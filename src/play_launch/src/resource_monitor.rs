use csv::Writer;
use eyre::{Result, WrapErr};
use nvml_wrapper::{
    enum_wrappers::device::{Clock, TemperatureSensor},
    Nvml,
};
use std::{
    collections::HashMap,
    fs::{self, File},
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, SystemTime},
};
use sysinfo::{Pid, System};
use tracing::{debug, warn};

/// GPU metrics tuple: (memory_bytes, gpu_util%, mem_util%, temp_celsius, power_mw, graphics_clock_mhz, memory_clock_mhz)
type GpuMetricsTuple = (
    Option<u64>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
    Option<u32>,
);

/// Resource metrics for a single process at a point in time
#[derive(Debug, Clone)]
pub struct ResourceMetrics {
    pub timestamp: SystemTime,
    pub pid: u32,

    // CPU metrics
    pub cpu_percent: f64,
    pub cpu_user_time: u64, // Total accumulated runtime in seconds

    // Memory metrics
    pub rss_bytes: u64, // Resident Set Size
    pub vms_bytes: u64, // Virtual Memory Size

    // I/O metrics (disk only - from sysinfo)
    pub io_read_bytes: u64,
    pub io_write_bytes: u64,

    // Total I/O metrics (all I/O including network - from /proc/[pid]/io)
    pub total_read_bytes: u64,  // rchar field
    pub total_write_bytes: u64, // wchar field

    // I/O rates (bytes per second, calculated from previous sample)
    pub total_read_rate_bps: Option<f64>,
    pub total_write_rate_bps: Option<f64>,

    // Process info
    pub state: ProcessState,
    pub num_threads: u32,
    pub num_fds: u32,       // File descriptors
    pub num_processes: u32, // Number of processes in tree (parent + children)

    // GPU metrics (optional - only populated if GPU monitoring enabled)
    pub gpu_memory_bytes: Option<u64>,
    pub gpu_utilization_percent: Option<u32>,
    pub gpu_memory_utilization_percent: Option<u32>,
    pub gpu_temperature_celsius: Option<u32>,
    pub gpu_power_milliwatts: Option<u32>,
    pub gpu_graphics_clock_mhz: Option<u32>,
    pub gpu_memory_clock_mhz: Option<u32>,

    // Network metrics (Linux-specific, counts of active connections)
    pub tcp_connections: u32,
    pub udp_connections: u32,
}

#[derive(Debug, Clone, Copy)]
pub enum ProcessState {
    Running,
    Sleeping,
    #[allow(dead_code)] // May be used in future OS implementations
    Waiting,
    Zombie,
    Stopped,
    Unknown,
}

impl ProcessState {
    fn as_str(&self) -> &'static str {
        match self {
            ProcessState::Running => "Running",
            ProcessState::Sleeping => "Sleeping",
            ProcessState::Waiting => "Waiting",
            ProcessState::Zombie => "Zombie",
            ProcessState::Stopped => "Stopped",
            ProcessState::Unknown => "Unknown",
        }
    }
}

/// Configuration for resource monitoring
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub enabled: bool,
    pub sample_interval_ms: u64,
}

/// Previous sample for rate calculation
#[derive(Debug, Clone)]
struct PreviousSample {
    timestamp: SystemTime,
    total_read_bytes: u64,
    total_write_bytes: u64,
}

/// Resource monitor with sysinfo and NVML integration
pub struct ResourceMonitor {
    system: System,
    csv_writers: HashMap<u32, Writer<File>>, // PID -> CSV writer
    nvml: Option<Nvml>,
    gpu_device_count: u32,
    previous_samples: HashMap<u32, PreviousSample>, // PID -> previous I/O sample
}

/// Count network connections from /proc/<pid>/net/{tcp,udp} files
/// Each line (except header) represents a connection
#[cfg(target_os = "linux")]
fn count_connections_in_file(path: &str) -> u32 {
    match std::fs::read_to_string(path) {
        Ok(content) => {
            // Count lines minus 1 for header
            let line_count = content.lines().count();
            if line_count > 0 {
                (line_count - 1) as u32
            } else {
                0
            }
        }
        Err(_) => 0, // File may not exist if no connections
    }
}

/// Find all subprocess PIDs recursively (Linux only)
#[cfg(target_os = "linux")]
fn find_subprocess_pids(parent_pid: u32) -> Vec<u32> {
    let mut pids = Vec::new();

    // Read /proc/<pid>/task/<tid>/children for each thread
    let task_dir = format!("/proc/{}/task", parent_pid);
    if let Ok(entries) = std::fs::read_dir(task_dir) {
        for entry in entries.flatten() {
            let children_path = entry.path().join("children");
            if let Ok(content) = std::fs::read_to_string(&children_path) {
                for pid_str in content.split_whitespace() {
                    if let Ok(child_pid) = pid_str.parse::<u32>() {
                        pids.push(child_pid);
                        // Recursively find grandchildren
                        let mut grandchildren = find_subprocess_pids(child_pid);
                        pids.append(&mut grandchildren);
                    }
                }
            }
        }
    }

    pids
}

#[cfg(not(target_os = "linux"))]
fn find_subprocess_pids(_parent_pid: u32) -> Vec<u32> {
    Vec::new()
}

impl ResourceMonitor {
    pub fn new(nvml: Option<Nvml>) -> Result<Self> {
        // Use System::new() instead of new_all() to avoid loading everything upfront
        // We'll refresh only the processes we need in the monitoring loop

        // Get GPU device count if NVML available
        let gpu_device_count = if let Some(ref nvml) = nvml {
            nvml.device_count()
                .wrap_err("Failed to get GPU device count")?
        } else {
            0
        };

        debug!(
            "ResourceMonitor initialized with {} GPU devices",
            gpu_device_count
        );

        Ok(Self {
            system: System::new(),
            csv_writers: HashMap::new(),
            nvml,
            gpu_device_count,
            previous_samples: HashMap::new(),
        })
    }

    fn collect_metrics(&mut self, pid: u32) -> Result<ResourceMetrics> {
        let pid_obj = Pid::from_u32(pid);

        // Get process from system
        let process = self
            .system
            .process(pid_obj)
            .ok_or_else(|| eyre::eyre!("Process {} not found", pid))?;

        // Discover subprocesses for aggregation
        let subprocess_pids = find_subprocess_pids(pid);

        // Aggregate metrics from parent + all children
        let mut cpu_percent = process.cpu_usage() as f64;
        let mut cpu_user_time = process.run_time();
        let mut rss_bytes = process.memory();
        let mut vms_bytes = process.virtual_memory();
        let disk_usage = process.disk_usage();
        let mut io_read_bytes = disk_usage.total_read_bytes;
        let mut io_write_bytes = disk_usage.total_written_bytes;
        let mut num_threads = process.tasks().map(|t| t.len() as u32).unwrap_or(1);

        // Aggregate from subprocesses
        for child_pid in &subprocess_pids {
            let child_pid_obj = Pid::from_u32(*child_pid);
            if let Some(child_process) = self.system.process(child_pid_obj) {
                cpu_percent += child_process.cpu_usage() as f64;
                cpu_user_time += child_process.run_time();
                rss_bytes += child_process.memory();
                vms_bytes += child_process.virtual_memory();
                let child_disk = child_process.disk_usage();
                io_read_bytes += child_disk.total_read_bytes;
                io_write_bytes += child_disk.total_written_bytes;
                num_threads += child_process.tasks().map(|t| t.len() as u32).unwrap_or(1);
            }
        }

        // Count open file descriptors (Linux-specific)
        #[cfg(target_os = "linux")]
        let num_fds = std::fs::read_dir(format!("/proc/{}/fd", pid))
            .map(|entries| entries.count() as u32)
            .unwrap_or(0);
        #[cfg(not(target_os = "linux"))]
        let num_fds = 0;

        // Map sysinfo process status to our ProcessState enum
        let state = match process.status() {
            sysinfo::ProcessStatus::Run => ProcessState::Running,
            sysinfo::ProcessStatus::Sleep => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Idle => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Zombie => ProcessState::Zombie,
            sysinfo::ProcessStatus::Stop => ProcessState::Stopped,
            _ => ProcessState::Unknown,
        };

        let num_processes = 1 + subprocess_pids.len() as u32;

        // Collect GPU metrics if NVML available
        let (
            gpu_memory_bytes,
            gpu_utilization_percent,
            gpu_memory_utilization_percent,
            gpu_temperature_celsius,
            gpu_power_milliwatts,
            gpu_graphics_clock_mhz,
            gpu_memory_clock_mhz,
        ) = self.collect_gpu_metrics(pid)?;

        // Collect network connection counts
        let (tcp_connections, udp_connections) = self.collect_network_connections(pid);

        // Parse /proc/[pid]/io for total I/O (including network)
        let (total_read_bytes, total_write_bytes) = self.parse_proc_io(pid).unwrap_or((0, 0));

        // Calculate I/O rates from previous sample
        let current_time = SystemTime::now();
        let (total_read_rate_bps, total_write_rate_bps) = if let Some(prev) =
            self.previous_samples.get(&pid)
        {
            let time_diff = current_time
                .duration_since(prev.timestamp)
                .unwrap_or(Duration::from_secs(0))
                .as_secs_f64();

            if time_diff > 0.0 {
                let read_rate =
                    (total_read_bytes.saturating_sub(prev.total_read_bytes)) as f64 / time_diff;
                let write_rate =
                    (total_write_bytes.saturating_sub(prev.total_write_bytes)) as f64 / time_diff;
                (Some(read_rate), Some(write_rate))
            } else {
                (None, None)
            }
        } else {
            // No previous sample, can't calculate rate
            (None, None)
        };

        // Store current sample for next iteration
        self.previous_samples.insert(
            pid,
            PreviousSample {
                timestamp: current_time,
                total_read_bytes,
                total_write_bytes,
            },
        );

        Ok(ResourceMetrics {
            timestamp: current_time,
            pid,
            cpu_percent,
            cpu_user_time,
            rss_bytes,
            vms_bytes,
            io_read_bytes,
            io_write_bytes,
            total_read_bytes,
            total_write_bytes,
            total_read_rate_bps,
            total_write_rate_bps,
            state,
            num_threads,
            num_fds,
            num_processes,
            gpu_memory_bytes,
            gpu_utilization_percent,
            gpu_memory_utilization_percent,
            gpu_temperature_celsius,
            gpu_power_milliwatts,
            gpu_graphics_clock_mhz,
            gpu_memory_clock_mhz,
            tcp_connections,
            udp_connections,
        })
    }

    fn collect_gpu_metrics(&self, pid: u32) -> Result<GpuMetricsTuple> {
        let nvml = match &self.nvml {
            Some(nvml) => nvml,
            None => {
                // No NVML, return all None
                return Ok((None, None, None, None, None, None, None));
            }
        };

        // Search all GPU devices for this process
        for device_index in 0..self.gpu_device_count {
            let device = nvml
                .device_by_index(device_index)
                .wrap_err_with(|| format!("Failed to get GPU device {}", device_index))?;

            // Get running compute processes
            let processes = device.running_compute_processes().wrap_err_with(|| {
                format!("Failed to get running processes for GPU {}", device_index)
            })?;

            // Check if our PID is using this GPU
            for proc in processes {
                if proc.pid == pid {
                    // Found process on this GPU - collect metrics
                    let utilization = device.utilization_rates().ok();
                    let temperature = device.temperature(TemperatureSensor::Gpu).ok();
                    let power = device.power_usage().ok();
                    let graphics_clock = device.clock_info(Clock::Graphics).ok();
                    let memory_clock = device.clock_info(Clock::Memory).ok();

                    // Extract GPU memory as u64 from UsedGpuMemory enum
                    let gpu_mem_bytes = match proc.used_gpu_memory {
                        nvml_wrapper::enums::device::UsedGpuMemory::Used(bytes) => Some(bytes),
                        nvml_wrapper::enums::device::UsedGpuMemory::Unavailable => None,
                    };

                    return Ok((
                        gpu_mem_bytes,
                        utilization.as_ref().map(|u| u.gpu),
                        utilization.as_ref().map(|u| u.memory),
                        temperature,
                        power,
                        graphics_clock,
                        memory_clock,
                    ));
                }
            }
        }

        // Process not found on any GPU
        Ok((None, None, None, None, None, None, None))
    }

    /// Count TCP and UDP connections for a process (Linux-specific)
    #[cfg(target_os = "linux")]
    fn collect_network_connections(&self, pid: u32) -> (u32, u32) {
        let tcp_count = count_connections_in_file(&format!("/proc/{}/net/tcp", pid))
            + count_connections_in_file(&format!("/proc/{}/net/tcp6", pid));
        let udp_count = count_connections_in_file(&format!("/proc/{}/net/udp", pid))
            + count_connections_in_file(&format!("/proc/{}/net/udp6", pid));

        (tcp_count, udp_count)
    }

    #[cfg(not(target_os = "linux"))]
    fn collect_network_connections(&self, _pid: u32) -> (u32, u32) {
        (0, 0)
    }

    /// Parse /proc/[pid]/io for total I/O bytes (rchar/wchar) including network
    /// Returns (rchar, wchar) which are cumulative byte counters
    #[cfg(target_os = "linux")]
    fn parse_proc_io(&self, pid: u32) -> Result<(u64, u64)> {
        let io_path = format!("/proc/{}/io", pid);
        let content = std::fs::read_to_string(&io_path)
            .wrap_err_with(|| format!("Failed to read {}", io_path))?;

        let mut rchar = 0u64;
        let mut wchar = 0u64;

        for line in content.lines() {
            let parts: Vec<&str> = line.split(':').collect();
            if parts.len() != 2 {
                continue;
            }

            let key = parts[0].trim();
            let value = parts[1].trim().parse::<u64>().unwrap_or(0);

            match key {
                "rchar" => rchar = value,
                "wchar" => wchar = value,
                _ => {}
            }
        }

        Ok((rchar, wchar))
    }

    #[cfg(not(target_os = "linux"))]
    fn parse_proc_io(&self, _pid: u32) -> Result<(u64, u64)> {
        Ok((0, 0))
    }

    fn write_csv(&mut self, output_dir: &Path, metrics: &ResourceMetrics) -> Result<()> {
        let pid = metrics.pid;

        // Get or create CSV writer for this PID
        if !self.csv_writers.contains_key(&pid) {
            let csv_path = self.get_csv_path(output_dir)?;

            // Create parent directories
            if let Some(parent) = csv_path.parent() {
                fs::create_dir_all(parent).wrap_err_with(|| {
                    format!("Failed to create directory: {}", parent.display())
                })?;
            }

            let file = File::create(&csv_path)
                .wrap_err_with(|| format!("Failed to create CSV file: {}", csv_path.display()))?;
            let mut writer = Writer::from_writer(file);

            // Write header
            writer
                .write_record([
                    "timestamp",
                    "pid",
                    "cpu_percent",
                    "cpu_user_secs",
                    "rss_bytes",
                    "vms_bytes",
                    "io_read_bytes",
                    "io_write_bytes",
                    "total_read_bytes",
                    "total_write_bytes",
                    "total_read_rate_bps",
                    "total_write_rate_bps",
                    "state",
                    "num_threads",
                    "num_fds",
                    "num_processes",
                    "gpu_memory_bytes",
                    "gpu_utilization_percent",
                    "gpu_memory_utilization_percent",
                    "gpu_temperature_celsius",
                    "gpu_power_milliwatts",
                    "gpu_graphics_clock_mhz",
                    "gpu_memory_clock_mhz",
                    "tcp_connections",
                    "udp_connections",
                ])
                .wrap_err("Failed to write CSV header")?;

            writer.flush().wrap_err("Failed to flush CSV header")?;
            self.csv_writers.insert(pid, writer);
        }

        let writer = self
            .csv_writers
            .get_mut(&pid)
            .ok_or_else(|| eyre::eyre!("CSV writer not found for PID {}", pid))?;

        // Format timestamp
        let timestamp_str = format_timestamp(metrics.timestamp);

        // Write data row
        writer
            .write_record(&[
                timestamp_str,
                metrics.pid.to_string(),
                format!("{:.2}", metrics.cpu_percent),
                metrics.cpu_user_time.to_string(),
                metrics.rss_bytes.to_string(),
                metrics.vms_bytes.to_string(),
                metrics.io_read_bytes.to_string(),
                metrics.io_write_bytes.to_string(),
                metrics.total_read_bytes.to_string(),
                metrics.total_write_bytes.to_string(),
                metrics
                    .total_read_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                metrics
                    .total_write_rate_bps
                    .map(|v| format!("{:.2}", v))
                    .unwrap_or_default(),
                metrics.state.as_str().to_string(),
                metrics.num_threads.to_string(),
                metrics.num_fds.to_string(),
                metrics.num_processes.to_string(),
                metrics
                    .gpu_memory_bytes
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_utilization_percent
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_memory_utilization_percent
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_temperature_celsius
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_power_milliwatts
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_graphics_clock_mhz
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics
                    .gpu_memory_clock_mhz
                    .map(|v| v.to_string())
                    .unwrap_or_default(),
                metrics.tcp_connections.to_string(),
                metrics.udp_connections.to_string(),
            ])
            .wrap_err_with(|| format!("Failed to write CSV row for PID {}", pid))?;

        writer
            .flush()
            .wrap_err_with(|| format!("Failed to flush CSV for PID {}", pid))?;

        Ok(())
    }

    fn get_csv_path(&self, output_dir: &Path) -> Result<PathBuf> {
        // Create path: <output_dir>/metrics.csv
        let csv_path = output_dir.join("metrics.csv");
        Ok(csv_path)
    }
}

/// Spawn monitoring thread
pub fn spawn_monitor_thread(
    config: MonitorConfig,
    process_registry: Arc<Mutex<HashMap<u32, PathBuf>>>,
    nvml: Option<Nvml>,
) -> Result<JoinHandle<()>> {
    if !config.enabled {
        return Err(eyre::eyre!("Monitoring is not enabled"));
    }

    let handle = thread::spawn(move || {
        let mut monitor = match ResourceMonitor::new(nvml) {
            Ok(m) => m,
            Err(e) => {
                warn!("Failed to create ResourceMonitor: {}", e);
                return;
            }
        };

        let interval = Duration::from_millis(config.sample_interval_ms);
        debug!(
            "Monitoring thread started with interval: {}ms",
            config.sample_interval_ms
        );

        loop {
            // Get snapshot of current processes to monitor
            let processes = process_registry.lock().unwrap().clone();

            // Refresh only the specific processes we're monitoring, not all system processes
            // This is much more efficient than refreshing all processes
            let pids_to_refresh: Vec<Pid> =
                processes.keys().map(|&pid| Pid::from_u32(pid)).collect();
            if !pids_to_refresh.is_empty() {
                monitor
                    .system
                    .refresh_processes(sysinfo::ProcessesToUpdate::Some(&pids_to_refresh), true);
            }

            for (pid, output_dir) in processes {
                match monitor.collect_metrics(pid) {
                    Ok(metrics) => {
                        if let Err(e) = monitor.write_csv(&output_dir, &metrics) {
                            warn!(
                                "Failed to write metrics for PID {} ({}): {}",
                                pid,
                                output_dir.display(),
                                e
                            );
                        }
                    }
                    Err(e) => {
                        debug!(
                            "Failed to collect metrics for PID {} ({}): {}",
                            pid,
                            output_dir.display(),
                            e
                        );
                        // Process may have exited, we'll keep trying in case it comes back
                    }
                }
            }

            thread::sleep(interval);
        }
    });

    Ok(handle)
}

/// Format SystemTime as ISO 8601 string
fn format_timestamp(time: SystemTime) -> String {
    match time.duration_since(SystemTime::UNIX_EPOCH) {
        Ok(duration) => {
            let secs = duration.as_secs();
            let millis = duration.subsec_millis();
            // Format as YYYY-MM-DDTHH:MM:SS.mmmZ
            let datetime =
                chrono::DateTime::<chrono::Utc>::from_timestamp(secs as i64, millis * 1_000_000)
                    .unwrap_or_default();
            datetime.to_rfc3339_opts(chrono::SecondsFormat::Millis, true)
        }
        Err(_) => String::from("1970-01-01T00:00:00.000Z"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_timestamp() {
        let time = SystemTime::UNIX_EPOCH + Duration::from_secs(1234567890);
        let formatted = format_timestamp(time);
        assert!(formatted.starts_with("2009-02-13"));
    }

    #[test]
    fn test_process_state_as_str() {
        assert_eq!(ProcessState::Running.as_str(), "Running");
        assert_eq!(ProcessState::Sleeping.as_str(), "Sleeping");
        assert_eq!(ProcessState::Zombie.as_str(), "Zombie");
    }
}
