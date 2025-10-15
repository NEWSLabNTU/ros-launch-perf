use csv::Writer;
use eyre::{Result, WrapErr};
use std::{
    collections::HashMap,
    fs::{self, File},
    path::PathBuf,
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, SystemTime},
};
use sysinfo::{Pid, System};
use tracing::{debug, warn};

/// Resource metrics for a single process at a point in time
#[derive(Debug, Clone)]
pub struct ResourceMetrics {
    pub timestamp: SystemTime,
    pub pid: u32,
    #[allow(dead_code)] // Used in CSV output
    pub node_name: String,

    // CPU metrics
    pub cpu_percent: f64,
    pub cpu_user_time: u64, // Total accumulated runtime in seconds

    // Memory metrics
    pub rss_bytes: u64, // Resident Set Size
    pub vms_bytes: u64, // Virtual Memory Size

    // I/O metrics
    pub io_read_bytes: u64,
    pub io_write_bytes: u64,

    // Process info
    pub state: ProcessState,
    pub num_threads: u32,
    pub num_fds: u32,       // File descriptors
    pub num_processes: u32, // Number of processes in tree (parent + children)
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
    pub log_dir: PathBuf,
}

/// Resource monitor with sysinfo integration
pub struct ResourceMonitor {
    system: System,
    log_dir: PathBuf,
    csv_writers: HashMap<String, Writer<File>>,
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
    pub fn new(log_dir: PathBuf) -> Result<Self> {
        Ok(Self {
            system: System::new_all(),
            log_dir,
            csv_writers: HashMap::new(),
        })
    }

    fn collect_metrics(&mut self, pid: u32, node_name: &str) -> Result<ResourceMetrics> {
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

        Ok(ResourceMetrics {
            timestamp: SystemTime::now(),
            pid,
            node_name: node_name.to_string(),
            cpu_percent,
            cpu_user_time,
            rss_bytes,
            vms_bytes,
            io_read_bytes,
            io_write_bytes,
            state,
            num_threads,
            num_fds,
            num_processes,
        })
    }

    fn write_csv(&mut self, node_name: &str, metrics: &ResourceMetrics) -> Result<()> {
        // Get or create CSV writer for this node
        if !self.csv_writers.contains_key(node_name) {
            let csv_path = self.get_csv_path(node_name)?;

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
                    "state",
                    "num_threads",
                    "num_fds",
                    "num_processes",
                ])
                .wrap_err("Failed to write CSV header")?;

            writer.flush().wrap_err("Failed to flush CSV header")?;
            self.csv_writers.insert(node_name.to_string(), writer);
        }

        let writer = self
            .csv_writers
            .get_mut(node_name)
            .ok_or_else(|| eyre::eyre!("CSV writer not found for {}", node_name))?;

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
                metrics.state.as_str().to_string(),
                metrics.num_threads.to_string(),
                metrics.num_fds.to_string(),
                metrics.num_processes.to_string(),
            ])
            .wrap_err_with(|| format!("Failed to write CSV row for {}", node_name))?;

        writer
            .flush()
            .wrap_err_with(|| format!("Failed to flush CSV for {}", node_name))?;

        Ok(())
    }

    fn get_csv_path(&self, node_name: &str) -> Result<PathBuf> {
        // Create path: metrics/<sanitized_node_name>.csv
        let sanitized = sanitize_node_name(node_name);
        let csv_path = self
            .log_dir
            .join("metrics")
            .join(format!("{}.csv", sanitized));
        Ok(csv_path)
    }
}

/// Spawn monitoring thread
pub fn spawn_monitor_thread(
    config: MonitorConfig,
    process_registry: Arc<Mutex<HashMap<u32, String>>>,
) -> Result<JoinHandle<()>> {
    if !config.enabled {
        return Err(eyre::eyre!("Monitoring is not enabled"));
    }

    let handle = thread::spawn(move || {
        let mut monitor = match ResourceMonitor::new(config.log_dir.clone()) {
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
            // Refresh all processes in one call for efficiency
            // In sysinfo 0.32, we need to specify which processes to update
            monitor
                .system
                .refresh_processes(sysinfo::ProcessesToUpdate::All, false);

            let processes = process_registry.lock().unwrap().clone();

            for (pid, node_name) in processes {
                match monitor.collect_metrics(pid, &node_name) {
                    Ok(metrics) => {
                        if let Err(e) = monitor.write_csv(&node_name, &metrics) {
                            warn!("Failed to write metrics for {}: {}", node_name, e);
                        }
                    }
                    Err(e) => {
                        debug!(
                            "Failed to collect metrics for {} (pid {}): {}",
                            node_name, pid, e
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

/// Sanitize node name for use in file paths
fn sanitize_node_name(node_name: &str) -> String {
    node_name.replace(['/', '\\', ':', '*', '?', '"', '<', '>', '|'], "_")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sanitize_node_name() {
        assert_eq!(
            sanitize_node_name("/planning/behavior"),
            "_planning_behavior"
        );
        assert_eq!(sanitize_node_name("simple_node"), "simple_node");
        assert_eq!(sanitize_node_name("/a/b/c"), "_a_b_c");
    }

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
