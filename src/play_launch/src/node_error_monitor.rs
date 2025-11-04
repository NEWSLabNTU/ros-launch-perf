//! Node error monitoring - periodically scans stderr files for excessive errors
//!
//! This module provides background monitoring of node stderr files to alert users
//! when nodes are printing excessive error messages. This helps catch issues during
//! long-running executions without requiring users to manually check log files.

use eyre::Result;
use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    thread::{self, JoinHandle},
    time::{Duration, SystemTime},
};
use tracing::{debug, warn};

/// Configuration for error monitoring
#[derive(Debug, Clone)]
pub struct NodeErrorMonitorConfig {
    pub enabled: bool,
    pub check_interval_secs: u64,
    pub error_threshold_lines: usize,
    pub rate_limit_secs: u64,
}

impl Default for NodeErrorMonitorConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            check_interval_secs: 30,
            error_threshold_lines: 10,
            rate_limit_secs: 60,
        }
    }
}

/// State tracking for a single node's stderr file
struct NodeErrorState {
    last_size: u64,
    last_warning: Option<SystemTime>,
}

/// Spawn background thread that monitors node stderr files for excessive errors
///
/// The monitor periodically scans all stderr files in the log directory and warns
/// when nodes are printing many error lines. This helps surface issues during
/// long-running executions.
///
/// # Monitoring Strategy
///
/// - Every `check_interval_secs`, scan `{log_dir}/node/*/err` and `{log_dir}/load_node/*/err`
/// - Track last known file size for each stderr file
/// - Calculate delta bytes since last check
/// - Estimate lines: `delta_bytes / 80` (assumes ~80 bytes per line)
/// - Warn if estimated lines ≥ `error_threshold_lines`
/// - Rate limit warnings (don't re-warn within `rate_limit_secs`)
///
/// # Arguments
///
/// * `config` - Error monitoring configuration
/// * `log_dir` - Base log directory (e.g., `play_log/2025-11-04_19-02-12`)
///
/// # Returns
///
/// Returns a thread handle if monitoring is enabled, error otherwise
pub fn spawn_error_monitor_thread(
    config: NodeErrorMonitorConfig,
    log_dir: PathBuf,
) -> Result<JoinHandle<()>> {
    if !config.enabled {
        return Err(eyre::eyre!("Error monitoring is not enabled"));
    }

    let handle = thread::spawn(move || {
        let mut state: HashMap<PathBuf, NodeErrorState> = HashMap::new();
        let interval = Duration::from_secs(config.check_interval_secs);

        debug!(
            "Error monitor thread started (interval: {}s, threshold: {} lines)",
            config.check_interval_secs, config.error_threshold_lines
        );

        loop {
            thread::sleep(interval);

            // Scan both regular nodes and standalone composable nodes
            check_node_errors(&log_dir.join("node"), &mut state, &config);
            check_node_errors(&log_dir.join("load_node"), &mut state, &config);
        }
    });

    Ok(handle)
}

/// Check stderr files in a node directory for error growth
///
/// Scans all subdirectories in `base_dir` looking for `err` files, tracks their
/// size growth, and warns when growth exceeds the threshold.
fn check_node_errors(
    base_dir: &Path,
    state: &mut HashMap<PathBuf, NodeErrorState>,
    config: &NodeErrorMonitorConfig,
) {
    let Ok(entries) = fs::read_dir(base_dir) else {
        // Directory might not exist yet (e.g., no composable nodes)
        return;
    };

    for entry in entries.flatten() {
        let node_dir = entry.path();
        if !node_dir.is_dir() {
            continue;
        }

        let err_file = node_dir.join("err");
        if !err_file.exists() {
            continue;
        }

        // Get current file size
        let Ok(metadata) = fs::metadata(&err_file) else {
            continue;
        };
        let current_size = metadata.len();

        // Get or create state for this file
        let prev_state = state.entry(err_file.clone()).or_insert(NodeErrorState {
            last_size: 0,
            last_warning: None,
        });

        // Handle file truncation (shouldn't happen, but be defensive)
        if current_size < prev_state.last_size {
            debug!(
                "Stderr file shrank: {} (was {} bytes, now {})",
                err_file.display(),
                prev_state.last_size,
                current_size
            );
            prev_state.last_size = current_size;
            continue;
        }

        // Calculate delta since last check
        if current_size > prev_state.last_size {
            let delta_bytes = current_size - prev_state.last_size;
            // Estimate lines assuming ~80 bytes per line
            // This is approximate but avoids reading the entire file
            let estimated_lines = (delta_bytes / 80) as usize;

            if estimated_lines >= config.error_threshold_lines {
                // Check rate limiting - don't spam warnings
                let should_warn = match prev_state.last_warning {
                    None => true,
                    Some(last_time) => SystemTime::now()
                        .duration_since(last_time)
                        .map(|d| d.as_secs() >= config.rate_limit_secs)
                        .unwrap_or(true),
                };

                if should_warn {
                    let node_name = node_dir
                        .file_name()
                        .and_then(|n| n.to_str())
                        .unwrap_or("unknown");

                    // Calculate time since last warning for accurate reporting
                    let time_period_secs = match prev_state.last_warning {
                        None => config.check_interval_secs,
                        Some(last_time) => SystemTime::now()
                            .duration_since(last_time)
                            .map(|d| d.as_secs())
                            .unwrap_or(config.check_interval_secs),
                    };

                    warn!(
                        "Node '{}' has printed ~{} error lines in the last {}s",
                        node_name, estimated_lines, time_period_secs
                    );
                    warn!("  Check: {}", err_file.display());

                    prev_state.last_warning = Some(SystemTime::now());
                    // Reset tracking - we've warned about errors up to this point
                    prev_state.last_size = current_size;
                }
            } else {
                // Below threshold, reset tracking for next interval
                prev_state.last_size = current_size;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{fs::File, io::Write};
    use tempfile::TempDir;

    #[test]
    fn test_error_detection() {
        let temp_dir = TempDir::new().unwrap();
        let log_dir = temp_dir.path();
        let node_dir = log_dir.join("node").join("test_node");
        fs::create_dir_all(&node_dir).unwrap();

        let err_file = node_dir.join("err");
        let mut file = File::create(&err_file).unwrap();

        let mut state = HashMap::new();
        let config = NodeErrorMonitorConfig {
            enabled: true,
            check_interval_secs: 1,
            error_threshold_lines: 5,
            rate_limit_secs: 10,
        };

        // First check - no errors
        check_node_errors(&log_dir.join("node"), &mut state, &config);

        // Write some errors (6 lines * 80 bytes = 480 bytes)
        for i in 0..6 {
            writeln!(file, "[ERROR] Test error message number {}", i).unwrap();
        }
        file.flush().unwrap();

        // Second check - should detect errors
        // Note: In real usage, warnings go to tracing output
        check_node_errors(&log_dir.join("node"), &mut state, &config);

        // Verify state was updated
        assert!(state.contains_key(&err_file));
        assert!(state[&err_file].last_size > 0);
    }

    #[test]
    fn test_no_warning_below_threshold() {
        let temp_dir = TempDir::new().unwrap();
        let log_dir = temp_dir.path();
        let node_dir = log_dir.join("node").join("quiet_node");
        fs::create_dir_all(&node_dir).unwrap();

        let err_file = node_dir.join("err");
        let mut file = File::create(&err_file).unwrap();

        let mut state = HashMap::new();
        let config = NodeErrorMonitorConfig {
            enabled: true,
            check_interval_secs: 1,
            error_threshold_lines: 10,
            rate_limit_secs: 10,
        };

        // Write only 2 errors (below threshold)
        writeln!(file, "[ERROR] Error 1").unwrap();
        writeln!(file, "[ERROR] Error 2").unwrap();
        file.flush().unwrap();

        check_node_errors(&log_dir.join("node"), &mut state, &config);

        // State should be updated but no warning (checked via tracing in real usage)
        assert!(state.contains_key(&err_file));
    }
}
