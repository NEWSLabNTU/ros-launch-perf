//! Module for reading /proc/[pid]/io files

use play_launch::ipc::{ProcIoError, ProcIoStats};
use tracing::warn;

/// Read I/O statistics from /proc/[pid]/io
///
/// This function reads and parses the /proc/[pid]/io file which requires
/// CAP_SYS_PTRACE capability when the target process has capabilities set
/// or its dumpable flag is cleared.
pub fn read_proc_io_stats(pid: u32) -> Result<ProcIoStats, ProcIoError> {
    let io_path = format!("/proc/{}/io", pid);

    // Read file content
    let content = std::fs::read_to_string(&io_path).map_err(|e| {
        use std::io::ErrorKind;
        match e.kind() {
            ErrorKind::NotFound => ProcIoError::ProcessNotFound,
            ErrorKind::PermissionDenied => {
                warn!(
                    "Permission denied reading {}. CAP_SYS_PTRACE may not be set on helper binary.",
                    io_path
                );
                ProcIoError::PermissionDenied
            }
            _ => ProcIoError::IoError(e.to_string()),
        }
    })?;

    // Parse the content
    ProcIoStats::parse(&content).map_err(ProcIoError::ParseError)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_own_proc_io() {
        // Should be able to read our own process without special permissions
        let pid = std::process::id();
        let result = read_proc_io_stats(pid);

        // Should succeed or fail with permission denied (but not process not found)
        match result {
            Ok(stats) => {
                // Basic sanity check (rchar is u64, always >= 0)
                println!("Successfully read own /proc/[pid]/io: {:?}", stats);
            }
            Err(ProcIoError::PermissionDenied) => {
                println!("Permission denied (expected if no CAP_SYS_PTRACE)");
            }
            Err(e) => {
                panic!("Unexpected error: {:?}", e);
            }
        }
    }

    #[test]
    fn test_read_nonexistent_process() {
        // PID 999999 should not exist
        let result = read_proc_io_stats(999999);

        assert!(matches!(result, Err(ProcIoError::ProcessNotFound)));
    }
}
