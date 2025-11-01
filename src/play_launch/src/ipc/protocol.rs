//! Protocol message definitions for play_launch IPC

use super::error::{HelperError, ProcIoError};
use serde::{Deserialize, Serialize};

/// Request from play_launch to helper daemon
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum Request {
    /// Read I/O stats for a single PID
    ReadProcIo { pid: u32 },

    /// Read I/O stats for multiple PIDs (batch request for efficiency)
    ReadProcIoBatch { pids: Vec<u32> },

    /// Health check - helper responds with Pong
    Ping,

    /// Request graceful shutdown
    Shutdown,
}

/// Response from helper daemon to play_launch
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum Response {
    /// I/O statistics for a single PID
    ProcIo(ProcIoResult),

    /// Batch response with results for multiple PIDs
    ProcIoBatch(Vec<ProcIoResult>),

    /// Response to Ping
    Pong,

    /// Acknowledgment of shutdown request
    ShutdownAck,

    /// Error occurred while processing request
    Error(HelperError),
}

/// Result for a single PID I/O query
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ProcIoResult {
    pub pid: u32,
    pub result: Result<ProcIoStats, ProcIoError>,
}

/// I/O statistics from /proc/[pid]/io
///
/// These fields mirror the contents of /proc/[pid]/io:
/// - rchar: Total bytes read (all I/O including cache)
/// - wchar: Total bytes written (all I/O including cache)
/// - syscr: Number of read syscalls
/// - syscw: Number of write syscalls
/// - read_bytes: Bytes actually read from storage
/// - write_bytes: Bytes actually written to storage
/// - cancelled_write_bytes: Bytes written but later truncated
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct ProcIoStats {
    pub rchar: u64,
    pub wchar: u64,
    pub syscr: u64,
    pub syscw: u64,
    pub read_bytes: u64,
    pub write_bytes: u64,
    pub cancelled_write_bytes: u64,
}

impl ProcIoStats {
    /// Parse /proc/[pid]/io content
    pub fn parse(content: &str) -> Result<Self, String> {
        let mut stats = ProcIoStats {
            rchar: 0,
            wchar: 0,
            syscr: 0,
            syscw: 0,
            read_bytes: 0,
            write_bytes: 0,
            cancelled_write_bytes: 0,
        };

        for line in content.lines() {
            let parts: Vec<&str> = line.split(':').collect();
            if parts.len() != 2 {
                continue;
            }

            let key = parts[0].trim();
            let value = parts[1]
                .trim()
                .parse::<u64>()
                .map_err(|e| format!("Failed to parse {}: {}", key, e))?;

            match key {
                "rchar" => stats.rchar = value,
                "wchar" => stats.wchar = value,
                "syscr" => stats.syscr = value,
                "syscw" => stats.syscw = value,
                "read_bytes" => stats.read_bytes = value,
                "write_bytes" => stats.write_bytes = value,
                "cancelled_write_bytes" => stats.cancelled_write_bytes = value,
                _ => {} // Ignore unknown fields
            }
        }

        Ok(stats)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_proc_io_stats() {
        let content = r#"rchar: 4208
wchar: 100
syscr: 10
syscw: 5
read_bytes: 0
write_bytes: 4096
cancelled_write_bytes: 0
"#;

        let stats = ProcIoStats::parse(content).unwrap();
        assert_eq!(stats.rchar, 4208);
        assert_eq!(stats.wchar, 100);
        assert_eq!(stats.syscr, 10);
        assert_eq!(stats.syscw, 5);
        assert_eq!(stats.read_bytes, 0);
        assert_eq!(stats.write_bytes, 4096);
        assert_eq!(stats.cancelled_write_bytes, 0);
    }

    #[test]
    fn test_parse_invalid_format() {
        let content = "invalid content";
        let stats = ProcIoStats::parse(content).unwrap();
        // Should return zeros for all fields
        assert_eq!(stats.rchar, 0);
    }
}
