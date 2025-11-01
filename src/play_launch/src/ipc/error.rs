//! Error types for play_launch IPC

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Errors that can occur when reading /proc/[pid]/io for a specific PID
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Error)]
pub enum ProcIoError {
    /// Process not found (exited or invalid PID)
    #[error("Process not found")]
    ProcessNotFound,

    /// Permission denied (even with CAP_SYS_PTRACE)
    #[error("Permission denied")]
    PermissionDenied,

    /// /proc filesystem not available or not mounted
    #[error("/proc filesystem not available")]
    ProcNotAvailable,

    /// Error parsing /proc/[pid]/io content
    #[error("Parse error: {0}")]
    ParseError(String),

    /// I/O error reading file
    #[error("I/O error: {0}")]
    IoError(String),
}

impl From<std::io::Error> for ProcIoError {
    fn from(err: std::io::Error) -> Self {
        use std::io::ErrorKind;

        match err.kind() {
            ErrorKind::NotFound => ProcIoError::ProcessNotFound,
            ErrorKind::PermissionDenied => ProcIoError::PermissionDenied,
            _ => ProcIoError::IoError(err.to_string()),
        }
    }
}

/// Errors from the helper daemon itself
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Error)]
pub enum HelperError {
    /// IPC communication error
    #[error("IPC error: {0}")]
    IpcError(String),

    /// Protocol version mismatch
    #[error("Protocol version mismatch")]
    VersionMismatch,

    /// Invalid request
    #[error("Invalid request: {0}")]
    InvalidRequest(String),

    /// Internal error in helper daemon
    #[error("Internal error: {0}")]
    InternalError(String),

    /// Helper daemon not available or not running
    #[error("Helper daemon not available")]
    NotAvailable,

    /// Connection timeout
    #[error("Connection timeout")]
    Timeout,
}
