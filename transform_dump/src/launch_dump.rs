use serde::Deserialize;
use std::{collections::HashMap, path::PathBuf};

#[derive(Debug, Clone, Deserialize)]
pub struct LaunchDump {
    pub process: Vec<ProcessInfo>,
    pub file_data: HashMap<PathBuf, String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ProcessInfo {
    pub kind: ProcessKind,
    pub cmdline: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcessKind {
    ComposableNodeContainer,
    LifecycleNode,
    Node,
    Unknown,
}
