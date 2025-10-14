// Simplified bindings for composition_interfaces/srv/LoadNode
//
// This module provides a direct FFI-based approach to calling the LoadNode service
// without implementing the full rosidl_runtime_rs Message trait (which would require
// complex parameter conversion). Instead, we keep things simple for Phase 1.
//
// TODO: Add full parameter/remap support in a later phase

use std::ffi::{CStr, CString};

// ============================================================================
// FFI Declarations for composition_interfaces/srv/LoadNode
// ============================================================================

#[repr(C)]
pub struct rosidl_runtime_c__String {
    pub data: *mut std::os::raw::c_char,
    pub size: usize,
    pub capacity: usize,
}

#[repr(C)]
pub struct rosidl_runtime_c__String__Sequence {
    pub data: *mut rosidl_runtime_c__String,
    pub size: usize,
    pub capacity: usize,
}

#[repr(C)]
pub struct rcl_interfaces__msg__Parameter__Sequence {
    pub data: *mut std::ffi::c_void, // Simplified - we don't handle parameters yet
    pub size: usize,
    pub capacity: usize,
}

#[repr(C)]
pub struct composition_interfaces__srv__LoadNode_Request {
    pub package_name: rosidl_runtime_c__String,
    pub plugin_name: rosidl_runtime_c__String,
    pub node_name: rosidl_runtime_c__String,
    pub node_namespace: rosidl_runtime_c__String,
    pub log_level: u8,
    pub remap_rules: rosidl_runtime_c__String__Sequence,
    pub parameters: rcl_interfaces__msg__Parameter__Sequence,
    pub extra_arguments: rcl_interfaces__msg__Parameter__Sequence,
}

#[repr(C)]
pub struct composition_interfaces__srv__LoadNode_Response {
    pub success: bool,
    pub error_message: rosidl_runtime_c__String,
    pub full_node_name: rosidl_runtime_c__String,
    pub unique_id: u64,
}

#[link(name = "composition_interfaces__rosidl_generator_c")]
extern "C" {
    fn composition_interfaces__srv__LoadNode_Request__init(
        msg: *mut composition_interfaces__srv__LoadNode_Request,
    ) -> bool;

    fn composition_interfaces__srv__LoadNode_Request__fini(
        msg: *mut composition_interfaces__srv__LoadNode_Request,
    );

    fn composition_interfaces__srv__LoadNode_Response__init(
        msg: *mut composition_interfaces__srv__LoadNode_Response,
    ) -> bool;

    fn composition_interfaces__srv__LoadNode_Response__fini(
        msg: *mut composition_interfaces__srv__LoadNode_Response,
    );
}

#[link(name = "rosidl_runtime_c")]
extern "C" {
    fn rosidl_runtime_c__String__assign(
        string: *mut rosidl_runtime_c__String,
        value: *const std::os::raw::c_char,
    ) -> bool;

    fn rosidl_runtime_c__String__Sequence__init(
        seq: *mut rosidl_runtime_c__String__Sequence,
        size: usize,
    ) -> bool;

    fn rosidl_runtime_c__String__Sequence__fini(seq: *mut rosidl_runtime_c__String__Sequence);
}

#[link(name = "composition_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__composition_interfaces__srv__LoadNode(
    ) -> *const std::ffi::c_void;
}

// ============================================================================
// Safe Rust Wrappers
// ============================================================================

/// High-level request for loading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct LoadNodeRequest {
    pub package_name: String,
    pub plugin_name: String,
    pub node_name: String,
    pub node_namespace: String,
    pub log_level: u8,
    pub remap_rules: Vec<String>,
    // TODO: Add parameters and extra_arguments
}

/// Response from loading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct LoadNodeResponse {
    pub success: bool,
    pub error_message: String,
    pub full_node_name: String,
    pub unique_id: u64,
}

impl LoadNodeRequest {
    /// Create a new LoadNodeRequest
    pub fn new(
        package_name: String,
        plugin_name: String,
        node_name: String,
        node_namespace: String,
    ) -> Self {
        Self {
            package_name,
            plugin_name,
            node_name,
            node_namespace,
            log_level: 0,
            remap_rules: Vec::new(),
        }
    }

    /// Convert to FFI request structure
    ///
    /// # Safety
    /// The returned structure must be freed with `drop_ffi_request`
    pub unsafe fn to_ffi(&self) -> composition_interfaces__srv__LoadNode_Request {
        let mut request = std::mem::zeroed();
        if !composition_interfaces__srv__LoadNode_Request__init(&mut request) {
            panic!("Failed to initialize LoadNode_Request");
        }

        let package_cstr = CString::new(self.package_name.as_str()).unwrap();
        rosidl_runtime_c__String__assign(&mut request.package_name, package_cstr.as_ptr());

        let plugin_cstr = CString::new(self.plugin_name.as_str()).unwrap();
        rosidl_runtime_c__String__assign(&mut request.plugin_name, plugin_cstr.as_ptr());

        let node_name_cstr = CString::new(self.node_name.as_str()).unwrap();
        rosidl_runtime_c__String__assign(&mut request.node_name, node_name_cstr.as_ptr());

        let namespace_cstr = CString::new(self.node_namespace.as_str()).unwrap();
        rosidl_runtime_c__String__assign(&mut request.node_namespace, namespace_cstr.as_ptr());

        request.log_level = self.log_level;

        // Initialize remap_rules sequence
        if !self.remap_rules.is_empty() {
            rosidl_runtime_c__String__Sequence__init(
                &mut request.remap_rules,
                self.remap_rules.len(),
            );
            for (i, rule) in self.remap_rules.iter().enumerate() {
                let rule_cstr = CString::new(rule.as_str()).unwrap();
                let rule_ptr = request.remap_rules.data.add(i);
                rosidl_runtime_c__String__assign(rule_ptr, rule_cstr.as_ptr());
            }
        }

        // TODO: Initialize parameters and extra_arguments sequences

        request
    }

    /// Free FFI request structure
    ///
    /// # Safety
    /// The request must have been created by `to_ffi`
    pub unsafe fn drop_ffi_request(mut request: composition_interfaces__srv__LoadNode_Request) {
        composition_interfaces__srv__LoadNode_Request__fini(&mut request);
    }
}

impl LoadNodeResponse {
    /// Convert from FFI response structure
    ///
    /// # Safety
    /// The response must be a valid initialized response
    pub unsafe fn from_ffi(response: &composition_interfaces__srv__LoadNode_Response) -> Self {
        let error_message =
            if response.error_message.data.is_null() || response.error_message.size == 0 {
                String::new()
            } else {
                let slice = std::slice::from_raw_parts(
                    response.error_message.data as *const u8,
                    response.error_message.size,
                );
                String::from_utf8_lossy(slice).into_owned()
            };

        let full_node_name =
            if response.full_node_name.data.is_null() || response.full_node_name.size == 0 {
                String::new()
            } else {
                let slice = std::slice::from_raw_parts(
                    response.full_node_name.data as *const u8,
                    response.full_node_name.size,
                );
                String::from_utf8_lossy(slice).into_owned()
            };

        Self {
            success: response.success,
            error_message,
            full_node_name,
            unique_id: response.unique_id,
        }
    }

    /// Free FFI response structure
    ///
    /// # Safety
    /// The response must be a valid initialized response
    pub unsafe fn drop_ffi_response(mut response: composition_interfaces__srv__LoadNode_Response) {
        composition_interfaces__srv__LoadNode_Response__fini(&mut response);
    }
}

/// Get the type support handle for LoadNode service
///
/// This is needed for rclrs service client creation
pub fn get_load_node_type_support() -> *const std::ffi::c_void {
    unsafe {
        rosidl_typesupport_c__get_service_type_support_handle__composition_interfaces__srv__LoadNode(
        )
    }
}
