fn main() {
    // Link against composition_interfaces library
    println!("cargo:rustc-link-search=native=/opt/ros/humble/lib");
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rosidl_runtime_c");
}
