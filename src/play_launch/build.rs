fn main() {
    // Link against composition_interfaces library
    println!("cargo:rustc-link-search=native=/opt/ros/humble/lib");
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rosidl_runtime_c");

    // Add RPATH for runtime library discovery (allows .deb package to work standalone)
    // Use old-style RPATH (not RUNPATH) so it applies to transitive dependencies
    println!("cargo:rustc-link-arg=-Wl,--disable-new-dtags");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/opt/ros/humble/lib");
}
