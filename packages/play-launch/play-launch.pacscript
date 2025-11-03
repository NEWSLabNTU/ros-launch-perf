#!/bin/bash

# play-launch - ROS 2 launch inspection tool
pkgname="play-launch"
gives="play_launch"
pkgver="0.1.0"
pkgrel="1"
epoch="0"
pkgdesc="ROS 2 launch inspection tool with resource monitoring"
arch=("amd64" "arm64")
url="https://github.com/NEWSLabNTU/ros-launch-perf"
license=("BSD")
maintainer="Jerry Lin <jerry73204@gmail.com>"

depends=("ros-humble-desktop")
makedepends=("git" "python3-pip" "python3-dev" "build-essential" "cmake" "curl" "pkg-config" "just")
optdepends=("nvidia-utils: For NVIDIA GPU monitoring")

source=("${url}/archive/refs/tags/v${pkgver}.tar.gz")
sha256sums=("62fb035f6c41e943103b41a6c69b43bcd7787a6f524f95192707066be14d3ca8")

prepare() {
  cd "${srcdir}" || exit 1

  # Handle extracted directory
  [[ -d "ros-launch-perf-${pkgver}" ]] && cd "ros-launch-perf-${pkgver}"

  # Initialize submodules
  git submodule update --init --recursive || {
    cd "${srcdir}"
    rm -rf "ros-launch-perf-${pkgver}"
    git clone --recursive --branch "v${pkgver}" "${url}" "ros-launch-perf-${pkgver}"
    cd "ros-launch-perf-${pkgver}"
  }
}

build() {
  cd "${srcdir}/ros-launch-perf-${pkgver}" || exit 1

  # Install Rust
  if ! command -v rustc &>/dev/null; then
    fancy_message info "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
  fi
  export PATH="$HOME/.cargo/bin:$PATH"

  # Install Python deps
  fancy_message info "Installing Python dependencies..."
  pip3 install --user --upgrade \
    "git+https://github.com/colcon/colcon-cargo.git" \
    "git+https://github.com/colcon/colcon-ros-cargo.git" \
    pyyaml lark packaging plotly pandas &>/dev/null

  # Install cargo-ament-build
  command -v cargo-ament-build &>/dev/null || cargo install cargo-ament-build

  # Source ROS 2
  [[ -f "/opt/ros/humble/setup.bash" ]] || {
    fancy_message error "ROS 2 Humble not found"
    return 1
  }
  source /opt/ros/humble/setup.bash

  export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
  export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:${PYTHONPATH}"

  # Build
  fancy_message info "Building (this takes 10-30 minutes)..."
  just build 2>&1 | tee build.log || {
    fancy_message error "Build failed"
    tail -50 build.log
    return 1
  }
}

install() {
  cd "${srcdir}/ros-launch-perf-${pkgver}" || exit 1

  # Install binaries
  install -Dm755 "install/play_launch/lib/play_launch/play_launch" \
    "${pkgdir}/usr/bin/play_launch"
  install -Dm755 "install/play_launch/lib/play_launch/play_launch_io_helper" \
    "${pkgdir}/usr/lib/${pkgname}/play_launch_io_helper"

  # Install Python packages
  [[ -d "install/dump_launch/lib/python3.10/site-packages/dump_launch" ]] && \
    cp -r "install/dump_launch/lib/python3.10/site-packages/dump_launch" \
      "${pkgdir}/usr/lib/python3/dist-packages/"

  [[ -d "install/play_launch_analyzer/lib/python3.10/site-packages/play_launch_analyzer" ]] && \
    cp -r "install/play_launch_analyzer/lib/python3.10/site-packages/play_launch_analyzer" \
      "${pkgdir}/usr/lib/python3/dist-packages/"

  # Install Python scripts
  [[ -f "install/dump_launch/lib/dump_launch/dump_launch" ]] && \
    install -Dm755 "install/dump_launch/lib/dump_launch/dump_launch" \
      "${pkgdir}/usr/bin/dump_launch"

  [[ -f "install/play_launch_analyzer/lib/play_launch_analyzer/play_launch_analyzer" ]] && \
    install -Dm755 "install/play_launch_analyzer/lib/play_launch_analyzer/play_launch_analyzer" \
      "${pkgdir}/usr/bin/play_launch_analyzer"

  # Install docs
  install -Dm644 "README.md" "${pkgdir}/usr/share/doc/${pkgname}/README.md"
  install -Dm644 "CLAUDE.md" "${pkgdir}/usr/share/doc/${pkgname}/CLAUDE.md"
  [[ -d "docs" ]] && cp -r docs/* "${pkgdir}/usr/share/doc/${pkgname}/"

  # Install examples
  [[ -d "test" ]] && {
    install -d "${pkgdir}/usr/share/${pkgname}/examples"
    cp -r test/* "${pkgdir}/usr/share/${pkgname}/examples/"
  }

  # Create I/O helper setup script
  cat > "${pkgdir}/usr/share/${pkgname}/setup_io_helper.sh" << 'EOF'
#!/bin/bash
echo "I/O Helper Setup - Grant CAP_SYS_PTRACE for container monitoring"
read -p "Enable I/O monitoring for containers? [y/N] " -n 1 -r
echo
[[ $REPLY =~ ^[Yy]$ ]] && {
  sudo setcap cap_sys_ptrace+ep /usr/lib/play-launch/play_launch_io_helper
  echo "✓ Enabled (reapply after upgrades)"
} || echo "Skipped"
EOF
  chmod +x "${pkgdir}/usr/share/${pkgname}/setup_io_helper.sh"
}

post_install() {
  cat << 'EOF'

play-launch installed successfully!

Usage:
  source /opt/ros/humble/setup.bash
  play_launch --help
  play_launch launch <package> <launch_file>
  play_launch plot

Optional:
  /usr/share/play-launch/setup_io_helper.sh  # I/O monitoring for containers

Docs: /usr/share/doc/play-launch/
Issues: https://github.com/NEWSLabNTU/ros-launch-perf/issues

EOF
}

post_remove() {
  fancy_message info "play-launch removed"
}
