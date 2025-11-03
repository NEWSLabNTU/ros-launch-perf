#!/usr/bin/env bash
set -e

# Check if project is built
if [ ! -f install/play_launch/lib/play_launch/play_launch ]; then
    echo "Error: Project not built. Run 'just build' first."
    exit 1
fi

VERSION=$(grep '^play-launch' debian/changelog | head -1 | sed 's/.*(\(.*\)).*/\1/')
ARCH=$(dpkg --print-architecture)
PKG_NAME="play-launch_${VERSION}_${ARCH}"
DEB_DIR="debian/play-launch"

echo "Building Debian package: ${PKG_NAME}.deb"
rm -rf "${DEB_DIR}"
mkdir -p "${DEB_DIR}/DEBIAN"

# Create control file from template
sed "s/VERSION_PLACEHOLDER/${VERSION}/g; s/ARCH_PLACEHOLDER/${ARCH}/g" \
    debian/control.template > "${DEB_DIR}/DEBIAN/control"

# Create postinst script to set capability on I/O helper
cat > "${DEB_DIR}/DEBIAN/postinst" << 'EOF'
#!/bin/bash
set -e

IO_HELPER="/usr/lib/play-launch/play_launch_io_helper"

if [ "$1" = "configure" ]; then
    echo "Setting up play-launch..."

    # Set capability on I/O helper for container monitoring
    if [ -f "${IO_HELPER}" ]; then
        if command -v setcap >/dev/null 2>&1; then
            echo "Setting CAP_SYS_PTRACE on I/O helper for container monitoring..."
            setcap cap_sys_ptrace+ep "${IO_HELPER}" 2>/dev/null || {
                echo "Warning: Failed to set CAP_SYS_PTRACE on ${IO_HELPER}"
                echo "I/O monitoring for privileged processes will be unavailable."
                echo "Run 'sudo setcap cap_sys_ptrace+ep ${IO_HELPER}' manually to enable."
            }
        else
            echo "Warning: setcap not found. I/O monitoring for containers disabled."
        fi
    fi

    echo "play-launch installed successfully!"
    echo ""
    echo "Usage:"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  play_launch --help"
    echo "  play_launch launch <package> <launch_file>"
    echo "  play_launch plot"
    echo ""
    echo "Documentation: /usr/share/doc/play-launch/"
fi

exit 0
EOF

chmod 755 "${DEB_DIR}/DEBIAN/postinst"

# Install binaries
install -Dm755 install/play_launch/lib/play_launch/play_launch \
    "${DEB_DIR}/usr/bin/play_launch"
install -Dm755 install/play_launch/lib/play_launch/play_launch_io_helper \
    "${DEB_DIR}/usr/lib/play-launch/play_launch_io_helper"

# Install Python packages (copy actual files, not symlinks)
if [ -d src/dump_launch/dump_launch ]; then
    mkdir -p "${DEB_DIR}/usr/lib/python3.10/dist-packages"
    cp -rL src/dump_launch/dump_launch \
        "${DEB_DIR}/usr/lib/python3.10/dist-packages/"
    # Remove __pycache__
    find "${DEB_DIR}/usr/lib/python3.10/dist-packages/dump_launch" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
fi

if [ -d src/play_launch_analyzer/play_launch_analyzer ]; then
    mkdir -p "${DEB_DIR}/usr/lib/python3.10/dist-packages"
    cp -rL src/play_launch_analyzer/play_launch_analyzer \
        "${DEB_DIR}/usr/lib/python3.10/dist-packages/"
    # Remove __pycache__
    find "${DEB_DIR}/usr/lib/python3.10/dist-packages/play_launch_analyzer" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
fi

# Install Python scripts
[ -f install/dump_launch/lib/dump_launch/dump_launch ] && \
    install -Dm755 install/dump_launch/lib/dump_launch/dump_launch \
        "${DEB_DIR}/usr/bin/dump_launch"

[ -f install/play_launch_analyzer/lib/play_launch_analyzer/play_launch_analyzer ] && \
    install -Dm755 install/play_launch_analyzer/lib/play_launch_analyzer/play_launch_analyzer \
        "${DEB_DIR}/usr/bin/play_launch_analyzer"

# Install docs
install -Dm644 README.md "${DEB_DIR}/usr/share/doc/play-launch/README.md"
install -Dm644 CLAUDE.md "${DEB_DIR}/usr/share/doc/play-launch/CLAUDE.md"
[ -d docs ] && cp -r docs/* "${DEB_DIR}/usr/share/doc/play-launch/" 2>/dev/null || true

# Install examples
if [ -d test ]; then
    mkdir -p "${DEB_DIR}/usr/share/play-launch/examples"
    cp -r test/* "${DEB_DIR}/usr/share/play-launch/examples/"
fi

# Build .deb
dpkg-deb --build "${DEB_DIR}" "${PKG_NAME}.deb"
rm -rf "${DEB_DIR}"

echo "✓ Debian package created: ${PKG_NAME}.deb"
ls -lh "${PKG_NAME}.deb"
