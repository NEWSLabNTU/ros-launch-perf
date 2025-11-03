#!/bin/bash
# Pacscript Validation Script
# Validates play-launch.pacscript against Pacstall requirements

set -e

PACSCRIPT="packages/play-launch/play-launch.pacscript"
ERRORS=0
WARNINGS=0

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         Pacscript Validation for play-launch              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

error() {
    echo -e "${RED}✗ ERROR:${NC} $1"
    ((ERRORS++))
}

warn() {
    echo -e "${YELLOW}⚠ WARNING:${NC} $1"
    ((WARNINGS++))
}

success() {
    echo -e "${GREEN}✓${NC} $1"
}

info() {
    echo "  $1"
}

# Check file exists
if [[ ! -f "$PACSCRIPT" ]]; then
    error "Pacscript not found at $PACSCRIPT"
    exit 1
fi

echo "Checking pacscript: $PACSCRIPT"
echo ""

# 1. Bash Syntax Check
echo "═══ Syntax Validation ═══"
if bash -n "$PACSCRIPT" 2>/dev/null; then
    success "Bash syntax valid"
else
    error "Bash syntax errors found"
    bash -n "$PACSCRIPT"
fi
echo ""

# 2. Required Fields Check
echo "═══ Required Metadata Fields ═══"

required_fields=(
    "pkgname"
    "pkgver"
    "pkgdesc"
    "arch"
    "url"
    "license"
)

for field in "${required_fields[@]}"; do
    if grep -q "^${field}=" "$PACSCRIPT"; then
        value=$(grep "^${field}=" "$PACSCRIPT" | head -1 | cut -d'=' -f2-)
        success "$field is defined: $value"
    else
        error "Required field '$field' not found"
    fi
done
echo ""

# 3. Function Checks
echo "═══ Required Functions ═══"

required_functions=(
    "prepare"
    "build"
    "install"
)

for func in "${required_functions[@]}"; do
    if grep -q "^${func}()" "$PACSCRIPT"; then
        success "Function ${func}() is defined"
    else
        error "Required function ${func}() not found"
    fi
done
echo ""

# 4. Optional but Recommended Fields
echo "═══ Optional Fields ═══"

optional_fields=(
    "gives"
    "pkgrel"
    "epoch"
    "maintainer"
    "depends"
    "makedepends"
    "optdepends"
)

for field in "${optional_fields[@]}"; do
    if grep -q "^${field}=" "$PACSCRIPT" || grep -q "^${field}=(" "$PACSCRIPT"; then
        success "$field is defined"
    else
        warn "$field not defined (optional)"
    fi
done
echo ""

# 5. Architecture Check
echo "═══ Architecture Support ═══"
if grep -q 'arch=.*amd64.*arm64' "$PACSCRIPT"; then
    success "Multi-architecture support (amd64, arm64)"
elif grep -q 'arch=.*amd64' "$PACSCRIPT"; then
    warn "Only amd64 architecture specified"
elif grep -q 'arch=.*arm64' "$PACSCRIPT"; then
    warn "Only arm64 architecture specified"
else
    error "No architecture specified"
fi
echo ""

# 6. Source Check
echo "═══ Source Configuration ═══"
if grep -q "^source=" "$PACSCRIPT"; then
    success "Source is defined"
    source_line=$(grep "^source=" "$PACSCRIPT")
    info "$source_line"
else
    error "Source not defined"
fi

if grep -q "^sha256sums=" "$PACSCRIPT"; then
    success "Checksums are defined"
else
    warn "No checksums defined (using SKIP is acceptable for testing)"
fi
echo ""

# 7. Dependencies Check
echo "═══ Dependencies Analysis ═══"
if grep -q "^depends=" "$PACSCRIPT"; then
    success "Runtime dependencies defined"
    depends=$(grep "^depends=" "$PACSCRIPT" -A 3 | grep -o '"[^"]*"' | wc -l)
    info "  Found $depends dependency declarations"
fi

if grep -q "^makedepends=" "$PACSCRIPT"; then
    success "Build dependencies defined"
    makedeps=$(grep "^makedepends=" "$PACSCRIPT" -A 10 | grep -o '"[^"]*"' | wc -l)
    info "  Found $makedeps build dependency declarations"
fi
echo ""

# 8. Post Install/Remove Hooks
echo "═══ Post-Install/Remove Hooks ═══"
if grep -q "^post_install()" "$PACSCRIPT"; then
    success "post_install() function defined"
else
    warn "No post_install() function (optional)"
fi

if grep -q "^post_remove()" "$PACSCRIPT"; then
    success "post_remove() function defined"
else
    warn "No post_remove() function (optional)"
fi
echo ""

# 9. File Size Check
echo "═══ Pacscript Statistics ═══"
lines=$(wc -l < "$PACSCRIPT")
size=$(du -h "$PACSCRIPT" | cut -f1)
success "Lines: $lines"
success "Size: $size"
echo ""

# 10. Best Practices Check
echo "═══ Best Practices ═══"

# Check for error handling
if grep -q "set -e" "$PACSCRIPT" || grep -q "set -o errexit" "$PACSCRIPT"; then
    success "Error handling enabled (set -e or errexit)"
else
    warn "Consider adding 'set -e' for better error handling"
fi

# Check for fancy_message usage
if grep -q "fancy_message" "$PACSCRIPT"; then
    success "Uses fancy_message for user feedback"
fi

# Check for pkgdir usage
if grep -q '${pkgdir}' "$PACSCRIPT"; then
    success "Properly uses \${pkgdir} for installation"
else
    error "Missing \${pkgdir} usage in install function"
fi

# Check for srcdir usage
if grep -q '${srcdir}' "$PACSCRIPT"; then
    success "Properly uses \${srcdir} for source directory"
else
    warn "Consider using \${srcdir} for source directory references"
fi
echo ""

# 11. Specific Checks for play-launch
echo "═══ Project-Specific Checks ═══"

# Check for ROS 2 dependency
if grep -q "ros-humble-desktop" "$PACSCRIPT"; then
    success "ROS 2 Humble dependency declared"
else
    warn "ROS 2 dependency not explicitly declared"
fi

# Check for 3-stage build mention
if grep -q "make build" "$PACSCRIPT"; then
    success "Uses 'make build' for 3-stage build process"
else
    warn "Build process should use 'make build'"
fi

# Check for Rust installation
if grep -q "install_rust" "$PACSCRIPT" || grep -q "rustup" "$PACSCRIPT"; then
    success "Handles Rust toolchain installation"
else
    warn "No Rust installation handling found"
fi

# Check for Python dependencies
if grep -q "pip3 install" "$PACSCRIPT"; then
    success "Installs Python dependencies"
else
    warn "No Python dependency installation found"
fi
echo ""

# Final Summary
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                   Validation Summary                       ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

if [[ $ERRORS -eq 0 ]] && [[ $WARNINGS -eq 0 ]]; then
    echo -e "${GREEN}✓ Perfect!${NC} Pacscript is valid with no issues."
    exit 0
elif [[ $ERRORS -eq 0 ]]; then
    echo -e "${YELLOW}⚠ Valid with warnings${NC}"
    echo "  Errors: $ERRORS"
    echo "  Warnings: $WARNINGS"
    echo ""
    echo "The pacscript will work, but consider addressing warnings."
    exit 0
else
    echo -e "${RED}✗ Validation failed${NC}"
    echo "  Errors: $ERRORS"
    echo "  Warnings: $WARNINGS"
    echo ""
    echo "Please fix the errors before proceeding."
    exit 1
fi
