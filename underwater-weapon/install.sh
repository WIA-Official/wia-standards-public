#!/bin/bash

################################################################################
# WIA-DEF-019: Underwater Weapon - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Defense & Security Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-DEF-019 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
SLATE='\033[38;5;102m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${SLATE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🌊 WIA-DEF-019: Underwater Weapon Installer              ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${RESET}"
}

print_error() {
    echo -e "${RED}✗ $1${RESET}"
}

print_info() {
    echo -e "${GRAY}  $1${RESET}"
}

# Check prerequisites
check_prerequisites() {
    print_section "Checking Prerequisites"

    local all_ok=true

    # Check bash
    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1)"
    else
        print_error "Bash not found"
        all_ok=false
    fi

    # Check bc (for calculations)
    if command -v bc &> /dev/null; then
        print_success "bc (calculator): installed"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Install: sudo apt-get install bc  (Debian/Ubuntu)"
        print_info "Install: sudo yum install bc      (RHEL/CentOS)"
        print_info "Install: brew install bc          (macOS)"
    fi

    # Check Node.js (for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (required for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    fi

    # Check npm
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (required for TypeScript SDK)"
    fi

    if [ "$all_ok" = false ]; then
        print_error "Some prerequisites are missing"
        return 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-def-019.sh"
    local install_dir="/usr/local/bin"
    local cli_target="$install_dir/wia-def-019"

    # Check if source exists
    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        return 1
    fi

    # Make executable
    chmod +x "$cli_source"
    print_success "Made CLI executable"

    # Try to install to /usr/local/bin (may require sudo)
    if [ -w "$install_dir" ]; then
        cp "$cli_source" "$cli_target"
        print_success "Installed CLI to: $cli_target"
    else
        print_warning "No write permission to $install_dir"
        print_info "Attempting installation with sudo..."

        if sudo cp "$cli_source" "$cli_target" 2>/dev/null; then
            print_success "Installed CLI to: $cli_target (with sudo)"
        else
            print_warning "Could not install to $install_dir"
            print_info "You can manually copy:"
            print_info "  sudo cp $cli_source $cli_target"
            print_info "Or run from source:"
            print_info "  $cli_source"
            return 1
        fi
    fi

    # Verify installation
    if command -v wia-def-019 &> /dev/null; then
        print_success "CLI tool installed successfully"
        wia-def-019 --version
    else
        print_warning "CLI installed but not in PATH"
        print_info "Add to PATH: export PATH=\$PATH:$install_dir"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local sdk_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_error "TypeScript SDK directory not found: $sdk_dir"
        return 1
    fi

    cd "$sdk_dir"

    # Check if package.json exists
    if [ ! -f "package.json" ]; then
        print_error "package.json not found in $sdk_dir"
        return 1
    fi

    print_info "Installing dependencies..."
    if npm install &> /dev/null; then
        print_success "Dependencies installed"
    else
        print_warning "Failed to install dependencies (continuing anyway)"
    fi

    print_info "Building TypeScript SDK..."
    if npm run build &> /dev/null; then
        print_success "TypeScript SDK built successfully"
    else
        print_warning "Build failed (you may need to build manually)"
        print_info "Run: cd $sdk_dir && npm install && npm run build"
    fi

    # Link globally (optional)
    print_info "Linking SDK globally..."
    if npm link &> /dev/null; then
        print_success "SDK linked globally (available as @wia/def-019)"
    else
        print_warning "Global link failed"
        print_info "Install locally: npm install $sdk_dir"
    fi

    cd "$SCRIPT_DIR"
}

# Create symlinks for documentation
setup_documentation() {
    print_section "Setting Up Documentation"

    local docs=(
        "README.md"
        "spec/WIA-DEF-019-v1.0.md"
    )

    for doc in "${docs[@]}"; do
        if [ -f "$SCRIPT_DIR/$doc" ]; then
            print_success "Found: $doc"
        else
            print_warning "Missing: $doc"
        fi
    done

    print_info "Documentation available in: $SCRIPT_DIR"
}

# Verify installation
verify_installation() {
    print_section "Verifying Installation"

    local all_ok=true

    # Check CLI
    if command -v wia-def-019 &> /dev/null; then
        print_success "CLI tool: wia-def-019 (installed)"
    else
        print_warning "CLI tool: not in PATH"
        all_ok=false
    fi

    # Check TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK: built"
    else
        print_warning "TypeScript SDK: not built"
        all_ok=false
    fi

    # Check documentation
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        print_success "Documentation: available"
    else
        print_warning "Documentation: missing"
        all_ok=false
    fi

    if [ "$all_ok" = true ]; then
        print_success "All components verified"
    else
        print_warning "Some components may need manual setup"
    fi
}

# Display usage instructions
show_usage() {
    print_section "Usage Instructions"

    echo -e "${CYAN}CLI Tool:${RESET}"
    echo -e "  wia-def-019 --help"
    echo -e "  wia-def-019 torpedo --type heavy --diameter 533"
    echo -e "  wia-def-019 mine-clearance --area \"35.0,129.0,5km\" --method UUV"
    echo

    echo -e "${CYAN}TypeScript SDK:${RESET}"
    echo -e "  npm install @wia/def-019"
    echo -e "  # or install from local path"
    echo -e "  npm install $SCRIPT_DIR/api/typescript"
    echo

    echo -e "${CYAN}Documentation:${RESET}"
    echo -e "  README: $SCRIPT_DIR/README.md"
    echo -e "  Spec:   $SCRIPT_DIR/spec/WIA-DEF-019-v1.0.md"
    echo

    echo -e "${CYAN}Examples:${RESET}"
    echo -e "  # Calculate torpedo intercept"
    echo -e "  wia-def-019 simulate-attack --torpedo-speed 50 --target-speed 20"
    echo

    echo -e "  # Plan mine clearance"
    echo -e "  wia-def-019 mine-clearance --area \"35.0,129.0,5km\" --purpose humanitarian"
    echo

    echo -e "  # Calculate sonar range"
    echo -e "  wia-def-019 sonar-range --freq 5000 --depth 100 --temp 15"
    echo
}

# Cleanup function
cleanup() {
    print_section "Cleanup"
    print_info "Removing temporary files..."
    # Add cleanup tasks if needed
    print_success "Cleanup complete"
}

# Main installation flow
main() {
    print_header

    echo -e "${GRAY}This installer will set up WIA-DEF-019 Underwater Weapon components${RESET}"
    echo -e "${GRAY}including CLI tool, TypeScript SDK, and documentation.${RESET}"
    echo

    # Check prerequisites
    if ! check_prerequisites; then
        print_error "Prerequisites check failed"
        print_info "Please install missing dependencies and try again"
        exit 1
    fi

    # Install components
    install_cli
    install_typescript_sdk
    setup_documentation

    # Verify installation
    verify_installation

    # Show usage
    show_usage

    print_section "Installation Complete"
    print_success "WIA-DEF-019 Underwater Weapon Standard installed"
    echo
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo
}

# Handle errors
trap 'print_error "Installation failed"; exit 1' ERR

# Run main installation
main "$@"
