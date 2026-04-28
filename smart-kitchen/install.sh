#!/bin/bash

################################################################################
# WIA-IND-008: Smart Kitchen Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry 4.0 Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-008 Smart Kitchen Standard CLI tool and
# TypeScript SDK on your system.
################################################################################

set -e

# Colors
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Configuration
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"
CLI_NAME="wia-ind-008"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🍳 WIA-IND-008: Smart Kitchen Installer                 ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
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
    echo -e "${CYAN}ℹ $1${RESET}"
}

print_step() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

# Check if running as root for system-wide install
check_permissions() {
    if [[ "$INSTALL_DIR" == "/usr/local/bin" ]] || [[ "$INSTALL_DIR" == "/usr/bin" ]]; then
        if [[ $EUID -ne 0 ]]; then
            print_warning "System-wide installation requires sudo privileges"
            print_info "Re-running with sudo..."
            sudo "$0" "$@"
            exit $?
        fi
    fi
}

# Detect operating system
detect_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS="linux"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS="macos"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        OS="windows"
    else
        OS="unknown"
    fi

    print_info "Detected OS: $OS"
}

# Check dependencies
check_dependencies() {
    print_step "Checking Dependencies"

    local missing_deps=()

    # Check for bc (calculator)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    else
        print_success "bc is installed"
    fi

    # Check for bash version
    if [[ ${BASH_VERSION%%.*} -lt 4 ]]; then
        print_warning "Bash version 4+ recommended (current: ${BASH_VERSION})"
    else
        print_success "Bash ${BASH_VERSION} is installed"
    fi

    # Check for Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        local node_version=$(node --version)
        print_success "Node.js ${node_version} is installed"
        HAS_NODE=true
    else
        print_warning "Node.js not found (optional, needed for TypeScript SDK)"
        HAS_NODE=false
    fi

    # Check for npm (optional)
    if command -v npm &> /dev/null; then
        local npm_version=$(npm --version)
        print_success "npm ${npm_version} is installed"
        HAS_NPM=true
    else
        if [[ "$HAS_NODE" == true ]]; then
            print_warning "npm not found (needed for TypeScript SDK)"
        fi
        HAS_NPM=false
    fi

    # Install missing dependencies if any
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        print_warning "Missing dependencies: ${missing_deps[*]}"
        install_dependencies "${missing_deps[@]}"
    fi
}

# Install system dependencies
install_dependencies() {
    local deps=("$@")

    print_info "Attempting to install missing dependencies..."

    if [[ "$OS" == "linux" ]]; then
        if command -v apt-get &> /dev/null; then
            sudo apt-get update
            sudo apt-get install -y "${deps[@]}"
        elif command -v yum &> /dev/null; then
            sudo yum install -y "${deps[@]}"
        elif command -v pacman &> /dev/null; then
            sudo pacman -S --noconfirm "${deps[@]}"
        else
            print_error "Could not find package manager. Please install: ${deps[*]}"
            exit 1
        fi
    elif [[ "$OS" == "macos" ]]; then
        if command -v brew &> /dev/null; then
            brew install "${deps[@]}"
        else
            print_error "Homebrew not found. Please install: ${deps[*]}"
            exit 1
        fi
    fi
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI Tool"

    local cli_script="${SCRIPT_DIR}/cli/${CLI_NAME}.sh"

    if [[ ! -f "$cli_script" ]]; then
        print_error "CLI script not found: $cli_script"
        exit 1
    fi

    # Make executable
    chmod +x "$cli_script"
    print_success "Made CLI script executable"

    # Copy to install directory
    if [[ -w "$INSTALL_DIR" ]] || [[ $EUID -eq 0 ]]; then
        cp "$cli_script" "${INSTALL_DIR}/${CLI_NAME}"
        print_success "Installed CLI to ${INSTALL_DIR}/${CLI_NAME}"
    else
        print_error "Cannot write to ${INSTALL_DIR}"
        print_info "Try running with sudo or set INSTALL_DIR to a writable location"
        exit 1
    fi

    # Verify installation
    if command -v "${CLI_NAME}" &> /dev/null; then
        print_success "CLI tool is now available: ${CLI_NAME}"
    else
        print_warning "CLI installed but not in PATH. Add ${INSTALL_DIR} to your PATH"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    if [[ "$HAS_NPM" != true ]]; then
        print_warning "Skipping TypeScript SDK installation (npm not available)"
        return
    fi

    print_step "Installing TypeScript SDK"

    local ts_dir="${SCRIPT_DIR}/api/typescript"

    if [[ ! -d "$ts_dir" ]]; then
        print_error "TypeScript directory not found: $ts_dir"
        return
    fi

    cd "$ts_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install
    print_success "Dependencies installed"

    # Build TypeScript
    print_info "Building TypeScript..."
    npm run build
    print_success "TypeScript SDK built successfully"

    # Link for local development (optional)
    if [[ "$1" == "--dev" ]]; then
        print_info "Linking SDK for development..."
        npm link
        print_success "SDK linked for development"
    fi

    cd - > /dev/null
}

# Create configuration directory
create_config() {
    print_step "Creating Configuration"

    local config_dir="${HOME}/.config/wia-ind-008"

    if [[ ! -d "$config_dir" ]]; then
        mkdir -p "$config_dir"
        print_success "Created config directory: $config_dir"
    fi

    # Create default config file
    local config_file="${config_dir}/config.json"
    if [[ ! -f "$config_file" ]]; then
        cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "energy_rate_kwh": 0.15,
  "currency": "USD",
  "timezone": "UTC",
  "units": {
    "temperature": "celsius",
    "volume": "liters",
    "weight": "grams"
  },
  "appliances": [],
  "preferences": {
    "energy_mode": "eco",
    "dietary_preference": "balanced"
  }
}
EOF
        print_success "Created default configuration"
    else
        print_info "Configuration already exists"
    fi
}

# Run post-install checks
post_install_checks() {
    print_step "Post-Installation Checks"

    # Test CLI
    if "${INSTALL_DIR}/${CLI_NAME}" version &> /dev/null; then
        print_success "CLI tool working correctly"
    else
        print_error "CLI tool test failed"
    fi

    # Show installed version
    echo ""
    "${INSTALL_DIR}/${CLI_NAME}" version
}

# Display next steps
show_next_steps() {
    print_step "Installation Complete!"

    cat << EOF

${GREEN}✓ WIA-IND-008 Smart Kitchen Standard installed successfully!${RESET}

${CYAN}Quick Start:${RESET}

  1. Check version:
     ${GRAY}$ wia-ind-008 version${RESET}

  2. Calculate cooking energy:
     ${GRAY}$ wia-ind-008 calc-energy --appliance oven --power 3.5 --time 1.5${RESET}

  3. Scale a recipe:
     ${GRAY}$ wia-ind-008 scale-recipe --from 4 --to 6${RESET}

  4. View all commands:
     ${GRAY}$ wia-ind-008 help${RESET}

${CYAN}TypeScript SDK:${RESET}

EOF

    if [[ "$HAS_NPM" == true ]]; then
        cat << EOF
  Install in your project:
     ${GRAY}$ npm install ${SCRIPT_DIR}/api/typescript${RESET}

  Or publish to npm registry and:
     ${GRAY}$ npm install @wia/ind-008${RESET}

EOF
    else
        echo "  Install Node.js and npm to use the TypeScript SDK"
        echo ""
    fi

    cat << EOF
${CYAN}Configuration:${RESET}

  Config file: ${GRAY}~/.config/wia-ind-008/config.json${RESET}
  Edit to customize energy rates, units, and preferences

${CYAN}Documentation:${RESET}

  Specification: ${GRAY}${SCRIPT_DIR}/spec/WIA-IND-008-v1.0.md${RESET}
  README:        ${GRAY}${SCRIPT_DIR}/README.md${RESET}
  Website:       ${GRAY}https://wiastandards.com/standards/smart-kitchen${RESET}

${INDIGO}弘益人間 (홍익인간)${RESET} - Benefit All Humanity

EOF
}

# Uninstall function
uninstall() {
    print_header
    print_step "Uninstalling WIA-IND-008"

    # Remove CLI
    if [[ -f "${INSTALL_DIR}/${CLI_NAME}" ]]; then
        rm "${INSTALL_DIR}/${CLI_NAME}"
        print_success "Removed CLI tool"
    fi

    # Remove config (ask first)
    local config_dir="${HOME}/.config/wia-ind-008"
    if [[ -d "$config_dir" ]]; then
        read -p "Remove configuration directory? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$config_dir"
            print_success "Removed configuration"
        fi
    fi

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    # Check for uninstall flag
    if [[ "$1" == "--uninstall" ]] || [[ "$1" == "uninstall" ]]; then
        uninstall
        exit 0
    fi

    print_header

    # Installation steps
    detect_os
    check_dependencies
    install_cli

    # Optional TypeScript SDK
    if [[ "$1" == "--with-sdk" ]] || [[ "$1" == "--dev" ]]; then
        install_typescript_sdk "$1"
    fi

    create_config
    post_install_checks
    show_next_steps
}

# Run main function
main "$@"
