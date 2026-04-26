#!/bin/bash

################################################################################
# WIA-IND-009: Food Delivery Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Food Delivery Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-009 Food Delivery Standard CLI tool
# and TypeScript SDK.
################################################################################

set -e

# Colors
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STANDARD_NAME="WIA-IND-009"
STANDARD_DIR="food-delivery"

# Print functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     🚚 WIA-IND-009: Food Delivery Standard Installer         ║"
    echo "║                      Version 1.0.0                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_step() {
    echo -e "\n${CYAN}▶ $1${RESET}"
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
    echo -e "  $1"
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Detect OS
detect_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "linux"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo "macos"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        echo "windows"
    else
        echo "unknown"
    fi
}

# Check prerequisites
check_prerequisites() {
    print_step "Checking prerequisites"

    local missing_deps=()

    # Check for bash
    if ! command_exists bash; then
        missing_deps+=("bash")
    else
        print_success "bash $(bash --version | head -n1)"
    fi

    # Check for bc (calculator)
    if ! command_exists bc; then
        missing_deps+=("bc")
        print_warning "bc (calculator) not found"
    else
        print_success "bc installed"
    fi

    # Check for node (optional, for TypeScript SDK)
    if command_exists node; then
        print_success "node $(node --version)"
    else
        print_warning "node.js not found (optional, for TypeScript SDK)"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command_exists npm; then
        print_success "npm $(npm --version)"
    else
        print_warning "npm not found (optional, for TypeScript SDK)"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        print_info "Please install missing dependencies:"

        local os=$(detect_os)
        case $os in
            linux)
                print_info "  sudo apt-get install ${missing_deps[*]}"
                print_info "  # or"
                print_info "  sudo yum install ${missing_deps[*]}"
                ;;
            macos)
                print_info "  brew install ${missing_deps[*]}"
                ;;
            windows)
                print_info "  Install Git Bash or WSL with required tools"
                ;;
        esac

        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool"

    local cli_script="$SCRIPT_DIR/cli/wia-ind-009.sh"

    if [ ! -f "$cli_script" ]; then
        print_error "CLI script not found at $cli_script"
        exit 1
    fi

    # Make executable
    chmod +x "$cli_script"
    print_success "Made CLI script executable"

    # Determine installation directory
    local install_dir="/usr/local/bin"
    local install_path="$install_dir/wia-ind-009"

    # Check if we can write to /usr/local/bin
    if [ -w "$install_dir" ]; then
        # Create symlink
        ln -sf "$cli_script" "$install_path"
        print_success "Installed CLI to $install_path"
    else
        # Try with sudo
        print_warning "Requesting sudo access to install to $install_dir"
        sudo ln -sf "$cli_script" "$install_path"
        print_success "Installed CLI to $install_path (with sudo)"
    fi

    # Verify installation
    if command_exists wia-ind-009; then
        print_success "CLI tool installed successfully"
        wia-ind-009 version
    else
        print_warning "CLI installed but not in PATH"
        print_info "Add this to your ~/.bashrc or ~/.zshrc:"
        print_info "  export PATH=\"\$PATH:$install_dir\""
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_step "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript SDK directory not found"
        return 1
    fi

    if ! command_exists npm; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "To install later, run: cd $ts_dir && npm install"
        return 0
    fi

    cd "$ts_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install

    # Build TypeScript
    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK built successfully"

    # Link for local development (optional)
    if [ -w "$(npm config get prefix)/lib" ]; then
        npm link
        print_success "SDK linked for local development (npm link)"
    else
        print_info "To use SDK globally, run: sudo npm link"
    fi

    cd "$SCRIPT_DIR"
}

# Run tests
run_tests() {
    print_step "Running tests"

    # Test CLI
    print_info "Testing CLI tool..."

    if command_exists wia-ind-009; then
        # Run a simple calculation
        wia-ind-009 calc-time --distance 5 --prep-time 15 > /dev/null 2>&1
        print_success "CLI test passed"
    else
        print_warning "CLI not in PATH, skipping tests"
    fi

    # Test TypeScript SDK (if installed)
    if command_exists npm && [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cd "$SCRIPT_DIR/api/typescript"
        if [ -f "package.json" ]; then
            print_info "TypeScript SDK tests available"
            print_info "Run: cd $SCRIPT_DIR/api/typescript && npm test"
        fi
        cd "$SCRIPT_DIR"
    fi
}

# Create sample configuration
create_sample_config() {
    print_step "Creating sample configuration"

    local config_dir="$HOME/.wia"
    local config_file="$config_dir/ind-009.conf"

    # Create config directory
    mkdir -p "$config_dir"

    # Create sample config
    cat > "$config_file" << 'EOF'
# WIA-IND-009 Food Delivery Configuration
# 弘益인간 (Benefit All Humanity)

# API Configuration
WIA_API_KEY=your_api_key_here
WIA_API_URL=https://api.wia-ind-009.com/v1
WIA_REGION=us-west

# Default Settings
DEFAULT_VEHICLE=ebike
DEFAULT_PREP_TIME=15
DEFAULT_TRAFFIC_FACTOR=1.2

# Temperature Monitoring
ENABLE_TEMP_MONITORING=true
TEMP_ALERT_THRESHOLD_HOT=60
TEMP_ALERT_THRESHOLD_COLD=4

# Route Optimization
ENABLE_ROUTE_OPTIMIZATION=true
OPTIMIZATION_ALGORITHM=tsp_2opt
MAX_BATCH_SIZE=4

# Performance
REQUEST_TIMEOUT=30000
RETRY_MAX_ATTEMPTS=3
EOF

    print_success "Sample configuration created at $config_file"
    print_info "Edit this file to customize your settings"
}

# Show installation summary
show_summary() {
    print_step "Installation Summary"

    echo ""
    print_success "WIA-IND-009 Food Delivery Standard installed successfully!"
    echo ""

    print_info "CLI Tool:"
    print_info "  Command: wia-ind-009"
    print_info "  Location: $(which wia-ind-009 || echo 'Not in PATH')"
    print_info "  Usage: wia-ind-009 help"
    echo ""

    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_info "TypeScript SDK:"
        print_info "  Package: @wia/ind-009"
        print_info "  Location: $SCRIPT_DIR/api/typescript"
        print_info "  Import: import { FoodDeliverySDK } from '@wia/ind-009';"
        echo ""
    fi

    print_info "Configuration:"
    print_info "  File: ~/.wia/ind-009.conf"
    echo ""

    print_info "Documentation:"
    print_info "  README: $SCRIPT_DIR/README.md"
    print_info "  Spec: $SCRIPT_DIR/spec/WIA-IND-009-v1.0.md"
    print_info "  Web: https://wiastandards.com/ind-009"
    echo ""

    print_info "Quick Start:"
    print_info "  # Calculate delivery time"
    print_info "  wia-ind-009 calc-time --distance 5.2 --prep-time 15"
    echo ""
    print_info "  # Calculate delivery cost"
    print_info "  wia-ind-009 calc-cost --distance 8.5 --surge 1.5"
    echo ""
    print_info "  # Optimize route"
    print_info "  wia-ind-009 optimize-route --orders 3 --avg-distance 4.0"
    echo ""

    echo -e "${INDIGO}弘익人間 (홍익인간) · Benefit All Humanity${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    print_step "Uninstalling WIA-IND-009"

    # Remove CLI symlink
    if [ -L "/usr/local/bin/wia-ind-009" ]; then
        sudo rm -f /usr/local/bin/wia-ind-009
        print_success "Removed CLI symlink"
    fi

    # Unlink npm package
    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cd "$SCRIPT_DIR/api/typescript"
        npm unlink 2>/dev/null || true
        cd "$SCRIPT_DIR"
    fi

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    print_header

    # Check for uninstall flag
    if [ "$1" = "--uninstall" ]; then
        uninstall
        exit 0
    fi

    echo "This script will install the WIA-IND-009 Food Delivery Standard."
    echo "Press Enter to continue or Ctrl+C to cancel..."
    read -r

    check_prerequisites
    install_cli
    install_typescript_sdk
    create_sample_config
    run_tests
    show_summary

    print_success "Installation complete!"
}

# Run main function
main "$@"
