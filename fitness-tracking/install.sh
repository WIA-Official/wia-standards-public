#!/bin/bash

################################################################################
# WIA-IND-012: Fitness Tracking Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Health & Fitness Working Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-012 Fitness Tracking Standard CLI tool
# and TypeScript SDK.
################################################################################

set -e

# Colors
INDIGO='\033[38;5;99m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Installation paths
INSTALL_DIR="/usr/local/bin"
CLI_NAME="wia-ind-012"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Print functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     🏃 WIA-IND-012: Fitness Tracking Standard Installer       ║"
    echo "║                    弘益人間 · Benefit All                      ║"
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
    echo -e "${GRAY}  $1${RESET}"
}

# Check if running as root/sudo for system-wide installation
check_permissions() {
    if [ "$EUID" -ne 0 ] && [ ! -w "$INSTALL_DIR" ]; then
        print_warning "Not running as root. Installing to user directory instead."
        INSTALL_DIR="$HOME/.local/bin"
        mkdir -p "$INSTALL_DIR"

        # Add to PATH if not already there
        if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
            print_info "Adding $INSTALL_DIR to PATH in ~/.bashrc"
            echo "" >> "$HOME/.bashrc"
            echo "# WIA-IND-012 Fitness Tracking CLI" >> "$HOME/.bashrc"
            echo "export PATH=\"\$PATH:$INSTALL_DIR\"" >> "$HOME/.bashrc"
            print_warning "Please run 'source ~/.bashrc' or restart your shell"
        fi
    fi
}

# Check dependencies
check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for bc (calculator)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    else
        print_success "bc is installed"
    fi

    # Check for Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        local node_version=$(node --version)
        print_success "Node.js is installed ($node_version)"
    else
        print_warning "Node.js not found (optional for TypeScript SDK)"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command -v npm &> /dev/null; then
        local npm_version=$(npm --version)
        print_success "npm is installed (v$npm_version)"
    else
        print_warning "npm not found (optional for TypeScript SDK)"
    fi

    # Install missing dependencies
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_warning "Missing dependencies: ${missing_deps[*]}"

        if command -v apt-get &> /dev/null; then
            print_info "Installing via apt-get..."
            sudo apt-get update
            sudo apt-get install -y "${missing_deps[@]}"
        elif command -v yum &> /dev/null; then
            print_info "Installing via yum..."
            sudo yum install -y "${missing_deps[@]}"
        elif command -v brew &> /dev/null; then
            print_info "Installing via Homebrew..."
            brew install "${missing_deps[@]}"
        else
            print_error "Cannot install dependencies automatically"
            print_info "Please install manually: ${missing_deps[*]}"
            exit 1
        fi
    fi
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool..."

    local cli_source="$SCRIPT_DIR/cli/$CLI_NAME.sh"
    local cli_dest="$INSTALL_DIR/$CLI_NAME"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI script not found at $cli_source"
        exit 1
    fi

    # Copy CLI script
    cp "$cli_source" "$cli_dest"
    chmod +x "$cli_dest"

    print_success "CLI tool installed to $cli_dest"

    # Verify installation
    if command -v "$CLI_NAME" &> /dev/null; then
        print_success "CLI tool is available in PATH"
    else
        print_warning "CLI tool installed but not in PATH"
        print_info "Add $INSTALL_DIR to your PATH to use the CLI"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_warning "TypeScript SDK directory not found"
        return
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return
    fi

    cd "$ts_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install

    # Build TypeScript
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript..."
        npm run build 2>/dev/null || print_warning "Build script not found, skipping build"
    fi

    cd "$SCRIPT_DIR"

    print_success "TypeScript SDK prepared"
    print_info "Use 'npm link' in $ts_dir for local development"
    print_info "Or publish to npm with 'npm publish'"
}

# Create example config
create_config() {
    print_step "Creating configuration..."

    local config_dir="$HOME/.config/wia-ind-012"
    local config_file="$config_dir/config.json"

    mkdir -p "$config_dir"

    if [ ! -f "$config_file" ]; then
        cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "user": {
    "age": 30,
    "gender": "male",
    "weight": 70,
    "height": 170,
    "restingHeartRate": 60,
    "fitnessLevel": "intermediate"
  },
  "goals": {
    "dailySteps": 10000,
    "weeklyWorkouts": 5,
    "weeklyDistance": 25000
  },
  "preferences": {
    "distanceUnit": "km",
    "weightUnit": "kg",
    "temperatureUnit": "celsius"
  }
}
EOF
        print_success "Configuration file created at $config_file"
        print_info "Edit this file to customize your settings"
    else
        print_info "Configuration file already exists"
    fi
}

# Copy documentation
copy_docs() {
    print_step "Copying documentation..."

    local docs_dir="$HOME/.local/share/wia-ind-012"
    mkdir -p "$docs_dir"

    # Copy README
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "README copied to $docs_dir"
    fi

    # Copy specification
    if [ -f "$SCRIPT_DIR/spec/WIA-IND-012-v1.0.md" ]; then
        mkdir -p "$docs_dir/spec"
        cp "$SCRIPT_DIR/spec/WIA-IND-012-v1.0.md" "$docs_dir/spec/"
        print_success "Specification copied to $docs_dir/spec"
    fi
}

# Run tests
run_tests() {
    print_step "Running tests..."

    # Test CLI installation
    if command -v "$CLI_NAME" &> /dev/null; then
        print_info "Testing CLI tool..."
        "$CLI_NAME" --version > /dev/null 2>&1 && print_success "CLI tool works correctly"
    fi

    # Test basic calculations
    print_info "Testing calculations..."
    "$CLI_NAME" calc-bmi --weight 70 --height 170 > /dev/null 2>&1 && print_success "BMI calculation works"
}

# Print installation summary
print_summary() {
    echo ""
    echo -e "${INDIGO}╔════════════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${INDIGO}║              Installation Complete! 🎉                         ║${RESET}"
    echo -e "${INDIGO}╚════════════════════════════════════════════════════════════════╝${RESET}"
    echo ""
    print_info "CLI Tool: $INSTALL_DIR/$CLI_NAME"
    print_info "Documentation: $HOME/.local/share/wia-ind-012"
    print_info "Configuration: $HOME/.config/wia-ind-012/config.json"
    echo ""
    echo -e "${CYAN}Quick Start:${RESET}"
    echo "  $CLI_NAME --help                    # Show help"
    echo "  $CLI_NAME calc-bmr --weight 70 --height 170 --age 30"
    echo "  $CLI_NAME calc-calories --activity running --duration 30"
    echo "  $CLI_NAME calc-hr-zones --age 30 --resting-hr 60"
    echo "  $CLI_NAME log-steps --steps 12500"
    echo ""
    echo -e "${CYAN}TypeScript SDK:${RESET}"
    echo "  npm install @wia/ind-012            # Install SDK"
    echo "  import { FitnessSDK } from '@wia/ind-012'"
    echo ""
    echo -e "${GRAY}弘익人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

# Uninstall function
uninstall() {
    print_header
    print_step "Uninstalling WIA-IND-012..."

    # Remove CLI
    if [ -f "$INSTALL_DIR/$CLI_NAME" ]; then
        rm -f "$INSTALL_DIR/$CLI_NAME"
        print_success "CLI tool removed"
    fi

    # Ask about config and docs
    read -p "Remove configuration and documentation? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$HOME/.config/wia-ind-012"
        rm -rf "$HOME/.local/share/wia-ind-012"
        print_success "Configuration and documentation removed"
    fi

    print_success "Uninstallation complete"
}

# Main installation flow
main() {
    print_header

    # Check for uninstall flag
    if [ "$1" = "--uninstall" ] || [ "$1" = "-u" ]; then
        uninstall
        exit 0
    fi

    # Installation steps
    check_permissions
    check_dependencies
    install_cli
    install_typescript_sdk
    create_config
    copy_docs
    run_tests
    print_summary
}

# Run main installation
main "$@"
