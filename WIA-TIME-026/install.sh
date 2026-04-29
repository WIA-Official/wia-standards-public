#!/bin/bash

###############################################################################
# WIA-TIME-026: Chronology Testing - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
INSTALL_DIR="/usr/local/bin"
CONFIG_DIR="$HOME/.wia/time-026"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${PURPLE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${PURPLE}║${NC}  🧪 WIA-TIME-026: Chronology Testing Installer          ${PURPLE}║${NC}"
    echo -e "${PURPLE}║${NC}  Version: 1.0.0                                         ${PURPLE}║${NC}"
    echo -e "${PURPLE}║${NC}  弘益人間 (Benefit All Humanity)                         ${PURPLE}║${NC}"
    echo -e "${PURPLE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_step() {
    echo -e "${CYAN}➜ $1${NC}"
}

check_requirements() {
    print_step "Checking system requirements..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for openssl (for random cert IDs)
    if ! command -v openssl &> /dev/null; then
        print_warning "openssl not found (optional, for certificate generation)"
    fi

    # Check for jq (for JSON processing)
    if ! command -v jq &> /dev/null; then
        print_warning "jq not found (optional, for JSON processing)"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install missing dependencies and try again."
        exit 1
    fi

    print_success "All required dependencies found"
}

create_directories() {
    print_step "Creating configuration directories..."

    mkdir -p "$CONFIG_DIR"
    mkdir -p "$CONFIG_DIR/data"
    mkdir -p "$CONFIG_DIR/reports"
    mkdir -p "$CONFIG_DIR/certificates"
    mkdir -p "$CONFIG_DIR/logs"

    print_success "Configuration directories created: $CONFIG_DIR"
}

install_cli() {
    print_step "Installing CLI tool..."

    local cli_source="$SCRIPT_DIR/cli/wia-time-026.sh"
    local cli_target="$INSTALL_DIR/wia-time-026"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI script not found: $cli_source"
        exit 1
    fi

    # Check if we need sudo
    if [ -w "$INSTALL_DIR" ]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
    else
        print_info "Installing to $INSTALL_DIR requires sudo..."
        sudo cp "$cli_source" "$cli_target"
        sudo chmod +x "$cli_target"
    fi

    print_success "CLI tool installed: $cli_target"
}

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_warning "TypeScript SDK directory not found, skipping..."
        return
    fi

    cd "$ts_dir"

    # Check if npm is available
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "To install TypeScript SDK manually:"
        print_info "  cd $ts_dir"
        print_info "  npm install"
        print_info "  npm run build"
        cd "$SCRIPT_DIR"
        return
    fi

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install --silent

    # Build TypeScript
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript SDK..."
        npm run build --silent || print_warning "Build failed, but continuing..."
    fi

    cd "$SCRIPT_DIR"
    print_success "TypeScript SDK installed"
}

create_config_file() {
    print_step "Creating default configuration..."

    local config_file="$CONFIG_DIR/config.json"

    if [ -f "$config_file" ]; then
        print_warning "Configuration file already exists, skipping..."
        return
    fi

    cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "settings": {
    "defaultTestSuite": "standard",
    "defaultTestDuration": 3600,
    "defaultCertificateValidity": 365,
    "safetyLevel": "maximum",
    "simulationQuality": "high",
    "enableLogging": true,
    "logLevel": "info"
  },
  "paths": {
    "dataDir": "~/.wia/time-026/data",
    "reportsDir": "~/.wia/time-026/reports",
    "certificatesDir": "~/.wia/time-026/certificates",
    "logsDir": "~/.wia/time-026/logs"
  },
  "testWeights": {
    "systemIntegration": 0.25,
    "equipmentCertification": 0.20,
    "safetyValidation": 0.20,
    "timelineSimulation": 0.20,
    "stressTesting": 0.15
  },
  "thresholds": {
    "minimumPassScore": 80,
    "minimumReliability": 0.95,
    "maximumParadoxRisk": 0.05
  },
  "certification": {
    "authority": "WIA Certification Authority",
    "issuer": "WIA Time Research Group",
    "baseUrl": "https://cert.wiastandards.com"
  }
}
EOF

    print_success "Default configuration created: $config_file"
}

create_sample_files() {
    print_step "Creating sample files..."

    # Sample device configuration
    local device_file="$CONFIG_DIR/data/sample-device.json"
    cat > "$device_file" << 'EOF'
{
  "id": "TM-2024-001",
  "manufacturer": "TemporalTech Industries",
  "model": "ChronoNavigator X1",
  "serialNumber": "TM-2024-001",
  "manufactureDate": "2024-01-01",
  "specifications": {
    "maxTemporalDistance": 10000,
    "maxSpatialDistance": 1000000,
    "maxOccupancy": 4,
    "energyCapacity": 1e18,
    "powerOutput": 1e9
  }
}
EOF

    print_success "Sample device file created: $device_file"
}

setup_completion() {
    print_step "Setting up shell completion..."

    # Bash completion
    local completion_dir="/etc/bash_completion.d"
    local completion_file="$completion_dir/wia-time-026"

    if [ -d "$completion_dir" ] && [ -w "$completion_dir" ]; then
        cat > "$completion_file" << 'EOF'
# Bash completion for wia-time-026

_wia_time_026_completion() {
    local cur prev commands
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    commands="test-system simulate certify validate-safety stress-test qa-check report benchmark version help"

    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "$commands" -- $cur) )
        return 0
    fi

    case "${prev}" in
        --suite)
            COMPREPLY=( $(compgen -W "basic standard comprehensive forensic" -- $cur) )
            return 0
            ;;
        --intensity|--stress-level)
            COMPREPLY=( $(compgen -W "low medium high extreme" -- $cur) )
            return 0
            ;;
        --format)
            COMPREPLY=( $(compgen -W "pdf json html" -- $cur) )
            return 0
            ;;
    esac
}

complete -F _wia_time_026_completion wia-time-026
EOF
        print_success "Bash completion installed"
    else
        print_warning "Bash completion directory not writable, skipping..."
    fi
}

verify_installation() {
    print_step "Verifying installation..."

    # Check if CLI is accessible
    if ! command -v wia-time-026 &> /dev/null; then
        print_error "CLI tool not found in PATH"
        print_info "You may need to add $INSTALL_DIR to your PATH"
        print_info "Add this to your ~/.bashrc or ~/.zshrc:"
        print_info "  export PATH=\"\$PATH:$INSTALL_DIR\""
        return 1
    fi

    # Test CLI
    if wia-time-026 version &> /dev/null; then
        print_success "CLI tool is working correctly"
    else
        print_warning "CLI tool installed but may have issues"
    fi

    return 0
}

show_next_steps() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}  ${GREEN}Installation Complete!${NC}                                   ${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Next Steps:${NC}"
    echo ""
    echo -e "1. ${GREEN}Test the installation:${NC}"
    echo -e "   ${BLUE}wia-time-026 version${NC}"
    echo ""
    echo -e "2. ${GREEN}Run a system test:${NC}"
    echo -e "   ${BLUE}wia-time-026 test-system --device TM-2024-001 --suite standard${NC}"
    echo ""
    echo -e "3. ${GREEN}Simulate a timeline:${NC}"
    echo -e "   ${BLUE}wia-time-026 simulate --target 1969-07-20 --scenario observation${NC}"
    echo ""
    echo -e "4. ${GREEN}View all commands:${NC}"
    echo -e "   ${BLUE}wia-time-026 help${NC}"
    echo ""
    echo -e "${CYAN}Configuration:${NC}"
    echo -e "   Config file: ${BLUE}$CONFIG_DIR/config.json${NC}"
    echo -e "   Data dir:    ${BLUE}$CONFIG_DIR/data${NC}"
    echo -e "   Reports dir: ${BLUE}$CONFIG_DIR/reports${NC}"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo -e "   Website:     ${BLUE}https://wiastandards.com${NC}"
    echo -e "   Docs:        ${BLUE}https://docs.wiastandards.com/WIA-TIME-026${NC}"
    echo -e "   GitHub:      ${BLUE}https://github.com/WIA-Official/wia-standards${NC}"
    echo ""
    echo -e "${PURPLE}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
}

uninstall() {
    print_header
    print_warning "Uninstalling WIA-TIME-026..."
    echo ""

    # Remove CLI
    if [ -f "$INSTALL_DIR/wia-time-026" ]; then
        print_step "Removing CLI tool..."
        if [ -w "$INSTALL_DIR" ]; then
            rm -f "$INSTALL_DIR/wia-time-026"
        else
            sudo rm -f "$INSTALL_DIR/wia-time-026"
        fi
        print_success "CLI tool removed"
    fi

    # Remove configuration (ask first)
    if [ -d "$CONFIG_DIR" ]; then
        echo ""
        read -p "Remove configuration directory ($CONFIG_DIR)? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$CONFIG_DIR"
            print_success "Configuration directory removed"
        else
            print_info "Configuration directory preserved"
        fi
    fi

    # Remove completion
    if [ -f "/etc/bash_completion.d/wia-time-026" ]; then
        print_step "Removing bash completion..."
        sudo rm -f "/etc/bash_completion.d/wia-time-026"
        print_success "Bash completion removed"
    fi

    echo ""
    print_success "Uninstall complete"
    echo ""
}

###############################################################################
# Main Installation Flow
###############################################################################

main() {
    # Check for uninstall
    if [ "${1:-}" = "uninstall" ]; then
        uninstall
        exit 0
    fi

    print_header

    print_info "Starting installation..."
    echo ""

    check_requirements
    create_directories
    install_cli
    install_typescript_sdk
    create_config_file
    create_sample_files
    setup_completion

    echo ""

    if verify_installation; then
        show_next_steps
    else
        print_error "Installation completed with warnings"
        print_info "Please check the output above for issues"
        exit 1
    fi
}

# Run main installation
main "$@"
