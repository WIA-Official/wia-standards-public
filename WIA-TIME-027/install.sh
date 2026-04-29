#!/bin/bash

###############################################################################
# WIA-TIME-027: Traveler Bio-Safety - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

# Configuration
STANDARD="WIA-TIME-027"
VERSION="1.0.0"
INSTALL_DIR="${HOME}/.wia/time-027"
BIN_DIR="${HOME}/.local/bin"

###############################################################################
# Utility Functions
###############################################################################

print_header() {
    echo -e "${PURPLE}"
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║  🏥 WIA-TIME-027: Traveler Bio-Safety Installation        ║"
    echo "║  Version: ${VERSION}                                         ║"
    echo "║  弘益人間 (Benefit All Humanity)                        ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo
}

print_success() {
    echo -e "${GREEN}✓${NC} $*"
}

print_error() {
    echo -e "${RED}✗${NC} $*"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $*"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $*"
}

check_requirements() {
    echo "Checking requirements..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for basic utilities
    for cmd in mkdir cp chmod; do
        if ! command -v $cmd &> /dev/null; then
            missing_deps+=("$cmd")
        fi
    done

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        exit 1
    fi

    print_success "All requirements satisfied"
}

create_directories() {
    echo "Creating directories..."

    mkdir -p "${INSTALL_DIR}"
    mkdir -p "${BIN_DIR}"
    mkdir -p "${INSTALL_DIR}/data"
    mkdir -p "${INSTALL_DIR}/logs"
    mkdir -p "${INSTALL_DIR}/certs"

    print_success "Directories created"
}

install_cli() {
    echo "Installing CLI tool..."

    local cli_source="./cli/wia-time-027.sh"
    local cli_dest="${BIN_DIR}/wia-time-027"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        exit 1
    fi

    cp "$cli_source" "$cli_dest"
    chmod +x "$cli_dest"

    print_success "CLI tool installed to ${cli_dest}"
}

install_typescript_sdk() {
    echo "Installing TypeScript SDK..."

    if command -v npm &> /dev/null; then
        cd api/typescript

        print_info "Installing dependencies..."
        npm install --silent

        print_info "Building TypeScript SDK..."
        npm run build --silent

        cd ../..

        print_success "TypeScript SDK installed"
        print_info "To use in your project: npm install @wia/time-027"
    else
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js to use TypeScript SDK"
    fi
}

create_config() {
    echo "Creating configuration..."

    local config_file="${INSTALL_DIR}/config.json"

    cat > "$config_file" << EOF
{
  "version": "${VERSION}",
  "standard": "${STANDARD}",
  "installDate": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "paths": {
    "data": "${INSTALL_DIR}/data",
    "logs": "${INSTALL_DIR}/logs",
    "certs": "${INSTALL_DIR}/certs"
  },
  "defaults": {
    "strictMode": false,
    "realTimeMonitoring": true,
    "alertThreshold": 0.85,
    "autoQuarantine": false
  },
  "limits": {
    "radiation": {
      "shortTerm": 50,
      "medium": 100,
      "extended": 200,
      "longDuration": 500
    },
    "quarantine": {
      "low": 24,
      "medium": 48,
      "high": 168,
      "critical": 336
    }
  }
}
EOF

    print_success "Configuration created"
}

setup_environment() {
    echo "Setting up environment..."

    local shell_rc=""

    if [ -f "${HOME}/.bashrc" ]; then
        shell_rc="${HOME}/.bashrc"
    elif [ -f "${HOME}/.zshrc" ]; then
        shell_rc="${HOME}/.zshrc"
    fi

    if [ -n "$shell_rc" ]; then
        # Check if PATH already includes BIN_DIR
        if ! grep -q "${BIN_DIR}" "$shell_rc"; then
            echo "" >> "$shell_rc"
            echo "# WIA-TIME-027 Bio-Safety" >> "$shell_rc"
            echo "export PATH=\"\$PATH:${BIN_DIR}\"" >> "$shell_rc"
            echo "export WIA_TIME_027_HOME=\"${INSTALL_DIR}\"" >> "$shell_rc"

            print_success "Environment variables added to $shell_rc"
            print_warning "Run 'source $shell_rc' or restart your shell"
        else
            print_info "Environment already configured"
        fi
    fi

    # Set for current session
    export PATH="$PATH:${BIN_DIR}"
    export WIA_TIME_027_HOME="${INSTALL_DIR}"
}

create_sample_data() {
    echo "Creating sample data..."

    local sample_dir="${INSTALL_DIR}/samples"
    mkdir -p "$sample_dir"

    # Sample screening result
    cat > "${sample_dir}/sample-screening.json" << 'EOF'
{
  "screeningId": "SCR-SAMPLE-001",
  "travelerId": "TR-SAMPLE",
  "screeningDate": "2024-01-01T00:00:00Z",
  "destination": {
    "era": "MEDIEVAL",
    "year": 1347,
    "location": { "x": 48.8566, "y": 2.3522, "z": 0 },
    "timeline": "PRIME"
  },
  "bioSafetyIndex": 0.895,
  "cleared": true,
  "clearanceLevel": "good",
  "recommendations": [
    "Plague vaccination required",
    "N100 respirator recommended",
    "Avoid physical contact with locals"
  ]
}
EOF

    print_success "Sample data created in ${sample_dir}"
}

run_tests() {
    echo "Running verification tests..."

    # Test CLI is accessible
    if command -v wia-time-027 &> /dev/null; then
        print_success "CLI command accessible"
    else
        print_warning "CLI not in PATH - may need to restart shell"
    fi

    # Test directories exist
    for dir in data logs certs; do
        if [ -d "${INSTALL_DIR}/${dir}" ]; then
            print_success "Directory exists: ${dir}"
        else
            print_error "Directory missing: ${dir}"
        fi
    done
}

print_completion() {
    echo
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║            Installation Complete! 🎉                       ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo
    echo "Installation Summary:"
    echo "  • Standard: ${STANDARD}"
    echo "  • Version: ${VERSION}"
    echo "  • Install Directory: ${INSTALL_DIR}"
    echo "  • CLI Location: ${BIN_DIR}/wia-time-027"
    echo
    echo "Quick Start:"
    echo "  1. Restart your shell or run: source ~/.bashrc"
    echo "  2. Test installation: wia-time-027 version"
    echo "  3. View help: wia-time-027 help"
    echo "  4. Screen traveler: wia-time-027 screen --traveler TR-001 --destination \"1347-EUROPE\" --duration 72h"
    echo
    echo "TypeScript SDK:"
    echo "  • Install: npm install @wia/time-027"
    echo "  • Import: import { BioSafetyMonitor } from '@wia/time-027';"
    echo
    echo "Documentation:"
    echo "  • Specification: ./spec/WIA-TIME-027-v1.0.md"
    echo "  • README: ./README.md"
    echo "  • Website: https://wiastandards.com/standards/WIA-TIME-027"
    echo
    echo "Configuration:"
    echo "  • Config file: ${INSTALL_DIR}/config.json"
    echo "  • Data directory: ${INSTALL_DIR}/data"
    echo "  • Logs: ${INSTALL_DIR}/logs"
    echo
    echo -e "${PURPLE}弘益人間 (Benefit All Humanity)${NC}"
    echo "WIA - World Certification Industry Association"
    echo "© 2025 SmileStory Inc. / WIA"
    echo
}

###############################################################################
# Main Installation Process
###############################################################################

main() {
    print_header

    echo "Starting installation of ${STANDARD} v${VERSION}..."
    echo

    check_requirements
    create_directories
    install_cli

    # Optional: Install TypeScript SDK if npm is available
    if [ "${SKIP_NPM:-false}" != "true" ]; then
        install_typescript_sdk
    fi

    create_config
    setup_environment
    create_sample_data
    run_tests

    print_completion
}

# Handle script arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-npm)
            SKIP_NPM=true
            shift
            ;;
        --help|-h)
            echo "WIA-TIME-027 Installation Script"
            echo
            echo "Usage: ./install.sh [OPTIONS]"
            echo
            echo "OPTIONS:"
            echo "  --skip-npm    Skip npm/TypeScript SDK installation"
            echo "  --help, -h    Show this help message"
            echo
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run './install.sh --help' for usage information"
            exit 1
            ;;
    esac
done

# Run main installation
main
