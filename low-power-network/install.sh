#!/usr/bin/env bash

##############################################################################
# WIA-COMM-018: Low-Power Network - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Communication Research Group
#
# 弘益人間 (Benefit All Humanity)
##############################################################################

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${HOME}/.local/bin"

print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  WIA-COMM-018 Installation${NC}"
    echo -e "${CYAN}  Low-Power Network Standard${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

##############################################################################
# Check Dependencies
##############################################################################

check_dependencies() {
    print_info "Checking dependencies..."

    local missing=()

    # Check for jq (required for CLI)
    if ! command -v jq &> /dev/null; then
        missing+=("jq")
    fi

    # Check for Node.js (optional, for TypeScript SDK)
    if ! command -v node &> /dev/null; then
        print_warning "Node.js not found (optional for TypeScript SDK)"
    else
        local node_version=$(node --version | sed 's/v//')
        print_success "Node.js ${node_version} found"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found (optional for TypeScript SDK)"
    fi

    if [ ${#missing[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing[*]}"
        echo ""
        print_info "Install missing dependencies:"
        echo "  macOS:   brew install ${missing[*]}"
        echo "  Ubuntu:  sudo apt-get install ${missing[*]}"
        echo "  Fedora:  sudo dnf install ${missing[*]}"
        return 1
    fi

    print_success "All required dependencies found"
}

##############################################################################
# Install CLI Tool
##############################################################################

install_cli() {
    print_info "Installing CLI tool..."

    # Create install directory
    mkdir -p "${INSTALL_DIR}"

    # Copy CLI script
    cp "${SCRIPT_DIR}/cli/wia-comm-018.sh" "${INSTALL_DIR}/wia-comm-018"
    chmod +x "${INSTALL_DIR}/wia-comm-018"

    print_success "CLI tool installed to ${INSTALL_DIR}/wia-comm-018"

    # Check if install dir is in PATH
    if [[ ":$PATH:" != *":${INSTALL_DIR}:"* ]]; then
        print_warning "${INSTALL_DIR} is not in your PATH"
        echo ""
        print_info "Add the following to your shell profile (~/.bashrc, ~/.zshrc, etc.):"
        echo "  export PATH=\"\$PATH:${INSTALL_DIR}\""
    fi
}

##############################################################################
# Install TypeScript SDK
##############################################################################

install_typescript_sdk() {
    if ! command -v npm &> /dev/null; then
        print_warning "Skipping TypeScript SDK (npm not found)"
        return 0
    fi

    print_info "Installing TypeScript SDK..."

    cd "${SCRIPT_DIR}/api/typescript"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install --silent

    # Build TypeScript
    if command -v npx &> /dev/null; then
        print_info "Building TypeScript SDK..."
        npx tsc || print_warning "TypeScript build failed (tsc not configured)"
    fi

    print_success "TypeScript SDK installed"
    print_info "Install globally: cd ${SCRIPT_DIR}/api/typescript && npm install -g"
}

##############################################################################
# Setup Configuration
##############################################################################

setup_config() {
    print_info "Setting up configuration..."

    local config_dir="${HOME}/.wia-comm-018"
    mkdir -p "${config_dir}"

    if [[ ! -f "${config_dir}/config.json" ]]; then
        cat > "${config_dir}/config.json" <<'EOF'
{
  "technology": "LoRaWAN",
  "deviceEUI": "0000000000000000",
  "appEUI": "0000000000000000",
  "appKey": "00000000000000000000000000000000",
  "region": "US915",
  "deviceClass": "A",
  "spreadingFactor": 10,
  "txPower": 14,
  "enableADR": true,
  "confirmed": false
}
EOF
        print_success "Created default configuration at ${config_dir}/config.json"
    else
        print_info "Configuration already exists at ${config_dir}/config.json"
    fi
}

##############################################################################
# Verify Installation
##############################################################################

verify_installation() {
    print_info "Verifying installation..."

    if [[ -x "${INSTALL_DIR}/wia-comm-018" ]]; then
        print_success "CLI tool is executable"

        # Test CLI
        if "${INSTALL_DIR}/wia-comm-018" version > /dev/null 2>&1; then
            print_success "CLI tool works correctly"
        else
            print_warning "CLI tool installed but may not work correctly"
        fi
    else
        print_error "CLI tool installation failed"
        return 1
    fi

    # Check TypeScript SDK
    if [[ -d "${SCRIPT_DIR}/api/typescript/node_modules" ]]; then
        print_success "TypeScript SDK dependencies installed"
    fi
}

##############################################################################
# Print Usage
##############################################################################

print_usage() {
    echo ""
    echo -e "${CYAN}Installation complete!${NC}"
    echo ""
    echo "Quick Start:"
    echo ""
    echo "  1. Initialize device:"
    echo "     wia-comm-018 init --tech LoRaWAN --region US915 --class A"
    echo ""
    echo "  2. Configure credentials (edit):"
    echo "     ${HOME}/.wia-comm-018/config.json"
    echo ""
    echo "  3. Join network (LoRaWAN):"
    echo "     wia-comm-018 join --otaa --retry 3"
    echo ""
    echo "  4. Send message:"
    echo "     wia-comm-018 send --port 1 --payload 01670210"
    echo ""
    echo "  5. Monitor messages:"
    echo "     wia-comm-018 monitor --show-rssi --show-snr"
    echo ""
    echo "For more information:"
    echo "  wia-comm-018 help"
    echo ""
    echo "TypeScript SDK:"
    echo "  cd ${SCRIPT_DIR}/api/typescript"
    echo "  npm install"
    echo "  npm run build"
    echo ""
    echo "Documentation:"
    echo "  README:  ${SCRIPT_DIR}/README.md"
    echo "  Spec:    ${SCRIPT_DIR}/spec/WIA-COMM-018-v1.0.md"
    echo ""
    echo -e "${CYAN}弘益人間 (Benefit All Humanity)${NC}"
    echo "© 2025 WIA - World Certification Industry Association"
    echo ""
}

##############################################################################
# Main Installation
##############################################################################

main() {
    print_header

    # Check dependencies
    check_dependencies || exit 1

    echo ""

    # Install components
    install_cli
    echo ""

    setup_config
    echo ""

    install_typescript_sdk
    echo ""

    # Verify installation
    verify_installation || exit 1

    # Print usage
    print_usage
}

# Run installation
main "$@"
