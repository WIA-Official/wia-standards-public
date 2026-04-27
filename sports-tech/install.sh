#!/usr/bin/env bash

################################################################################
# WIA-IND-011: Sports Tech Standard - Installation Script
#
# Version: 1.0.0
# Author: WIA Technical Committee - Sports Technology Division
# License: MIT
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This script installs the WIA-IND-011 Sports Tech CLI tool and SDK
################################################################################

set -euo pipefail

# Colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m'

# Installation paths
readonly INSTALL_DIR="${HOME}/.local/bin"
readonly CLI_NAME="wia-ind-011"
readonly SDK_DIR="${HOME}/.wia-sports"

# Version
readonly VERSION="1.0.0"

################################################################################
# Utility Functions
################################################################################

print_banner() {
    cat << "EOF"
    ⚽ WIA-IND-011 Sports Tech Standard
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Installation Script v1.0.0

    弘益人間 (Benefit All Humanity)

EOF
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $*"
}

print_success() {
    echo -e "${GREEN}✓${NC} $*"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $*"
}

print_error() {
    echo -e "${RED}✗${NC} $*" >&2
}

################################################################################
# System Detection
################################################################################

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

detect_package_manager() {
    if command -v apt-get &> /dev/null; then
        echo "apt"
    elif command -v yum &> /dev/null; then
        echo "yum"
    elif command -v brew &> /dev/null; then
        echo "brew"
    elif command -v pacman &> /dev/null; then
        echo "pacman"
    else
        echo "none"
    fi
}

################################################################################
# Dependency Installation
################################################################################

install_dependencies() {
    print_info "Checking dependencies..."

    local deps=("curl" "jq" "bc")
    local missing=()
    local pkg_manager
    pkg_manager=$(detect_package_manager)

    # Check which dependencies are missing
    for dep in "${deps[@]}"; do
        if ! command -v "${dep}" &> /dev/null; then
            missing+=("${dep}")
        fi
    done

    if [[ ${#missing[@]} -eq 0 ]]; then
        print_success "All dependencies already installed"
        return 0
    fi

    print_warning "Missing dependencies: ${missing[*]}"

    # Install based on package manager
    case ${pkg_manager} in
        apt)
            print_info "Installing via apt-get..."
            sudo apt-get update -qq
            sudo apt-get install -y "${missing[@]}"
            ;;
        yum)
            print_info "Installing via yum..."
            sudo yum install -y "${missing[@]}"
            ;;
        brew)
            print_info "Installing via Homebrew..."
            brew install "${missing[@]}"
            ;;
        pacman)
            print_info "Installing via pacman..."
            sudo pacman -S --noconfirm "${missing[@]}"
            ;;
        none)
            print_error "No supported package manager found"
            print_info "Please manually install: ${missing[*]}"
            return 1
            ;;
    esac

    print_success "Dependencies installed"
}

################################################################################
# CLI Installation
################################################################################

install_cli() {
    print_info "Installing CLI tool..."

    # Create install directory if it doesn't exist
    if [[ ! -d "${INSTALL_DIR}" ]]; then
        mkdir -p "${INSTALL_DIR}"
        print_info "Created directory: ${INSTALL_DIR}"
    fi

    # Copy CLI script
    local cli_source="./cli/${CLI_NAME}.sh"
    local cli_dest="${INSTALL_DIR}/${CLI_NAME}"

    if [[ ! -f "${cli_source}" ]]; then
        print_error "CLI script not found: ${cli_source}"
        return 1
    fi

    cp "${cli_source}" "${cli_dest}"
    chmod +x "${cli_dest}"
    print_success "Installed CLI to: ${cli_dest}"

    # Check if install dir is in PATH
    if [[ ":$PATH:" != *":${INSTALL_DIR}:"* ]]; then
        print_warning "${INSTALL_DIR} is not in your PATH"
        print_info "Add the following to your ~/.bashrc or ~/.zshrc:"
        echo
        echo -e "    ${CYAN}export PATH=\"\${PATH}:${INSTALL_DIR}\"${NC}"
        echo
    fi
}

################################################################################
# SDK Installation
################################################################################

install_sdk() {
    print_info "Installing TypeScript SDK..."

    # Create SDK directory
    if [[ ! -d "${SDK_DIR}" ]]; then
        mkdir -p "${SDK_DIR}"
        print_info "Created directory: ${SDK_DIR}"
    fi

    # Copy SDK files
    if [[ -d "./api/typescript" ]]; then
        cp -r ./api/typescript/* "${SDK_DIR}/"
        print_success "Copied SDK files to: ${SDK_DIR}"

        # Install npm dependencies if npm is available
        if command -v npm &> /dev/null; then
            print_info "Installing npm dependencies..."
            (cd "${SDK_DIR}" && npm install --silent) || print_warning "npm install failed (non-critical)"
            print_success "npm dependencies installed"
        else
            print_warning "npm not found. Skipping npm install."
            print_info "To use the SDK, install Node.js and run: cd ${SDK_DIR} && npm install"
        fi
    else
        print_warning "SDK source not found. Skipping SDK installation."
    fi
}

################################################################################
# Configuration
################################################################################

setup_config() {
    print_info "Setting up configuration..."

    local config_dir="${HOME}/.wia-sports"
    local config_file="${config_dir}/config.json"

    # Create config directory
    mkdir -p "${config_dir}"

    # Create default config if it doesn't exist
    if [[ ! -f "${config_file}" ]]; then
        cat > "${config_file}" << EOF
{
  "version": "${VERSION}",
  "standard_id": "WIA-IND-011",
  "api_base_url": "https://api.wia-sports.com/v1",
  "api_token": "",
  "default_athlete_id": "",
  "created": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "philosophy": "弘익人間 (Benefit All Humanity)"
}
EOF
        print_success "Created config file: ${config_file}"
    else
        print_info "Config file already exists: ${config_file}"
    fi
}

################################################################################
# Verification
################################################################################

verify_installation() {
    print_info "Verifying installation..."

    local errors=0

    # Check CLI
    if [[ -x "${INSTALL_DIR}/${CLI_NAME}" ]]; then
        print_success "CLI tool installed correctly"
    else
        print_error "CLI tool not found or not executable"
        ((errors++))
    fi

    # Check dependencies
    local deps=("curl" "jq" "bc")
    for dep in "${deps[@]}"; do
        if command -v "${dep}" &> /dev/null; then
            print_success "${dep} available"
        else
            print_error "${dep} not found"
            ((errors++))
        fi
    done

    # Check config
    if [[ -f "${HOME}/.wia-sports/config.json" ]]; then
        print_success "Configuration file created"
    else
        print_warning "Configuration file not found"
    fi

    return ${errors}
}

################################################################################
# Uninstall
################################################################################

uninstall() {
    print_info "Uninstalling WIA-IND-011 Sports Tech..."

    # Remove CLI
    if [[ -f "${INSTALL_DIR}/${CLI_NAME}" ]]; then
        rm "${INSTALL_DIR}/${CLI_NAME}"
        print_success "Removed CLI tool"
    fi

    # Ask before removing config
    read -p "Remove configuration files? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if [[ -d "${HOME}/.wia-sports" ]]; then
            rm -rf "${HOME}/.wia-sports"
            print_success "Removed configuration and SDK files"
        fi
    else
        print_info "Configuration files preserved"
    fi

    print_success "Uninstall complete"
}

################################################################################
# Main Installation Flow
################################################################################

main() {
    print_banner

    # Detect OS
    local os
    os=$(detect_os)
    print_info "Detected OS: ${os}"

    # Check if uninstall requested
    if [[ $# -gt 0 ]] && [[ "$1" == "uninstall" ]]; then
        uninstall
        exit 0
    fi

    echo
    print_info "Starting installation..."
    echo

    # Install dependencies
    install_dependencies || {
        print_error "Failed to install dependencies"
        exit 1
    }

    echo

    # Install CLI
    install_cli || {
        print_error "Failed to install CLI"
        exit 1
    }

    echo

    # Install SDK
    install_sdk

    echo

    # Setup configuration
    setup_config

    echo

    # Verify installation
    if verify_installation; then
        echo
        print_success "✓ Installation completed successfully!"
        echo
        print_info "Get started:"
        echo
        echo -e "  ${CYAN}wia-ind-011 --help${NC}          Show help"
        echo -e "  ${CYAN}wia-ind-011 version${NC}         Show version"
        echo -e "  ${CYAN}wia-ind-011 track --help${NC}    Track performance"
        echo
        print_info "Documentation: https://wiastandards.com/sports-tech"
        echo
        echo -e "${CYAN}弘益人間 (Benefit All Humanity)${NC}"
        echo
    else
        echo
        print_warning "Installation completed with some issues"
        print_info "Please review the errors above"
        exit 1
    fi
}

################################################################################
# Entry Point
################################################################################

# Handle script arguments
if [[ $# -gt 0 ]]; then
    case "$1" in
        --help|-h)
            cat << EOF
WIA-IND-011 Sports Tech Installation Script

Usage:
  ./install.sh              Install WIA-IND-011 Sports Tech
  ./install.sh uninstall    Uninstall WIA-IND-011 Sports Tech
  ./install.sh --help       Show this help message

弘益人間 (Benefit All Humanity)
EOF
            exit 0
            ;;
    esac
fi

# Run main installation
main "$@"
