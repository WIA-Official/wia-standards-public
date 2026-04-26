#!/bin/bash

################################################################################
# WIA-QUA-016: FTL Communication - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Future Technologies Research Group
#
# 弘익人間 (Benefit All Humanity)
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
STANDARD="WIA-QUA-016"
STANDARD_NAME="ftl-communication"
VERSION="1.0.0"
INSTALL_DIR="${HOME}/.wia/standards/${STANDARD_NAME}"
BIN_DIR="${HOME}/.wia/bin"

print_header() {
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}  🚀 ${MAGENTA}WIA-QUA-016: FTL Communication${NC}                     ${CYAN}║${NC}"
    echo -e "${CYAN}║${NC}     Installation Script v${VERSION}                         ${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ ERROR: $1${NC}"
}

check_dependencies() {
    print_info "Checking dependencies..."

    # Check for bash
    if ! command -v bash &> /dev/null; then
        print_error "bash is required but not installed"
        exit 1
    fi

    # Check for node (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js found: $(node --version)"
    else
        print_warning "Node.js not found - TypeScript SDK will not be available"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command -v npm &> /dev/null; then
        print_success "npm found: $(npm --version)"
    else
        print_warning "npm not found - TypeScript SDK installation skipped"
    fi

    echo ""
}

create_directories() {
    print_info "Creating directories..."

    mkdir -p "${INSTALL_DIR}"
    mkdir -p "${BIN_DIR}"

    print_success "Directories created"
    echo ""
}

install_cli() {
    print_info "Installing CLI tool..."

    # Get the directory where this script is located
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Copy CLI script
    cp "${SCRIPT_DIR}/cli/wia-qua-016.sh" "${BIN_DIR}/wia-qua-016"
    chmod +x "${BIN_DIR}/wia-qua-016"

    # Copy standard files
    cp -r "${SCRIPT_DIR}"/* "${INSTALL_DIR}/"

    print_success "CLI tool installed to ${BIN_DIR}/wia-qua-016"
    echo ""
}

install_typescript_sdk() {
    if ! command -v npm &> /dev/null; then
        print_warning "Skipping TypeScript SDK installation (npm not found)"
        return
    fi

    print_info "Installing TypeScript SDK..."

    cd "${INSTALL_DIR}/api/typescript"

    # Install dependencies
    npm install --production

    # Build if TypeScript compiler is available
    if [ -f "tsconfig.json" ]; then
        npm run build 2>/dev/null || print_warning "Build skipped (TypeScript not configured)"
    fi

    print_success "TypeScript SDK installed"
    echo ""
}

setup_path() {
    print_info "Setting up PATH..."

    # Detect shell configuration file
    SHELL_RC=""
    if [ -f "${HOME}/.bashrc" ]; then
        SHELL_RC="${HOME}/.bashrc"
    elif [ -f "${HOME}/.bash_profile" ]; then
        SHELL_RC="${HOME}/.bash_profile"
    elif [ -f "${HOME}/.zshrc" ]; then
        SHELL_RC="${HOME}/.zshrc"
    fi

    if [ -z "$SHELL_RC" ]; then
        print_warning "Could not detect shell configuration file"
        print_info "Please manually add ${BIN_DIR} to your PATH"
        return
    fi

    # Check if already in PATH
    if grep -q "\.wia/bin" "$SHELL_RC"; then
        print_info "PATH already configured in $SHELL_RC"
    else
        echo "" >> "$SHELL_RC"
        echo "# WIA Standards - Added by install script" >> "$SHELL_RC"
        echo "export PATH=\"\${HOME}/.wia/bin:\${PATH}\"" >> "$SHELL_RC"
        print_success "Added ${BIN_DIR} to PATH in $SHELL_RC"
    fi

    echo ""
}

show_theoretical_warning() {
    echo ""
    echo -e "${YELLOW}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║                                                            ║${NC}"
    echo -e "${YELLOW}║                  ⚠️  IMPORTANT NOTICE ⚠️                    ║${NC}"
    echo -e "${YELLOW}║                                                            ║${NC}"
    echo -e "${YELLOW}║  This standard implements THEORETICAL frameworks for FTL  ║${NC}"
    echo -e "${YELLOW}║  communication. Current physics indicates FTL is           ║${NC}"
    echo -e "${YELLOW}║  IMPOSSIBLE. This tool is for:                            ║${NC}"
    echo -e "${YELLOW}║                                                            ║${NC}"
    echo -e "${YELLOW}║  • Educational purposes                                    ║${NC}"
    echo -e "${YELLOW}║  • Theoretical research                                    ║${NC}"
    echo -e "${YELLOW}║  • Science fiction world-building                         ║${NC}"
    echo -e "${YELLOW}║  • Thought experiments                                     ║${NC}"
    echo -e "${YELLOW}║                                                            ║${NC}"
    echo -e "${YELLOW}║  No known physical mechanisms support FTL communication.   ║${NC}"
    echo -e "${YELLOW}║                                                            ║${NC}"
    echo -e "${YELLOW}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

show_completion_message() {
    print_success "Installation complete!"
    echo ""
    echo -e "${CYAN}Next steps:${NC}"
    echo ""
    echo -e "  1. Reload your shell or run: ${GREEN}source ~/.bashrc${NC}"
    echo -e "  2. Verify installation: ${GREEN}wia-qua-016 version${NC}"
    echo -e "  3. View help: ${GREEN}wia-qua-016 help${NC}"
    echo -e "  4. Get information: ${GREEN}wia-qua-016 info${NC}"
    echo ""
    echo -e "${CYAN}Quick examples:${NC}"
    echo ""
    echo -e "  ${BLUE}wia-qua-016 quantum-channel --distance 4.24${NC}"
    echo -e "  ${BLUE}wia-qua-016 energy-calc --method alcubierre --distance 10${NC}"
    echo -e "  ${BLUE}wia-qua-016 causality-check --speed 10c${NC}"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo ""
    echo -e "  Spec: ${INSTALL_DIR}/spec/WIA-QUA-016-v1.0.md"
    echo -e "  README: ${INSTALL_DIR}/README.md"
    echo -e "  Online: https://docs.wiastandards.com/qua-016"
    echo ""
    echo -e "${MAGENTA}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
}

main() {
    print_header

    check_dependencies
    create_directories
    install_cli
    install_typescript_sdk
    setup_path
    show_theoretical_warning
    show_completion_message
}

main "$@"

################################################################################
# 弘益人間 (Benefit All Humanity)
################################################################################
