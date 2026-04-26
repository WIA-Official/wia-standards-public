#!/bin/bash

################################################################################
# WIA-COMM-011: Edge Computing - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Edge Computing Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-COMM-011 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      📱 WIA-COMM-011: Edge Computing Installer                ║"
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
        print_info "Install with: apt-get install bc (Debian/Ubuntu) or brew install bc (macOS)"
        all_ok=false
    fi

    # Check Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional, for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    fi

    # Check npm (optional, for TypeScript SDK)
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional, for TypeScript SDK)"
    fi

    # Check kubectl (optional, for edge orchestration)
    if command -v kubectl &> /dev/null; then
        print_success "kubectl: $(kubectl version --client --short 2>/dev/null || echo 'installed')"
    else
        print_warning "kubectl not found (optional, for edge orchestration)"
        print_info "Install from: https://kubernetes.io/docs/tasks/tools/"
    fi

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-comm-011.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    # Make executable
    chmod +x "$cli_path"
    print_success "Made CLI script executable"

    # Check if we can write to /usr/local/bin
    if [ -w "$install_dir" ] || sudo -n true 2>/dev/null; then
        # Try to create symlink
        if sudo ln -sf "$cli_path" "$install_dir/wia-comm-011" 2>/dev/null; then
            print_success "Installed CLI to $install_dir/wia-comm-011"
            print_info "You can now run: wia-comm-011 --help"
        else
            print_warning "Could not install to $install_dir (permission denied)"
            print_info "You can run the CLI directly from: $cli_path"
            print_info "Or add an alias: alias wia-comm-011='$cli_path'"
        fi
    else
        print_warning "No write permission to $install_dir"
        print_info "You can run the CLI directly from: $cli_path"
        print_info "Or add to your PATH: export PATH=\"\$PATH:$SCRIPT_DIR/cli\""
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript SDK directory not found"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    if npm install --silent 2>/dev/null; then
        print_success "Dependencies installed"
    else
        print_warning "Could not install dependencies"
        print_info "Run 'npm install' manually in $ts_dir"
        return 0
    fi

    print_info "Building TypeScript SDK..."
    if npm run build --silent 2>/dev/null; then
        print_success "TypeScript SDK built successfully"
        print_info "Import with: import { EdgeComputingSDK } from '@wia/comm-011';"
    else
        print_warning "Could not build TypeScript SDK"
        print_info "Run 'npm run build' manually in $ts_dir"
    fi

    cd "$SCRIPT_DIR"
}

# Install documentation
install_docs() {
    print_section "Installing Documentation"

    local spec_file="$SCRIPT_DIR/spec/WIA-COMM-011-v1.0.md"
    local readme_file="$SCRIPT_DIR/README.md"

    if [ -f "$spec_file" ]; then
        print_success "Specification: $spec_file"
    else
        print_warning "Specification not found"
    fi

    if [ -f "$readme_file" ]; then
        print_success "README: $readme_file"
    else
        print_warning "README not found"
    fi

    print_info "Online documentation: https://wiastandards.com/standards/WIA-COMM-011"
}

# Verify installation
verify_installation() {
    print_section "Verifying Installation"

    local all_ok=true

    # Check CLI
    if command -v wia-comm-011 &> /dev/null; then
        print_success "CLI tool: wia-comm-011 (accessible in PATH)"
        wia-comm-011 --version | grep -q "1.0.0" && print_info "  Version check passed"
    elif [ -x "$SCRIPT_DIR/cli/wia-comm-011.sh" ]; then
        print_success "CLI tool: $SCRIPT_DIR/cli/wia-comm-011.sh (direct access)"
    else
        print_error "CLI tool not found or not executable"
        all_ok=false
    fi

    # Check TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK: Built and ready"
    else
        print_warning "TypeScript SDK: Not built (optional)"
    fi

    # Check documentation
    if [ -f "$SCRIPT_DIR/spec/WIA-COMM-011-v1.0.md" ]; then
        print_success "Documentation: Available"
    else
        print_warning "Documentation: Not found"
    fi

    if [ "$all_ok" = true ]; then
        echo ""
        print_success "Installation verification completed successfully!"
    else
        echo ""
        print_warning "Installation completed with warnings"
    fi
}

# Show next steps
show_next_steps() {
    print_section "Next Steps"

    echo ""
    echo -e "${GREEN}Quick Start Commands:${RESET}"
    echo ""
    echo "  # Calculate edge latency"
    echo "  wia-comm-011 calc-latency device access 100 5g"
    echo ""
    echo "  # Optimize workload placement"
    echo "  wia-comm-011 optimize video-analytics minimize-latency maxLatency=5ms"
    echo ""
    echo "  # Deploy edge AI model"
    echo "  wia-comm-011 deploy-model mobilenet-v2 int8 edge-tpu"
    echo ""
    echo "  # Monitor edge nodes"
    echo "  wia-comm-011 monitor all cpu,memory,latency"
    echo ""
    echo -e "${GREEN}TypeScript Usage:${RESET}"
    echo ""
    echo "  import { EdgeComputingSDK } from '@wia/comm-011';"
    echo ""
    echo "  const sdk = new EdgeComputingSDK();"
    echo "  const latency = sdk.calculateEdgeLatency({...});"
    echo ""
    echo -e "${GREEN}Documentation:${RESET}"
    echo ""
    echo "  README:  $SCRIPT_DIR/README.md"
    echo "  Spec:    $SCRIPT_DIR/spec/WIA-COMM-011-v1.0.md"
    echo "  Online:  https://wiastandards.com/standards/WIA-COMM-011"
    echo ""
}

# Print footer
print_footer() {
    echo ""
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}WIA - World Certification Industry Association${RESET}"
    echo -e "${GRAY}© 2025 MIT License${RESET}"
    echo ""
}

# Main installation process
main() {
    print_header

    check_prerequisites
    install_cli
    install_typescript_sdk
    install_docs
    verify_installation
    show_next_steps
    print_footer
}

# Run main installation
main
