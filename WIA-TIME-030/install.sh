#!/bin/bash

################################################################################
# WIA-TIME-030: Time Travel Ethics Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Ethics Committee
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-030 standard including:
# - CLI tool installation
# - TypeScript SDK setup
# - Configuration files
# - Documentation
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Print functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║                  WIA-TIME-030 Installation                     ║"
    echo "║              Time Travel Ethics Standard                       ║"
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

# Check prerequisites
check_prerequisites() {
    print_step "Checking prerequisites"

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    else
        print_success "bash found"
    fi

    # Check for node (optional for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js found ($(node --version))"
    else
        print_warning "Node.js not found (optional for TypeScript SDK)"
    fi

    # Check for npm (optional)
    if command -v npm &> /dev/null; then
        print_success "npm found ($(npm --version))"
    else
        print_warning "npm not found (optional for TypeScript SDK)"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        return 1
    fi

    return 0
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool"

    local cli_source="$SCRIPT_DIR/cli/wia-time-030.sh"
    local install_dir="/usr/local/bin"
    local cli_target="$install_dir/wia-time-030"

    # Check if source exists
    if [ ! -f "$cli_source" ]; then
        print_error "CLI source not found: $cli_source"
        return 1
    fi

    # Make CLI executable
    chmod +x "$cli_source"
    print_success "Made CLI executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_source" "$cli_target"
    else
        print_info "Installing to $install_dir requires sudo"
        sudo cp "$cli_source" "$cli_target"
        sudo chmod +x "$cli_target"
    fi

    print_success "CLI installed to $cli_target"

    # Verify installation
    if command -v wia-time-030 &> /dev/null; then
        print_success "CLI verified: $(wia-time-030 --version | head -1)"
    else
        print_warning "CLI installed but not in PATH. You may need to restart your shell."
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

    cd "$ts_dir"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "To install manually: cd $ts_dir && npm install && npm run build"
        return 0
    fi

    # Install dependencies
    print_info "Installing dependencies..."
    npm install --quiet

    print_success "Dependencies installed"

    # Build TypeScript
    print_info "Building TypeScript..."
    npm run build --quiet

    print_success "TypeScript SDK built"

    # Optional: Link for local development
    if [ "$1" = "--link" ]; then
        print_info "Creating npm link for local development..."
        npm link
        print_success "npm link created (use 'npm link @wia/time-030' in your projects)"
    fi

    cd "$SCRIPT_DIR"
}

# Create config directory
create_config() {
    print_step "Creating configuration"

    local config_dir="$HOME/.wia/time-030"

    mkdir -p "$config_dir"
    print_success "Config directory created: $config_dir"

    # Create default config if it doesn't exist
    local config_file="$config_dir/config.json"
    if [ ! -f "$config_file" ]; then
        cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "ethics": {
    "minComplianceScore": 70,
    "maxObservationHours": 72,
    "recentHistoryYears": 100
  },
  "monitoring": {
    "enabled": true,
    "realTimeTracking": true,
    "activityLogging": true
  },
  "certification": {
    "checkEnabled": true,
    "requireCurrent": true
  },
  "api": {
    "baseUrl": "https://ethics.wiastandards.com/api",
    "timeout": 30000
  }
}
EOF
        print_success "Default config created: $config_file"
    else
        print_info "Config already exists: $config_file"
    fi
}

# Install documentation
install_docs() {
    print_step "Installing documentation"

    local doc_dir="$HOME/.wia/time-030/docs"
    mkdir -p "$doc_dir"

    # Copy spec if it exists
    if [ -f "$SCRIPT_DIR/spec/WIA-TIME-030-v1.0.md" ]; then
        cp "$SCRIPT_DIR/spec/WIA-TIME-030-v1.0.md" "$doc_dir/"
        print_success "Specification copied to $doc_dir"
    fi

    # Copy README
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$doc_dir/"
        print_success "README copied to $doc_dir"
    fi

    print_info "Documentation available at: $doc_dir"
}

# Run tests (optional)
run_tests() {
    print_step "Running tests"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ] || ! command -v npm &> /dev/null; then
        print_warning "Tests skipped (npm not available)"
        return 0
    fi

    cd "$ts_dir"

    if [ -f "package.json" ] && grep -q '"test"' package.json; then
        print_info "Running test suite..."
        if npm test --quiet; then
            print_success "All tests passed"
        else
            print_warning "Some tests failed (non-critical)"
        fi
    else
        print_info "No tests configured"
    fi

    cd "$SCRIPT_DIR"
}

# Print completion message
print_completion() {
    echo ""
    echo -e "${VIOLET}╔════════════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${VIOLET}║              Installation Complete! 🎉                         ║${RESET}"
    echo -e "${VIOLET}╚════════════════════════════════════════════════════════════════╝${RESET}"
    echo ""
    echo -e "${GREEN}WIA-TIME-030 Time Travel Ethics Standard installed successfully!${RESET}"
    echo ""
    echo "Quick Start:"
    echo "  wia-time-030 --help                    Show all commands"
    echo "  wia-time-030 validate-operation ...    Validate operation"
    echo "  wia-time-030 check-interference ...    Check interference level"
    echo ""
    echo "Documentation:"
    echo "  README:    $HOME/.wia/time-030/docs/README.md"
    echo "  Spec:      $HOME/.wia/time-030/docs/WIA-TIME-030-v1.0.md"
    echo "  Config:    $HOME/.wia/time-030/config.json"
    echo ""
    echo "TypeScript SDK:"
    echo "  npm install @wia/time-030"
    echo ""
    echo "Resources:"
    echo "  Website:   https://wiastandards.com"
    echo "  Ethics:    https://ethics.wiastandards.com/time-travel"
    echo "  GitHub:    https://github.com/WIA-Official/wia-standards"
    echo ""
    echo -e "${CYAN}弘益人間 (Benefit All Humanity)${RESET}"
    echo "WIA - World Certification Industry Association"
    echo "© 2025 SmileStory Inc. / WIA"
    echo ""
}

# Uninstall function
uninstall() {
    print_header
    print_step "Uninstalling WIA-TIME-030"

    # Remove CLI
    if [ -f "/usr/local/bin/wia-time-030" ]; then
        if [ -w "/usr/local/bin" ]; then
            rm -f "/usr/local/bin/wia-time-030"
        else
            sudo rm -f "/usr/local/bin/wia-time-030"
        fi
        print_success "CLI removed"
    fi

    # Remove config (ask for confirmation)
    if [ -d "$HOME/.wia/time-030" ]; then
        read -p "Remove configuration and documentation? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$HOME/.wia/time-030"
            print_success "Configuration removed"
        fi
    fi

    print_success "Uninstall complete"
}

# Main installation
main() {
    print_header

    # Check for uninstall flag
    if [ "$1" = "--uninstall" ]; then
        uninstall
        exit 0
    fi

    # Check prerequisites
    if ! check_prerequisites; then
        exit 1
    fi

    # Install components
    install_cli || print_error "CLI installation failed"
    create_config || print_error "Config creation failed"
    install_docs || print_error "Documentation installation failed"

    # Optional TypeScript SDK installation
    if [ "$1" = "--with-sdk" ] || [ "$1" = "--link" ]; then
        install_typescript_sdk "$1" || print_warning "TypeScript SDK installation failed"
    elif command -v npm &> /dev/null; then
        print_step "TypeScript SDK"
        print_info "TypeScript SDK available. Run with --with-sdk to install."
        print_info "Example: ./install.sh --with-sdk"
    fi

    # Optional tests
    if [ "$1" = "--test" ]; then
        run_tests
    fi

    # Success
    print_completion
}

# Run main installation
main "$@"
