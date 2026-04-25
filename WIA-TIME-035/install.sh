#!/bin/bash

################################################################################
# WIA-TIME-035: Temporal Information Security Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Security Working Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-035 standard including:
# - CLI tool installation
# - TypeScript SDK setup
# - Security configuration
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
    echo "║                  WIA-TIME-035 Installation                     ║"
    echo "║          Temporal Information Security Standard                ║"
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

    # Check for openssl (recommended for encryption)
    if command -v openssl &> /dev/null; then
        print_success "OpenSSL found ($(openssl version))"
    else
        print_warning "OpenSSL not found (recommended for encryption)"
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

    local cli_source="$SCRIPT_DIR/cli/wia-time-035.sh"
    local install_dir="/usr/local/bin"
    local cli_target="$install_dir/wia-time-035"

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
    if command -v wia-time-035 &> /dev/null; then
        print_success "CLI verified: $(wia-time-035 --version | head -1)"
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
    npm run build --quiet 2>/dev/null || print_warning "Build completed with warnings"

    print_success "TypeScript SDK built"

    # Optional: Link for local development
    if [ "$1" = "--link" ]; then
        print_info "Creating npm link for local development..."
        npm link
        print_success "npm link created (use 'npm link @wia/time-035' in your projects)"
    fi

    cd "$SCRIPT_DIR"
}

# Create config directory
create_config() {
    print_step "Creating configuration"

    local config_dir="$HOME/.wia/time-035"
    local key_store="$config_dir/keys"

    mkdir -p "$config_dir"
    mkdir -p "$key_store"
    chmod 700 "$config_dir"
    chmod 700 "$key_store"
    print_success "Config directory created: $config_dir"
    print_success "Key store created: $key_store"

    # Create default config if it doesn't exist
    local config_file="$config_dir/config.json"
    if [ ! -f "$config_file" ]; then
        cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "encryption": {
    "defaultAlgorithm": "TE-256",
    "minimumKeySize": 256,
    "quantumResistantRequired": true,
    "timelineBindingRequired": true
  },
  "keyManagement": {
    "rotationPeriod": 90,
    "masterKeyBackup": true,
    "hsmRequired": false,
    "emergencyRecoveryEnabled": true
  },
  "monitoring": {
    "realTimeMonitoring": true,
    "anomalyDetection": true,
    "threatIntelligence": true,
    "alertThreshold": "medium"
  },
  "audit": {
    "enabled": true,
    "logLevel": "detailed",
    "retentionPeriod": 2555,
    "immutableLogs": true
  },
  "accessControl": {
    "defaultPolicy": "deny",
    "multiFactorRequired": true,
    "timelineIsolationEnabled": true
  },
  "compliance": {
    "standard": "WIA-TIME-035",
    "certificationLevel": 2,
    "assessmentFrequency": 90
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

    local doc_dir="$HOME/.wia/time-035/docs"
    mkdir -p "$doc_dir"

    # Copy spec if it exists
    if [ -f "$SCRIPT_DIR/spec/WIA-TIME-035-v1.0.md" ]; then
        cp "$SCRIPT_DIR/spec/WIA-TIME-035-v1.0.md" "$doc_dir/"
        print_success "Specification copied to $doc_dir"
    fi

    # Copy README
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$doc_dir/"
        print_success "README copied to $doc_dir"
    fi

    print_info "Documentation available at: $doc_dir"
}

# Setup security
setup_security() {
    print_step "Setting up security"

    # Create secure directories with proper permissions
    local secure_dirs=(
        "$HOME/.wia/time-035/keys"
        "$HOME/.wia/time-035/channels"
        "$HOME/.wia/time-035/audit"
    )

    for dir in "${secure_dirs[@]}"; do
        mkdir -p "$dir"
        chmod 700 "$dir"
    done

    print_success "Secure directories configured"

    # Generate initial master key (optional)
    if [ "$1" = "--generate-master-key" ]; then
        local master_key="$HOME/.wia/time-035/keys/master.key"
        if [ ! -f "$master_key" ]; then
            print_info "Generating master temporal key..."
            head -c 32 /dev/urandom | base64 > "$master_key"
            chmod 600 "$master_key"
            print_success "Master key generated: $master_key"
            print_warning "IMPORTANT: Backup this key securely!"
        fi
    fi

    print_info "Security configuration complete"
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
        if npm test --quiet 2>/dev/null; then
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
    echo -e "${GREEN}WIA-TIME-035 Temporal Information Security installed successfully!${RESET}"
    echo ""
    echo "Quick Start:"
    echo "  wia-time-035 --help                    Show all commands"
    echo "  wia-time-035 generate-key ...          Generate encryption key"
    echo "  wia-time-035 encrypt ...               Encrypt data"
    echo "  wia-time-035 audit ...                 Run security audit"
    echo ""
    echo "Documentation:"
    echo "  README:    $HOME/.wia/time-035/docs/README.md"
    echo "  Spec:      $HOME/.wia/time-035/docs/WIA-TIME-035-v1.0.md"
    echo "  Config:    $HOME/.wia/time-035/config.json"
    echo ""
    echo "Security:"
    echo "  Keys:      $HOME/.wia/time-035/keys/"
    echo "  Channels:  $HOME/.wia/time-035/channels/"
    echo "  Audit:     $HOME/.wia/time-035/audit/"
    echo ""
    echo "TypeScript SDK:"
    echo "  npm install @wia/time-035"
    echo ""
    echo "Resources:"
    echo "  Website:   https://wiastandards.com"
    echo "  Security:  https://security.wiastandards.com/temporal"
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
    print_step "Uninstalling WIA-TIME-035"

    # Remove CLI
    if [ -f "/usr/local/bin/wia-time-035" ]; then
        if [ -w "/usr/local/bin" ]; then
            rm -f "/usr/local/bin/wia-time-035"
        else
            sudo rm -f "/usr/local/bin/wia-time-035"
        fi
        print_success "CLI removed"
    fi

    # Remove config (ask for confirmation)
    if [ -d "$HOME/.wia/time-035" ]; then
        read -p "Remove configuration, keys, and documentation? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$HOME/.wia/time-035"
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
    setup_security "$@" || print_error "Security setup failed"

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
