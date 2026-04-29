#!/bin/bash

################################################################################
# WIA-TIME-018: Temporal Communication - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Temporal Communication Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-018 Temporal Communication standard,
# including the CLI tool, TypeScript SDK, and supporting files.
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Installation paths
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"
CONFIG_DIR="$HOME/.wia/time-018"
LIB_DIR="$HOME/.wia/time-018/lib"

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Print functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     📡 WIA-TIME-018: Temporal Communication Installer         ║"
    echo "║                      Version 1.0.0                            ║"
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

# Check dependencies
check_dependencies() {
    print_section "Checking Dependencies"

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    else
        print_success "bash: $(bash --version | head -n1)"
    fi

    # Check for bc (for calculations)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    else
        print_success "bc: installed"
    fi

    # Check for date
    if ! command -v date &> /dev/null; then
        missing_deps+=("date")
    else
        print_success "date: installed"
    fi

    # Optional: Node.js for TypeScript SDK
    if command -v node &> /dev/null; then
        print_success "node: $(node --version) (optional)"
    else
        print_warning "node: not found (TypeScript SDK will not be available)"
    fi

    # Optional: npm for TypeScript SDK
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version) (optional)"
    else
        print_warning "npm: not found (TypeScript SDK will not be available)"
    fi

    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        print_info "Please install missing dependencies and try again:"
        print_info "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        print_info "  CentOS/RHEL:   sudo yum install ${missing_deps[*]}"
        print_info "  macOS:         brew install ${missing_deps[*]}"
        exit 1
    fi

    print_success "All required dependencies are installed"
}

# Create directories
create_directories() {
    print_section "Creating Directories"

    mkdir -p "$CONFIG_DIR"
    print_success "Created: $CONFIG_DIR"

    mkdir -p "$LIB_DIR"
    print_success "Created: $LIB_DIR"

    mkdir -p "$CONFIG_DIR/messages"
    print_success "Created: $CONFIG_DIR/messages"

    mkdir -p "$CONFIG_DIR/channels"
    print_success "Created: $CONFIG_DIR/channels"

    mkdir -p "$CONFIG_DIR/timelines"
    print_success "Created: $CONFIG_DIR/timelines"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_source="$SCRIPT_DIR/cli/wia-time-018.sh"
    local cli_target="$INSTALL_DIR/wia-time-018"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI script not found: $cli_source"
        return 1
    fi

    # Check if install directory is writable
    if [ ! -w "$INSTALL_DIR" ]; then
        print_warning "Install directory not writable: $INSTALL_DIR"
        print_info "Attempting to install with sudo..."

        sudo cp "$cli_source" "$cli_target"
        sudo chmod +x "$cli_target"
    else
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
    fi

    print_success "Installed CLI tool: $cli_target"

    # Verify installation
    if command -v wia-time-018 &> /dev/null; then
        print_success "CLI tool is in PATH and executable"
    else
        print_warning "CLI tool installed but not in PATH"
        print_info "Add $INSTALL_DIR to your PATH:"
        print_info "  export PATH=\"\$PATH:$INSTALL_DIR\""
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_warning "TypeScript SDK directory not found: $ts_dir"
        return 0
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing npm dependencies..."
    npm install --silent

    print_info "Building TypeScript SDK..."
    npm run build --silent 2>&1 | grep -v "npm WARN" || true

    print_success "TypeScript SDK installed"
    print_info "To use in your project:"
    print_info "  npm install $ts_dir"
    print_info "or"
    print_info "  npm install @wia/time-018"

    cd "$SCRIPT_DIR"
}

# Create configuration files
create_config() {
    print_section "Creating Configuration"

    local config_file="$CONFIG_DIR/config.json"

    if [ -f "$config_file" ]; then
        print_warning "Configuration already exists: $config_file"
        return 0
    fi

    cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "timelineId": "main",
  "quantumEntanglement": true,
  "encryption": "temporal-aes-256",
  "securityLevel": 3,
  "novikovChecking": true,
  "routingOptimization": "balanced",
  "rateLimit": 1000,
  "maxMessageSize": 10485760,
  "autoAck": true,
  "endpoints": {
    "api": "https://api.wiastandards.com/time-018",
    "quantum": "https://quantum.wiastandards.com",
    "wormhole": "https://wormhole.wiastandards.com"
  }
}
EOF

    print_success "Created configuration: $config_file"
}

# Create sample files
create_samples() {
    print_section "Creating Sample Files"

    local samples_dir="$CONFIG_DIR/samples"
    mkdir -p "$samples_dir"

    # Sample message
    cat > "$samples_dir/sample-message.json" << 'EOF'
{
  "content": "Hello from 2024! This is a test message sent to the future.",
  "target": {
    "time": "2050-01-01T00:00:00Z",
    "timeline": "main"
  },
  "options": {
    "priority": "medium",
    "requireAck": true,
    "encryption": 3
  }
}
EOF

    print_success "Created: $samples_dir/sample-message.json"

    # Sample script
    cat > "$samples_dir/send-future-message.sh" << 'EOF'
#!/bin/bash
# Sample script to send message to future

wia-time-018 send \
  --target "2050-01-01" \
  --message "Greetings from the past! Hope the future is bright." \
  --priority high \
  --require-ack \
  --timeline main

echo "Message sent to future!"
EOF

    chmod +x "$samples_dir/send-future-message.sh"
    print_success "Created: $samples_dir/send-future-message.sh"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    print_info "Testing CLI tool..."

    # Test version command
    if wia-time-018 version &> /dev/null; then
        print_success "CLI version command works"
    else
        print_error "CLI version command failed"
        return 1
    fi

    # Test help command
    if wia-time-018 help &> /dev/null; then
        print_success "CLI help command works"
    else
        print_error "CLI help command failed"
        return 1
    fi

    print_success "All tests passed"
}

# Print installation summary
print_summary() {
    print_section "Installation Summary"

    echo -e "${VIOLET}Installation Complete!${RESET}"
    echo ""
    echo -e "${GREEN}✓${RESET} CLI Tool:        wia-time-018"
    echo -e "${GREEN}✓${RESET} Configuration:   $CONFIG_DIR"
    echo -e "${GREEN}✓${RESET} Documentation:   $SCRIPT_DIR/spec/"

    if command -v npm &> /dev/null && [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        echo -e "${GREEN}✓${RESET} TypeScript SDK:  $SCRIPT_DIR/api/typescript"
    else
        echo -e "${YELLOW}⚠${RESET} TypeScript SDK:  Not installed (npm required)"
    fi

    echo ""
    print_info "Quick Start:"
    echo ""
    print_info "1. Send a message to the future:"
    print_info "   wia-time-018 send --target \"2050-01-01\" --message \"Hello!\""
    echo ""
    print_info "2. Create a quantum channel:"
    print_info "   wia-time-018 create-channel --type quantum \\"
    print_info "     --from \"2024-01-01\" --to \"2030-01-01\""
    echo ""
    print_info "3. Listen for messages:"
    print_info "   wia-time-018 listen --timeline main"
    echo ""
    print_info "4. View help:"
    print_info "   wia-time-018 help"
    echo ""
    echo -e "${VIOLET}Documentation:${RESET}"
    print_info "Specification: $SCRIPT_DIR/spec/WIA-TIME-018-v1.0.md"
    print_info "README:        $SCRIPT_DIR/README.md"
    print_info "Website:       https://wiastandards.com"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
}

# Uninstall function
uninstall() {
    print_section "Uninstalling WIA-TIME-018"

    print_warning "This will remove the CLI tool and configuration"
    read -p "Continue? (y/N) " -n 1 -r
    echo

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Uninstall cancelled"
        return 0
    fi

    # Remove CLI tool
    if [ -f "$INSTALL_DIR/wia-time-018" ]; then
        if [ -w "$INSTALL_DIR" ]; then
            rm "$INSTALL_DIR/wia-time-018"
        else
            sudo rm "$INSTALL_DIR/wia-time-018"
        fi
        print_success "Removed CLI tool"
    fi

    # Remove configuration (optional)
    read -p "Remove configuration directory? (y/N) " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$CONFIG_DIR"
        print_success "Removed configuration"
    fi

    print_success "Uninstall complete"
}

# Main installation function
main() {
    local command="${1:-install}"

    case "$command" in
        install)
            print_header
            check_dependencies
            create_directories
            install_cli
            install_typescript_sdk
            create_config
            create_samples
            run_tests
            print_summary
            ;;

        uninstall)
            print_header
            uninstall
            ;;

        test)
            print_header
            run_tests
            ;;

        --help|-h)
            print_header
            echo "USAGE:"
            echo "  ./install.sh [command]"
            echo ""
            echo "COMMANDS:"
            echo "  install     Install WIA-TIME-018 (default)"
            echo "  uninstall   Remove WIA-TIME-018"
            echo "  test        Run installation tests"
            echo "  --help      Show this help"
            echo ""
            echo "ENVIRONMENT VARIABLES:"
            echo "  INSTALL_DIR   Installation directory (default: /usr/local/bin)"
            echo ""
            echo "EXAMPLES:"
            echo "  ./install.sh"
            echo "  ./install.sh install"
            echo "  ./install.sh uninstall"
            echo "  INSTALL_DIR=~/bin ./install.sh"
            ;;

        *)
            print_error "Unknown command: $command"
            echo "Run './install.sh --help' for usage"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
