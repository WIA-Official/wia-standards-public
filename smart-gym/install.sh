#!/bin/bash

################################################################################
# WIA-IND-014: Smart Gym - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Fitness Technology Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-014 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
INDIGO='\033[38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║       🏋️  WIA-IND-014: Smart Gym Installer                    ║"
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

    # Check bc (for calculations in CLI)
    if command -v bc &> /dev/null; then
        print_success "bc (calculator): installed"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Install with: apt-get install bc (Debian/Ubuntu) or brew install bc (macOS)"
        all_ok=false
    fi

    # Check curl (for API calls)
    if command -v curl &> /dev/null; then
        print_success "curl: $(curl --version | head -n1)"
    else
        print_warning "curl not found (recommended for API calls)"
        print_info "Install with: apt-get install curl (Debian/Ubuntu) or brew install curl (macOS)"
    fi

    # Check jq (for JSON processing)
    if command -v jq &> /dev/null; then
        print_success "jq: $(jq --version)"
    else
        print_warning "jq not found (optional, for JSON processing)"
        print_info "Install with: apt-get install jq (Debian/Ubuntu) or brew install jq (macOS)"
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

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-ind-014.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    # Make executable
    chmod +x "$cli_path"
    print_success "Made CLI script executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-ind-014"
        print_success "Installed CLI to $install_dir/wia-ind-014"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-ind-014"
        print_success "Installed CLI to $install_dir/wia-ind-014 (with sudo)"
    fi

    # Verify installation
    if command -v wia-ind-014 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-ind-014'"
        print_info "Try: wia-ind-014 --help"
    else
        print_warning "CLI installed but not in PATH. You may need to restart your terminal."
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found at $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build 2>/dev/null || {
        print_warning "Build failed (this is normal if no build script exists yet)"
    }

    print_success "TypeScript SDK setup complete"
    print_info "To use the SDK globally: npm link"
    print_info "Or in your project: npm install $ts_dir"

    cd "$SCRIPT_DIR"
}

# Create symlinks for documentation
install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/ind-014/docs"

    # Create directory
    mkdir -p "$docs_dir"

    # Copy documentation
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "Installed README.md"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Installed specification files"
    fi

    print_info "Documentation available at: $docs_dir"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-ind-014 &> /dev/null; then
        print_info "Testing CLI tool..."
        wia-ind-014 version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
    fi

    # Test TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/node_modules" ]; then
        print_success "TypeScript SDK dependencies installed"
    fi
}

# Show post-installation message
show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}The WIA-IND-014 standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-ind-014 member checkin${RESET}       - Check in member"
    echo -e "${CYAN}  wia-ind-014 workout start${RESET}        - Start workout session"
    echo -e "${CYAN}  wia-ind-014 workout monitor${RESET}      - Monitor live workout"
    echo -e "${CYAN}  wia-ind-014 plan create${RESET}          - Generate workout plan"
    echo -e "${CYAN}  wia-ind-014 facility status${RESET}      - View facility status"
    echo -e "${CYAN}  wia-ind-014 equipment maintenance${RESET} - Check equipment"
    echo -e "${CYAN}  wia-ind-014 analytics member${RESET}     - View member analytics"
    echo -e "${CYAN}  wia-ind-014 virtual list-classes${RESET} - List virtual classes"
    echo -e "${CYAN}  wia-ind-014 calc${RESET}                 - Fitness calculations"
    echo -e "${CYAN}  wia-ind-014 help${RESET}                 - Show help message"
    echo ""
    echo "Quick start examples:"
    echo ""
    echo -e "${GRAY}  # Check in a member${RESET}"
    echo -e "${CYAN}  wia-ind-014 member checkin --id MEM-12345 --method rfid${RESET}"
    echo ""
    echo -e "${GRAY}  # Start a workout session${RESET}"
    echo -e "${CYAN}  wia-ind-014 workout start --member MEM-12345 --equipment TREAD-001 --duration 45${RESET}"
    echo ""
    echo -e "${GRAY}  # Generate an AI workout plan${RESET}"
    echo -e "${CYAN}  wia-ind-014 plan create --member MEM-12345 --goal strength --weeks 8${RESET}"
    echo ""
    echo -e "${GRAY}  # View live facility status${RESET}"
    echo -e "${CYAN}  wia-ind-014 facility status --live${RESET}"
    echo ""
    echo -e "${GRAY}  # Calculate one-rep max${RESET}"
    echo -e "${CYAN}  wia-ind-014 calc --metric one-rep-max --weight 100 --reps 8${RESET}"
    echo ""
    echo -e "${GRAY}  # Check equipment maintenance${RESET}"
    echo -e "${CYAN}  wia-ind-014 equipment maintenance --check-all${RESET}"
    echo ""
    echo -e "${GRAY}  # View member analytics${RESET}"
    echo -e "${CYAN}  wia-ind-014 analytics member --id MEM-12345 --period 30days${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { SmartGymSDK } from '@wia/ind-014';${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/ind-014/docs/${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Check if user wants to skip certain steps
    SKIP_CLI=false
    SKIP_TS=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-cli) SKIP_CLI=true; shift ;;
            --skip-typescript) SKIP_TS=true; shift ;;
            --help)
                echo "Usage: ./install.sh [options]"
                echo ""
                echo "Options:"
                echo "  --skip-cli          Skip CLI installation"
                echo "  --skip-typescript   Skip TypeScript SDK installation"
                echo "  --help              Show this help message"
                echo ""
                exit 0
                ;;
            *) shift ;;
        esac
    done

    # Run installation steps
    check_prerequisites

    if [ "$SKIP_CLI" = false ]; then
        install_cli
    else
        print_warning "Skipping CLI installation"
    fi

    if [ "$SKIP_TS" = false ]; then
        install_typescript
    else
        print_warning "Skipping TypeScript SDK installation"
    fi

    install_docs
    run_tests
    show_completion
}

# Run main
main "$@"
