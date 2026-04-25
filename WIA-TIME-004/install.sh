#!/usr/bin/env bash

###############################################################################
# WIA-TIME-004: Temporal Coordinate System - Installation Script
#
# @version 1.0.0
# @license MIT
# @organization WIA - World Certification Industry Association
# @philosophy 弘益人間 (Hongik Ingan) - Benefit All Humanity
###############################################################################

set -euo pipefail

# Colors
readonly VIOLET='\033[0;35m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly RED='\033[0;31m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'
readonly BOLD='\033[1m'

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${INSTALL_DIR:-$HOME/.local/bin}"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${VIOLET}${BOLD}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                                                              ║"
    echo "║      WIA-TIME-004: Temporal Coordinate System               ║"
    echo "║      🗺️  Universal Spacetime Navigation                     ║"
    echo "║                                                              ║"
    echo "║      弘益人間 (Hongik Ingan)                                 ║"
    echo "║      Benefit All Humanity                                    ║"
    echo "║                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_step() {
    echo -e "\n${VIOLET}${BOLD}▸ $1${NC}"
}

###############################################################################
# Installation Steps
###############################################################################

check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for required commands
    if ! command -v node &> /dev/null; then
        missing_deps+=("node")
    fi

    if ! command -v npm &> /dev/null; then
        missing_deps+=("npm")
    fi

    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    fi

    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        print_info "Please install the missing dependencies and try again."
        exit 1
    fi

    print_success "All dependencies found"
}

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    cd "$SCRIPT_DIR/api/typescript"

    if [[ -f "package.json" ]]; then
        print_info "Installing npm packages..."
        npm install

        print_info "Building TypeScript SDK..."
        npm run build

        print_success "TypeScript SDK installed successfully"
    else
        print_warning "package.json not found, skipping npm install"
    fi

    cd "$SCRIPT_DIR"
}

install_cli_tool() {
    print_step "Installing CLI tool..."

    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"

    # Copy CLI script
    local cli_source="$SCRIPT_DIR/cli/wia-time-004.sh"
    local cli_target="$INSTALL_DIR/wia-time-004"

    if [[ -f "$cli_source" ]]; then
        cp "$cli_source" "$cli_target"
        chmod +x "$cli_target"
        print_success "CLI tool installed to: $cli_target"
    else
        print_error "CLI script not found: $cli_source"
        exit 1
    fi
}

check_path() {
    print_step "Checking PATH configuration..."

    if [[ ":$PATH:" == *":$INSTALL_DIR:"* ]]; then
        print_success "$INSTALL_DIR is in PATH"
    else
        print_warning "$INSTALL_DIR is not in PATH"
        print_info "Add the following to your shell profile (~/.bashrc, ~/.zshrc, etc.):"
        echo ""
        echo "    export PATH=\"\$PATH:$INSTALL_DIR\""
        echo ""
    fi
}

run_tests() {
    print_step "Running tests..."

    cd "$SCRIPT_DIR/api/typescript"

    if [[ -f "package.json" ]] && npm run test &> /dev/null; then
        print_success "All tests passed"
    else
        print_warning "Tests not available or failed"
    fi

    cd "$SCRIPT_DIR"
}

display_summary() {
    print_step "Installation Summary"

    echo ""
    echo -e "${BOLD}Installed Components:${NC}"
    echo "  📦 TypeScript SDK: $SCRIPT_DIR/api/typescript"
    echo "  🔧 CLI Tool: $INSTALL_DIR/wia-time-004"
    echo "  📚 Documentation: $SCRIPT_DIR/spec/WIA-TIME-004-v1.0.md"
    echo ""

    echo -e "${BOLD}Quick Start:${NC}"
    echo "  # Get current temporal coordinate"
    echo "  $ wia-time-004 current"
    echo ""
    echo "  # Calculate distance between two points"
    echo "  $ wia-time-004 distance \\"
    echo "      --from \"139.6917,35.6895,40,1735084800\" \\"
    echo "      --to \"-0.1276,51.5074,11,1735084800\""
    echo ""
    echo "  # Transform coordinate systems"
    echo "  $ wia-time-004 transform \\"
    echo "      --coord \"0,0,0,0\" \\"
    echo "      --from \"EARTH_J2000\" \\"
    echo "      --to \"GALACTIC_CENTER\""
    echo ""
    echo "  # Show help"
    echo "  $ wia-time-004 help"
    echo ""

    echo -e "${BOLD}TypeScript SDK:${NC}"
    echo "  import { createTemporalCoordinate } from '@wia/time-004';"
    echo ""
    echo "  const coord = createTemporalCoordinate({"
    echo "    x: 139.6917, y: 35.6895, z: 40,"
    echo "    t: 1735084800"
    echo "  });"
    echo ""

    echo -e "${BOLD}Documentation:${NC}"
    echo "  📖 Specification: $SCRIPT_DIR/spec/WIA-TIME-004-v1.0.md"
    echo "  🌐 Website: https://wiastandards.com/standards/WIA-TIME-004"
    echo "  💻 GitHub: https://github.com/WIA-Official/wia-standards"
    echo ""

    echo -e "${VIOLET}${BOLD}弘益人間 (Hongik Ingan) - Benefit All Humanity${NC}"
    echo -e "${BOLD}WIA - World Certification Industry Association${NC}"
    echo -e "© 2025 MIT License"
    echo ""
}

###############################################################################
# Uninstallation
###############################################################################

uninstall() {
    print_header
    print_step "Uninstalling WIA-TIME-004..."

    # Remove CLI tool
    local cli_target="$INSTALL_DIR/wia-time-004"
    if [[ -f "$cli_target" ]]; then
        rm "$cli_target"
        print_success "Removed CLI tool: $cli_target"
    fi

    # Remove node_modules (optional)
    if [[ -d "$SCRIPT_DIR/api/typescript/node_modules" ]]; then
        read -p "Remove node_modules? (y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$SCRIPT_DIR/api/typescript/node_modules"
            print_success "Removed node_modules"
        fi
    fi

    print_success "Uninstallation complete"
}

###############################################################################
# Main
###############################################################################

main() {
    case "${1:-install}" in
        install)
            print_header
            check_dependencies
            install_typescript_sdk
            install_cli_tool
            check_path
            run_tests
            display_summary
            print_success "Installation complete! 🎉"
            ;;
        uninstall)
            uninstall
            ;;
        --help|-h)
            print_header
            echo "Usage: $0 [install|uninstall]"
            echo ""
            echo "Commands:"
            echo "  install     Install WIA-TIME-004 (default)"
            echo "  uninstall   Uninstall WIA-TIME-004"
            echo ""
            echo "Environment Variables:"
            echo "  INSTALL_DIR  Installation directory (default: ~/.local/bin)"
            echo ""
            echo "Example:"
            echo "  INSTALL_DIR=/usr/local/bin $0 install"
            echo ""
            ;;
        *)
            print_error "Unknown command: $1"
            echo "Usage: $0 [install|uninstall]"
            exit 1
            ;;
    esac
}

# Run main
main "$@"
