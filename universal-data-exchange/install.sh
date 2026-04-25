#!/bin/bash

################################################################################
# WIA-CORE-003: Universal Data Exchange - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Core Standards Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-CORE-003 Universal Data Exchange standard
# including the TypeScript SDK and CLI tools.
################################################################################

set -e

# Colors
INDIGO='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${INSTALL_DIR:-/usr/local/bin}"

# Helper functions
print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    🔄 WIA-CORE-003: Universal Data Exchange Installer        ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${INDIGO}▶ $1${RESET}"
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

# Check if command exists
command_exists() {
    command -v "$1" &> /dev/null
}

# Check system requirements
check_requirements() {
    print_section "Checking System Requirements"

    local missing_deps=()

    # Check Node.js
    if command_exists node; then
        local node_version=$(node --version | sed 's/v//')
        print_success "Node.js: $node_version"
    else
        print_warning "Node.js: Not installed (optional for TypeScript SDK)"
        missing_deps+=("nodejs")
    fi

    # Check npm
    if command_exists npm; then
        local npm_version=$(npm --version)
        print_success "npm: $npm_version"
    else
        if command_exists node; then
            print_warning "npm: Not installed (recommended for TypeScript SDK)"
        fi
    fi

    # Check jq (required for CLI)
    if command_exists jq; then
        local jq_version=$(jq --version | sed 's/jq-//')
        print_success "jq: $jq_version"
    else
        print_error "jq: Not installed (required for CLI)"
        missing_deps+=("jq")
    fi

    # Check bc (optional for CLI)
    if command_exists bc; then
        print_success "bc: Installed"
    else
        print_warning "bc: Not installed (optional for CLI calculations)"
    fi

    # Check git
    if command_exists git; then
        local git_version=$(git --version | awk '{print $3}')
        print_success "git: $git_version"
    else
        print_warning "git: Not installed (optional)"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        echo ""
        print_warning "Missing required dependencies: ${missing_deps[*]}"
        print_info "Install with:"
        print_info "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        print_info "  macOS: brew install ${missing_deps[*]}"
        print_info "  CentOS/RHEL: sudo yum install ${missing_deps[*]}"
        echo ""
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    if [ ! -d "$SCRIPT_DIR/api/typescript" ]; then
        print_error "TypeScript SDK directory not found"
        return 1
    fi

    cd "$SCRIPT_DIR/api/typescript"

    if command_exists npm; then
        print_info "Installing dependencies..."
        npm install

        print_info "Building SDK..."
        npm run build

        print_success "TypeScript SDK installed successfully"
        print_info "Install globally with: npm install -g"
        print_info "Or add to project: npm install @wia/core-003"
    else
        print_warning "npm not available, skipping TypeScript SDK installation"
    fi

    cd "$SCRIPT_DIR"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_script="$SCRIPT_DIR/cli/wia-core-003.sh"

    if [ ! -f "$cli_script" ]; then
        print_error "CLI script not found: $cli_script"
        return 1
    fi

    # Make executable
    chmod +x "$cli_script"
    print_success "CLI script is executable"

    # Create symlink
    if [ -w "$INSTALL_DIR" ]; then
        ln -sf "$cli_script" "$INSTALL_DIR/wia-core-003"
        print_success "CLI installed to: $INSTALL_DIR/wia-core-003"
    else
        print_warning "Cannot write to $INSTALL_DIR (need sudo)"
        print_info "Run manually with sudo to install globally:"
        print_info "  sudo ln -sf $cli_script $INSTALL_DIR/wia-core-003"
        print_info ""
        print_info "Or use directly from: $cli_script"
    fi
}

# Verify installation
verify_installation() {
    print_section "Verifying Installation"

    local success=true

    # Check CLI
    if command_exists wia-core-003; then
        local cli_version=$(wia-core-003 --version 2>&1 | head -1)
        print_success "CLI: $cli_version"
    elif [ -x "$SCRIPT_DIR/cli/wia-core-003.sh" ]; then
        print_warning "CLI: Installed locally (not in PATH)"
        print_info "Add to PATH: export PATH=\"$SCRIPT_DIR/cli:\$PATH\""
    else
        print_error "CLI: Not installed"
        success=false
    fi

    # Check TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK: Built successfully"
    else
        print_warning "TypeScript SDK: Not built"
    fi

    if [ "$success" = true ]; then
        return 0
    else
        return 1
    fi
}

# Show usage examples
show_examples() {
    print_section "Usage Examples"

    echo "TypeScript SDK:"
    echo ""
    cat << 'EOF'
  import { UniversalDataExchange } from '@wia/core-003';

  const ude = new UniversalDataExchange();

  const envelope = ude.createEnvelope({
    schema: 'https://schema.org/Person',
    data: {
      name: 'John Doe',
      email: 'john@example.com'
    }
  });

  console.log('Envelope ID:', envelope.meta.id);
EOF

    echo ""
    echo "CLI Tool:"
    echo ""
    echo "  # Create envelope"
    echo "  wia-core-003 create https://schema.org/Person data.json envelope.json"
    echo ""
    echo "  # Validate envelope"
    echo "  wia-core-003 validate envelope.json"
    echo ""
    echo "  # Verify integrity"
    echo "  wia-core-003 verify envelope.json"
    echo ""
    echo "  # Check compatibility"
    echo "  wia-core-003 compatibility 1.0.0 2.0.0"
    echo ""
}

# Display installation summary
show_summary() {
    print_section "Installation Complete"

    echo "🎉 WIA-CORE-003 Universal Data Exchange has been installed!"
    echo ""
    print_info "Standard ID: WIA-CORE-003"
    print_info "Version: $VERSION"
    print_info "Category: CORE (Universal Integration Standards)"
    print_info "Color: Indigo (#6366F1)"
    echo ""
    print_info "Installation directory: $SCRIPT_DIR"
    echo ""
    print_info "Documentation:"
    print_info "  - README: $SCRIPT_DIR/README.md"
    print_info "  - Specification: $SCRIPT_DIR/spec/WIA-CORE-003-v1.0.md"
    print_info "  - TypeScript SDK: $SCRIPT_DIR/api/typescript/"
    print_info "  - CLI Tool: $SCRIPT_DIR/cli/wia-core-003.sh"
    echo ""
    print_info "Next steps:"
    print_info "  1. Read the specification: $SCRIPT_DIR/spec/WIA-CORE-003-v1.0.md"
    print_info "  2. Try the CLI: wia-core-003 --help"
    print_info "  3. Install TypeScript SDK: npm install @wia/core-003"
    print_info "  4. Get certified: https://cert.wiastandards.com"
    echo ""
    print_info "弘益人間 (Benefit All Humanity)"
    print_info "WIA - World Certification Industry Association"
    echo ""
}

# Uninstall function
uninstall() {
    print_section "Uninstalling WIA-CORE-003"

    # Remove symlink
    if [ -L "$INSTALL_DIR/wia-core-003" ]; then
        if [ -w "$INSTALL_DIR" ]; then
            rm -f "$INSTALL_DIR/wia-core-003"
            print_success "Removed CLI from $INSTALL_DIR"
        else
            print_warning "Need sudo to remove from $INSTALL_DIR"
            print_info "Run: sudo rm -f $INSTALL_DIR/wia-core-003"
        fi
    fi

    # Clean TypeScript build
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        rm -rf "$SCRIPT_DIR/api/typescript/dist"
        print_success "Removed TypeScript build artifacts"
    fi

    if [ -d "$SCRIPT_DIR/api/typescript/node_modules" ]; then
        rm -rf "$SCRIPT_DIR/api/typescript/node_modules"
        print_success "Removed TypeScript dependencies"
    fi

    print_success "Uninstall complete"
}

# Main installation flow
main() {
    print_header

    # Parse arguments
    case "${1:-}" in
        --uninstall)
            uninstall
            exit 0
            ;;
        --help)
            echo "Usage: ./install.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --uninstall    Uninstall WIA-CORE-003"
            echo "  --help         Show this help message"
            echo ""
            exit 0
            ;;
    esac

    # Run installation steps
    check_requirements
    install_typescript_sdk
    install_cli

    echo ""

    if verify_installation; then
        show_examples
        show_summary
        exit 0
    else
        print_error "Installation completed with warnings"
        print_info "Some components may not be fully functional"
        exit 1
    fi
}

# Run main
main "$@"
