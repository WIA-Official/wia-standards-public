#!/bin/bash

###############################################################################
# WIA-OCEAN_CONSERVATION Installation Script
# Version 1.0
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-OCEAN_CONSERVATION standard tools and SDKs
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
INSTALL_DIR="${HOME}/.wia/ocean-conservation"
BIN_DIR="${HOME}/.local/bin"
VERSION="1.0.0"

# Functions
print_header() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║   WIA-OCEAN_CONSERVATION Installation                   ║"
    echo "║   弘益人間 - Benefit All Humanity                        ║"
    echo "║   Version ${VERSION}                                       ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# Check prerequisites
check_prerequisites() {
    print_info "Checking prerequisites..."

    # Check for curl
    if ! command -v curl &> /dev/null; then
        print_error "curl is not installed. Please install curl and try again."
        exit 1
    fi
    print_success "curl found"

    # Check for jq (optional but recommended)
    if ! command -v jq &> /dev/null; then
        print_warning "jq is not installed. JSON output will not be formatted."
        print_info "Install jq for better CLI experience: sudo apt install jq (Ubuntu/Debian)"
    else
        print_success "jq found"
    fi

    # Check for Node.js (for TypeScript SDK)
    if command -v node &> /dev/null; then
        NODE_VERSION=$(node -v)
        print_success "Node.js found: $NODE_VERSION"
    else
        print_warning "Node.js not found. TypeScript SDK will not be installed."
    fi
}

# Create installation directory
create_directories() {
    print_info "Creating installation directories..."

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$BIN_DIR"
    mkdir -p "$INSTALL_DIR/spec"
    mkdir -p "$INSTALL_DIR/api"
    mkdir -p "$INSTALL_DIR/ebook"

    print_success "Directories created"
}

# Install CLI tool
install_cli() {
    print_info "Installing CLI tool..."

    # Get the directory of this script
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Copy CLI script
    if [ -f "$SCRIPT_DIR/cli/wia-ocean-conservation.sh" ]; then
        cp "$SCRIPT_DIR/cli/wia-ocean-conservation.sh" "$BIN_DIR/wia-ocean-conservation"
        chmod +x "$BIN_DIR/wia-ocean-conservation"
        print_success "CLI tool installed to $BIN_DIR/wia-ocean-conservation"
    else
        print_error "CLI tool not found at $SCRIPT_DIR/cli/wia-ocean-conservation.sh"
        exit 1
    fi
}

# Install specifications
install_specs() {
    print_info "Installing specifications..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec"/* "$INSTALL_DIR/spec/"
        print_success "Specifications installed"
    else
        print_warning "Specifications directory not found"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        return
    fi

    print_info "Installing TypeScript SDK..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cd "$SCRIPT_DIR/api/typescript"

        # Install dependencies
        npm install

        # Build SDK
        npm run build

        print_success "TypeScript SDK installed"
        print_info "To use the SDK globally: cd $SCRIPT_DIR/api/typescript && npm link"
    else
        print_warning "TypeScript SDK directory not found"
    fi
}

# Install ebook
install_ebook() {
    print_info "Installing ebook documentation..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if [ -d "$SCRIPT_DIR/ebook" ]; then
        cp -r "$SCRIPT_DIR/ebook"/* "$INSTALL_DIR/ebook/"
        print_success "Ebook documentation installed"
        print_info "View ebook: file://$INSTALL_DIR/ebook/en/index.html"
    else
        print_warning "Ebook directory not found"
    fi
}

# Configure PATH
configure_path() {
    print_info "Configuring PATH..."

    # Detect shell
    SHELL_RC=""
    if [ -n "$BASH_VERSION" ]; then
        SHELL_RC="$HOME/.bashrc"
    elif [ -n "$ZSH_VERSION" ]; then
        SHELL_RC="$HOME/.zshrc"
    else
        print_warning "Unknown shell. Please add $BIN_DIR to your PATH manually."
        return
    fi

    # Check if BIN_DIR is already in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        echo "" >> "$SHELL_RC"
        echo "# WIA Ocean Conservation CLI" >> "$SHELL_RC"
        echo "export PATH=\"\$PATH:$BIN_DIR\"" >> "$SHELL_RC"
        print_success "PATH configured in $SHELL_RC"
        print_warning "Please restart your shell or run: source $SHELL_RC"
    else
        print_success "PATH already configured"
    fi
}

# Create API key configuration
configure_api_key() {
    print_info "API Key Configuration"
    echo ""
    echo "To use the WIA-OCEAN_CONSERVATION CLI, you need an API key."
    echo "Visit https://wia.org/api-keys to obtain your key."
    echo ""
    read -p "Enter your API key (or press Enter to skip): " API_KEY

    if [ -n "$API_KEY" ]; then
        echo "export WIA_API_KEY=\"$API_KEY\"" >> "$HOME/.wia_env"

        # Add to shell RC
        SHELL_RC=""
        if [ -n "$BASH_VERSION" ]; then
            SHELL_RC="$HOME/.bashrc"
        elif [ -n "$ZSH_VERSION" ]; then
            SHELL_RC="$HOME/.zshrc"
        fi

        if [ -n "$SHELL_RC" ]; then
            echo "" >> "$SHELL_RC"
            echo "# WIA API Key" >> "$SHELL_RC"
            echo "source $HOME/.wia_env" >> "$SHELL_RC"
        fi

        print_success "API key configured"
    else
        print_info "Skipped API key configuration"
        print_info "You can set it later with: export WIA_API_KEY='your-key'"
    fi
}

# Print completion message
print_completion() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║   Installation Complete!                                ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    print_info "Installation directory: $INSTALL_DIR"
    print_info "CLI installed at: $BIN_DIR/wia-ocean-conservation"
    echo ""
    print_info "Next steps:"
    echo "  1. Restart your shell or run: source ~/.bashrc (or ~/.zshrc)"
    echo "  2. Test the CLI: wia-ocean-conservation --help"
    echo "  3. Set your API key: export WIA_API_KEY='your-key'"
    echo "  4. View documentation: file://$INSTALL_DIR/ebook/en/index.html"
    echo ""
    print_info "Example commands:"
    echo "  wia-ocean-conservation area-monitor mpa-uuid-123"
    echo "  wia-ocean-conservation species-track IUCN-Chelonia-mydas"
    echo "  wia-ocean-conservation reef-status reef-uuid-456"
    echo ""
    print_success "弘益人間 - Benefit All Humanity"
    echo ""
}

# Main installation flow
main() {
    print_header

    check_prerequisites
    create_directories
    install_cli
    install_specs
    install_typescript_sdk
    install_ebook
    configure_path
    configure_api_key

    print_completion
}

# Run main function
main "$@"
