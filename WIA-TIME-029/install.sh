#!/bin/bash

###############################################################################
# WIA-TIME-029: Time Adaptation Standard - Installation Script
#
# Version: 1.0.0
# License: MIT
# Author: WIA Time Adaptation Research Group
#
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
INSTALL_DIR="${HOME}/.wia/time-029"
BIN_DIR="${HOME}/.local/bin"
VERSION="1.0.0"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo ""
    echo -e "${PURPLE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${PURPLE}║${NC}  🔄 WIA-TIME-029: Time Adaptation Standard           ${PURPLE}║${NC}"
    echo -e "${PURPLE}║${NC}     Installation Script v${VERSION}                        ${PURPLE}║${NC}"
    echo -e "${PURPLE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ Error: $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ Warning: $1${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ $1${NC}"
}

print_step() {
    echo -e "${BLUE}▶ $1${NC}"
}

###############################################################################
# Installation Functions
###############################################################################

check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=0

    # Check for bash
    if ! command -v bash &> /dev/null; then
        print_error "bash is required but not installed"
        missing_deps=1
    fi

    # Check for node (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        local node_version=$(node -v | cut -d'v' -f2 | cut -d'.' -f1)
        if [ "$node_version" -ge 16 ]; then
            print_success "Node.js $(node -v) detected"
        else
            print_warning "Node.js version 16+ recommended (found: $(node -v))"
        fi
    else
        print_warning "Node.js not found (optional, needed for TypeScript SDK)"
    fi

    # Check for npm (optional)
    if command -v npm &> /dev/null; then
        print_success "npm $(npm -v) detected"
    else
        print_warning "npm not found (optional, needed for TypeScript SDK)"
    fi

    if [ $missing_deps -eq 0 ]; then
        print_success "All required dependencies satisfied"
    else
        print_error "Missing required dependencies"
        exit 1
    fi
}

create_directories() {
    print_step "Creating installation directories..."

    # Create main installation directory
    mkdir -p "$INSTALL_DIR"
    print_success "Created $INSTALL_DIR"

    # Create bin directory if it doesn't exist
    mkdir -p "$BIN_DIR"
    print_success "Created $BIN_DIR"

    # Create subdirectories
    mkdir -p "$INSTALL_DIR/cli"
    mkdir -p "$INSTALL_DIR/spec"
    mkdir -p "$INSTALL_DIR/api/typescript"

    print_success "Directory structure created"
}

install_cli() {
    print_step "Installing CLI tool..."

    # Get script directory
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Copy CLI script
    if [ -f "$SCRIPT_DIR/cli/wia-time-029.sh" ]; then
        cp "$SCRIPT_DIR/cli/wia-time-029.sh" "$INSTALL_DIR/cli/"
        chmod +x "$INSTALL_DIR/cli/wia-time-029.sh"
        print_success "CLI tool copied to $INSTALL_DIR/cli/"
    else
        print_error "CLI tool not found at $SCRIPT_DIR/cli/wia-time-029.sh"
        exit 1
    fi

    # Create symlink in bin directory
    ln -sf "$INSTALL_DIR/cli/wia-time-029.sh" "$BIN_DIR/wia-time-029"
    print_success "Symlink created in $BIN_DIR"

    # Verify installation
    if command -v wia-time-029 &> /dev/null; then
        print_success "CLI tool installed successfully"
    else
        print_warning "CLI tool installed but not in PATH"
        print_info "Add $BIN_DIR to your PATH to use 'wia-time-029' command"
    fi
}

install_spec() {
    print_step "Installing specification documents..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Copy spec files
    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec/"* "$INSTALL_DIR/spec/"
        print_success "Specification documents installed"
    else
        print_warning "Specification directory not found"
    fi
}

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    if [ ! -d "$SCRIPT_DIR/api/typescript" ]; then
        print_warning "TypeScript SDK source not found"
        return
    fi

    # Copy TypeScript files
    cp -r "$SCRIPT_DIR/api/typescript/"* "$INSTALL_DIR/api/typescript/"
    print_success "TypeScript SDK files copied"

    # Check if npm is available
    if command -v npm &> /dev/null; then
        print_info "Building TypeScript SDK..."

        cd "$INSTALL_DIR/api/typescript"

        # Install dependencies
        if [ -f "package.json" ]; then
            print_info "Installing dependencies..."
            npm install --silent 2>&1 | grep -v "^npm WARN" || true
            print_success "Dependencies installed"

            # Build TypeScript
            if [ -f "tsconfig.json" ] || grep -q "\"build\"" package.json; then
                print_info "Building TypeScript..."
                npm run build --silent 2>&1 | grep -v "^npm WARN" || true
                print_success "TypeScript SDK built"
            fi
        fi

        cd - > /dev/null
    else
        print_warning "npm not available, skipping SDK build"
        print_info "Install Node.js and npm to build the TypeScript SDK"
    fi
}

setup_path() {
    print_step "Setting up PATH..."

    local shell_rc=""
    local shell_name=$(basename "$SHELL")

    case "$shell_name" in
        bash)
            shell_rc="$HOME/.bashrc"
            ;;
        zsh)
            shell_rc="$HOME/.zshrc"
            ;;
        fish)
            shell_rc="$HOME/.config/fish/config.fish"
            ;;
        *)
            print_warning "Unknown shell: $shell_name"
            print_info "Please manually add $BIN_DIR to your PATH"
            return
            ;;
    esac

    # Check if PATH already includes bin directory
    if echo "$PATH" | grep -q "$BIN_DIR"; then
        print_success "PATH already configured"
        return
    fi

    # Add to shell RC file
    if [ -f "$shell_rc" ]; then
        if ! grep -q "$BIN_DIR" "$shell_rc"; then
            echo "" >> "$shell_rc"
            echo "# WIA Tools" >> "$shell_rc"
            echo "export PATH=\"\$PATH:$BIN_DIR\"" >> "$shell_rc"
            print_success "Added $BIN_DIR to $shell_rc"
            print_warning "Please restart your shell or run: source $shell_rc"
        else
            print_success "PATH already configured in $shell_rc"
        fi
    else
        print_warning "Shell configuration file not found: $shell_rc"
        print_info "Please manually add $BIN_DIR to your PATH"
    fi
}

create_readme() {
    print_step "Creating installation README..."

    cat > "$INSTALL_DIR/README.txt" << 'EOF'
WIA-TIME-029: Time Adaptation Standard
=======================================

Installation completed successfully!

弘益人間 (Benefit All Humanity)

USAGE:
------
Command-line tool:
  wia-time-029 --help

TypeScript SDK:
  cd api/typescript
  npm link  # To use globally

  In your project:
  npm link @wia/time-029

DOCUMENTATION:
--------------
Specification: spec/WIA-TIME-029-v1.0.md

Online: https://wiastandards.com/standards/WIA-TIME-029

EXAMPLES:
---------
# Create adaptation program
wia-time-029 create-program --target 1920-01-01 --duration 90

# Assess culture shock risk
wia-time-029 assess --from 2025-01-01 --to 1920-01-01

# Get historical context
wia-time-029 context-brief --era 1920 --location "New York"

SUPPORT:
--------
GitHub: https://github.com/WIA-Official/wia-standards
Issues: https://github.com/WIA-Official/wia-standards/issues

LICENSE:
--------
MIT License
© 2025 SmileStory Inc. / WIA
EOF

    print_success "README created at $INSTALL_DIR/README.txt"
}

verify_installation() {
    print_step "Verifying installation..."

    local errors=0

    # Check CLI tool
    if [ -x "$INSTALL_DIR/cli/wia-time-029.sh" ]; then
        print_success "CLI tool: OK"
    else
        print_error "CLI tool: FAILED"
        errors=$((errors + 1))
    fi

    # Check symlink
    if [ -L "$BIN_DIR/wia-time-029" ]; then
        print_success "Symlink: OK"
    else
        print_warning "Symlink: Not found (may need PATH configuration)"
    fi

    # Check spec
    if [ -d "$INSTALL_DIR/spec" ] && [ "$(ls -A $INSTALL_DIR/spec)" ]; then
        print_success "Specification: OK"
    else
        print_warning "Specification: Not found"
    fi

    # Check TypeScript SDK
    if [ -d "$INSTALL_DIR/api/typescript/src" ]; then
        print_success "TypeScript SDK: OK"
    else
        print_warning "TypeScript SDK: Not found"
    fi

    if [ $errors -eq 0 ]; then
        print_success "Installation verification passed"
        return 0
    else
        print_error "Installation verification failed with $errors error(s)"
        return 1
    fi
}

show_completion_message() {
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}                 Installation Complete!                    ${GREEN}║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Installation Directory:${NC} $INSTALL_DIR"
    echo -e "${CYAN}CLI Command:${NC} wia-time-029"
    echo ""
    echo -e "${YELLOW}Quick Start:${NC}"
    echo -e "  wia-time-029 --help"
    echo -e "  wia-time-029 create-program --target 1920-01-01 --duration 90"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo -e "  Local: $INSTALL_DIR/spec/WIA-TIME-029-v1.0.md"
    echo -e "  Online: https://wiastandards.com/standards/WIA-TIME-029"
    echo ""
    echo -e "${PURPLE}弘益人間 · Benefit All Humanity${NC}"
    echo ""
}

###############################################################################
# Main Installation Process
###############################################################################

main() {
    print_header

    echo -e "${CYAN}This script will install WIA-TIME-029 Time Adaptation Standard${NC}"
    echo -e "${CYAN}Installation directory: $INSTALL_DIR${NC}"
    echo ""

    # Ask for confirmation
    read -p "Proceed with installation? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Installation cancelled"
        exit 0
    fi

    echo ""

    # Run installation steps
    check_dependencies
    create_directories
    install_cli
    install_spec
    install_typescript_sdk
    setup_path
    create_readme

    echo ""

    # Verify and complete
    if verify_installation; then
        show_completion_message
        exit 0
    else
        print_error "Installation completed with warnings"
        print_info "Please check the messages above for details"
        exit 1
    fi
}

###############################################################################
# Uninstall Function
###############################################################################

uninstall() {
    print_header
    echo -e "${YELLOW}This will remove WIA-TIME-029 from your system${NC}"
    echo -e "${YELLOW}Installation directory: $INSTALL_DIR${NC}"
    echo ""

    read -p "Are you sure you want to uninstall? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Uninstall cancelled"
        exit 0
    fi

    print_step "Removing installation directory..."
    if [ -d "$INSTALL_DIR" ]; then
        rm -rf "$INSTALL_DIR"
        print_success "Removed $INSTALL_DIR"
    fi

    print_step "Removing symlink..."
    if [ -L "$BIN_DIR/wia-time-029" ]; then
        rm "$BIN_DIR/wia-time-029"
        print_success "Removed $BIN_DIR/wia-time-029"
    fi

    print_success "Uninstall complete"
    print_info "You may want to remove the PATH entry from your shell configuration"
}

###############################################################################
# Script Entry Point
###############################################################################

if [ "$1" = "uninstall" ]; then
    uninstall
else
    main
fi
