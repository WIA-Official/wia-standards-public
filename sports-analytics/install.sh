#!/usr/bin/env bash

################################################################################
# WIA-IND-013: Sports Analytics Installation Script
#
# Standard: WIA-IND-013 v1.0
# Category: IND / Industry
# Purpose: Install and configure Sports Analytics tools
#
# 弘益人間 (Benefit All Humanity)
# Making sports analytics accessible to all levels of competition
#
# © 2025 SmileStory Inc. / WIA
# MIT License
################################################################################

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
STANDARD="WIA-IND-013"
VERSION="1.0.0"
INSTALL_DIR="${HOME}/.wia/sports-analytics"
BIN_DIR="${HOME}/.local/bin"

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}📊 WIA-IND-013: Sports Analytics Installer${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}Version: ${VERSION}${NC}"
    echo -e "${CYAN}弘익인간 (Benefit All Humanity)${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_step() {
    echo ""
    echo -e "${CYAN}▶ $1${NC}"
}

################################################################################
# Dependency Checks
################################################################################

check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for bc (calculator)
    if ! command -v bc &> /dev/null; then
        missing_deps+=("bc")
    fi

    # Optional: Node.js for TypeScript SDK
    if command -v node &> /dev/null; then
        local node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        print_success "Node.js v$(node --version) found"
        if [ "$node_version" -ge 16 ]; then
            print_success "Node.js version is compatible (>=16)"
        else
            print_warning "Node.js version is old. Recommended: >=16"
        fi
    else
        print_warning "Node.js not found (optional for TypeScript SDK)"
    fi

    # Check for npm
    if command -v npm &> /dev/null; then
        print_success "npm $(npm --version) found"
    else
        print_warning "npm not found (optional for TypeScript SDK)"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install missing dependencies:"
        echo "  Ubuntu/Debian: sudo apt-get install ${missing_deps[*]}"
        echo "  macOS:         brew install ${missing_deps[*]}"
        echo "  Fedora:        sudo dnf install ${missing_deps[*]}"
        exit 1
    fi

    print_success "All required dependencies are installed"
}

################################################################################
# Directory Setup
################################################################################

setup_directories() {
    print_step "Setting up directories..."

    # Create installation directory
    if [ ! -d "$INSTALL_DIR" ]; then
        mkdir -p "$INSTALL_DIR"
        print_success "Created installation directory: $INSTALL_DIR"
    else
        print_info "Installation directory already exists: $INSTALL_DIR"
    fi

    # Create bin directory
    if [ ! -d "$BIN_DIR" ]; then
        mkdir -p "$BIN_DIR"
        print_success "Created bin directory: $BIN_DIR"
    else
        print_info "Bin directory already exists: $BIN_DIR"
    fi

    # Create data directories
    mkdir -p "$INSTALL_DIR/data"
    mkdir -p "$INSTALL_DIR/cache"
    mkdir -p "$INSTALL_DIR/logs"

    print_success "Directory structure created"
}

################################################################################
# Install CLI Tool
################################################################################

install_cli() {
    print_step "Installing CLI tool..."

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local cli_source="$script_dir/cli/wia-ind-013.sh"
    local cli_target="$BIN_DIR/wia-ind-013"

    if [ ! -f "$cli_source" ]; then
        print_error "CLI source file not found: $cli_source"
        exit 1
    fi

    # Copy CLI script
    cp "$cli_source" "$cli_target"
    chmod +x "$cli_target"

    print_success "CLI tool installed: $cli_target"

    # Check if BIN_DIR is in PATH
    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        print_warning "$BIN_DIR is not in your PATH"
        echo ""
        echo "Add the following line to your ~/.bashrc or ~/.zshrc:"
        echo "  export PATH=\"\$PATH:$BIN_DIR\""
        echo ""
    else
        print_success "$BIN_DIR is in your PATH"
    fi
}

################################################################################
# Install TypeScript SDK
################################################################################

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        return
    fi

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local sdk_dir="$script_dir/api/typescript"

    if [ ! -d "$sdk_dir" ]; then
        print_error "TypeScript SDK directory not found: $sdk_dir"
        return
    fi

    cd "$sdk_dir"

    # Install dependencies
    print_info "Installing npm dependencies..."
    if npm install --silent; then
        print_success "TypeScript SDK dependencies installed"
    else
        print_warning "Failed to install TypeScript SDK dependencies"
        return
    fi

    # Build SDK
    print_info "Building TypeScript SDK..."
    if npm run build --silent 2>/dev/null; then
        print_success "TypeScript SDK built successfully"
    else
        print_warning "Failed to build TypeScript SDK (tsconfig may be needed)"
    fi

    cd - > /dev/null

    print_info "To use the SDK in your project:"
    echo "  npm install @wia/ind-013"
    echo "  or locally: npm link $sdk_dir"
}

################################################################################
# Install Documentation
################################################################################

install_documentation() {
    print_step "Installing documentation..."

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Copy documentation files
    if [ -f "$script_dir/README.md" ]; then
        cp "$script_dir/README.md" "$INSTALL_DIR/"
        print_success "README.md copied"
    fi

    if [ -d "$script_dir/spec" ]; then
        cp -r "$script_dir/spec" "$INSTALL_DIR/"
        print_success "Specification copied"
    fi

    print_success "Documentation installed"
}

################################################################################
# Configuration
################################################################################

create_config() {
    print_step "Creating configuration..."

    local config_file="$INSTALL_DIR/config.json"

    if [ -f "$config_file" ]; then
        print_info "Configuration file already exists, skipping"
        return
    fi

    cat > "$config_file" << 'EOF'
{
  "version": "1.0.0",
  "standard": "WIA-IND-013",
  "sport": "football",
  "features": {
    "realTimeTracking": true,
    "videoAnalytics": false,
    "predictions": true,
    "injuryMonitoring": true,
    "scouting": true
  },
  "dataRetention": {
    "trackingData": 90,
    "videoData": 30,
    "aggregatedStats": 1825
  },
  "privacy": {
    "anonymizePublicData": true,
    "gdprCompliant": true
  }
}
EOF

    print_success "Configuration file created: $config_file"
}

################################################################################
# Post-Installation
################################################################################

post_install() {
    print_step "Running post-installation tasks..."

    # Verify installation
    if command -v wia-ind-013 &> /dev/null; then
        print_success "CLI tool is accessible"
        local version=$(wia-ind-013 --version 2>&1 | head -n1)
        print_info "$version"
    else
        print_warning "CLI tool not found in PATH. You may need to restart your shell."
    fi

    print_success "Post-installation complete"
}

################################################################################
# Display Summary
################################################################################

display_summary() {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}Installation Complete! 🎉${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${CYAN}Installed Components:${NC}"
    echo -e "  ${GREEN}✓${NC} CLI Tool: wia-ind-013"
    echo -e "  ${GREEN}✓${NC} TypeScript SDK: @wia/ind-013"
    echo -e "  ${GREEN}✓${NC} Documentation"
    echo -e "  ${GREEN}✓${NC} Configuration"
    echo ""
    echo -e "${CYAN}Installation Directory:${NC}"
    echo -e "  $INSTALL_DIR"
    echo ""
    echo -e "${CYAN}Quick Start:${NC}"
    echo -e "  ${YELLOW}wia-ind-013 --help${NC}           # Show all commands"
    echo -e "  ${YELLOW}wia-ind-013 calc-ppr --help${NC}  # Calculate player rating"
    echo -e "  ${YELLOW}wia-ind-013 calc-xg --help${NC}   # Calculate expected goals"
    echo ""
    echo -e "${CYAN}Example Usage:${NC}"
    echo -e "  ${YELLOW}wia-ind-013 calc-ppr --goals 15 --assists 10 --games 38 --pass-accuracy 87${NC}"
    echo -e "  ${YELLOW}wia-ind-013 calc-xg --distance 18 --angle 20 --pressure low${NC}"
    echo -e "  ${YELLOW}wia-ind-013 win-probability --home-score 1 --away-score 0 --minutes-left 30${NC}"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo -e "  README:  $INSTALL_DIR/README.md"
    echo -e "  Spec:    $INSTALL_DIR/spec/WIA-IND-013-v1.0.md"
    echo -e "  Website: ${BLUE}https://wiastandards.com/standards/sports-analytics${NC}"
    echo ""
    echo -e "${CYAN}弘익인간 (Benefit All Humanity)${NC}"
    echo -e "${CYAN}Making sports analytics accessible to everyone${NC}"
    echo ""
    echo -e "${CYAN}© 2025 SmileStory Inc. / WIA · MIT License${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

################################################################################
# Main Installation
################################################################################

main() {
    print_header

    check_dependencies
    setup_directories
    install_cli
    install_typescript_sdk
    install_documentation
    create_config
    post_install

    display_summary
}

# Run installation
main "$@"

################################################################################
# 弘익人間 (홍익인간) · Benefit All Humanity
#
# This installer sets up the WIA-IND-013 Sports Analytics standard,
# making advanced analytics accessible to teams at all levels.
#
# WIA - World Certification Industry Association
# © 2025 SmileStory Inc. / WIA
# MIT License
################################################################################
