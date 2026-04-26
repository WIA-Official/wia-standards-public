#!/bin/bash

################################################################################
# WIA-IND-029: Additive Manufacturing - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-029 standard components:
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
AMBER='\033[0;33m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${AMBER}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    🖨️  WIA-IND-029: Additive Manufacturing Installer          ║"
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

# Check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        print_warning "Running as root - installing system-wide"
        INSTALL_DIR="/usr/local/bin"
    else
        print_info "Running as user - installing to ~/.local/bin"
        INSTALL_DIR="$HOME/.local/bin"
        mkdir -p "$INSTALL_DIR"
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local CLI_SOURCE="$SCRIPT_DIR/cli/wia-ind-029.sh"
    local CLI_TARGET="$INSTALL_DIR/wia-ind-029"

    if [[ -f "$CLI_SOURCE" ]]; then
        cp "$CLI_SOURCE" "$CLI_TARGET"
        chmod +x "$CLI_TARGET"
        print_success "CLI tool installed to $CLI_TARGET"
    else
        print_error "CLI source not found: $CLI_SOURCE"
        return 1
    fi

    # Verify installation
    if command -v wia-ind-029 &> /dev/null; then
        print_success "CLI tool is accessible in PATH"
        print_info "Version: $(wia-ind-029 --version 2>&1 | head -1 || echo 'Unknown')"
    else
        print_warning "CLI tool installed but not in PATH"
        print_info "Add $INSTALL_DIR to your PATH:"
        print_info "  echo 'export PATH=\"\$PATH:$INSTALL_DIR\"' >> ~/.bashrc"
        print_info "  source ~/.bashrc"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local TS_DIR="$SCRIPT_DIR/api/typescript"

    if [[ -d "$TS_DIR" ]]; then
        cd "$TS_DIR"

        # Check for package manager
        if command -v npm &> /dev/null; then
            print_info "Installing dependencies with npm..."
            npm install
            print_success "TypeScript SDK dependencies installed"

            print_info "Building TypeScript SDK..."
            npm run build 2>/dev/null || print_warning "Build script not found (optional)"

            print_info "To use the SDK in your project:"
            print_info "  npm install @wia/ind-029"
            print_info "  or link locally:"
            print_info "  cd $TS_DIR && npm link"
        elif command -v yarn &> /dev/null; then
            print_info "Installing dependencies with yarn..."
            yarn install
            print_success "TypeScript SDK dependencies installed"

            print_info "Building TypeScript SDK..."
            yarn build 2>/dev/null || print_warning "Build script not found (optional)"
        else
            print_warning "npm/yarn not found - skipping TypeScript SDK installation"
            print_info "Install Node.js to use the TypeScript SDK"
        fi

        cd "$SCRIPT_DIR"
    else
        print_error "TypeScript SDK directory not found: $TS_DIR"
    fi
}

# Install documentation
install_documentation() {
    print_section "Installing Documentation"

    local DOC_FILES=(
        "README.md"
        "spec/WIA-IND-029-v1.0.md"
    )

    for doc in "${DOC_FILES[@]}"; do
        if [[ -f "$SCRIPT_DIR/$doc" ]]; then
            print_success "Found: $doc"
        else
            print_warning "Missing: $doc"
        fi
    done

    print_info "Documentation is available in: $SCRIPT_DIR"
}

# Create configuration directory
create_config() {
    print_section "Creating Configuration"

    local CONFIG_DIR="$HOME/.config/wia-ind-029"
    mkdir -p "$CONFIG_DIR"

    # Create default config if not exists
    if [[ ! -f "$CONFIG_DIR/config.json" ]]; then
        cat > "$CONFIG_DIR/config.json" <<EOF
{
  "version": "1.0.0",
  "printFarmId": "FARM-001",
  "apiEndpoint": "http://localhost:8080",
  "defaultPrinter": "PRINTER-001",
  "defaultMaterial": "PLA",
  "defaultProfile": "fdm-standard",
  "slicerEngine": "cura",
  "qualityInspection": true,
  "autoPostProcessing": false
}
EOF
        print_success "Created default configuration: $CONFIG_DIR/config.json"
    else
        print_info "Configuration already exists: $CONFIG_DIR/config.json"
    fi
}

# Run system checks
system_check() {
    print_section "System Check"

    # Check for required tools
    local tools=("bash" "curl" "jq")
    for tool in "${tools[@]}"; do
        if command -v "$tool" &> /dev/null; then
            print_success "$tool: $(command -v $tool)"
        else
            print_warning "$tool: not found (optional)"
        fi
    done

    # Check for Node.js
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_info "Node.js: not found (needed for TypeScript SDK)"
    fi

    # Check for Python (for slicer integration)
    if command -v python3 &> /dev/null; then
        print_success "Python: $(python3 --version)"
    else
        print_info "Python: not found (needed for some slicers)"
    fi
}

# Main installation
main() {
    print_header

    print_section "WIA-IND-029 Additive Manufacturing Standard"
    print_info "Standard ID: WIA-IND-029"
    print_info "Category: IND (Industry)"
    print_info "Version: 1.0.0"
    echo ""
    print_info "弘益人間 (Benefit All Humanity)"

    system_check
    check_root
    install_cli
    install_typescript_sdk
    install_documentation
    create_config

    print_section "Installation Complete"
    echo ""
    print_success "WIA-IND-029 installation successful!"
    echo ""
    print_info "Next steps:"
    print_info "  1. Test CLI: wia-ind-029 --help"
    print_info "  2. Read documentation: $SCRIPT_DIR/README.md"
    print_info "  3. Configure: ~/.config/wia-ind-029/config.json"
    print_info "  4. Start printing: wia-ind-029 upload --file model.stl"
    echo ""
    print_info "Resources:"
    print_info "  - Website: https://wiastandards.com"
    print_info "  - GitHub: https://github.com/WIA-Official/wia-standards"
    print_info "  - Docs: https://docs.wiastandards.com"
    echo ""
    echo -e "${AMBER}弘益人間 (홍익인간) · Benefit All Humanity${RESET}"
    echo ""
}

# Run installation
main "$@"
