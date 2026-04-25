#!/bin/bash

################################################################################
# WIA-TIME-005: Timeline Anchor Standard - Installation Script
#
# Version: 1.0.0
# License: MIT
# Description: Install the Timeline Anchor SDK and CLI tools
#
# 弘益人間 (Benefit All Humanity)
# © 2025 SmileStory Inc. / WIA
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Installation directories
INSTALL_DIR="${WIA_INSTALL_DIR:-$HOME/.wia}"
BIN_DIR="$INSTALL_DIR/bin"
TIME_005_DIR="$INSTALL_DIR/time-005"

################################################################################
# Helper Functions
################################################################################

log() {
    echo -e "${VIOLET}[WIA-TIME-005 INSTALL]${NC} $1"
}

success() {
    echo -e "${GREEN}✓${NC} $1"
}

warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

error() {
    echo -e "${RED}✗${NC} $1" >&2
    exit 1
}

check_command() {
    if command -v "$1" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

################################################################################
# Banner
################################################################################

show_banner() {
    cat << "EOF"

    ⚓ WIA-TIME-005 Timeline Anchor Standard

    弘益人間 - Benefit All Humanity

    Installing Timeline Anchor SDK and CLI...

EOF
}

################################################################################
# Installation Steps
################################################################################

check_dependencies() {
    log "Checking dependencies..."

    local missing_deps=()

    if ! check_command "curl"; then
        missing_deps+=("curl")
    fi

    if ! check_command "git"; then
        warn "git not found (optional but recommended)"
    fi

    if ! check_command "node"; then
        warn "Node.js not found (required for TypeScript SDK)"
    fi

    if ! check_command "npm"; then
        warn "npm not found (required for TypeScript SDK)"
    fi

    if ! check_command "jq"; then
        warn "jq not found (optional, enhances CLI output formatting)"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        error "Missing required dependencies: ${missing_deps[*]}"
    fi

    success "Dependencies check passed"
}

create_directories() {
    log "Creating installation directories..."

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$BIN_DIR"
    mkdir -p "$TIME_005_DIR"
    mkdir -p "$TIME_005_DIR/config"
    mkdir -p "$TIME_005_DIR/logs"

    success "Directories created at $INSTALL_DIR"
}

install_cli() {
    log "Installing CLI tool..."

    # Get the directory where this script is located
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Copy CLI script
    if [ -f "$script_dir/cli/wia-time-005.sh" ]; then
        cp "$script_dir/cli/wia-time-005.sh" "$BIN_DIR/wia-time-005"
        chmod +x "$BIN_DIR/wia-time-005"
        success "CLI tool installed to $BIN_DIR/wia-time-005"
    else
        error "CLI script not found at $script_dir/cli/wia-time-005.sh"
    fi
}

install_sdk() {
    log "Installing TypeScript SDK..."

    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    if [ -d "$script_dir/api/typescript" ]; then
        # Copy SDK files
        cp -r "$script_dir/api/typescript" "$TIME_005_DIR/sdk"

        # Install npm dependencies if npm is available
        if check_command "npm"; then
            log "Installing npm dependencies..."
            cd "$TIME_005_DIR/sdk"
            npm install --production 2>&1 | grep -v "npm WARN" || true
            success "SDK dependencies installed"

            # Build SDK if dev dependencies are available
            if npm run build &> /dev/null; then
                success "SDK built successfully"
            else
                warn "SDK build skipped (dev dependencies not installed)"
            fi

            cd - > /dev/null
        else
            warn "npm not available, SDK installed but dependencies not installed"
        fi

        success "TypeScript SDK installed to $TIME_005_DIR/sdk"
    else
        error "SDK source not found at $script_dir/api/typescript"
    fi
}

install_docs() {
    log "Installing documentation..."

    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    if [ -f "$script_dir/README.md" ]; then
        cp "$script_dir/README.md" "$TIME_005_DIR/README.md"
    fi

    if [ -f "$script_dir/spec/WIA-TIME-005-v1.0.md" ]; then
        mkdir -p "$TIME_005_DIR/spec"
        cp "$script_dir/spec/WIA-TIME-005-v1.0.md" "$TIME_005_DIR/spec/"
        success "Specification installed"
    fi

    success "Documentation installed"
}

setup_path() {
    log "Setting up PATH..."

    local shell_rc=""
    local shell_name
    shell_name=$(basename "$SHELL")

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
            warn "Unknown shell: $shell_name"
            return
            ;;
    esac

    if [ -f "$shell_rc" ]; then
        # Check if already in PATH
        if ! grep -q "$BIN_DIR" "$shell_rc"; then
            echo "" >> "$shell_rc"
            echo "# WIA Standards CLI tools" >> "$shell_rc"
            echo "export PATH=\"\$PATH:$BIN_DIR\"" >> "$shell_rc"
            success "Added $BIN_DIR to PATH in $shell_rc"
            warn "Please run: source $shell_rc"
        else
            success "PATH already configured"
        fi
    fi
}

create_config() {
    log "Creating default configuration..."

    local config_file="$TIME_005_DIR/config/config.json"

    cat > "$config_file" << EOF
{
  "version": "1.0.0",
  "api_url": "https://api.wiastandards.com/time-005/v1",
  "environment": "production",
  "timeout": 30000,
  "debug": false
}
EOF

    success "Configuration created at $config_file"
}

show_next_steps() {
    cat << EOF

${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}

${VIOLET}✓ Installation Complete!${NC}

${YELLOW}Next Steps:${NC}

1. ${BLUE}Set your API key:${NC}
   export WIA_API_KEY="your-api-key-here"

   Or add to your shell rc file (~/.bashrc, ~/.zshrc, etc.):
   echo 'export WIA_API_KEY="your-api-key"' >> ~/.bashrc

2. ${BLUE}Add CLI to PATH${NC} (if not already done):
   export PATH="\$PATH:$BIN_DIR"

   Or source your shell rc file:
   source ~/.bashrc  # or ~/.zshrc

3. ${BLUE}Test the CLI:${NC}
   wia-time-005 version
   wia-time-005 help

4. ${BLUE}Use the TypeScript SDK:${NC}
   cd $TIME_005_DIR/sdk
   npm link  # Make SDK available globally

   Then in your project:
   npm link @wia/time-005

${YELLOW}Example Usage:${NC}

${BLUE}CLI:${NC}
  # Create an anchor
  wia-time-005 create-anchor --name "Origin_2025" --strength 2.42

  # Monitor anchor health
  wia-time-005 monitor --anchor-id anchor_abc123

${BLUE}TypeScript:${NC}
  import { TimelineAnchorSDK } from '@wia/time-005';

  const sdk = new TimelineAnchorSDK({
    apiKey: process.env.WIA_API_KEY,
    environment: 'production'
  });

  const anchor = await sdk.createAnchor({ ... });

${YELLOW}Documentation:${NC}
  • README:          $TIME_005_DIR/README.md
  • Specification:   $TIME_005_DIR/spec/WIA-TIME-005-v1.0.md
  • Online docs:     https://wiastandards.com/time-005

${YELLOW}Support:${NC}
  • GitHub:  https://github.com/WIA-Official/wia-standards
  • Email:   support@wiastandards.com
  • Discord: https://discord.gg/wia-standards

${VIOLET}弘益人間 (Benefit All Humanity)${NC}
${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}

EOF
}

################################################################################
# Main Installation
################################################################################

main() {
    show_banner

    log "Installing WIA-TIME-005 Timeline Anchor Standard..."
    echo ""

    check_dependencies
    create_directories
    install_cli
    install_sdk
    install_docs
    setup_path
    create_config

    echo ""
    show_next_steps
}

# Run installation
main "$@"

# 弘益人間 (Benefit All Humanity)
