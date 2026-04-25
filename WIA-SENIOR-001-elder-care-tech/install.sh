#!/bin/bash

###############################################################################
# WIA-SENIOR-001: Elder Care Technology Standard - Installation Script
# 弘益人間 (Benefit All Humanity)
###############################################################################

set -e

# Colors
ORANGE='\033[0;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

print_header() {
    echo -e "${ORANGE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║   WIA-SENIOR-001: Elder Care Technology Standard         ║"
    echo "║                  Installation Script                      ║"
    echo "║                   弘益人間 (Benefit All)                  ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${ORANGE}ℹ $1${NC}"
}

# Check prerequisites
check_prerequisites() {
    print_info "Checking prerequisites..."

    # Check Node.js
    if command -v node &> /dev/null; then
        NODE_VERSION=$(node -v)
        print_success "Node.js found: $NODE_VERSION"
    else
        print_error "Node.js not found. Please install Node.js 16+ first."
        exit 1
    fi

    # Check npm
    if command -v npm &> /dev/null; then
        NPM_VERSION=$(npm -v)
        print_success "npm found: $NPM_VERSION"
    else
        print_error "npm not found. Please install npm first."
        exit 1
    fi

    # Check curl (for CLI)
    if command -v curl &> /dev/null; then
        print_success "curl found"
    else
        print_error "curl not found. Please install curl first."
        exit 1
    fi

    # Check jq (for CLI)
    if command -v jq &> /dev/null; then
        print_success "jq found"
    else
        print_info "jq not found. Installing jq for CLI JSON parsing..."
        if [[ "$OSTYPE" == "darwin"* ]]; then
            brew install jq
        elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
            sudo apt-get install -y jq || sudo yum install -y jq
        fi
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_info "Installing TypeScript SDK..."

    cd api/typescript

    # Install dependencies
    npm install

    # Build
    npm run build

    print_success "TypeScript SDK installed"

    cd ../..
}

# Install CLI tool
install_cli() {
    print_info "Installing CLI tool..."

    # Make CLI executable
    chmod +x cli/wia-senior-001.sh

    # Create symlink to /usr/local/bin
    if [ -w /usr/local/bin ]; then
        ln -sf "$(pwd)/cli/wia-senior-001.sh" /usr/local/bin/wia-senior-001
        print_success "CLI tool installed to /usr/local/bin/wia-senior-001"
    else
        print_info "Cannot write to /usr/local/bin. You can manually add to PATH:"
        echo "  export PATH=\"\$PATH:$(pwd)/cli\""
    fi
}

# Setup configuration
setup_config() {
    print_info "Setting up configuration..."

    # Create config directory
    CONFIG_DIR="$HOME/.wia/senior-001"
    mkdir -p "$CONFIG_DIR"

    # Create default config if not exists
    if [ ! -f "$CONFIG_DIR/config.json" ]; then
        cat > "$CONFIG_DIR/config.json" <<EOF
{
  "apiUrl": "https://api.wia.org/senior-001",
  "apiKey": "",
  "locale": "en",
  "timezone": "UTC"
}
EOF
        print_success "Configuration file created at $CONFIG_DIR/config.json"
        print_info "Please edit the config file and add your API key"
    else
        print_info "Configuration file already exists"
    fi
}

# Main installation
main() {
    print_header

    check_prerequisites
    echo ""

    install_typescript_sdk
    echo ""

    install_cli
    echo ""

    setup_config
    echo ""

    print_success "Installation complete!"
    echo ""
    echo -e "${ORANGE}Next steps:${NC}"
    echo "1. Add your API key to $HOME/.wia/senior-001/config.json"
    echo "2. Or set environment variable: export WIA_API_KEY='your-api-key'"
    echo "3. Test CLI: wia-senior-001 help"
    echo "4. Test SDK: npm test"
    echo "5. View documentation: open index.html"
    echo ""
    echo -e "${ORANGE}弘益人間 (Benefit All Humanity)${NC}"
}

main
