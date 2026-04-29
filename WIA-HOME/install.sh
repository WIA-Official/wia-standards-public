#!/bin/bash

#######################################################
# WIA-HOME Installation Script
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
#######################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored message
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

print_message "$BLUE" "======================================"
print_message "$BLUE" "  WIA-HOME Installation Script"
print_message "$BLUE" "  弘益人間 (Benefit All Humanity)"
print_message "$BLUE" "======================================"
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
   print_message "$YELLOW" "Warning: Running as root. Consider running as a regular user."
fi

# Detect OS
OS="$(uname -s)"
case "${OS}" in
    Linux*)     MACHINE=Linux;;
    Darwin*)    MACHINE=Mac;;
    *)          MACHINE="UNKNOWN:${OS}"
esac

print_message "$GREEN" "Detected OS: $MACHINE"

# Check Node.js version
print_message "$BLUE" "Checking Node.js..."
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    print_message "$GREEN" "Node.js $NODE_VERSION found"

    # Extract major version number
    NODE_MAJOR=$(echo $NODE_VERSION | cut -d'.' -f1 | sed 's/v//')
    if [ "$NODE_MAJOR" -lt 18 ]; then
        print_message "$YELLOW" "Warning: Node.js 18+ recommended. Current version: $NODE_VERSION"
    fi
else
    print_message "$RED" "Error: Node.js not found"
    print_message "$YELLOW" "Please install Node.js 18+ from https://nodejs.org"
    exit 1
fi

# Check npm
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    print_message "$GREEN" "npm $NPM_VERSION found"
else
    print_message "$RED" "Error: npm not found"
    exit 1
fi

# Install TypeScript SDK
print_message "$BLUE" "\nInstalling TypeScript SDK..."
cd api/typescript

if [ -f "package.json" ]; then
    npm install
    print_message "$GREEN" "Dependencies installed successfully"

    # Build TypeScript
    print_message "$BLUE" "Building TypeScript..."
    npm run build
    print_message "$GREEN" "Build completed successfully"
else
    print_message "$RED" "Error: package.json not found"
    exit 1
fi

cd ../..

# Make CLI executable
print_message "$BLUE" "\nConfiguring CLI..."
if [ -f "cli/wia-home.sh" ]; then
    chmod +x cli/wia-home.sh
    print_message "$GREEN" "CLI made executable"
else
    print_message "$YELLOW" "Warning: CLI script not found at cli/wia-home.sh"
fi

# Add CLI to PATH (optional)
CLI_PATH="$(pwd)/cli"
print_message "$BLUE" "\nOptional: Add CLI to PATH"
print_message "$YELLOW" "Add the following line to your ~/.bashrc or ~/.zshrc:"
print_message "$NC" "export PATH=\"\$PATH:$CLI_PATH\""

# Create .env file if it doesn't exist
if [ ! -f ".env" ]; then
    print_message "$BLUE" "\nCreating .env file..."
    cat > .env << EOF
# WIA-HOME Configuration
WIA_DEFI_API_KEY=your_api_key_here
WIA_DEFI_NETWORK=mainnet
MAINNET_RPC_URL=https://eth-mainnet.g.alchemy.com/v2/YOUR_KEY
SEPOLIA_RPC_URL=https://eth-sepolia.g.alchemy.com/v2/YOUR_KEY
ARBITRUM_RPC_URL=https://arb-mainnet.g.alchemy.com/v2/YOUR_KEY
OPTIMISM_RPC_URL=https://opt-mainnet.g.alchemy.com/v2/YOUR_KEY
BASE_RPC_URL=https://base-mainnet.g.alchemy.com/v2/YOUR_KEY
EOF
    print_message "$GREEN" ".env file created"
    print_message "$YELLOW" "Please edit .env and add your API keys"
else
    print_message "$GREEN" ".env file already exists"
fi

# Test CLI
print_message "$BLUE" "\nTesting CLI..."
if ./cli/wia-home.sh --version &> /dev/null; then
    print_message "$GREEN" "CLI test passed"
else
    print_message "$YELLOW" "Warning: CLI test failed"
fi

# Summary
echo ""
print_message "$GREEN" "======================================"
print_message "$GREEN" "  Installation Complete!"
print_message "$GREEN" "======================================"
echo ""
print_message "$NC" "Quick Start:"
print_message "$NC" "  1. Edit .env and add your API keys"
print_message "$NC" "  2. Run CLI: ./cli/wia-home.sh --help"
print_message "$NC" "  3. Use SDK:"
echo ""
print_message "$NC" "     import { WIADeFiSDK } from '@wia/home-sdk';"
print_message "$NC" "     const sdk = new WIADeFiSDK({ apiKey: '...', network: 'mainnet' });"
echo ""
print_message "$NC" "Documentation:"
print_message "$NC" "  - Spec: ./spec/WIA-HOME-PHASE1.md"
print_message "$NC" "  - API:  ./api/typescript/README.md"
print_message "$NC" "  - Ebook: ./ebook/en/index.html"
echo ""
print_message "$BLUE" "弘益人間 (Hongik Ingan) - Benefit All Humanity"
echo ""
