#!/bin/bash

# WIA-LEG-008 Data Portability Standard - Installation Script
# Version: 1.0.0
# License: MIT

set -e

echo "==================================="
echo "WIA-LEG-008 Data Portability Standard"
echo "Installation Script v1.0.0"
echo "==================================="
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo "Warning: Running as root. Consider running as regular user."
fi

# Detect OS
OS="$(uname -s)"
case "${OS}" in
    Linux*)     MACHINE=Linux;;
    Darwin*)    MACHINE=Mac;;
    CYGWIN*)    MACHINE=Cygwin;;
    MINGW*)     MACHINE=MinGw;;
    *)          MACHINE="UNKNOWN:${OS}"
esac

echo "Detected OS: ${MACHINE}"
echo ""

# Install CLI tool
echo "[1/3] Installing CLI tool..."
CLI_SOURCE="./cli/wia-leg-008.sh"
CLI_DEST="/usr/local/bin/wia-leg-008"

if [[ -f "$CLI_SOURCE" ]]; then
    if [[ -w "/usr/local/bin" ]]; then
        cp "$CLI_SOURCE" "$CLI_DEST"
        chmod +x "$CLI_DEST"
        echo "✓ CLI tool installed to $CLI_DEST"
    else
        echo "✗ Cannot write to /usr/local/bin. Try with sudo or install manually."
        echo "  Manual install: sudo cp $CLI_SOURCE $CLI_DEST && sudo chmod +x $CLI_DEST"
    fi
else
    echo "✗ CLI source file not found: $CLI_SOURCE"
fi

# Install TypeScript SDK
echo ""
echo "[2/3] Installing TypeScript SDK..."
if [[ -d "./api/typescript" ]]; then
    cd ./api/typescript
    
    if command -v npm &> /dev/null; then
        echo "Installing dependencies..."
        npm install
        
        echo "Building SDK..."
        npm run build
        
        echo "✓ TypeScript SDK built successfully"
        echo ""
        echo "To use the SDK in your project:"
        echo "  npm install @wia/leg-008"
        echo "or link locally:"
        echo "  npm link"
    else
        echo "✗ npm not found. Please install Node.js and npm first."
    fi
    
    cd ../..
else
    echo "✗ TypeScript SDK directory not found"
fi

# Setup complete
echo ""
echo "[3/3] Installation complete!"
echo ""
echo "==================================="
echo "Quick Start:"
echo "==================================="
echo ""
echo "CLI Usage:"
echo "  wia-leg-008 --help"
echo "  wia-leg-008 export --service facebook --output export.json"
echo ""
echo "TypeScript Usage:"
echo "  import { DataPortabilitySDK } from '@wia/leg-008';"
echo "  const sdk = new DataPortabilitySDK();"
echo ""
echo "Documentation:"
echo "  Spec: ./spec/WIA-LEG-008-v1.0.md"
echo "  README: ./README.md"
echo "  Ebook: ./ebook/index.html"
echo ""
echo "Links:"
echo "  Website: https://wiastandards.com"
echo "  GitHub: https://github.com/WIA-Official/wia-standards"
echo "  Docs: https://docs.wiastandards.com/leg-008"
echo ""
echo "弘益人間 (Benefit All Humanity)"
echo "© 2025 WIA - World Certification Industry Association"
echo ""
