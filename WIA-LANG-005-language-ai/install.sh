#!/bin/bash

# WIA-LANG Installation Script
# 弘益人間 · Benefit All Humanity

set -e

echo "════════════════════════════════════════════"
echo "  WIA-LANG Installation"
echo "  弘益人間 · Benefit All Humanity"
echo "════════════════════════════════════════════"
echo ""

# Check system
echo "Checking system requirements..."
command -v node >/dev/null 2>&1 || { echo "Node.js is required but not installed"; exit 1; }
command -v npm >/dev/null 2>&1 || { echo "npm is required but not installed"; exit 1; }

echo "✓ Node.js: $(node --version)"
echo "✓ npm: $(npm --version)"
echo ""

# Install dependencies
echo "Installing dependencies..."
if [ -d "api/typescript" ]; then
    cd api/typescript
    npm install
    npm run build
    cd ../..
fi

echo "✓ Dependencies installed"
echo ""

# Create directories
echo "Creating directories..."
mkdir -p data/recordings
mkdir -p data/archive
mkdir -p logs

echo "✓ Directories created"
echo ""

# Set permissions
echo "Setting permissions..."
chmod +x cli/*.sh

echo "✓ Permissions set"
echo ""

# Configuration
echo "Creating configuration..."
cat > config.json << CONFIGEOF
{
  "version": "1.0.0",
  "dataDir": "./data",
  "logDir": "./logs",
  "api": {
    "endpoint": "https://api.wia-lang.org/v1",
    "timeout": 30000
  }
}
CONFIGEOF

echo "✓ Configuration created"
echo ""

echo "════════════════════════════════════════════"
echo "  Installation Complete!"
echo "════════════════════════════════════════════"
echo ""
echo "Next steps:"
echo "  1. Set your API key: export WIA_API_KEY=your-key"
echo "  2. Run CLI: ./cli/*.sh help"
echo "  3. Open simulator: Open simulator/index.html"
echo "  4. Read docs: Open ebook/en/index.html"
echo ""
echo "© 2025 SmileStory Inc. / WIA"
