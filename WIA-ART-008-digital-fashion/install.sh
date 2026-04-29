#!/bin/bash
# WIA-ART-008: Digital Fashion Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WIA-ART-008 Installation"
echo "  Digital Fashion Standard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Install CLI tool
echo "📦 Installing CLI tool..."
sudo cp cli/wia-art-008.sh /usr/local/bin/wia-art-008
sudo chmod +x /usr/local/bin/wia-art-008

# Install TypeScript SDK
if command -v npm &> /dev/null; then
  echo "📚 Installing TypeScript SDK..."
  cd api/typescript
  npm install
  npm run build
  cd ../..
fi

echo ""
echo "✅ Installation complete!"
echo ""
echo "Usage:"
echo "  wia-art-008 --help"
echo ""
echo "弘益人間 (Benefit All Humanity)"
echo "© 2025 WIA - MIT License"
