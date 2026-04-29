#!/bin/bash
# WIA-ART-011: Virtual Exhibition Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WIA-ART-011 Installation"
echo "  Virtual Exhibition Standard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Install CLI tool
echo "📦 Installing CLI tool..."
sudo cp cli/wia-art-011.sh /usr/local/bin/wia-art-011
sudo chmod +x /usr/local/bin/wia-art-011

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
echo "  wia-art-011 --help"
echo ""
echo "弘益人間 (Benefit All Humanity)"
echo "© 2025 WIA - MIT License"
