#!/bin/bash
# WIA-ART-007: Creative AI Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WIA-ART-007 Installation"
echo "  Creative AI Standard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Install CLI tool
echo "📦 Installing CLI tool..."
sudo cp cli/wia-art-007.sh /usr/local/bin/wia-art-007
sudo chmod +x /usr/local/bin/wia-art-007

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
echo "  wia-art-007 --help"
echo ""
echo "弘益人間 (Benefit All Humanity)"
echo "© 2025 WIA - MIT License"
