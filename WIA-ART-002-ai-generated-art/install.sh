#!/bin/bash
# WIA-ART-002: AI-Generated Art Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  WIA-ART-002 Installation"
echo "  AI-Generated Art Standard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Install CLI tool
echo "📦 Installing CLI tool..."
sudo cp cli/wia-art-002.sh /usr/local/bin/wia-art-002
sudo chmod +x /usr/local/bin/wia-art-002

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
echo "  wia-art-002 --help"
echo ""
echo "弘益人間 (Benefit All Humanity)"
echo "© 2025 WIA - MIT License"
