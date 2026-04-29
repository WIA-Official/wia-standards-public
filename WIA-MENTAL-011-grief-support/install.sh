#!/bin/bash
# WIA-MENTAL-011-grief-support Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-011-grief-support..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/grief-support.sh

echo "✓ Installation complete!"
echo "Run: ./cli/grief-support.sh"
