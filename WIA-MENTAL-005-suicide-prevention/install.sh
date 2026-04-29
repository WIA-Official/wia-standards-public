#!/bin/bash
# WIA-MENTAL-005-suicide-prevention Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-005-suicide-prevention..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/suicide-prevention.sh

echo "✓ Installation complete!"
echo "Run: ./cli/suicide-prevention.sh"
