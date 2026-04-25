#!/bin/bash
# WIA-MENTAL-004-anxiety-management Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-004-anxiety-management..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/anxiety-management.sh

echo "✓ Installation complete!"
echo "Run: ./cli/anxiety-management.sh"
