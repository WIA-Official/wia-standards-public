#!/bin/bash
# WIA-MENTAL-003-depression-detection Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-003-depression-detection..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/depression-detection.sh

echo "✓ Installation complete!"
echo "Run: ./cli/depression-detection.sh"
