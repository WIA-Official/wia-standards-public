#!/bin/bash
# WIA-MENTAL-010-mindfulness-app Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-010-mindfulness-app..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/mindfulness-app.sh

echo "✓ Installation complete!"
echo "Run: ./cli/mindfulness-app.sh"
