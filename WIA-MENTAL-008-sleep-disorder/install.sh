#!/bin/bash
# WIA-MENTAL-008-sleep-disorder Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-008-sleep-disorder..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/sleep-disorder.sh

echo "✓ Installation complete!"
echo "Run: ./cli/sleep-disorder.sh"
