#!/bin/bash
# WIA-MENTAL-006-ptsd-support Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-006-ptsd-support..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/ptsd-support.sh

echo "✓ Installation complete!"
echo "Run: ./cli/ptsd-support.sh"
