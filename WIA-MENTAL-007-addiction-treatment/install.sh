#!/bin/bash
# WIA-MENTAL-007-addiction-treatment Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-007-addiction-treatment..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/addiction-treatment.sh

echo "✓ Installation complete!"
echo "Run: ./cli/addiction-treatment.sh"
