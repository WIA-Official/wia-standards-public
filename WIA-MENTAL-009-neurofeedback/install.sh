#!/bin/bash
# WIA-MENTAL-009-neurofeedback Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-009-neurofeedback..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/neurofeedback.sh

echo "✓ Installation complete!"
echo "Run: ./cli/neurofeedback.sh"
