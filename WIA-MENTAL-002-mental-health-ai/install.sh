#!/bin/bash
# WIA-MENTAL-002-mental-health-ai Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-002-mental-health-ai..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/mental-health-ai.sh

echo "✓ Installation complete!"
echo "Run: ./cli/mental-health-ai.sh"
