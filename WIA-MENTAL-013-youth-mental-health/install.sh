#!/bin/bash
# WIA-MENTAL-013-youth-mental-health Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-013-youth-mental-health..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/youth-mental-health.sh

echo "✓ Installation complete!"
echo "Run: ./cli/youth-mental-health.sh"
