#!/bin/bash
# WIA-MENTAL-015-mental-data-privacy Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-015-mental-data-privacy..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/mental-data-privacy.sh

echo "✓ Installation complete!"
echo "Run: ./cli/mental-data-privacy.sh"
