#!/bin/bash
# WIA-MENTAL-012-workplace-wellbeing Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-012-workplace-wellbeing..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/workplace-wellbeing.sh

echo "✓ Installation complete!"
echo "Run: ./cli/workplace-wellbeing.sh"
