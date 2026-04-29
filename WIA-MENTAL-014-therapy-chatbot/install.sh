#!/bin/bash
# WIA-MENTAL-014-therapy-chatbot Installation Script
# 弘益人間 · Benefit All Humanity

echo "🧠💜 Installing WIA-MENTAL-014-therapy-chatbot..."
echo "弘益人間 · Benefit All Humanity"

cd api/typescript
npm install 2>/dev/null || echo "npm install skipped"

chmod +x ../cli/therapy-chatbot.sh

echo "✓ Installation complete!"
echo "Run: ./cli/therapy-chatbot.sh"
