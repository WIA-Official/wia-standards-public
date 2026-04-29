#!/bin/bash
# WIA-SENIOR-005 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-005: Loneliness Prevention..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-005.sh
echo "Installation complete! 弘益人間"
