#!/bin/bash
# WIA-SENIOR-009 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-009: Senior Mobility..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-009.sh
echo "Installation complete! 弘益人間"
