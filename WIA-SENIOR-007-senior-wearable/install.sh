#!/bin/bash
# WIA-SENIOR-007 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-007: Senior Wearable..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-007.sh
echo "Installation complete! 弘益人間"
