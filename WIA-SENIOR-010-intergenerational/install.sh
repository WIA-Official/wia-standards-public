#!/bin/bash
# WIA-SENIOR-010 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-010: Intergenerational..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-010.sh
echo "Installation complete! 弘益人間"
