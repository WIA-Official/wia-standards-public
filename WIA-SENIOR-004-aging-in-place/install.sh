#!/bin/bash
# WIA-SENIOR-004 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-004: Aging in Place..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-004.sh
echo "Installation complete! 弘益人間"
