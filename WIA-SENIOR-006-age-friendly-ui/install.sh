#!/bin/bash
# WIA-SENIOR-006 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-006: Age-Friendly UI..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-006.sh
echo "Installation complete! 弘益人間"
