#!/bin/bash
# WIA-SENIOR-002 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-002: Dementia Care..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-002.sh
echo "Installation complete! 弘益人間"
