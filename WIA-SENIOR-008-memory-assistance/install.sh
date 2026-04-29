#!/bin/bash
# WIA-SENIOR-008 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-008: Memory Assistance..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-008.sh
echo "Installation complete! 弘益人間"
