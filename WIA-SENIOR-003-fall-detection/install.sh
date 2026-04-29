#!/bin/bash
# WIA-SENIOR-003 Installation Script
# 弘益人間

echo "Installing WIA-SENIOR-003: Fall Detection..."
cd api/typescript && npm install && npm run build
chmod +x ../cli/wia-senior-003.sh
echo "Installation complete! 弘益人間"
