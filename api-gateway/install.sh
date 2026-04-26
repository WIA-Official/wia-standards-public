#!/bin/bash
# WIA-COMP-010 Installation
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Installing WIA-COMP-010 API Gateway CLI..."
sudo cp "$SCRIPT_DIR/cli/wia-comp-010.sh" /usr/local/bin/wia-comp-010
echo "✓ Installation complete! Try: wia-comp-010 --help"
echo "弘익인간 (Benefit All Humanity)"
