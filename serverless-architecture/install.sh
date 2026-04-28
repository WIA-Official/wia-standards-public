#!/bin/bash
# WIA-COMP-008 Installation
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Installing WIA-COMP-008 Serverless CLI..."
sudo cp "$SCRIPT_DIR/cli/wia-comp-008.sh" /usr/local/bin/wia-comp-008
echo "✓ Installation complete! Try: wia-comp-008 --help"
echo "弘익人間 (Benefit All Humanity)"
