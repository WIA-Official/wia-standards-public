#!/bin/bash
# WIA-COMP-009 Installation
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Installing WIA-COMP-009 Microservices CLI..."
sudo cp "$SCRIPT_DIR/cli/wia-comp-009.sh" /usr/local/bin/wia-comp-009
echo "✓ Installation complete! Try: wia-comp-009 --help"
echo "弘익人간 (Benefit All Humanity)"
