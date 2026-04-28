#!/bin/bash
# WIA-COMP-007: Virtualization - Installation Script
# 弘익人間 (Benefit All Humanity)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Installing WIA-COMP-007 Virtualization CLI..."
sudo cp "$SCRIPT_DIR/cli/wia-comp-007.sh" /usr/local/bin/wia-comp-007
echo "✓ Installation complete! Try: wia-comp-007 --help"
echo "弘益人間 (Benefit All Humanity)"
