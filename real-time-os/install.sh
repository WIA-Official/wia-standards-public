#!/bin/bash
set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}Installing WIA-COMP-019...${RESET}"

# Install CLI
CLI_PATH="$SCRIPT_DIR/cli/wia-comp-019.sh"
chmod +x "$CLI_PATH"

if [ -w "/usr/local/bin" ]; then
    cp "$CLI_PATH" "/usr/local/bin/wia-comp-019"
else
    sudo cp "$CLI_PATH" "/usr/local/bin/wia-comp-019"
fi

echo -e "${GREEN}✓ CLI installed to /usr/local/bin/wia-comp-019${RESET}"

# Install TypeScript SDK if npm is available
if command -v npm &> /dev/null; then
    cd "$SCRIPT_DIR/api/typescript"
    npm install 2>/dev/null || echo "npm install skipped"
    npm run build 2>/dev/null || echo "build skipped"
    cd "$SCRIPT_DIR"
    echo -e "${GREEN}✓ TypeScript SDK built${RESET}"
fi

echo -e "${GREEN}Installation complete!${RESET}"
echo ""
echo "Try: wia-comp-019 --version"
echo ""
