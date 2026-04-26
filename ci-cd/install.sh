#!/bin/bash

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${BLUE}║         WIA-COMP-012 Installer                              ║${RESET}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${RESET}"
}

install_cli() {
    echo -e "${GREEN}Installing CLI...${RESET}"
    chmod +x "$SCRIPT_DIR/cli/wia-comp-012.sh"
    if [ -w "/usr/local/bin" ]; then
        cp "$SCRIPT_DIR/cli/wia-comp-012.sh" "/usr/local/bin/wia-comp-012"
    else
        sudo cp "$SCRIPT_DIR/cli/wia-comp-012.sh" "/usr/local/bin/wia-comp-012"
    fi
    echo -e "${GREEN}✓ CLI installed${RESET}"
}

install_typescript() {
    if command -v npm &> /dev/null && [ -d "$SCRIPT_DIR/api/typescript" ]; then
        echo -e "${GREEN}Installing TypeScript SDK...${RESET}"
        cd "$SCRIPT_DIR/api/typescript"
        npm install && npm run build
        echo -e "${GREEN}✓ TypeScript SDK built${RESET}"
    fi
}

print_header
install_cli
install_typescript
echo -e "${BLUE}弘익人間 (Benefit All Humanity)${RESET}"
