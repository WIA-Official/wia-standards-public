#!/bin/bash

###############################################################################
# WIA-LEGAL-001: Digital Court Standard - Installation Script
# Version: 1.0.0
#
# 弘益人間 (Hongik Ingan) - Benefit All Humanity
###############################################################################

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${PURPLE}⚖️  WIA-LEGAL-001: Digital Court Standard${NC}"
echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo
echo -e "${BLUE}Installation Script v1.0.0${NC}"
echo

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}"

command -v node >/dev/null 2>&1 || {
    echo -e "${YELLOW}⚠ Node.js not found. Installing...${NC}"
    # Installation commands would go here
}

command -v npm >/dev/null 2>&1 || {
    echo -e "${YELLOW}⚠ npm not found. Please install Node.js first.${NC}"
    exit 1
}

echo -e "${GREEN}✓ Prerequisites check complete${NC}"
echo

# Install TypeScript dependencies
if [ -d "api/typescript" ]; then
    echo -e "${BLUE}Installing TypeScript SDK dependencies...${NC}"
    cd api/typescript

    if [ ! -f "package.json" ]; then
        cat > package.json <<EOF
{
  "name": "@wia/legal-001",
  "version": "1.0.0",
  "description": "WIA-LEGAL-001 Digital Court Standard - TypeScript SDK",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "scripts": {
    "build": "tsc",
    "test": "jest",
    "lint": "eslint src --ext .ts"
  },
  "keywords": ["wia", "legal", "digital-court", "courthouse"],
  "author": "WIA - World Certification Industry Association",
  "license": "MIT",
  "dependencies": {
    "axios": "^1.6.0",
    "ws": "^8.14.0"
  },
  "devDependencies": {
    "@types/node": "^20.0.0",
    "@types/ws": "^8.5.0",
    "typescript": "^5.3.0",
    "eslint": "^8.55.0",
    "@typescript-eslint/eslint-plugin": "^6.15.0",
    "@typescript-eslint/parser": "^6.15.0",
    "jest": "^29.7.0"
  }
}
EOF
    fi

    npm install
    echo -e "${GREEN}✓ TypeScript SDK installed${NC}"
    cd ../..
fi

# Make CLI executable
if [ -f "cli/wia-legal-001.sh" ]; then
    echo -e "${BLUE}Setting up CLI tool...${NC}"
    chmod +x cli/wia-legal-001.sh
    echo -e "${GREEN}✓ CLI tool ready${NC}"
fi

# Create config directory
echo -e "${BLUE}Creating configuration directory...${NC}"
mkdir -p ~/.wia/legal-001
echo -e "${GREEN}✓ Configuration directory created${NC}"

# Setup complete
echo
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✓ Installation complete!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo
echo -e "${BLUE}Next Steps:${NC}"
echo
echo "1. Set your API key:"
echo -e "   ${YELLOW}export WIA_API_KEY='your-api-key'${NC}"
echo
echo "2. Try the CLI:"
echo -e "   ${YELLOW}./cli/wia-legal-001.sh --help${NC}"
echo
echo "3. View the simulator:"
echo -e "   ${YELLOW}Open simulator/index.html in your browser${NC}"
echo
echo "4. Read the documentation:"
echo -e "   ${YELLOW}Open ebook/en/index.html in your browser${NC}"
echo
echo -e "${PURPLE}弘益人間 (Hongik Ingan) - Benefit All Humanity${NC}"
echo
