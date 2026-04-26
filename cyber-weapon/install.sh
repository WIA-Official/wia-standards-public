#!/bin/bash

################################################################################
# WIA-DEF-004: Cyber Weapon Standard Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Defense Research Group
#
# 弘익人間 (Benefit All Humanity)
################################################################################

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
RESET='\033[0m'

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║      WIA-DEF-004: Cyber Weapon Standard Installation          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${RESET}"
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}⚠ Warning: Running as root is not recommended${RESET}"
    echo -e "${GRAY}  Consider running as a regular user with sudo when needed${RESET}"
    echo ""
fi

# Detect OS
echo -e "${CYAN}▶ Detecting operating system...${RESET}"
OS="unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    echo -e "${GREEN}✓ Linux detected${RESET}"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
    echo -e "${GREEN}✓ macOS detected${RESET}"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then
    OS="windows"
    echo -e "${GREEN}✓ Windows detected${RESET}"
else
    echo -e "${YELLOW}⚠ Unknown OS: $OSTYPE${RESET}"
fi
echo ""

# Check dependencies
echo -e "${CYAN}▶ Checking dependencies...${RESET}"

# Check for bash
if command -v bash &> /dev/null; then
    BASH_VERSION=$(bash --version | head -n1)
    echo -e "${GREEN}✓ Bash: $BASH_VERSION${RESET}"
else
    echo -e "${RED}✗ Bash not found${RESET}"
    exit 1
fi

# Check for Node.js (optional)
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    echo -e "${GREEN}✓ Node.js: $NODE_VERSION${RESET}"
    HAS_NODE=true
else
    echo -e "${YELLOW}⚠ Node.js not found (optional for TypeScript SDK)${RESET}"
    HAS_NODE=false
fi

# Check for npm (optional)
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    echo -e "${GREEN}✓ npm: $NPM_VERSION${RESET}"
    HAS_NPM=true
else
    echo -e "${YELLOW}⚠ npm not found (optional for TypeScript SDK)${RESET}"
    HAS_NPM=false
fi

echo ""

# Get installation directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "${CYAN}▶ Installation directory: ${SCRIPT_DIR}${RESET}"
echo ""

# Make CLI executable
echo -e "${CYAN}▶ Making CLI tool executable...${RESET}"
chmod +x "${SCRIPT_DIR}/cli/wia-def-004.sh"
echo -e "${GREEN}✓ CLI tool is now executable${RESET}"
echo ""

# Create symlink (optional)
echo -e "${CYAN}▶ Creating symlink...${RESET}"
INSTALL_DIR="/usr/local/bin"

if [ -w "$INSTALL_DIR" ]; then
    ln -sf "${SCRIPT_DIR}/cli/wia-def-004.sh" "${INSTALL_DIR}/wia-def-004"
    echo -e "${GREEN}✓ Symlink created: ${INSTALL_DIR}/wia-def-004${RESET}"
    echo -e "${GRAY}  You can now run 'wia-def-004' from anywhere${RESET}"
else
    echo -e "${YELLOW}⚠ Cannot write to ${INSTALL_DIR}${RESET}"
    echo -e "${GRAY}  Run with sudo to install globally, or use:${RESET}"
    echo -e "${GRAY}  ${SCRIPT_DIR}/cli/wia-def-004.sh${RESET}"
    echo ""
    echo -e "${CYAN}  To install globally, run:${RESET}"
    echo -e "${GRAY}  sudo ln -sf ${SCRIPT_DIR}/cli/wia-def-004.sh ${INSTALL_DIR}/wia-def-004${RESET}"
fi
echo ""

# Install TypeScript SDK (optional)
if [ "$HAS_NODE" = true ] && [ "$HAS_NPM" = true ]; then
    echo -e "${CYAN}▶ TypeScript SDK installation${RESET}"
    echo -e "${GRAY}  Do you want to install the TypeScript SDK? (y/N)${RESET}"
    read -r -p "  " response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        cd "${SCRIPT_DIR}/api/typescript"

        echo -e "${CYAN}  Installing dependencies...${RESET}"
        npm install
        echo -e "${GREEN}✓ Dependencies installed${RESET}"

        echo -e "${CYAN}  Building TypeScript SDK...${RESET}"
        npm run build
        echo -e "${GREEN}✓ SDK built successfully${RESET}"

        echo ""
        echo -e "${GREEN}✓ TypeScript SDK installed${RESET}"
        echo -e "${GRAY}  To use in your project: npm install ${SCRIPT_DIR}/api/typescript${RESET}"
    else
        echo -e "${GRAY}  Skipping TypeScript SDK installation${RESET}"
    fi
else
    echo -e "${YELLOW}⚠ Node.js and npm required for TypeScript SDK${RESET}"
    echo -e "${GRAY}  Install Node.js from https://nodejs.org/${RESET}"
fi
echo ""

# Verify installation
echo -e "${CYAN}▶ Verifying installation...${RESET}"

if command -v wia-def-004 &> /dev/null; then
    echo -e "${GREEN}✓ CLI tool installed successfully${RESET}"
    echo ""
    echo -e "${CYAN}  Test the installation:${RESET}"
    echo -e "${GRAY}  wia-def-004 version${RESET}"
elif [ -x "${SCRIPT_DIR}/cli/wia-def-004.sh" ]; then
    echo -e "${GREEN}✓ CLI tool is executable${RESET}"
    echo ""
    echo -e "${CYAN}  Test the installation:${RESET}"
    echo -e "${GRAY}  ${SCRIPT_DIR}/cli/wia-def-004.sh version${RESET}"
else
    echo -e "${RED}✗ Installation verification failed${RESET}"
    exit 1
fi
echo ""

# Show next steps
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${RESET}"
echo -e "${GREEN}║              Installation Complete! 🎉                          ║${RESET}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${RESET}"
echo ""
echo -e "${CYAN}Next Steps:${RESET}"
echo ""
echo -e "${GRAY}1. View help:${RESET}"
echo -e "   wia-def-004 help"
echo ""
echo -e "${GRAY}2. Analyze a threat:${RESET}"
echo -e "   wia-def-004 analyze-threat --type ransomware --severity high"
echo ""
echo -e "${GRAY}3. Classify malware:${RESET}"
echo -e "   wia-def-004 classify-malware --behavior encryption"
echo ""
echo -e "${GRAY}4. Assess vulnerability:${RESET}"
echo -e "   wia-def-004 assess-vuln --cve CVE-2024-12345 --cvss 9.8"
echo ""
echo -e "${GRAY}5. Read the documentation:${RESET}"
echo -e "   cat ${SCRIPT_DIR}/README.md"
echo ""
echo -e "${GRAY}6. Read the specification:${RESET}"
echo -e "   cat ${SCRIPT_DIR}/spec/WIA-DEF-004-v1.0.md"
echo ""

# Legal notice
echo -e "${YELLOW}╔════════════════════════════════════════════════════════════════╗${RESET}"
echo -e "${YELLOW}║                    ⚠ LEGAL NOTICE ⚠                            ║${RESET}"
echo -e "${YELLOW}╚════════════════════════════════════════════════════════════════╝${RESET}"
echo ""
echo -e "${GRAY}This tool is for DEFENSIVE CYBERSECURITY PURPOSES ONLY.${RESET}"
echo ""
echo -e "${GRAY}Prohibited uses include:${RESET}"
echo -e "${GRAY}  • Unauthorized access to computer systems${RESET}"
echo -e "${GRAY}  • Development or distribution of malware${RESET}"
echo -e "${GRAY}  • Any violation of applicable laws${RESET}"
echo ""
echo -e "${GRAY}Users must comply with all applicable laws including:${RESET}"
echo -e "${GRAY}  • Computer Fraud and Abuse Act (CFAA)${RESET}"
echo -e "${GRAY}  • Budapest Convention on Cybercrime${RESET}"
echo -e "${GRAY}  • Local and international cybersecurity laws${RESET}"
echo ""

# Philosophy
echo -e "${CYAN}弘益人間 (홍익인간) · Benefit All Humanity${RESET}"
echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
echo ""

exit 0
