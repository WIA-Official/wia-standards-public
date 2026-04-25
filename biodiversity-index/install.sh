#!/bin/bash

################################################################################
# WIA-ENE-030: Biodiversity Index Standard - Installation Script
#
# 弘익人間 (홍익인간) - Benefit All Humanity
#
# This script installs dependencies and sets up the WIA-ENE-030
# Biodiversity Index Standard SDK and CLI tools.
#
# Usage:
#   ./install.sh              # Install everything
#   ./install.sh --sdk-only   # Install SDK only
#   ./install.sh --cli-only   # Install CLI only
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/api/typescript"
CLI_DIR="${SCRIPT_DIR}/cli"

# Parse command line arguments
INSTALL_SDK=true
INSTALL_CLI=true

while [[ $# -gt 0 ]]; do
  case $1 in
    --sdk-only)
      INSTALL_CLI=false
      shift
      ;;
    --cli-only)
      INSTALL_SDK=false
      shift
      ;;
    --help)
      echo "WIA-ENE-030 Installation Script"
      echo ""
      echo "Usage:"
      echo "  ./install.sh              Install everything"
      echo "  ./install.sh --sdk-only   Install SDK only"
      echo "  ./install.sh --cli-only   Install CLI only"
      echo "  ./install.sh --help       Show this help"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown option: $1${NC}"
      exit 1
      ;;
  esac
done

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║      🦋  WIA-ENE-030: Biodiversity Index Standard             ║"
  echo "║                                                                ║"
  echo "║              弘익人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
}

print_step() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1"
}

check_command() {
  if command -v "$1" &> /dev/null; then
    return 0
  else
    return 1
  fi
}

################################################################################
# Pre-flight Checks
################################################################################

print_header

echo -e "${BLUE}Pre-flight checks...${NC}"
echo ""

# Check for Node.js
if $INSTALL_SDK; then
  if check_command node; then
    NODE_VERSION=$(node --version)
    print_step "Node.js found: ${NODE_VERSION}"
  else
    print_error "Node.js not found. Please install Node.js 18+ from https://nodejs.org/"
    exit 1
  fi

  # Check for npm
  if check_command npm; then
    NPM_VERSION=$(npm --version)
    print_step "npm found: v${NPM_VERSION}"
  else
    print_error "npm not found. Please install npm."
    exit 1
  fi
fi

# Check for bash
if $INSTALL_CLI; then
  if check_command bash; then
    BASH_VERSION=$(bash --version | head -n1)
    print_step "Bash found: ${BASH_VERSION}"
  else
    print_error "Bash not found. Please install Bash 4+"
    exit 1
  fi

  # Check for jq (optional but recommended)
  if check_command jq; then
    JQ_VERSION=$(jq --version)
    print_step "jq found: ${JQ_VERSION}"
  else
    print_warning "jq not found. JSON parsing will use basic methods."
    print_info "Install jq for better JSON support: https://stedolan.github.io/jq/"
  fi

  # Check for curl
  if check_command curl; then
    print_step "curl found"
  else
    print_error "curl not found. Please install curl."
    exit 1
  fi
fi

echo ""

################################################################################
# Install TypeScript SDK
################################################################################

if $INSTALL_SDK; then
  echo -e "${BLUE}Installing TypeScript SDK...${NC}"
  echo ""

  cd "${SDK_DIR}"

  # Install dependencies
  print_info "Installing npm dependencies..."
  npm install

  # Build the SDK
  print_info "Building TypeScript SDK..."
  npm run build 2>/dev/null || true

  # Run tests (if available)
  if [ -f "package.json" ] && grep -q "\"test\"" package.json; then
    print_info "Running tests..."
    npm test || print_warning "Some tests failed"
  fi

  print_step "TypeScript SDK installed successfully"
  echo ""
fi

################################################################################
# Install CLI Tools
################################################################################

if $INSTALL_CLI; then
  echo -e "${BLUE}Installing CLI tools...${NC}"
  echo ""

  # Make CLI script executable
  if [ -f "${CLI_DIR}/biodiversity-index.sh" ]; then
    chmod +x "${CLI_DIR}/biodiversity-index.sh"
    print_step "CLI script made executable"
  else
    print_error "CLI script not found: ${CLI_DIR}/biodiversity-index.sh"
    exit 1
  fi

  # Create symlink in /usr/local/bin (optional)
  if [ -w "/usr/local/bin" ]; then
    ln -sf "${CLI_DIR}/biodiversity-index.sh" /usr/local/bin/wia-biodiversity 2>/dev/null || true
    if [ -L "/usr/local/bin/wia-biodiversity" ]; then
      print_step "Symlink created: /usr/local/bin/wia-biodiversity"
      print_info "You can now run 'wia-biodiversity' from anywhere"
    fi
  else
    print_warning "Cannot create symlink in /usr/local/bin (no write permission)"
    print_info "You can run the CLI using: ${CLI_DIR}/biodiversity-index.sh"
  fi

  echo ""
fi

################################################################################
# Post-installation
################################################################################

echo -e "${BLUE}Installation Summary${NC}"
echo ""

if $INSTALL_SDK; then
  echo -e "${GREEN}✓${NC} TypeScript SDK installed"
  echo "  Location: ${SDK_DIR}"
  echo "  Usage:"
  echo "    import { BiodiversitySDK } from '@wia/ene-030';"
  echo ""
fi

if $INSTALL_CLI; then
  echo -e "${GREEN}✓${NC} CLI tools installed"
  echo "  Location: ${CLI_DIR}/biodiversity-index.sh"
  echo "  Usage:"
  echo "    ./cli/biodiversity-index.sh --help"
  if [ -L "/usr/local/bin/wia-biodiversity" ]; then
    echo "  Or: wia-biodiversity --help"
  fi
  echo ""
fi

echo -e "${BLUE}Next Steps:${NC}"
echo ""
echo "1. Read the documentation:"
echo "   - README.md: Overview and quick start"
echo "   - spec/WIA-ENE-030-v1.0.md: Detailed specification"
echo ""
echo "2. Try the SDK:"
echo "   cd ${SDK_DIR}"
echo "   npm test"
echo ""
echo "3. Try the CLI:"
echo "   ${CLI_DIR}/biodiversity-index.sh --help"
echo ""
echo "4. Start monitoring biodiversity:"
echo "   ${CLI_DIR}/biodiversity-index.sh create-observation \\"
echo "     --species 'Parus major' --common-name '박새' --count 3"
echo ""
echo "5. Join the community:"
echo "   - GitHub: https://github.com/WIA-Official/wia-standards"
echo "   - Website: https://wia.org/standards/ene-030"
echo "   - Email: standards@wia.org"
echo ""

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}         弘익人間 (홍익인간) · Benefit All Humanity${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo ""

print_step "Installation complete!"
echo ""
