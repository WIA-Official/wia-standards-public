#!/bin/bash

################################################################################
# WIA-CITY-011: Building Energy Management Standard - Installation Script
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This script installs dependencies and sets up the WIA-CITY-011
# Building Energy Management Standard SDK and CLI tools.
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
MAGENTA='\033[0;35m'
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
      echo "WIA-CITY-011 Installation Script"
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
  echo "║      ⚡  WIA-CITY-011: Building Energy Management Standard    ║"
  echo "║                                                                ║"
  echo "║              弘益人間 · Benefit All Humanity                   ║"
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
    CURL_VERSION=$(curl --version | head -n1)
    print_step "curl found: ${CURL_VERSION}"
  else
    print_error "curl not found. Please install curl."
    exit 1
  fi
fi

echo ""

################################################################################
# Install SDK
################################################################################

if $INSTALL_SDK; then
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo -e "${BLUE}Installing TypeScript SDK...${NC}"
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo ""

  cd "$SDK_DIR"

  print_info "Installing npm dependencies..."
  npm install

  echo ""
  print_info "Building SDK..."
  npm run build

  echo ""
  print_step "SDK installation complete!"
  echo ""
  print_info "To use the SDK in your project:"
  echo "  npm install ${SDK_DIR}"
  echo ""
  echo "  import { BuildingEnergySDK } from '@wia/city-011';"
  echo ""

  cd "$SCRIPT_DIR"
fi

################################################################################
# Install CLI
################################################################################

if $INSTALL_CLI; then
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo -e "${BLUE}Installing CLI tools...${NC}"
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo ""

  # Make CLI script executable
  chmod +x "${CLI_DIR}/building-energy-management.sh"
  print_step "CLI script made executable"

  # Create symbolic link (optional)
  if [ -w "/usr/local/bin" ]; then
    ln -sf "${CLI_DIR}/building-energy-management.sh" /usr/local/bin/wia-bems
    print_step "Symbolic link created: /usr/local/bin/wia-bems"
    echo ""
    print_info "You can now use the CLI with:"
    echo "  wia-bems dashboard --building-id BLD-SEOUL-001"
  else
    print_warning "Cannot create symbolic link in /usr/local/bin (insufficient permissions)"
    print_info "You can use the CLI with:"
    echo "  ${CLI_DIR}/building-energy-management.sh dashboard --building-id BLD-SEOUL-001"
  fi

  echo ""
  print_step "CLI installation complete!"
  echo ""
fi

################################################################################
# Configuration
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Configuration${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_info "To configure your API credentials, run:"
echo "  ${CLI_DIR}/building-energy-management.sh config"
echo ""
print_info "Or set environment variables:"
echo "  export WIA_API_KEY=your-api-key"
echo "  export WIA_API_ENDPOINT=https://api.wia.org/city-011/v1"
echo ""

################################################################################
# Documentation
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Documentation${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_info "README: ${SCRIPT_DIR}/README.md"
print_info "Specification: ${SCRIPT_DIR}/spec/WIA-CITY-011-v1.0.md"
print_info "Website: https://wia.org/standards/city-011"
echo ""

################################################################################
# Quick Start Examples
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Quick Start Examples${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if $INSTALL_CLI; then
  echo -e "${BLUE}CLI Examples:${NC}"
  echo ""
  echo "  # View real-time dashboard"
  echo "  ${CLI_DIR}/building-energy-management.sh dashboard --building-id BLD-SEOUL-001"
  echo ""
  echo "  # Check energy consumption"
  echo "  ${CLI_DIR}/building-energy-management.sh energy --building-id BLD-SEOUL-001 --type electricity"
  echo ""
  echo "  # Monitor solar PV"
  echo "  ${CLI_DIR}/building-energy-management.sh solar --pv-id PV-ROOF-001"
  echo ""
  echo "  # View carbon emissions"
  echo "  ${CLI_DIR}/building-energy-management.sh carbon --building-id BLD-SEOUL-001 --period monthly"
  echo ""
fi

if $INSTALL_SDK; then
  echo -e "${BLUE}TypeScript SDK Example:${NC}"
  echo ""
  cat << 'EOF'
  import { BuildingEnergySDK } from '@wia/city-011';

  const sdk = new BuildingEnergySDK({
    apiKey: process.env.WIA_API_KEY,
    endpoint: 'https://api.wia.org/city-011/v1'
  });

  // Get real-time dashboard
  const dashboard = await sdk.getRealtimeDashboard('BLD-SEOUL-001');
  console.log('Current demand:', dashboard.data.energy.currentDemand, 'kW');

  // Control HVAC
  await sdk.setHVACSetpoint({
    buildingId: 'BLD-SEOUL-001',
    systemId: 'HVAC-AHU-001',
    mode: 'cooling',
    setpoint: 26.0
  });
EOF
  echo ""
fi

################################################################################
# Completion
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Installation Complete! ✓${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_step "WIA-CITY-011 Building Energy Management Standard is ready to use!"
echo ""
echo -e "${BLUE}弘益人間 (홍익인간) - Benefit All Humanity${NC}"
echo ""
echo -e "${BLUE}© 2025 SmileStory Inc. / WIA${NC}"
echo -e "${BLUE}License: MIT${NC}"
echo ""
