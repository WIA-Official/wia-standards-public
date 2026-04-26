#!/bin/bash

################################################################################
# WIA-CITY-012: Elevator System Standard - Installation Script
#
# 弘익人間 (홍익인간) - Benefit All Humanity
#
# This script installs all dependencies and sets up the WIA-CITY-012
# Elevator System Standard development environment.
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║       🛗  WIA-CITY-012: Elevator System Standard              ║"
  echo "║                  Installation Script                           ║"
  echo "║                                                                ║"
  echo "║              弘益人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
  echo ""
}

print_success() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1" >&2
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

print_step() {
  echo ""
  echo -e "${CYAN}▶ $1${NC}"
  echo "────────────────────────────────────────────────────────────────"
}

# Check if command exists
command_exists() {
  command -v "$1" &> /dev/null
}

# Check Node.js version
check_node_version() {
  local required_major=18
  local node_version=$(node -v | cut -d'v' -f2 | cut -d'.' -f1)

  if [ "$node_version" -lt "$required_major" ]; then
    print_error "Node.js $required_major or higher is required (current: v$node_version)"
    return 1
  fi

  return 0
}

################################################################################
# Installation Steps
################################################################################

# Step 1: Check Prerequisites
step_check_prerequisites() {
  print_step "Step 1: Checking Prerequisites"

  # Check for Node.js
  if ! command_exists node; then
    print_error "Node.js is not installed"
    echo ""
    echo "Please install Node.js 18+ from:"
    echo "  https://nodejs.org/"
    echo ""
    exit 1
  fi

  if ! check_node_version; then
    echo ""
    echo "Please upgrade Node.js to version 18 or higher"
    exit 1
  fi

  print_success "Node.js $(node -v) detected"

  # Check for npm
  if ! command_exists npm; then
    print_error "npm is not installed"
    exit 1
  fi

  print_success "npm $(npm -v) detected"

  # Check for curl (optional, for CLI)
  if command_exists curl; then
    print_success "curl detected (CLI tool will work)"
  else
    print_warning "curl not found (CLI tool requires curl)"
  fi

  # Check for jq (optional, for CLI)
  if command_exists jq; then
    print_success "jq detected (CLI JSON parsing enabled)"
  else
    print_warning "jq not found (CLI will use basic JSON parsing)"
  fi
}

# Step 2: Install TypeScript SDK Dependencies
step_install_typescript_sdk() {
  print_step "Step 2: Installing TypeScript SDK Dependencies"

  cd "$SCRIPT_DIR/api/typescript"

  if [ ! -f "package.json" ]; then
    print_error "package.json not found in api/typescript/"
    exit 1
  fi

  print_info "Installing npm packages..."
  npm install

  print_success "TypeScript SDK dependencies installed"
}

# Step 3: Build TypeScript SDK
step_build_typescript_sdk() {
  print_step "Step 3: Building TypeScript SDK"

  cd "$SCRIPT_DIR/api/typescript"

  print_info "Building SDK..."
  npm run build

  if [ -d "dist" ]; then
    print_success "TypeScript SDK built successfully"
  else
    print_error "Build failed - dist directory not created"
    exit 1
  fi
}

# Step 4: Set Up CLI Tool
step_setup_cli() {
  print_step "Step 4: Setting Up CLI Tool"

  cd "$SCRIPT_DIR"

  # Make CLI executable
  chmod +x cli/elevator-system.sh
  print_success "CLI tool made executable"

  # Create symlink (optional)
  local bin_dir="$HOME/.local/bin"

  if [ -d "$bin_dir" ]; then
    local symlink="$bin_dir/wia-elevator"

    if [ -L "$symlink" ]; then
      rm "$symlink"
    fi

    ln -s "$SCRIPT_DIR/cli/elevator-system.sh" "$symlink"
    print_success "Created symlink: $symlink"

    # Check if bin_dir is in PATH
    if [[ ":$PATH:" == *":$bin_dir:"* ]]; then
      print_success "$bin_dir is in PATH"
    else
      print_warning "$bin_dir is not in PATH"
      echo "           Add this line to your ~/.bashrc or ~/.zshrc:"
      echo "           export PATH=\"\$HOME/.local/bin:\$PATH\""
    fi
  else
    print_info "Skipping symlink creation ($bin_dir does not exist)"
  fi
}

# Step 5: Run Tests (optional)
step_run_tests() {
  print_step "Step 5: Running Tests (optional)"

  cd "$SCRIPT_DIR/api/typescript"

  if npm run test --if-present 2>/dev/null; then
    print_success "Tests passed"
  else
    print_warning "No tests configured or tests failed (this is optional)"
  fi
}

# Step 6: Create Configuration Directory
step_create_config() {
  print_step "Step 6: Creating Configuration Directory"

  local config_dir="$HOME/.wia"

  if [ ! -d "$config_dir" ]; then
    mkdir -p "$config_dir"
    print_success "Created configuration directory: $config_dir"
  else
    print_info "Configuration directory already exists: $config_dir"
  fi

  # Create sample config file
  local config_file="$config_dir/elevator-system.conf.sample"

  cat > "$config_file" <<EOF
# WIA-CITY-012 Elevator System CLI Configuration
# Copy this file to elevator-system.conf and edit the values

WIA_API_KEY_CONF="your-api-key-here"
WIA_API_ENDPOINT_CONF="https://api.wia.org/city-012/v1"
EOF

  print_success "Created sample config: $config_file"
}

# Step 7: Verify Installation
step_verify_installation() {
  print_step "Step 7: Verifying Installation"

  local all_good=true

  # Check TypeScript SDK build
  if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
    print_success "TypeScript SDK build verified"
  else
    print_error "TypeScript SDK build not found"
    all_good=false
  fi

  # Check CLI executable
  if [ -x "$SCRIPT_DIR/cli/elevator-system.sh" ]; then
    print_success "CLI tool is executable"
  else
    print_error "CLI tool is not executable"
    all_good=false
  fi

  # Check spec file
  if [ -f "$SCRIPT_DIR/spec/WIA-CITY-012-v1.0.md" ]; then
    print_success "Specification document found"
  else
    print_warning "Specification document not found"
  fi

  if [ "$all_good" = true ]; then
    echo ""
    print_success "Installation verified successfully!"
  else
    echo ""
    print_error "Installation verification failed"
    exit 1
  fi
}

# Step 8: Print Summary
step_print_summary() {
  print_step "Installation Complete!"

  echo ""
  echo -e "${GREEN}✓ WIA-CITY-012 Elevator System Standard installed successfully!${NC}"
  echo ""
  echo -e "${CYAN}Next Steps:${NC}"
  echo ""
  echo "1. Configure API credentials:"
  echo "   ${YELLOW}export WIA_API_KEY=your-api-key${NC}"
  echo "   or run: ${YELLOW}./cli/elevator-system.sh config${NC}"
  echo ""
  echo "2. Test the CLI tool:"
  echo "   ${YELLOW}./cli/elevator-system.sh help${NC}"
  echo ""
  echo "3. Use the TypeScript SDK:"
  echo "   ${YELLOW}cd api/typescript${NC}"
  echo "   ${YELLOW}npm link${NC}  (to use globally)"
  echo ""
  echo "4. Read the documentation:"
  echo "   ${YELLOW}README.md${NC} - Quick start guide"
  echo "   ${YELLOW}spec/WIA-CITY-012-v1.0.md${NC} - Full specification"
  echo ""
  echo -e "${CYAN}Resources:${NC}"
  echo "  • Documentation: https://docs.wia.org/city-012"
  echo "  • GitHub: https://github.com/WIA-Official/wia-standards"
  echo "  • Website: https://wia.org/standards/city-012"
  echo ""
  echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
  echo -e "${GREEN}         弘익人間 (홍익인간) · Benefit All Humanity${NC}"
  echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
  echo ""
}

################################################################################
# Main Installation Flow
################################################################################

main() {
  print_header

  step_check_prerequisites
  step_install_typescript_sdk
  step_build_typescript_sdk
  step_setup_cli
  step_run_tests
  step_create_config
  step_verify_installation
  step_print_summary
}

# Run installation
main "$@"
