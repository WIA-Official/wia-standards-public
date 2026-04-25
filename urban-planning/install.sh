#!/bin/bash

##############################################################################
# WIA-CITY-016: Urban Planning Standard - Installation Script
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Version: 1.0.0
# License: MIT
##############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Print functions
print_header() {
    echo -e "${MAGENTA}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║  WIA-CITY-016: Urban Planning Standard                        ║"
    echo "║  Installation Script                                          ║"
    echo "║                                                                ║"
    echo "║  弘益人間 (홍익인간) - Benefit All Humanity                      ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_step() {
    echo -e "${CYAN}▶${NC} $1"
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
check_prerequisites() {
    print_step "Checking prerequisites..."

    local missing_deps=()

    # Check for Node.js
    if ! command_exists node; then
        missing_deps+=("node")
        print_warning "Node.js is not installed"
    else
        local node_version=$(node --version | cut -d'v' -f2)
        print_success "Node.js ${node_version} found"
    fi

    # Check for npm
    if ! command_exists npm; then
        missing_deps+=("npm")
        print_warning "npm is not installed"
    else
        local npm_version=$(npm --version)
        print_success "npm ${npm_version} found"
    fi

    # Check for jq (optional, for CLI)
    if ! command_exists jq; then
        print_warning "jq is not installed (optional for CLI JSON parsing)"
        print_info "Install with: sudo apt-get install jq (Ubuntu/Debian) or brew install jq (macOS)"
    else
        local jq_version=$(jq --version | cut -d'-' -f2)
        print_success "jq ${jq_version} found"
    fi

    # Check for curl
    if ! command_exists curl; then
        missing_deps+=("curl")
        print_warning "curl is not installed"
    else
        local curl_version=$(curl --version | head -n1 | awk '{print $2}')
        print_success "curl ${curl_version} found"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        print_info "Please install the missing dependencies and try again."
        exit 1
    fi

    print_success "All prerequisites satisfied"
    echo ""
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    if [ ! -d "$SCRIPT_DIR/api/typescript" ]; then
        print_warning "TypeScript SDK directory not found. Skipping..."
        return
    fi

    cd "$SCRIPT_DIR/api/typescript"

    # Install dependencies
    print_info "Installing npm dependencies..."
    npm install

    # Build the SDK
    if [ -f "tsconfig.json" ]; then
        print_info "Building TypeScript SDK..."
        npm run build 2>/dev/null || print_warning "Build script not available, skipping build"
    fi

    print_success "TypeScript SDK installed successfully"
    echo ""
}

# Setup CLI tool
setup_cli() {
    print_step "Setting up CLI tool..."

    if [ ! -f "$SCRIPT_DIR/cli/urban-planning.sh" ]; then
        print_warning "CLI script not found. Skipping..."
        return
    fi

    # Make CLI executable
    chmod +x "$SCRIPT_DIR/cli/urban-planning.sh"
    print_success "CLI tool made executable"

    # Create symlink (optional)
    local bin_dir="$HOME/.local/bin"
    if [ -d "$bin_dir" ]; then
        local symlink="$bin_dir/wia-urban-planning"
        if [ -L "$symlink" ]; then
            rm "$symlink"
        fi
        ln -s "$SCRIPT_DIR/cli/urban-planning.sh" "$symlink"
        print_success "Symlink created: $symlink"
    else
        print_info "To use CLI from anywhere, add to PATH:"
        print_info "  export PATH=\"\$PATH:$SCRIPT_DIR/cli\""
    fi

    echo ""
}

# Display usage information
display_usage() {
    print_step "Installation complete!"
    echo ""

    print_info "═══════════════════════════════════════════════════════════════"
    print_info "                    Quick Start Guide                          "
    print_info "═══════════════════════════════════════════════════════════════"
    echo ""

    echo -e "${CYAN}1. TypeScript SDK:${NC}"
    echo ""
    echo "   # Install in your project"
    echo "   cd your-project"
    echo "   npm install @wia/city-urban-planning"
    echo ""
    echo "   # Or link locally for development"
    echo "   cd $SCRIPT_DIR/api/typescript"
    echo "   npm link"
    echo "   cd your-project"
    echo "   npm link @wia/city-urban-planning"
    echo ""

    echo -e "${CYAN}2. CLI Tool:${NC}"
    echo ""
    echo "   # Set API key"
    echo "   export WIA_URBAN_PLANNING_API_KEY=\"your-api-key\""
    echo ""
    echo "   # List land use"
    echo "   $SCRIPT_DIR/cli/urban-planning.sh list-land-use"
    echo ""
    echo "   # Check zoning compatibility"
    echo "   $SCRIPT_DIR/cli/urban-planning.sh check-compatibility residential_low commercial_central"
    echo ""
    echo "   # Calculate density"
    echo "   $SCRIPT_DIR/cli/urban-planning.sh calculate-density zone-001"
    echo ""
    echo "   # Run growth simulation"
    echo "   $SCRIPT_DIR/cli/urban-planning.sh simulate-growth 2025 2050 compact"
    echo ""
    echo "   # Get help"
    echo "   $SCRIPT_DIR/cli/urban-planning.sh help"
    echo ""

    echo -e "${CYAN}3. Example Code:${NC}"
    echo ""
    cat << 'EOF'
   import { UrbanPlanningSDK } from '@wia/city-urban-planning';

   const sdk = new UrbanPlanningSDK({
     apiKey: 'your-api-key',
     endpoint: 'https://api.wia.org/city-016/v1'
   });

   // Get land use data
   const landUse = await sdk.landUse.get('parcel-001');

   // Check zoning compatibility
   const compatibility = await sdk.zoning.checkCompatibility(
     'residential_low',
     'commercial_central'
   );

   // Run growth simulation
   const simulation = await sdk.simulation.simulateGrowth({
     startYear: 2025,
     endYear: 2050,
     scenario: 'compact',
     populationGrowthRate: 1.5,
     economicGrowthRate: 2.0,
     constraints: {}
   });
EOF
    echo ""

    echo -e "${CYAN}4. Documentation:${NC}"
    echo ""
    echo "   - README:       $SCRIPT_DIR/README.md"
    echo "   - Specification: $SCRIPT_DIR/spec/WIA-CITY-016-v1.0.md"
    echo "   - API Docs:      https://api.wia.org/city-016/docs"
    echo ""

    print_info "═══════════════════════════════════════════════════════════════"
    echo ""

    print_success "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""
}

# Cleanup function
cleanup() {
    if [ $? -ne 0 ]; then
        print_error "Installation failed!"
        exit 1
    fi
}

trap cleanup EXIT

# Main installation flow
main() {
    print_header

    check_prerequisites
    install_typescript_sdk
    setup_cli
    display_usage

    print_success "Installation completed successfully! 🏙️"
    echo ""
}

main "$@"
