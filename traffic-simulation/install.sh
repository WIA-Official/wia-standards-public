#!/bin/bash

##############################################################################
# WIA-CITY-017: Traffic Simulation Standard - Installation Script
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
    echo "║  WIA-CITY-017: 교통 시뮬레이션 표준                           ║"
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

    # Check for curl
    if ! command_exists curl; then
        missing_deps+=("curl")
        print_warning "curl is not installed"
    else
        print_success "curl found"
    fi

    # Check for jq
    if ! command_exists jq; then
        missing_deps+=("jq")
        print_warning "jq is not installed (optional, for CLI tool)"
    else
        print_success "jq found"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        echo
        print_warning "Missing dependencies: ${missing_deps[*]}"
        echo
        print_info "Please install the following dependencies:"
        echo
        echo "  Ubuntu/Debian:"
        echo "    sudo apt-get update"
        echo "    sudo apt-get install -y nodejs npm curl jq"
        echo
        echo "  macOS:"
        echo "    brew install node curl jq"
        echo
        echo "  CentOS/RHEL:"
        echo "    sudo yum install -y nodejs npm curl jq"
        echo
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi

    echo
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    cd "$SCRIPT_DIR/api/typescript"

    if [ -f "package.json" ]; then
        print_info "Installing dependencies..."

        if command_exists npm; then
            npm install
            print_success "Dependencies installed"
        else
            print_warning "npm not found, skipping dependency installation"
        fi
    else
        print_warning "package.json not found, skipping TypeScript SDK installation"
    fi

    cd "$SCRIPT_DIR"
    echo
}

# Setup CLI tool
setup_cli_tool() {
    print_step "Setting up CLI tool..."

    local cli_path="$SCRIPT_DIR/cli/traffic-simulation.sh"

    if [ -f "$cli_path" ]; then
        chmod +x "$cli_path"
        print_success "CLI tool permissions set"

        # Create symlink in /usr/local/bin (if sudo available)
        if command_exists sudo && [ -w "/usr/local/bin" ] || sudo -n true 2>/dev/null; then
            read -p "Create symlink in /usr/local/bin? (y/N) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                sudo ln -sf "$cli_path" /usr/local/bin/traffic-simulation
                print_success "Symlink created: /usr/local/bin/traffic-simulation"
            fi
        else
            print_info "To use CLI globally, add to your PATH:"
            echo "  export PATH=\"$SCRIPT_DIR/cli:\$PATH\""
        fi
    else
        print_warning "CLI tool not found at $cli_path"
    fi

    echo
}

# Create example configuration
create_example_config() {
    print_step "Creating example configuration..."

    local config_file="$SCRIPT_DIR/.env.example"

    cat > "$config_file" <<EOF
# WIA-CITY-017 교통 시뮬레이션 설정

# API Configuration
WIA_TRAFFIC_SIM_ENDPOINT=https://api.wia.org/city-017/v1
WIA_TRAFFIC_SIM_API_KEY=your-api-key-here

# Logging (optional)
LOG_LEVEL=info

# Simulation Defaults (optional)
DEFAULT_TIME_STEP=0.5
DEFAULT_WARMUP_PERIOD=900
DEFAULT_CAR_FOLLOWING_MODEL=idm
DEFAULT_LANE_CHANGE_MODEL=mobil
EOF

    print_success "Example configuration created: .env.example"
    print_info "Copy .env.example to .env and update with your API key:"
    echo "  cp .env.example .env"
    echo "  nano .env"

    echo
}

# Print usage information
print_usage() {
    print_step "Usage information"

    echo "TypeScript SDK:"
    echo "  import { TrafficSimulationSDK } from '@wia/city-traffic-simulation';"
    echo
    echo "CLI Tool:"
    echo "  export WIA_TRAFFIC_SIM_API_KEY='your-api-key'"
    echo "  traffic-simulation create-simulation \"강남 출근\" network-001 \"2025-12-26T07:00:00\""
    echo "  traffic-simulation traffic-realtime"
    echo "  traffic-simulation optimize-signals signal-001,signal-002"
    echo
    echo "Documentation:"
    echo "  README.md                   - Overview and quick start"
    echo "  spec/WIA-CITY-017-v1.0.md  - Full specification"
    echo
    echo "API Endpoint:"
    echo "  https://api.wia.org/city-017/v1"
    echo
}

# Verify installation
verify_installation() {
    print_step "Verifying installation..."

    local all_good=true

    # Check spec file
    if [ -f "$SCRIPT_DIR/spec/WIA-CITY-017-v1.0.md" ]; then
        print_success "Specification found"
    else
        print_error "Specification not found"
        all_good=false
    fi

    # Check TypeScript types
    if [ -f "$SCRIPT_DIR/api/typescript/src/types.ts" ]; then
        print_success "TypeScript types found"
    else
        print_error "TypeScript types not found"
        all_good=false
    fi

    # Check SDK
    if [ -f "$SCRIPT_DIR/api/typescript/src/index.ts" ]; then
        print_success "TypeScript SDK found"
    else
        print_error "TypeScript SDK not found"
        all_good=false
    fi

    # Check CLI tool
    if [ -f "$SCRIPT_DIR/cli/traffic-simulation.sh" ] && [ -x "$SCRIPT_DIR/cli/traffic-simulation.sh" ]; then
        print_success "CLI tool found and executable"
    else
        print_error "CLI tool not found or not executable"
        all_good=false
    fi

    # Check README
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        print_success "README found"
    else
        print_error "README not found"
        all_good=false
    fi

    echo

    if [ "$all_good" = true ]; then
        print_success "Installation verified successfully!"
    else
        print_warning "Some files are missing. Installation may be incomplete."
    fi

    echo
}

# Print final message
print_final_message() {
    echo -e "${GREEN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║  Installation Complete! 🎉                                    ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo
    echo -e "${CYAN}Next steps:${NC}"
    echo "  1. Get your API key from: https://wia.org/developers"
    echo "  2. Set environment variable:"
    echo "     export WIA_TRAFFIC_SIM_API_KEY='your-api-key'"
    echo "  3. Try the CLI tool:"
    echo "     ./cli/traffic-simulation.sh traffic-realtime"
    echo
    echo -e "${CYAN}Quick start examples:${NC}"
    echo "  # Create a simulation"
    echo "  ./cli/traffic-simulation.sh create-simulation \\"
    echo "    \"강남 출근 시간대\" network-gangnam \"2025-12-26T07:00:00\" 7200"
    echo
    echo "  # Get real-time traffic"
    echo "  ./cli/traffic-simulation.sh traffic-realtime"
    echo
    echo "  # Optimize signals"
    echo "  ./cli/traffic-simulation.sh optimize-signals signal-001,signal-002"
    echo
    echo "  # Forecast traffic"
    echo "  ./cli/traffic-simulation.sh forecast-traffic link-001 short"
    echo
    echo -e "${CYAN}Resources:${NC}"
    echo "  📖 Documentation: https://wia.org/standards/city-017"
    echo "  💬 Community: https://github.com/WIA-Official/wia-standards"
    echo "  📧 Support: standards@wia-official.org"
    echo
    echo -e "${MAGENTA}弘益人間 (홍익인간) - Benefit All Humanity${NC}"
    echo
}

# Clean installation (optional)
clean_installation() {
    print_step "Cleaning previous installation..."

    # Remove node_modules
    if [ -d "$SCRIPT_DIR/api/typescript/node_modules" ]; then
        rm -rf "$SCRIPT_DIR/api/typescript/node_modules"
        print_success "Removed node_modules"
    fi

    # Remove package-lock.json
    if [ -f "$SCRIPT_DIR/api/typescript/package-lock.json" ]; then
        rm -f "$SCRIPT_DIR/api/typescript/package-lock.json"
        print_success "Removed package-lock.json"
    fi

    # Remove .env
    if [ -f "$SCRIPT_DIR/.env" ]; then
        rm -f "$SCRIPT_DIR/.env"
        print_success "Removed .env"
    fi

    echo
}

# Main installation flow
main() {
    print_header

    # Parse arguments
    CLEAN_INSTALL=false
    SKIP_VERIFY=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                CLEAN_INSTALL=true
                shift
                ;;
            --skip-verify)
                SKIP_VERIFY=true
                shift
                ;;
            --help)
                echo "Usage: $0 [options]"
                echo
                echo "Options:"
                echo "  --clean        Clean previous installation before installing"
                echo "  --skip-verify  Skip installation verification"
                echo "  --help         Show this help message"
                echo
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done

    # Clean if requested
    if [ "$CLEAN_INSTALL" = true ]; then
        clean_installation
    fi

    # Run installation steps
    check_prerequisites
    install_typescript_sdk
    setup_cli_tool
    create_example_config

    # Verify installation
    if [ "$SKIP_VERIFY" = false ]; then
        verify_installation
    fi

    # Print usage
    print_usage

    # Final message
    print_final_message
}

# Run main function
main "$@"
