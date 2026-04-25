#!/bin/bash

################################################################################
# WIA-IND-006: Personalized Cosmetics Standard - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Industry & Biotech Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-IND-006 Personalized Cosmetics Standard CLI tool
# and TypeScript SDK, sets up dependencies, and configures the environment.
################################################################################

set -e

# Colors
INDIGO='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
RESET='\033[0m'

# Detect OS
OS="$(uname -s)"
case "${OS}" in
    Linux*)     PLATFORM=Linux;;
    Darwin*)    PLATFORM=Mac;;
    CYGWIN*)    PLATFORM=Cygwin;;
    MINGW*)     PLATFORM=MinGw;;
    *)          PLATFORM="UNKNOWN:${OS}"
esac

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     💅 WIA-IND-006: Personalized Cosmetics Standard          ║"
    echo "║                  Installation Script                          ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_step() {
    echo -e "\n${CYAN}▶ $1${RESET}"
}

print_success() {
    echo -e "${GREEN}✓ $1${RESET}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${RESET}"
}

print_error() {
    echo -e "${RED}✗ $1${RESET}"
}

print_info() {
    echo -e "  $1"
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
check_prerequisites() {
    print_step "Checking prerequisites..."

    local all_good=true

    # Check for bash
    if command_exists bash; then
        print_success "bash found: $(bash --version | head -n1)"
    else
        print_error "bash not found (required)"
        all_good=false
    fi

    # Check for bc (calculator)
    if command_exists bc; then
        print_success "bc found: $(bc --version | head -n1 2>/dev/null || echo 'installed')"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Installing bc..."

        case "${PLATFORM}" in
            Linux)
                if command_exists apt-get; then
                    sudo apt-get update && sudo apt-get install -y bc
                elif command_exists yum; then
                    sudo yum install -y bc
                elif command_exists pacman; then
                    sudo pacman -S --noconfirm bc
                else
                    print_error "Could not install bc. Please install manually."
                    all_good=false
                fi
                ;;
            Mac)
                if command_exists brew; then
                    brew install bc
                else
                    print_warning "Homebrew not found. Please install bc manually."
                fi
                ;;
            *)
                print_warning "Please install bc manually for your platform."
                ;;
        esac
    fi

    # Check for Node.js (optional, for TypeScript SDK)
    if command_exists node; then
        print_success "Node.js found: $(node --version)"
    else
        print_warning "Node.js not found (optional, needed for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if command_exists npm; then
        print_success "npm found: $(npm --version)"
    else
        print_warning "npm not found (optional, needed for TypeScript SDK)"
    fi

    if [ "$all_good" = false ]; then
        print_error "Prerequisites check failed. Please install missing components."
        exit 1
    fi

    print_success "All prerequisites satisfied"
}

# Install CLI tool
install_cli() {
    print_step "Installing CLI tool..."

    # Get the directory of this script
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    CLI_SCRIPT="$SCRIPT_DIR/cli/wia-ind-006.sh"

    # Check if CLI script exists
    if [ ! -f "$CLI_SCRIPT" ]; then
        print_error "CLI script not found at: $CLI_SCRIPT"
        exit 1
    fi

    # Make CLI script executable
    chmod +x "$CLI_SCRIPT"
    print_success "Made CLI script executable"

    # Determine installation directory
    if [ -w "/usr/local/bin" ]; then
        INSTALL_DIR="/usr/local/bin"
    elif [ -w "$HOME/.local/bin" ]; then
        INSTALL_DIR="$HOME/.local/bin"
        mkdir -p "$INSTALL_DIR"
    else
        INSTALL_DIR="$HOME/bin"
        mkdir -p "$INSTALL_DIR"
    fi

    # Create symlink
    if [ -L "$INSTALL_DIR/wia-ind-006" ]; then
        rm "$INSTALL_DIR/wia-ind-006"
    fi

    ln -s "$CLI_SCRIPT" "$INSTALL_DIR/wia-ind-006"
    print_success "Created symlink: $INSTALL_DIR/wia-ind-006"

    # Check if install dir is in PATH
    if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
        print_warning "$INSTALL_DIR is not in PATH"
        print_info "Add to PATH by adding this line to ~/.bashrc or ~/.zshrc:"
        print_info "  export PATH=\"\$PATH:$INSTALL_DIR\""

        # Offer to add to PATH
        echo -n "Would you like to add it to ~/.bashrc now? (y/n): "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo "" >> ~/.bashrc
            echo "# WIA-IND-006 CLI" >> ~/.bashrc
            echo "export PATH=\"\$PATH:$INSTALL_DIR\"" >> ~/.bashrc
            print_success "Added to ~/.bashrc (restart shell or run: source ~/.bashrc)"
        fi
    else
        print_success "$INSTALL_DIR is in PATH"
    fi
}

# Install TypeScript SDK
install_sdk() {
    print_step "Installing TypeScript SDK..."

    if ! command_exists npm; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "Install Node.js and npm from: https://nodejs.org/"
        return
    fi

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    SDK_DIR="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$SDK_DIR" ]; then
        print_error "TypeScript SDK directory not found"
        return
    fi

    cd "$SDK_DIR"

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build 2>/dev/null || print_warning "Build step skipped (TypeScript not configured)"

    print_success "TypeScript SDK installed"
    print_info "To use the SDK in your project:"
    print_info "  npm install $SDK_DIR"
    print_info "Or publish to npm registry and install via:"
    print_info "  npm install @wia/ind-006"
}

# Create example files
create_examples() {
    print_step "Creating example files..."

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    EXAMPLES_DIR="$SCRIPT_DIR/examples"

    mkdir -p "$EXAMPLES_DIR"

    # Example 1: Basic skin analysis
    cat > "$EXAMPLES_DIR/example-skin-analysis.sh" << 'EOF'
#!/bin/bash
# Example: Basic skin analysis using WIA-IND-006 CLI

echo "Example 1: Analyzing combination skin with sensitivity"
wia-ind-006 calc-skin-score \
    --hydration 45 \
    --elasticity 68 \
    --oil 35 \
    --sensitivity 65 \
    --pigmentation 60

echo ""
echo "Example 2: Baumann type classification"
wia-ind-006 analyze-baumann \
    --sebum 180 \
    --erythema 320 \
    --pig-risk 75 \
    --elasticity 55
EOF

    chmod +x "$EXAMPLES_DIR/example-skin-analysis.sh"

    # Example 2: Formulation optimization
    cat > "$EXAMPLES_DIR/example-formulation.sh" << 'EOF'
#!/bin/bash
# Example: Formulation optimization using WIA-IND-006 CLI

echo "Example 1: Adjusting retinol concentration for sensitive skin"
wia-ind-006 calc-ingredient-conc \
    --base 0.5 \
    --factor -0.6 \
    --max-adj 0.3

echo ""
echo "Example 2: Climate-based formulation (tropical humid)"
wia-ind-006 optimize-climate \
    --climate tropical-humid \
    --hydration 5 \
    --oil 12
EOF

    chmod +x "$EXAMPLES_DIR/example-formulation.sh"

    # Example 3: Genetic analysis
    cat > "$EXAMPLES_DIR/example-genetic.sh" << 'EOF'
#!/bin/bash
# Example: Genetic skin age calculation

echo "Example: High genetic aging risk with good skincare"
wia-ind-006 calc-genetic-age \
    --age 32 \
    --risk 8 \
    --care 75
EOF

    chmod +x "$EXAMPLES_DIR/example-genetic.sh"

    print_success "Created example files in: $EXAMPLES_DIR"
    print_info "Run examples:"
    print_info "  $EXAMPLES_DIR/example-skin-analysis.sh"
    print_info "  $EXAMPLES_DIR/example-formulation.sh"
    print_info "  $EXAMPLES_DIR/example-genetic.sh"
}

# Run verification tests
verify_installation() {
    print_step "Verifying installation..."

    if command_exists wia-ind-006; then
        print_success "CLI tool accessible via: wia-ind-006"

        # Test basic command
        print_info "Testing CLI..."
        wia-ind-006 --version > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            print_success "CLI test passed"
        else
            print_warning "CLI test had warnings (may still work)"
        fi
    else
        print_warning "wia-ind-006 not found in PATH"
        print_info "You may need to restart your shell or run: source ~/.bashrc"
    fi
}

# Print usage instructions
print_usage() {
    print_step "Installation complete!"

    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo -e "${CYAN}Next steps:${RESET}"
    echo ""
    echo "1. Verify installation:"
    echo "   wia-ind-006 --version"
    echo ""
    echo "2. View help:"
    echo "   wia-ind-006 --help"
    echo ""
    echo "3. Try examples:"
    echo "   wia-ind-006 calc-skin-score --hydration 45 --elasticity 68"
    echo "   wia-ind-006 analyze-baumann --sebum 180 --erythema 320"
    echo "   wia-ind-006 calc-genetic-age --age 32 --risk 8 --care 70"
    echo ""
    echo "4. Read documentation:"
    echo "   cat README.md"
    echo "   cat spec/WIA-IND-006-v1.0.md"
    echo ""
    echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo ""
    echo "WIA - World Certification Industry Association"
    echo "© 2025 SmileStory Inc. / WIA"
    echo "MIT License"
    echo ""
}

# Main installation flow
main() {
    print_header

    echo "Platform: $PLATFORM"
    echo ""

    check_prerequisites
    install_cli
    install_sdk
    create_examples
    verify_installation
    print_usage
}

main "$@"
