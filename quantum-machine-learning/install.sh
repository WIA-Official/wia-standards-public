#!/bin/bash

################################################################################
# WIA-QUA-006: Quantum Machine Learning - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Quantum ML Research Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-QUA-006 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
################################################################################

set -e

# Colors for output
INDIGO='\033[0;38;5;99m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${INDIGO}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    🤖 WIA-QUA-006: Quantum Machine Learning Installer        ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
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
    echo -e "${GRAY}  $1${RESET}"
}

# Check prerequisites
check_prerequisites() {
    print_section "Checking Prerequisites"

    local all_ok=true

    # Check bash
    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1)"
    else
        print_error "Bash not found"
        all_ok=false
    fi

    # Check bc (for calculations)
    if command -v bc &> /dev/null; then
        print_success "bc (calculator): installed"
    else
        print_warning "bc not found (required for CLI calculations)"
        print_info "Install with: apt-get install bc (Debian/Ubuntu) or brew install bc (macOS)"
        all_ok=false
    fi

    # Check Node.js (optional, for TypeScript SDK)
    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional, for TypeScript SDK)"
        print_info "Install from: https://nodejs.org/"
    fi

    # Check npm (optional, for TypeScript SDK)
    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional, for TypeScript SDK)"
    fi

    # Check Python (optional, for quantum simulators)
    if command -v python3 &> /dev/null; then
        print_success "Python: $(python3 --version)"
    else
        print_warning "Python not found (optional, for quantum simulators)"
    fi

    if [ "$all_ok" = false ]; then
        echo ""
        print_error "Some prerequisites are missing. Please install them and try again."
        exit 1
    fi
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-qua-006.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    # Make executable
    chmod +x "$cli_path"
    print_success "Made CLI script executable"

    # Check if we need sudo
    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-qua-006"
        print_success "Installed CLI to $install_dir/wia-qua-006"
    else
        print_info "Installing to $install_dir requires sudo..."
        sudo cp "$cli_path" "$install_dir/wia-qua-006"
        print_success "Installed CLI to $install_dir/wia-qua-006 (with sudo)"
    fi

    # Verify installation
    if command -v wia-qua-006 &> /dev/null; then
        print_success "CLI tool is now available as 'wia-qua-006'"
        print_info "Try: wia-qua-006 --help"
    else
        print_warning "CLI installed but not in PATH. You may need to restart your terminal."
    fi
}

# Install TypeScript SDK
install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found at $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found. Skipping TypeScript SDK installation."
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install

    print_info "Building TypeScript SDK..."
    npm run build

    print_success "TypeScript SDK built successfully"
    print_info "To use the SDK globally: npm link"
    print_info "Or in your project: npm install $ts_dir"

    cd "$SCRIPT_DIR"
}

# Create symlinks for documentation
install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/qua-006/docs"

    # Create directory
    mkdir -p "$docs_dir"

    # Copy documentation
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "Installed README.md"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Installed specification files"
    fi

    print_info "Documentation available at: $docs_dir"
}

# Install sample data
install_samples() {
    print_section "Installing Sample Data"

    local samples_dir="$HOME/.wia/qua-006/samples"
    mkdir -p "$samples_dir"

    # Create sample CSV files for testing
    cat > "$samples_dir/train.csv" <<EOF
0.1,0.2,0.3,0.4,0
0.5,0.6,0.7,0.8,1
0.2,0.3,0.4,0.5,0
0.6,0.7,0.8,0.9,1
0.3,0.4,0.5,0.6,0
0.7,0.8,0.9,0.1,1
EOF

    cat > "$samples_dir/test.csv" <<EOF
0.15,0.25,0.35,0.45
0.55,0.65,0.75,0.85
0.25,0.35,0.45,0.55
EOF

    print_success "Created sample training data: $samples_dir/train.csv"
    print_success "Created sample test data: $samples_dir/test.csv"
    print_info "Sample data available at: $samples_dir"
}

# Run tests
run_tests() {
    print_section "Running Tests"

    # Test CLI
    if command -v wia-qua-006 &> /dev/null; then
        print_info "Testing CLI tool..."
        wia-qua-006 version &> /dev/null && print_success "CLI test passed" || print_error "CLI test failed"
    fi

    # Test TypeScript SDK
    if [ -d "$SCRIPT_DIR/api/typescript/dist" ]; then
        print_success "TypeScript SDK build verified"
    fi

    # Test sample data
    if [ -f "$HOME/.wia/qua-006/samples/train.csv" ]; then
        print_success "Sample data verified"
    fi
}

# Show post-installation message
show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}The WIA-QUA-006 standard has been successfully installed!${RESET}"
    echo ""
    echo "Available commands:"
    echo -e "${CYAN}  wia-qua-006 train-qnn${RESET}        - Train Quantum Neural Network"
    echo -e "${CYAN}  wia-qua-006 quantum-kernel${RESET}   - Compute quantum kernel matrix"
    echo -e "${CYAN}  wia-qua-006 vqc${RESET}              - Run Variational Quantum Classifier"
    echo -e "${CYAN}  wia-qua-006 feature-map${RESET}      - Generate quantum feature map"
    echo -e "${CYAN}  wia-qua-006 qbm${RESET}              - Simulate Quantum Boltzmann Machine"
    echo -e "${CYAN}  wia-qua-006 help${RESET}             - Show help message"
    echo ""
    echo "Quick start:"
    echo -e "${GRAY}  # Train a 4-qubit QNN${RESET}"
    echo -e "${CYAN}  wia-qua-006 train-qnn --qubits 4 --layers 3 --data ~/.wia/qua-006/samples/train.csv${RESET}"
    echo ""
    echo -e "${GRAY}  # Compute quantum kernel${RESET}"
    echo -e "${CYAN}  wia-qua-006 quantum-kernel --qubits 3 --feature-map angle --data ~/.wia/qua-006/samples/test.csv${RESET}"
    echo ""
    echo -e "${GRAY}  # Run VQC${RESET}"
    echo -e "${CYAN}  wia-qua-006 vqc --qubits 4 --ansatz hardware-efficient --train ~/.wia/qua-006/samples/train.csv${RESET}"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${GRAY}  import { QuantumMLSDK, createQNN, trainQNN } from '@wia/qua-006';${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/qua-006/docs/${RESET}"
    echo ""
    echo "Sample data:"
    echo -e "${GRAY}  $HOME/.wia/qua-006/samples/${RESET}"
    echo ""
    echo -e "${INDIGO}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation flow
main() {
    print_header

    # Check if user wants to skip certain steps
    SKIP_CLI=false
    SKIP_TS=false
    SKIP_SAMPLES=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-cli) SKIP_CLI=true; shift ;;
            --skip-typescript) SKIP_TS=true; shift ;;
            --skip-samples) SKIP_SAMPLES=true; shift ;;
            --help)
                echo "Usage: ./install.sh [options]"
                echo ""
                echo "Options:"
                echo "  --skip-cli          Skip CLI installation"
                echo "  --skip-typescript   Skip TypeScript SDK installation"
                echo "  --skip-samples      Skip sample data installation"
                echo "  --help              Show this help message"
                echo ""
                exit 0
                ;;
            *) shift ;;
        esac
    done

    # Run installation steps
    check_prerequisites

    if [ "$SKIP_CLI" = false ]; then
        install_cli
    else
        print_warning "Skipping CLI installation"
    fi

    if [ "$SKIP_TS" = false ]; then
        install_typescript
    else
        print_warning "Skipping TypeScript SDK installation"
    fi

    install_docs

    if [ "$SKIP_SAMPLES" = false ]; then
        install_samples
    else
        print_warning "Skipping sample data installation"
    fi

    run_tests
    show_completion
}

# Run main
main "$@"

# 弘益人間 (홍익인간) · Benefit All Humanity
