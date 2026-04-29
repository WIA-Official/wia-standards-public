#!/usr/bin/env bash

################################################################################
# WIA-MACHINE_VISION Installation Script
#
# Installs the WIA Machine Vision standard including:
# - TypeScript SDK
# - CLI tools
# - Dependencies
# - Optional deep learning frameworks
#
# Philosophy: 弘益人間 (Benefit All Humanity)
################################################################################

set -euo pipefail

VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

################################################################################
# Helper Functions
################################################################################

log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

command_exists() {
    command -v "$1" &> /dev/null
}

check_requirements() {
    log_info "Checking system requirements..."

    local missing_deps=()

    # Check Node.js
    if ! command_exists node; then
        missing_deps+=("node")
    else
        local node_version
        node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [ "$node_version" -lt 18 ]; then
            log_warning "Node.js version 18+ required, found v$(node --version)"
            missing_deps+=("node>=18")
        fi
    fi

    # Check npm
    if ! command_exists npm; then
        missing_deps+=("npm")
    fi

    # Check Python (optional but recommended)
    if ! command_exists python3; then
        log_warning "Python 3 not found (optional, needed for some examples)"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "Missing required dependencies: ${missing_deps[*]}"
        log_info "Please install missing dependencies and try again"
        return 1
    fi

    log_success "All required dependencies found"
    return 0
}

install_typescript_sdk() {
    log_info "Installing TypeScript SDK..."

    cd "$SCRIPT_DIR/api/typescript"

    # Install dependencies
    log_info "Installing npm dependencies..."
    npm install

    # Build SDK
    log_info "Building TypeScript SDK..."
    npm run build

    # Run tests
    log_info "Running tests..."
    npm test || log_warning "Some tests failed"

    log_success "TypeScript SDK installed successfully"
}

install_cli_tools() {
    log_info "Installing CLI tools..."

    local cli_path="$SCRIPT_DIR/cli/wia-machine-vision.sh"
    local install_path="/usr/local/bin/wia-machine-vision"

    # Make CLI executable
    chmod +x "$cli_path"

    # Try to install globally
    if [ -w "/usr/local/bin" ]; then
        ln -sf "$cli_path" "$install_path"
        log_success "CLI tool installed to $install_path"
    else
        log_warning "Cannot write to /usr/local/bin, trying with sudo..."
        if command_exists sudo; then
            sudo ln -sf "$cli_path" "$install_path"
            log_success "CLI tool installed to $install_path (with sudo)"
        else
            log_warning "Could not install CLI globally"
            log_info "Add to PATH manually: export PATH=\"$SCRIPT_DIR/cli:\$PATH\""
        fi
    fi

    # Test CLI
    if command_exists wia-machine-vision; then
        log_success "CLI tool is available: $(which wia-machine-vision)"
    fi
}

install_python_dependencies() {
    log_info "Installing optional Python dependencies..."

    if ! command_exists python3; then
        log_warning "Python 3 not found, skipping Python dependencies"
        return
    fi

    if ! command_exists pip3; then
        log_warning "pip3 not found, skipping Python dependencies"
        return
    fi

    # Create virtual environment (optional)
    if [ "$INSTALL_VENV" = "true" ]; then
        log_info "Creating Python virtual environment..."
        python3 -m venv "$SCRIPT_DIR/venv"
        source "$SCRIPT_DIR/venv/bin/activate"
    fi

    # Install Python packages
    log_info "Installing Python packages..."
    pip3 install --upgrade pip
    pip3 install \
        onnxruntime \
        onnxruntime-gpu \
        opencv-python \
        pillow \
        numpy \
        torch torchvision \
        tensorflow \
        || log_warning "Some Python packages failed to install"

    log_success "Python dependencies installed"
}

install_optional_frameworks() {
    log_info "Optional: Install deep learning frameworks? (y/n)"
    read -r install_dl

    if [ "$install_dl" != "y" ]; then
        log_info "Skipping optional frameworks"
        return
    fi

    log_info "Select frameworks to install:"
    log_info "1) PyTorch"
    log_info "2) TensorFlow"
    log_info "3) Both"
    log_info "4) Skip"
    read -r framework_choice

    case $framework_choice in
        1)
            log_info "Installing PyTorch..."
            pip3 install torch torchvision torchaudio || log_error "PyTorch installation failed"
            ;;
        2)
            log_info "Installing TensorFlow..."
            pip3 install tensorflow || log_error "TensorFlow installation failed"
            ;;
        3)
            log_info "Installing PyTorch and TensorFlow..."
            pip3 install torch torchvision torchaudio tensorflow || log_error "Framework installation failed"
            ;;
        *)
            log_info "Skipping framework installation"
            ;;
    esac
}

setup_environment() {
    log_info "Setting up environment..."

    # Create config directory
    mkdir -p "$HOME/.wia-machine-vision"

    # Create default config
    cat > "$HOME/.wia-machine-vision/config.json" << EOF
{
  "version": "$VERSION",
  "backend": "cpu",
  "device": "cpu",
  "modelCache": "$HOME/.wia-machine-vision/models",
  "dataCache": "$HOME/.wia-machine-vision/data",
  "logLevel": "info"
}
EOF

    # Create cache directories
    mkdir -p "$HOME/.wia-machine-vision/models"
    mkdir -p "$HOME/.wia-machine-vision/data"

    log_success "Environment configured at $HOME/.wia-machine-vision"
}

download_example_models() {
    log_info "Download example models? (y/n)"
    read -r download_models

    if [ "$download_models" != "y" ]; then
        log_info "Skipping model download"
        return
    fi

    local models_dir="$HOME/.wia-machine-vision/models"

    log_info "Downloading example models (this may take a while)..."

    # Download YOLO26-N (smallest model)
    log_info "Downloading YOLO26-N..."
    # In production, replace with actual download URLs
    log_warning "Model download not implemented in this version"
    log_info "Please download models manually from:"
    log_info "  - YOLO: https://github.com/ultralytics/ultralytics"
    log_info "  - PyTorch Hub: https://pytorch.org/hub/"
    log_info "  - TensorFlow Hub: https://tfhub.dev/"
}

show_completion_message() {
    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                                                           ║${NC}"
    echo -e "${GREEN}║      WIA-MACHINE_VISION Installation Complete! 🎉        ║${NC}"
    echo -e "${GREEN}║                                                           ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${NC}"
    echo ""
    echo "Quick Start:"
    echo ""
    echo "  # Test CLI"
    echo "  wia-machine-vision version"
    echo "  wia-machine-vision help"
    echo ""
    echo "  # Use TypeScript SDK"
    echo "  cd api/typescript"
    echo "  npm run example"
    echo ""
    echo "  # View documentation"
    echo "  open ebook/en/index.html"
    echo ""
    echo "Configuration: $HOME/.wia-machine-vision/config.json"
    echo "Models cache: $HOME/.wia-machine-vision/models"
    echo ""
    echo "For more information, visit:"
    echo "  https://github.com/WIA-Official/wia-standards"
    echo ""
}

################################################################################
# Main Installation
################################################################################

main() {
    echo ""
    echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                                                           ║${NC}"
    echo -e "${BLUE}║      WIA-MACHINE_VISION Installer v${VERSION}               ║${NC}"
    echo -e "${BLUE}║                                                           ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}弘益人間 (Benefit All Humanity)${NC}"
    echo ""

    # Parse command line arguments
    INSTALL_VENV=false
    SKIP_PYTHON=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --venv)
                INSTALL_VENV=true
                shift
                ;;
            --skip-python)
                SKIP_PYTHON=true
                shift
                ;;
            --help)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --venv          Create Python virtual environment"
                echo "  --skip-python   Skip Python dependency installation"
                echo "  --help          Show this help message"
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    # Check requirements
    if ! check_requirements; then
        exit 1
    fi

    # Install components
    install_typescript_sdk
    install_cli_tools

    if [ "$SKIP_PYTHON" != "true" ]; then
        install_python_dependencies
        install_optional_frameworks
    fi

    setup_environment
    download_example_models

    # Show completion message
    show_completion_message
}

# Run main installation
main "$@"
