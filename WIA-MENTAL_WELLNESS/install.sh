#!/bin/bash

# WIA-MENTAL_WELLNESS Standard Installation Script
# 弘益人間 (Benefit All Humanity)

set -e

VERSION="1.0.0"
INSTALL_DIR="${HOME}/.wia/mental-wellness"
BIN_DIR="${HOME}/.local/bin"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║       WIA-MENTAL_WELLNESS Standard Installation          ║"
    echo "║              弘益人間 · Benefit All Humanity              ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_header

echo "Installing WIA-MENTAL_WELLNESS Standard v${VERSION}..."
echo ""

# Create directories
print_info "Creating directories..."
mkdir -p "${INSTALL_DIR}"
mkdir -p "${BIN_DIR}"
mkdir -p "${INSTALL_DIR}/spec"
mkdir -p "${INSTALL_DIR}/cli"
mkdir -p "${INSTALL_DIR}/ebook"

# Copy files
print_info "Copying standard files..."
cp -r spec/* "${INSTALL_DIR}/spec/" 2>/dev/null || true
cp -r cli/* "${INSTALL_DIR}/cli/" 2>/dev/null || true
cp -r ebook/* "${INSTALL_DIR}/ebook/" 2>/dev/null || true
cp README.md "${INSTALL_DIR}/" 2>/dev/null || true

# Make CLI executable
chmod +x "${INSTALL_DIR}/cli/wia-mental-wellness.sh" 2>/dev/null || true

# Create symlink in bin directory
if [ -f "${INSTALL_DIR}/cli/wia-mental-wellness.sh" ]; then
    ln -sf "${INSTALL_DIR}/cli/wia-mental-wellness.sh" "${BIN_DIR}/wia-mental-wellness"
    print_success "CLI tool installed"
fi

# Add to PATH if not already there
if [[ ":$PATH:" != *":${BIN_DIR}:"* ]]; then
    print_warning "Adding ${BIN_DIR} to PATH..."
    
    # Detect shell
    if [ -n "$BASH_VERSION" ]; then
        SHELL_RC="${HOME}/.bashrc"
    elif [ -n "$ZSH_VERSION" ]; then
        SHELL_RC="${HOME}/.zshrc"
    else
        SHELL_RC="${HOME}/.profile"
    fi
    
    echo "" >> "$SHELL_RC"
    echo "# WIA-MENTAL_WELLNESS CLI" >> "$SHELL_RC"
    echo "export PATH=\"\$PATH:${BIN_DIR}\"" >> "$SHELL_RC"
    
    print_info "Please restart your shell or run: source $SHELL_RC"
fi

echo ""
print_success "Installation complete!"
echo ""

# Print usage information
echo "═══════════════════════════════════════════════════════════"
echo "Getting Started:"
echo ""
echo "  View specification:"
echo "    cat ${INSTALL_DIR}/spec/WIA-MENTAL_WELLNESS-v1.0.md"
echo ""
echo "  Open ebook guide:"
echo "    open ${INSTALL_DIR}/ebook/en/index.html"
echo "    (or browse to file:://${INSTALL_DIR}/ebook/en/index.html)"
echo ""
echo "  Use CLI tool:"
echo "    wia-mental-wellness assess-mood"
echo "    wia-mental-wellness crisis-help"
echo "    wia-mental-wellness meditation-session"
echo "    wia-mental-wellness help"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

print_info "Crisis Resources:"
echo "  US: 988 (Suicide & Crisis Lifeline)"
echo "  Text: HOME to 741741 (Crisis Text Line)"
echo "  International: https://www.iasp.info/resources/Crisis_Centres/"
echo ""

print_success "弘益人間 · Benefit All Humanity"
echo ""
