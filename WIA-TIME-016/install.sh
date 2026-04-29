#!/bin/bash

################################################################################
# WIA-TIME-016: Temporal Material - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Temporal Materials Science Group
#
# 弘익人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-016 standard components:
# - CLI tool
# - TypeScript SDK
# - Documentation
# - Configuration
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Installation paths
INSTALL_DIR="${HOME}/.wia/time-016"
BIN_DIR="${HOME}/.local/bin"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${PURPLE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${PURPLE}║${NC}  🔬 WIA-TIME-016: Temporal Material Standard            ${PURPLE}║${NC}"
    echo -e "${PURPLE}║${NC}     Installation Script v1.0.0                          ${PURPLE}║${NC}"
    echo -e "${PURPLE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${CYAN}ℹ${NC} $1"
}

print_step() {
    echo -e "${BLUE}▸${NC} $1"
}

check_dependencies() {
    print_step "Checking dependencies..."

    local missing_deps=()

    # Check for bash
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    # Check for node (optional, for TypeScript SDK)
    if ! command -v node &> /dev/null; then
        print_warning "Node.js not found - TypeScript SDK will not be built"
    else
        print_success "Node.js found: $(node --version)"
    fi

    # Check for npm (optional, for TypeScript SDK)
    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - TypeScript SDK will not be built"
    else
        print_success "npm found: $(npm --version)"
    fi

    # Check for bc (for calculations)
    if ! command -v bc &> /dev/null; then
        print_warning "bc not found - some CLI calculations may not work"
    else
        print_success "bc found"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        echo ""
        echo "Please install the missing dependencies and try again."
        exit 1
    fi

    print_success "All required dependencies are installed"
    echo ""
}

create_directories() {
    print_step "Creating directories..."

    mkdir -p "$INSTALL_DIR"
    mkdir -p "$BIN_DIR"
    mkdir -p "$INSTALL_DIR/docs"
    mkdir -p "$INSTALL_DIR/data"

    print_success "Directories created"
    echo ""
}

install_cli() {
    print_step "Installing CLI tool..."

    # Copy CLI script
    cp "$SCRIPT_DIR/cli/wia-time-016.sh" "$BIN_DIR/wia-time-016"
    chmod +x "$BIN_DIR/wia-time-016"

    print_success "CLI tool installed to $BIN_DIR/wia-time-016"
    echo ""
}

install_docs() {
    print_step "Installing documentation..."

    # Copy documentation files
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$INSTALL_DIR/docs/"
        print_success "README.md copied"
    fi

    if [ -f "$SCRIPT_DIR/spec/WIA-TIME-016-v1.0.md" ]; then
        cp "$SCRIPT_DIR/spec/WIA-TIME-016-v1.0.md" "$INSTALL_DIR/docs/"
        print_success "Specification copied"
    fi

    echo ""
}

install_typescript_sdk() {
    print_step "Installing TypeScript SDK..."

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found - skipping TypeScript SDK installation"
        echo ""
        return
    fi

    if [ -d "$SCRIPT_DIR/api/typescript" ]; then
        cd "$SCRIPT_DIR/api/typescript"

        # Install dependencies
        print_info "Installing npm dependencies..."
        npm install --quiet 2>&1 | grep -v "npm WARN" || true

        # Build TypeScript
        if [ -f "tsconfig.json" ] || [ -f "$SCRIPT_DIR/api/typescript/package.json" ]; then
            print_info "Building TypeScript SDK..."

            # Create tsconfig.json if it doesn't exist
            if [ ! -f "tsconfig.json" ]; then
                cat > tsconfig.json <<EOF
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "declaration": true,
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "moduleResolution": "node",
    "resolveJsonModule": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules", "dist"]
}
EOF
            fi

            npm run build --if-present 2>&1 | grep -v "npm WARN" || true
            print_success "TypeScript SDK built"
        fi

        cd - > /dev/null
    fi

    echo ""
}

create_config() {
    print_step "Creating configuration..."

    local config_file="$INSTALL_DIR/config.json"

    if [ ! -f "$config_file" ]; then
        cat > "$config_file" <<EOF
{
  "version": "1.0.0",
  "installDate": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "database": "$INSTALL_DIR/data/materials.db",
  "certificationAuthority": "WIA Temporal Materials Certification Center",
  "defaultTests": [
    "temporal-stress-test",
    "quantum-stability-test",
    "radiation-resistance-test",
    "thermal-cycling-test",
    "chrono-fatigue-test"
  ],
  "certificationLevels": {
    "WIA-TEMP-CERT-1": {
      "scope": "< 1 Tesla",
      "tsiMinimum": 0.70,
      "serviceLife": 1000
    },
    "WIA-TEMP-CERT-2": {
      "scope": "1-5 Tesla",
      "tsiMinimum": 0.85,
      "serviceLife": 2000
    },
    "WIA-TEMP-CERT-3": {
      "scope": "5-10 Tesla",
      "tsiMinimum": 0.90,
      "serviceLife": 5000
    },
    "WIA-TEMP-CERT-4": {
      "scope": "> 10 Tesla",
      "tsiMinimum": 0.95,
      "serviceLife": 10000
    }
  },
  "materials": {
    "exoticMatter": {
      "productionMethods": ["casimir-effect", "squeezed-light", "dynamic-casimir"],
      "confinementRequired": true,
      "hazardClass": "extreme"
    },
    "temporalAlloys": {
      "CTA-7": { "tsi": 0.95, "maxField": 10, "density": 8.2 },
      "CTA-9": { "tsi": 0.98, "maxField": 12, "density": 8.4 },
      "TS-316": { "tsi": 0.88, "maxField": 8, "density": 8.0 },
      "NC-1": { "tsi": 0.99, "maxField": 15, "density": 15.0 }
    },
    "quantumComposites": {
      "CWC-88": { "coherenceTime": 800, "tempRange": [4, 1500] },
      "DNV-Q1": { "coherenceTime": 10000, "tempRange": [1, 300] },
      "TIC-Bi2Se3": { "coherenceTime": 5000, "tempRange": [1, 100] }
    }
  },
  "safetyProtocols": {
    "exoticMatter": {
      "containmentLevel": 4,
      "ppeRequired": ["temporal-suit", "scba", "double-gloves", "dosimeter"],
      "evacuationDistance": 500
    },
    "temporalAlloys": {
      "hazardClass": "moderate",
      "ppeRequired": ["lab-coat", "safety-glasses", "gloves", "n95-mask"],
      "ventilationRequired": true
    }
  }
}
EOF
        print_success "Configuration file created"
    else
        print_info "Configuration file already exists"
    fi

    echo ""
}

check_path() {
    print_step "Checking PATH configuration..."

    if [[ ":$PATH:" != *":$BIN_DIR:"* ]]; then
        print_warning "$BIN_DIR is not in your PATH"
        echo ""
        echo "To add it, add this line to your ~/.bashrc or ~/.zshrc:"
        echo ""
        echo -e "  ${CYAN}export PATH=\"\$HOME/.local/bin:\$PATH\"${NC}"
        echo ""
        echo "Then run: source ~/.bashrc (or ~/.zshrc)"
    else
        print_success "$BIN_DIR is in your PATH"
    fi

    echo ""
}

create_sample_data() {
    print_step "Creating sample material database..."

    local db_file="$INSTALL_DIR/data/materials.db"

    cat > "$db_file" <<EOF
# WIA-TIME-016 Material Database
# Format: name|category|tsi|maxField|density|status

# Temporal Alloys
CTA-7|temporal-alloy|0.95|10|8.2|certified
CTA-9|temporal-alloy|0.98|12|8.4|certified
TS-316|temporal-alloy|0.88|8|8.0|certified
NC-1|temporal-alloy|0.99|15|15.0|certified

# Quantum Composites
CWC-88|quantum-composite|0.88|12|15.6|certified
DNV-Q1|quantum-composite|0.99|15|3.5|certified
TIC-Bi2Se3|quantum-composite|0.95|10|7.5|certified

# Chrono-Shielding
Multi-Layer-Shield|chrono-shield|0.99|10|7.5|certified
EOF

    print_success "Sample database created"
    echo ""
}

verify_installation() {
    print_step "Verifying installation..."

    # Check if CLI is executable
    if [ -x "$BIN_DIR/wia-time-016" ]; then
        print_success "CLI tool is executable"
    else
        print_error "CLI tool is not executable"
    fi

    # Check if config exists
    if [ -f "$INSTALL_DIR/config.json" ]; then
        print_success "Configuration file exists"
    else
        print_error "Configuration file not found"
    fi

    # Check if docs exist
    if [ -d "$INSTALL_DIR/docs" ]; then
        print_success "Documentation directory exists"
    else
        print_error "Documentation directory not found"
    fi

    echo ""
}

print_completion() {
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║${NC}  Installation Complete! 🎉                               ${GREEN}║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}Installation Summary:${NC}"
    echo -e "  • CLI tool: ${GREEN}$BIN_DIR/wia-time-016${NC}"
    echo -e "  • Config: ${GREEN}$INSTALL_DIR/config.json${NC}"
    echo -e "  • Documentation: ${GREEN}$INSTALL_DIR/docs/${NC}"
    echo -e "  • Data: ${GREEN}$INSTALL_DIR/data/${NC}"
    echo ""
    echo -e "${CYAN}Quick Start:${NC}"
    echo ""
    echo -e "  ${YELLOW}#${NC} List available materials"
    echo -e "  ${GREEN}\$${NC} wia-time-016 list-materials"
    echo ""
    echo -e "  ${YELLOW}#${NC} Analyze exotic matter"
    echo -e "  ${GREEN}\$${NC} wia-time-016 exotic-matter --density -1.5e-8 --quantity 1e-7"
    echo ""
    echo -e "  ${YELLOW}#${NC} Design temporal alloy"
    echo -e "  ${GREEN}\$${NC} wia-time-016 alloy-design CTA-7"
    echo ""
    echo -e "  ${YELLOW}#${NC} Calculate chrono-shield"
    echo -e "  ${GREEN}\$${NC} wia-time-016 shield-calc 10 spherical"
    echo ""
    echo -e "  ${YELLOW}#${NC} Show help"
    echo -e "  ${GREEN}\$${NC} wia-time-016 --help"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo -e "  • Specification: $INSTALL_DIR/docs/WIA-TIME-016-v1.0.md"
    echo -e "  • README: $INSTALL_DIR/docs/README.md"
    echo -e "  • Website: ${BLUE}https://wiastandards.com/time-016${NC}"
    echo ""
    echo -e "${PURPLE}弘益人間 (홍익인간) · Benefit All Humanity${NC}"
    echo ""
}

################################################################################
# Main Installation Flow
################################################################################

main() {
    print_header

    print_info "Installing WIA-TIME-016 to $INSTALL_DIR"
    echo ""

    check_dependencies
    create_directories
    install_cli
    install_docs
    install_typescript_sdk
    create_config
    create_sample_data
    verify_installation
    check_path
    print_completion
}

# Run installation
main "$@"
