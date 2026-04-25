#!/bin/bash

################################################################################
# WIA-AUG-007: Bionic Limb - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Human Augmentation Bionics Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║      🦾 WIA-AUG-007: Bionic Limb Installer                    ║"
    echo "║                      Version 1.0.0                             ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() { echo -e "\n${CYAN}▶ $1${RESET}"; }
print_success() { echo -e "${GREEN}✓ $1${RESET}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${RESET}"; }
print_error() { echo -e "${RED}✗ $1${RESET}"; }
print_info() { echo -e "${GRAY}  $1${RESET}"; }

check_prerequisites() {
    print_section "Checking Prerequisites"

    if command -v bash &> /dev/null; then
        print_success "Bash: $(bash --version | head -n1 | cut -d' ' -f1-4)"
    else
        print_error "Bash not found"
        exit 1
    fi

    if command -v bc &> /dev/null; then
        print_success "bc: installed"
    else
        print_warning "bc not found (install for calculations)"
    fi

    if command -v node &> /dev/null; then
        print_success "Node.js: $(node --version)"
    else
        print_warning "Node.js not found (optional for TypeScript SDK)"
    fi

    if command -v npm &> /dev/null; then
        print_success "npm: $(npm --version)"
    else
        print_warning "npm not found (optional for TypeScript SDK)"
    fi
}

install_cli() {
    print_section "Installing CLI Tool"

    local cli_path="$SCRIPT_DIR/cli/wia-aug-007.sh"
    local install_dir="/usr/local/bin"

    if [ ! -f "$cli_path" ]; then
        print_error "CLI script not found at $cli_path"
        return 1
    fi

    chmod +x "$cli_path"
    print_success "Made CLI executable"

    if [ -w "$install_dir" ]; then
        cp "$cli_path" "$install_dir/wia-aug-007"
    else
        sudo cp "$cli_path" "$install_dir/wia-aug-007"
    fi
    print_success "Installed to $install_dir/wia-aug-007"
}

install_typescript() {
    print_section "Installing TypeScript SDK"

    local ts_dir="$SCRIPT_DIR/api/typescript"

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK"
        return 0
    fi

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found"
        return 1
    fi

    cd "$ts_dir"

    print_info "Installing dependencies..."
    npm install 2>&1 | grep -v "^npm WARN" || true

    print_info "Building SDK..."
    npm run build 2>/dev/null || print_warning "Build skipped (tsup not configured)"

    print_success "TypeScript SDK ready"
    cd "$SCRIPT_DIR"
}

install_docs() {
    print_section "Installing Documentation"

    local docs_dir="$HOME/.wia/aug-007/docs"
    mkdir -p "$docs_dir"

    if [ -f "$SCRIPT_DIR/README.md" ]; then
        cp "$SCRIPT_DIR/README.md" "$docs_dir/"
        print_success "Installed README"
    fi

    if [ -d "$SCRIPT_DIR/spec" ]; then
        cp -r "$SCRIPT_DIR/spec" "$docs_dir/"
        print_success "Installed specifications"
    fi

    print_info "Documentation at: $docs_dir"
}

create_examples() {
    print_section "Creating Example Scripts"

    local examples_dir="$HOME/.wia/aug-007/examples"
    mkdir -p "$examples_dir"

    # Create example 1: Classify limb
    cat > "$examples_dir/01-classify-limb.sh" << 'EOF'
#!/bin/bash
# Example: Classify a bionic forearm

wia-aug-007 classify \
    --type FOREARM \
    --control pattern_recognition \
    --dof 8
EOF

    # Create example 2: Calibrate control
    cat > "$examples_dir/02-calibrate-control.sh" << 'EOF'
#!/bin/bash
# Example: Calibrate myoelectric control

wia-aug-007 calibrate \
    --limb-id BL-2025-001 \
    --method myoelectric
EOF

    # Create example 3: Test feedback
    cat > "$examples_dir/03-test-feedback.sh" << 'EOF'
#!/bin/bash
# Example: Test pressure feedback

wia-aug-007 feedback \
    --type pressure \
    --intensity 0.75 \
    --location palm
EOF

    # Create example 4: Analyze gait
    cat > "$examples_dir/04-analyze-gait.sh" << 'EOF'
#!/bin/bash
# Example: Analyze gait for lower limb

wia-aug-007 gait \
    --limb-id BL-2025-002 \
    --duration 60
EOF

    # Create example 5: Select grip
    cat > "$examples_dir/05-select-grip.sh" << 'EOF'
#!/bin/bash
# Example: Select power grip

wia-aug-007 grip \
    --pattern power_grip \
    --force 120
EOF

    # Make examples executable
    chmod +x "$examples_dir"/*.sh

    print_success "Created $(ls "$examples_dir" | wc -l) example scripts"
    print_info "Examples at: $examples_dir"
}

run_tests() {
    print_section "Running Installation Tests"

    print_info "Testing CLI installation..."
    if command -v wia-aug-007 &> /dev/null; then
        print_success "CLI command available"

        print_info "Testing version command..."
        wia-aug-007 version &> /dev/null && print_success "Version command works"
    else
        print_error "CLI command not found in PATH"
        return 1
    fi

    if [ -d "$SCRIPT_DIR/api/typescript" ] && command -v npm &> /dev/null; then
        cd "$SCRIPT_DIR/api/typescript"
        if [ -d "dist" ]; then
            print_success "TypeScript build verified"
        else
            print_warning "TypeScript build not found"
        fi
        cd "$SCRIPT_DIR"
    fi
}

show_completion() {
    print_section "Installation Complete!"

    echo ""
    echo -e "${GREEN}🦾 WIA-AUG-007 Bionic Limb Standard installed!${RESET}"
    echo ""
    echo "Available Commands:"
    echo -e "${CYAN}  wia-aug-007 classify${RESET}    - Classify bionic limb"
    echo -e "${CYAN}  wia-aug-007 calibrate${RESET}   - Calibrate control system"
    echo -e "${CYAN}  wia-aug-007 feedback${RESET}    - Test sensory feedback"
    echo -e "${CYAN}  wia-aug-007 gait${RESET}        - Analyze gait (lower limbs)"
    echo -e "${CYAN}  wia-aug-007 grip${RESET}        - Select grip pattern"
    echo -e "${CYAN}  wia-aug-007 maintenance${RESET} - Schedule maintenance"
    echo ""
    echo "TypeScript SDK:"
    echo -e "${CYAN}  npm install @wia/aug-007${RESET}"
    echo ""
    echo "Documentation:"
    echo -e "${GRAY}  $HOME/.wia/aug-007/docs/${RESET}"
    echo ""
    echo "Examples:"
    echo -e "${GRAY}  $HOME/.wia/aug-007/examples/${RESET}"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA${RESET}"
    echo ""
}

main() {
    print_header
    check_prerequisites
    install_cli
    install_typescript
    install_docs
    create_examples
    run_tests
    show_completion
}

main "$@"
