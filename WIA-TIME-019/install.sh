#!/bin/bash

################################################################################
# WIA-TIME-019: Timeline Synchronization - Installation Script
#
# @version 1.0.0
# @license MIT
# @author WIA Time Research Group
#
# 弘益人間 (Benefit All Humanity)
#
# This script installs the WIA-TIME-019 Timeline Synchronization standard
# including CLI tools and TypeScript SDK.
################################################################################

set -e

# Colors
VIOLET='\033[0;35m'
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

# Constants
VERSION="1.0.0"
INSTALL_DIR="/usr/local/bin"
LIB_DIR="/usr/local/lib/wia-time-019"

# Helper functions
print_header() {
    echo -e "${VIOLET}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║         🔄  WIA-TIME-019: Timeline Synchronization            ║"
    echo "║                    Installation Script                        ║"
    echo "║                      Version $VERSION                            ║"
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

    local missing_deps=()

    # Check for required commands
    if ! command -v bash &> /dev/null; then
        missing_deps+=("bash")
    fi

    if ! command -v jq &> /dev/null; then
        print_warning "jq not found (optional, for JSON processing)"
        print_info "Install with: sudo apt-get install jq (Debian/Ubuntu)"
        print_info "           or: brew install jq (macOS)"
    else
        print_success "jq found: $(jq --version)"
    fi

    if ! command -v bc &> /dev/null; then
        print_warning "bc not found (optional, for calculations)"
        print_info "Install with: sudo apt-get install bc (Debian/Ubuntu)"
        print_info "           or: brew install bc (macOS)"
    else
        print_success "bc found"
    fi

    # Check for Node.js (for TypeScript SDK)
    if command -v node &> /dev/null; then
        local node_version=$(node --version)
        print_success "Node.js found: $node_version"

        if command -v npm &> /dev/null; then
            local npm_version=$(npm --version)
            print_success "npm found: v$npm_version"
        else
            print_warning "npm not found (needed for TypeScript SDK)"
        fi
    else
        print_warning "Node.js not found (needed for TypeScript SDK)"
        print_info "Visit: https://nodejs.org/"
    fi

    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing required dependencies: ${missing_deps[*]}"
        exit 1
    fi

    print_success "All required prerequisites met"
}

# Install CLI tool
install_cli() {
    print_section "Installing CLI Tool"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local cli_script="$script_dir/cli/wia-time-019.sh"

    if [ ! -f "$cli_script" ]; then
        print_error "CLI script not found: $cli_script"
        exit 1
    fi

    # Make executable
    chmod +x "$cli_script"
    print_success "Made CLI script executable"

    # Copy to install directory (requires sudo for /usr/local/bin)
    if [ -w "$INSTALL_DIR" ]; then
        cp "$cli_script" "$INSTALL_DIR/wia-time-019"
        print_success "Installed CLI to $INSTALL_DIR/wia-time-019"
    else
        print_info "Installing to $INSTALL_DIR requires sudo..."
        sudo cp "$cli_script" "$INSTALL_DIR/wia-time-019"
        print_success "Installed CLI to $INSTALL_DIR/wia-time-019 (with sudo)"
    fi

    # Verify installation
    if command -v wia-time-019 &> /dev/null; then
        print_success "CLI tool installed successfully"
        print_info "Run: wia-time-019 --version"
    else
        print_warning "CLI tool installed but not in PATH"
        print_info "You may need to add $INSTALL_DIR to your PATH"
    fi
}

# Install TypeScript SDK
install_typescript_sdk() {
    print_section "Installing TypeScript SDK"

    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local ts_dir="$script_dir/api/typescript"

    if [ ! -d "$ts_dir" ]; then
        print_error "TypeScript directory not found: $ts_dir"
        return 1
    fi

    if ! command -v npm &> /dev/null; then
        print_warning "npm not found, skipping TypeScript SDK installation"
        print_info "Install Node.js and npm to use the TypeScript SDK"
        return 0
    fi

    cd "$ts_dir"

    # Install dependencies
    print_info "Installing dependencies..."
    if npm install --silent; then
        print_success "Dependencies installed"
    else
        print_warning "Failed to install dependencies"
        return 1
    fi

    # Build SDK
    print_info "Building SDK..."
    if npm run build --silent; then
        print_success "SDK built successfully"
    else
        print_warning "Failed to build SDK"
        return 1
    fi

    # Link globally (optional)
    print_info "Linking SDK globally..."
    if npm link --silent 2>/dev/null; then
        print_success "SDK linked globally as @wia/time-019"
        print_info "Import with: import { TimelineSynchronizer } from '@wia/time-019';"
    else
        print_warning "Could not link globally (may require sudo)"
        print_info "You can still use it locally or publish to npm"
    fi

    cd - > /dev/null
}

# Create example configurations
create_examples() {
    print_section "Creating Example Configurations"

    local examples_dir="$HOME/.wia-time-019/examples"
    mkdir -p "$examples_dir"

    # Example 1: Basic sync
    cat > "$examples_dir/basic-sync.sh" << 'EOF'
#!/bin/bash
# Basic timeline synchronization example

# Initialize synchronizer
wia-time-019 init --reference alpha-001 --precision nanosecond

# Synchronize a timeline
wia-time-019 sync --timeline beta-002 --correct-drift --show-metrics

# Check status
wia-time-019 status
EOF

    chmod +x "$examples_dir/basic-sync.sh"

    # Example 2: Continuous monitoring
    cat > "$examples_dir/continuous-monitoring.sh" << 'EOF'
#!/bin/bash
# Continuous timeline monitoring example

# Monitor multiple timelines
wia-time-019 monitor \
  --timelines alpha-001,beta-002,gamma-003 \
  --interval 1000 \
  --alert-on-drift
EOF

    chmod +x "$examples_dir/continuous-monitoring.sh"

    # Example 3: TypeScript usage
    cat > "$examples_dir/example.ts" << 'EOF'
import { createSynchronizer } from '@wia/time-019';

async function main() {
  // Create synchronizer
  const sync = await createSynchronizer({
    referenceTimeline: 'alpha-001',
    syncMode: 'continuous',
    precision: 'nanosecond',
    driftTolerance: 1000,
  });

  // Synchronize timeline
  const result = await sync.syncTimeline({
    timelineId: 'beta-002',
    strategy: 'clock-sync',
    correctDrift: true,
    mergeOnConflict: false,
    priority: 'normal',
  });

  console.log(`Sync status: ${result.status}`);
  console.log(`Drift corrected: ${result.driftCorrected}ns`);
  console.log(`Quality: ${result.quality * 100}%`);

  // Detect divergence
  const divergence = await sync.detectDivergence({
    timelineA: 'alpha-001',
    timelineB: 'beta-002',
    threshold: 0.05,
  });

  if (divergence.detected) {
    console.log(`Divergence detected: ${divergence.magnitude}`);
  }

  // Close synchronizer
  await sync.close();
}

main().catch(console.error);
EOF

    print_success "Created example scripts in $examples_dir"
    print_info "basic-sync.sh - Basic synchronization"
    print_info "continuous-monitoring.sh - Continuous monitoring"
    print_info "example.ts - TypeScript usage example"
}

# Setup completion
setup_completion() {
    print_section "Setting Up Shell Completion"

    local completion_script="$HOME/.wia-time-019/completion.bash"
    mkdir -p "$(dirname "$completion_script")"

    cat > "$completion_script" << 'EOF'
# Bash completion for wia-time-019

_wia_time_019_completions() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Main commands
    local commands="init sync monitor divergence merge status version help"

    # Complete commands
    if [ $COMP_CWORD -eq 1 ]; then
        COMPREPLY=( $(compgen -W "${commands}" -- ${cur}) )
        return 0
    fi

    # Complete options based on command
    case "${COMP_WORDS[1]}" in
        init)
            opts="--reference --precision"
            ;;
        sync)
            opts="--timeline --strategy --correct-drift --show-metrics"
            ;;
        monitor)
            opts="--timelines --interval --alert-on-drift"
            ;;
        divergence)
            opts="--timeline-a --timeline-b --threshold --detailed"
            ;;
        merge)
            opts="--source --target --strategy --resolve-conflicts"
            ;;
        status)
            opts="--all"
            ;;
        *)
            opts=""
            ;;
    esac

    COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
    return 0
}

complete -F _wia_time_019_completions wia-time-019
EOF

    print_success "Shell completion script created"
    print_info "Add to ~/.bashrc: source $completion_script"

    # Offer to add to .bashrc
    if [ -f "$HOME/.bashrc" ]; then
        if grep -q "wia-time-019/completion.bash" "$HOME/.bashrc"; then
            print_info "Already added to ~/.bashrc"
        else
            read -p "Add to ~/.bashrc? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                echo "" >> "$HOME/.bashrc"
                echo "# WIA-TIME-019 completion" >> "$HOME/.bashrc"
                echo "[ -f $completion_script ] && source $completion_script" >> "$HOME/.bashrc"
                print_success "Added to ~/.bashrc"
                print_info "Run: source ~/.bashrc"
            fi
        fi
    fi
}

# Print installation summary
print_summary() {
    print_section "Installation Summary"

    print_success "WIA-TIME-019 Timeline Synchronization installed!"

    echo ""
    echo "Quick Start:"
    echo ""
    echo "  1. Initialize synchronizer:"
    echo "     $ wia-time-019 init --reference alpha-001"
    echo ""
    echo "  2. Synchronize a timeline:"
    echo "     $ wia-time-019 sync --timeline beta-002"
    echo ""
    echo "  3. Monitor continuous sync:"
    echo "     $ wia-time-019 monitor --timelines alpha-001,beta-002"
    echo ""
    echo "  4. Detect divergence:"
    echo "     $ wia-time-019 divergence --timeline-a alpha-001 --timeline-b beta-002"
    echo ""
    echo "  5. Check status:"
    echo "     $ wia-time-019 status"
    echo ""

    if command -v node &> /dev/null && command -v npm &> /dev/null; then
        echo "TypeScript SDK:"
        echo ""
        echo "  Import: import { TimelineSynchronizer } from '@wia/time-019';"
        echo "  Example: $HOME/.wia-time-019/examples/example.ts"
        echo ""
    fi

    echo "Documentation:"
    echo "  Specification: ./spec/WIA-TIME-019-v1.0.md"
    echo "  README: ./README.md"
    echo ""

    echo "For help:"
    echo "  $ wia-time-019 help"
    echo ""

    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

# Main installation
main() {
    print_header

    # Check if running as root (not recommended)
    if [ "$EUID" -eq 0 ]; then
        print_warning "Running as root is not recommended"
        print_info "Consider running as a regular user with sudo when needed"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi

    # Run installation steps
    check_prerequisites
    install_cli
    install_typescript_sdk
    create_examples
    setup_completion
    print_summary

    print_success "Installation complete!"
}

# Run main installation
main "$@"
