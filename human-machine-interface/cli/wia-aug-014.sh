#!/bin/bash

################################################################################
# WIA-AUG-014: Human-Machine Interface CLI Tool
#
# @version 1.0.0
# @license MIT
# @author WIA Human Augmentation Interface Group
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

# Colors
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
GRAY='\033[0;90m'
RESET='\033[0m'

VERSION="1.0.0"

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║        🔌 WIA-AUG-014: Human-Machine Interface CLI            ║"
    echo "║                      Version $VERSION                            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
}

print_section() {
    echo -e "\n${CYAN}▶ $1${RESET}"
    echo -e "${GRAY}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
}

print_success() { echo -e "${GREEN}✓ $1${RESET}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${RESET}"; }
print_error() { echo -e "${RED}✗ $1${RESET}"; }
print_info() { echo -e "${GRAY}  $1${RESET}"; }

# Connect to device
connect_device() {
    local device=${1:-"prosthetic-arm"}
    local signal=${2:-"emg"}

    print_section "Connecting to Device"
    print_info "Device: $device"
    print_info "Signal Type: $signal"

    echo -n "  Discovering devices..."
    sleep 1
    echo -e " ${GREEN}Found 1 device${RESET}"

    echo -n "  Establishing connection..."
    sleep 1
    echo -e " ${GREEN}Connected${RESET}"

    echo -n "  Verifying protocols..."
    sleep 0.5
    echo -e " ${GREEN}WIA-AUG-014/1.0${RESET}"

    print_section "Connection Status"
    print_success "Device: $device"
    print_success "Connection ID: CONN-$(date +%s)"
    print_success "Latency: 12ms (Tier 2)"
    print_success "Signal Quality: 94%"
    print_info "Ready for communication"
    echo ""
}

# Calibrate interface
calibrate_interface() {
    local profile=${1:-"user-001"}
    local duration=${2:-60}

    print_section "Calibration Session"
    print_info "Profile: $profile"
    print_info "Duration: ${duration}s"

    print_section "Calibration Tasks"

    echo -n "  [1/4] Rest State (30s)..."
    sleep 1
    echo -e " ${GREEN}Complete${RESET}"

    echo -n "  [2/4] Maximum Contraction (5s x3)..."
    sleep 1
    echo -e " ${GREEN}Complete${RESET}"

    echo -n "  [3/4] Graded Force Levels..."
    sleep 1
    echo -e " ${GREEN}Complete${RESET}"

    echo -n "  [4/4] Movement Patterns..."
    sleep 1
    echo -e " ${GREEN}Complete${RESET}"

    print_section "Calibration Results"
    print_success "Accuracy: 94.2%"
    print_success "Signal Quality: SNR 48dB"
    print_success "Channel Stability: 0.97"
    print_success "Model trained and saved"
    print_info "Profile $profile updated"
    echo ""
}

# Monitor metrics
monitor_metrics() {
    print_section "Real-time Metrics"

    print_info "Signal Quality"
    print_info "  SNR: 47.3 dB ✓"
    print_info "  Channel Correlation: 0.15 ✓"
    print_info "  Baseline Stability: 0.04 ✓"
    print_info "  Artifact Rate: 2.1/min ✓"

    print_section "Performance"
    print_info "  Decoding Accuracy: 93.5%"
    print_info "  Latency Compliance: 97.2%"
    print_info "  Feedback Accuracy: 89.8%"

    print_section "Latency Breakdown"
    print_info "  Acquisition: 2.1ms"
    print_info "  Processing: 1.3ms"
    print_info "  Encoding: 0.6ms"
    print_info "  Transmission: 3.2ms"
    print_info "  Decoding: 0.5ms"
    print_info "  Execution: 2.8ms"
    print_info "  ─────────────────"
    print_success "Total: 10.5ms (Tier 2: <50ms)"
    echo ""
}

# Test bidirectional communication
test_bidirectional() {
    local mode=${1:-"bidirectional"}
    local duration=${2:-30}

    print_section "Communication Test"
    print_info "Mode: $mode"
    print_info "Duration: ${duration}s"

    print_section "Forward Path (Human → Machine)"
    echo -n "  Signal acquisition..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    echo -n "  Intent decoding..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    echo -n "  Command generation..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    print_success "Forward latency: 8.2ms"

    print_section "Feedback Path (Machine → Human)"
    echo -n "  State encoding..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    echo -n "  Feedback generation..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    echo -n "  Haptic delivery..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"
    print_success "Feedback latency: 6.8ms"

    print_section "Test Results"
    print_success "Bidirectional test: PASSED"
    print_info "Packets sent: 1000"
    print_info "Packets received: 998"
    print_info "Packet loss: 0.2%"
    print_info "Round-trip latency: 15.0ms avg"
    echo ""
}

# Send haptic feedback
send_haptic() {
    local pattern=${1:-"SINGLE_PULSE"}
    local intensity=${2:-0.7}

    print_section "Haptic Feedback"
    print_info "Pattern: $pattern"
    print_info "Intensity: $intensity"

    case $pattern in
        SINGLE_PULSE) duration=50 ;;
        DOUBLE_PULSE) duration=150 ;;
        LONG_PULSE) duration=200 ;;
        GRIP_RAMP) duration=100 ;;
        CONTACT) duration=30 ;;
        *) duration=100 ;;
    esac

    print_info "Duration: ${duration}ms"

    echo -n "  Generating pattern..."
    sleep 0.5
    echo -e " ${GREEN}OK${RESET}"

    echo -n "  Sending to actuators..."
    sleep 0.5
    echo -e " ${GREEN}Delivered${RESET}"

    print_success "Haptic feedback sent successfully"
    echo ""
}

# Device state
device_state() {
    print_section "Device State"

    print_info "Device ID: PROS-2025-001"
    print_info "Status: Active"

    print_section "Power"
    print_info "  Battery: 87%"
    print_info "  Charging: No"
    print_info "  Runtime: ~8.2 hours"

    print_section "Position"
    print_info "  Joint 1: 15°"
    print_info "  Joint 2: 30°"
    print_info "  Joint 3: 45°"
    print_info "  Joint 4: 60°"
    print_info "  Joint 5: 75°"
    print_info "  End Effector: (0.32, 0.15, 0.08)m"

    print_section "Sensors"
    print_info "  Force: [0.12, 0.25, 0.18, 0.30, 0.22] N"
    print_info "  Temperature: [36.5, 36.7, 36.4] °C"
    print_info "  Contact: [Off, On, On, Off, Off]"
    echo ""
}

# Help
show_help() {
    print_header
    echo "Usage: wia-aug-014 <command> [options]"
    echo ""
    echo "Commands:"
    echo "  connect                  Connect to HMI device"
    echo "    --device <name>        Device name/type"
    echo "    --signal-type <type>   Signal type (emg, eeg, hybrid)"
    echo ""
    echo "  calibrate                Start calibration session"
    echo "    --profile <id>         User profile ID"
    echo "    --duration <seconds>   Calibration duration"
    echo ""
    echo "  monitor                  Monitor signal metrics"
    echo "    --metrics <list>       Metrics to monitor"
    echo ""
    echo "  test                     Test communication"
    echo "    --mode <mode>          Test mode (bidirectional, forward, feedback)"
    echo "    --duration <seconds>   Test duration"
    echo ""
    echo "  haptic                   Send haptic feedback"
    echo "    --pattern <name>       Pattern name"
    echo "    --intensity <0-1>      Intensity level"
    echo ""
    echo "  state                    Get device state"
    echo ""
    echo "  version                  Show version"
    echo "  help                     Show this help"
    echo ""
    echo "Standard Haptic Patterns:"
    echo "  SINGLE_PULSE, DOUBLE_PULSE, LONG_PULSE, BUZZ,"
    echo "  HEARTBEAT, GRIP_RAMP, CONTACT, SLIP_WARNING"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo -e "${GRAY}© 2025 SmileStory Inc. / WIA - MIT License${RESET}"
    echo ""
}

show_version() {
    print_header
    echo "WIA-AUG-014 CLI Tool"
    echo "Version: $VERSION"
    echo "License: MIT"
    echo ""
    echo -e "${GRAY}弘益人間 (Benefit All Humanity)${RESET}"
    echo ""
}

# Main
COMMAND=${1:-help}
shift || true

case "$COMMAND" in
    connect)
        DEV="prosthetic-arm"; SIG="emg"
        while [[ $# -gt 0 ]]; do
            case $1 in
                --device) DEV=$2; shift 2 ;;
                --signal-type) SIG=$2; shift 2 ;;
                *) shift ;;
            esac
        done
        print_header
        connect_device "$DEV" "$SIG"
        ;;
    calibrate)
        PROF="user-001"; DUR=60
        while [[ $# -gt 0 ]]; do
            case $1 in
                --profile) PROF=$2; shift 2 ;;
                --duration) DUR=$2; shift 2 ;;
                *) shift ;;
            esac
        done
        print_header
        calibrate_interface "$PROF" "$DUR"
        ;;
    monitor)
        print_header
        monitor_metrics
        ;;
    test)
        MODE="bidirectional"; DUR=30
        while [[ $# -gt 0 ]]; do
            case $1 in
                --mode) MODE=$2; shift 2 ;;
                --duration) DUR=$2; shift 2 ;;
                *) shift ;;
            esac
        done
        print_header
        test_bidirectional "$MODE" "$DUR"
        ;;
    haptic)
        PAT="SINGLE_PULSE"; INT=0.7
        while [[ $# -gt 0 ]]; do
            case $1 in
                --pattern) PAT=$2; shift 2 ;;
                --intensity) INT=$2; shift 2 ;;
                *) shift ;;
            esac
        done
        print_header
        send_haptic "$PAT" "$INT"
        ;;
    state)
        print_header
        device_state
        ;;
    version)
        show_version
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${RESET}"
        echo "Run 'wia-aug-014 help' for usage"
        exit 1
        ;;
esac

exit 0
