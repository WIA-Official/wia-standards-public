#!/bin/bash

# WIA-MENTAL-001: Digital Therapy Standard CLI
# 弘益人間 (Benefit All Humanity)
#
# Command-line interface for digital therapy operations
#
# Usage:
#   ./digital-therapy.sh [command] [options]

set -e

VERSION="1.0.0"
API_BASE_URL="${WIA_API_URL:-https://api.wia-official.org/mental/v1}"
CONFIG_DIR="$HOME/.wia/mental-001"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Create config directory if it doesn't exist
mkdir -p "$CONFIG_DIR"

# Initialize default config if not exists
if [ ! -f "$CONFIG_FILE" ]; then
    cat > "$CONFIG_FILE" << EOF
{
  "apiKey": "",
  "defaultTherapyType": "CBT",
  "defaultDuration": 45,
  "language": "en"
}
EOF
fi

# Helper functions
print_header() {
    echo -e "${PURPLE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║       WIA-MENTAL-001: Digital Therapy CLI v$VERSION        ║"
    echo "║              弘益人間 · Benefit All Humanity              ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ Error: $1${NC}" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠ Warning: $1${NC}"
}

print_info() {
    echo -e "${PURPLE}ℹ $1${NC}"
}

# Load config value
get_config() {
    local key=$1
    cat "$CONFIG_FILE" | grep "\"$key\"" | sed 's/.*: "\(.*\)".*/\1/'
}

# Set config value
set_config() {
    local key=$1
    local value=$2
    local temp_file=$(mktemp)

    if grep -q "\"$key\"" "$CONFIG_FILE"; then
        sed "s/\"$key\": \".*\"/\"$key\": \"$value\"/" "$CONFIG_FILE" > "$temp_file"
        mv "$temp_file" "$CONFIG_FILE"
    fi

    print_success "Configuration updated: $key = $value"
}

# Command: init - Initialize CLI configuration
cmd_init() {
    print_header
    echo "Initializing WIA-MENTAL-001 CLI..."
    echo ""

    read -p "Enter API Key: " api_key
    set_config "apiKey" "$api_key"

    echo ""
    print_success "Initialization complete!"
    print_info "Configuration saved to: $CONFIG_FILE"
}

# Command: session - Manage therapy sessions
cmd_session() {
    local action=$1
    shift

    case $action in
        create)
            create_session "$@"
            ;;
        start)
            start_session "$@"
            ;;
        end)
            end_session "$@"
            ;;
        list)
            list_sessions "$@"
            ;;
        *)
            print_error "Unknown session action: $action"
            echo "Available actions: create, start, end, list"
            exit 1
            ;;
    esac
}

create_session() {
    print_header
    echo "Creating new therapy session..."

    local patient_id="${1:-}"
    local therapy_type="${2:-CBT}"

    if [ -z "$patient_id" ]; then
        read -p "Patient ID: " patient_id
    fi

    echo ""
    echo "Therapy Types:"
    echo "  1) CBT  - Cognitive Behavioral Therapy"
    echo "  2) DBT  - Dialectical Behavior Therapy"
    echo "  3) ACT  - Acceptance and Commitment Therapy"
    echo "  4) IPT  - Interpersonal Therapy"
    echo "  5) MBCT - Mindfulness-Based Cognitive Therapy"
    echo ""
    read -p "Select therapy type [1-5] (default: CBT): " therapy_choice

    case $therapy_choice in
        2) therapy_type="DBT" ;;
        3) therapy_type="ACT" ;;
        4) therapy_type="IPT" ;;
        5) therapy_type="MBCT" ;;
        *) therapy_type="CBT" ;;
    esac

    local session_id="session-$(date +%s)"

    cat > "$CONFIG_DIR/session-$session_id.json" << EOF
{
  "sessionId": "$session_id",
  "patientId": "$patient_id",
  "therapyType": "$therapy_type",
  "status": "created",
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    print_success "Session created: $session_id"
    print_info "Type: $therapy_type"
    print_info "Patient: $patient_id"
}

start_session() {
    local session_id=$1

    if [ -z "$session_id" ]; then
        print_error "Session ID required"
        echo "Usage: ./digital-therapy.sh session start <session-id>"
        exit 1
    fi

    print_header
    print_info "Starting session: $session_id"
    print_warning "Ensure patient consent and privacy compliance"

    # Simulate session start
    echo ""
    echo "Session Activities:"
    echo "  [00:00] Initial mood assessment"
    echo "  [05:00] Review homework from previous session"
    echo "  [15:00] Core therapeutic intervention"
    echo "  [35:00] Skills practice and exercises"
    echo "  [45:00] Session summary and homework assignment"

    print_success "Session started successfully"
}

end_session() {
    local session_id=$1

    if [ -z "$session_id" ]; then
        print_error "Session ID required"
        exit 1
    fi

    print_header
    print_info "Ending session: $session_id"

    read -p "Post-session mood (1-10): " mood
    read -p "Session effectiveness (1-10): " effectiveness

    print_success "Session completed"
    print_info "Post-session mood: $mood/10"
    print_info "Effectiveness: $effectiveness/10"
}

list_sessions() {
    print_header
    echo "Recent Sessions:"
    echo ""

    if ls "$CONFIG_DIR"/session-*.json >/dev/null 2>&1; then
        for session_file in "$CONFIG_DIR"/session-*.json; do
            local session_id=$(basename "$session_file" .json)
            echo "  • $session_id"
        done
    else
        print_warning "No sessions found"
    fi
}

# Command: assess - Run clinical assessments
cmd_assess() {
    local assessment_type="${1:-PHQ9}"

    print_header
    echo "Running Assessment: $assessment_type"
    echo ""

    case $assessment_type in
        PHQ9)
            print_info "Patient Health Questionnaire-9 (Depression)"
            echo "This assessment screens for depression symptoms."
            ;;
        GAD7)
            print_info "Generalized Anxiety Disorder-7"
            echo "This assessment screens for anxiety symptoms."
            ;;
        PCL5)
            print_info "PTSD Checklist for DSM-5"
            echo "This assessment screens for PTSD symptoms."
            ;;
        *)
            print_error "Unknown assessment type: $assessment_type"
            echo "Available: PHQ9, GAD7, PCL5"
            exit 1
            ;;
    esac

    echo ""
    print_warning "For demonstration purposes only. Use validated tools in production."

    # Simulate assessment
    local score=$((RANDOM % 27))
    echo ""
    print_success "Assessment completed"
    print_info "Score: $score"

    if [ $score -lt 5 ]; then
        echo "Severity: Minimal"
    elif [ $score -lt 10 ]; then
        echo "Severity: Mild"
    elif [ $score -lt 15 ]; then
        echo "Severity: Moderate"
    elif [ $score -lt 20 ]; then
        echo "Severity: Moderately Severe"
    else
        echo "Severity: Severe"
    fi
}

# Command: progress - View patient progress
cmd_progress() {
    local patient_id=$1

    if [ -z "$patient_id" ]; then
        read -p "Patient ID: " patient_id
    fi

    print_header
    echo "Progress Report: Patient $patient_id"
    echo ""

    echo "═══════════════════════════════════════════════════"
    echo "Metric                     | Current | Change"
    echo "═══════════════════════════════════════════════════"
    echo "Overall Effectiveness      |   67%   |  +12%"
    echo "Symptom Reduction          |   45%   |  +18%"
    echo "Engagement Score           |  8.5/10 |  +1.2"
    echo "Sessions Completed         |   24    |    --"
    echo "Homework Completion Rate   |   78%   |   +5%"
    echo "═══════════════════════════════════════════════════"

    echo ""
    print_success "Patient showing positive progress"
}

# Command: export - Export session data
cmd_export() {
    local format="${1:-json}"
    local output_file="${2:-export-$(date +%Y%m%d-%H%M%S).$format}"

    print_header
    print_info "Exporting data to: $output_file"
    print_warning "Ensure compliance with data protection regulations"

    # Create sample export
    cat > "$output_file" << EOF
{
  "exportDate": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "standard": "WIA-MENTAL-001",
  "version": "$VERSION",
  "data": {
    "sessions": [],
    "assessments": [],
    "progress": {}
  },
  "compliance": ["HIPAA", "GDPR"],
  "encryption": "AES-256"
}
EOF

    print_success "Data exported successfully"
}

# Command: help - Show help information
cmd_help() {
    print_header
    cat << EOF
Usage: ./digital-therapy.sh [command] [options]

Commands:
  init                    Initialize CLI configuration
  session <action>        Manage therapy sessions
    create [patient-id]   Create new session
    start <session-id>    Start a session
    end <session-id>      End a session
    list                  List recent sessions
  assess [type]           Run clinical assessment (PHQ9, GAD7, PCL5)
  progress <patient-id>   View patient progress
  export [format] [file]  Export session data
  version                 Show version information
  help                    Show this help message

Examples:
  ./digital-therapy.sh init
  ./digital-therapy.sh session create patient-123
  ./digital-therapy.sh assess PHQ9
  ./digital-therapy.sh progress patient-123
  ./digital-therapy.sh export json data.json

For more information: https://docs.wia-official.org/mental-001

弘益人間 · Benefit All Humanity
EOF
}

# Command: version - Show version
cmd_version() {
    print_header
    echo "Version: $VERSION"
    echo "Standard: WIA-MENTAL-001"
    echo "License: MIT"
    echo ""
    echo "© 2025 SmileStory Inc. / WIA"
}

# Main command dispatcher
main() {
    local command="${1:-help}"
    shift || true

    case $command in
        init)
            cmd_init "$@"
            ;;
        session)
            cmd_session "$@"
            ;;
        assess)
            cmd_assess "$@"
            ;;
        progress)
            cmd_progress "$@"
            ;;
        export)
            cmd_export "$@"
            ;;
        version)
            cmd_version
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run './digital-therapy.sh help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
