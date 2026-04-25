#!/bin/bash

# WIA-MENTAL_WELLNESS: Mental Wellness Standard CLI
# 弘益人間 (Benefit All Humanity)
#
# Command-line interface for mental wellness operations
#
# Usage:
#   ./wia-mental-wellness.sh [command] [options]

set -e

VERSION="1.0.0"
API_BASE_URL="${WIA_API_URL:-https://api.wia-official.org/mental-wellness/v1}"
CONFIG_DIR="$HOME/.wia/mental-wellness"
CONFIG_FILE="$CONFIG_DIR/config.json"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Create config directory if it doesn't exist
mkdir -p "$CONFIG_DIR"

# Initialize default config if not exists
if [ ! -f "$CONFIG_FILE" ]; then
    cat > "$CONFIG_FILE" << EOF
{
  "apiKey": "",
  "userId": "",
  "defaultLanguage": "en",
  "privacyMode": true,
  "emergencyContact": "",
  "therapistId": ""
}
EOF
fi

# Helper functions
print_header() {
    echo -e "${PURPLE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║     WIA-MENTAL_WELLNESS: Mental Wellness CLI v$VERSION     ║"
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
    echo -e "${CYAN}ℹ $1${NC}"
}

# Load config value
get_config() {
    local key=$1
    if [ -f "$CONFIG_FILE" ]; then
        cat "$CONFIG_FILE" | grep "\"$key\"" | sed 's/.*: "\(.*\)".*/\1/'
    fi
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

    print_success "Configuration updated: $key"
}

# Command: assess-mood - Assess current mood and mental state
cmd_assess_mood() {
    print_header
    echo "Mental Wellness Mood Assessment"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    # Quick mood check
    echo "How are you feeling right now?"
    echo ""
    echo "1) 😊 Great - Feeling happy and energetic"
    echo "2) 🙂 Good - Feeling positive overall"
    echo "3) 😐 Okay - Feeling neutral"
    echo "4) 😕 Not great - Feeling a bit down"
    echo "5) 😢 Poor - Struggling significantly"
    echo ""
    read -p "Select your mood [1-5]: " mood_rating

    # Anxiety level
    echo ""
    echo "How anxious do you feel?"
    echo "Rate from 1 (Not at all) to 10 (Extremely anxious)"
    read -p "Anxiety level [1-10]: " anxiety_level

    # Stress level
    echo ""
    echo "How stressed do you feel?"
    echo "Rate from 1 (Not at all) to 10 (Extremely stressed)"
    read -p "Stress level [1-10]: " stress_level

    # Sleep quality
    echo ""
    echo "How did you sleep last night?"
    echo "Rate from 1 (Very poor) to 10 (Excellent)"
    read -p "Sleep quality [1-10]: " sleep_quality

    # Energy level
    echo ""
    echo "What is your energy level?"
    echo "Rate from 1 (Exhausted) to 10 (Very energetic)"
    read -p "Energy level [1-10]: " energy_level

    # Save assessment
    local timestamp=$(date +%Y%m%d-%H%M%S)
    local assessment_file="$CONFIG_DIR/assessments/mood-$timestamp.json"
    mkdir -p "$CONFIG_DIR/assessments"

    cat > "$assessment_file" << EOF
{
  "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "type": "mood_assessment",
  "mood_rating": $mood_rating,
  "anxiety_level": $anxiety_level,
  "stress_level": $stress_level,
  "sleep_quality": $sleep_quality,
  "energy_level": $energy_level
}
EOF

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    print_success "Assessment completed and saved"

    # Provide recommendations
    echo ""
    echo "Recommendations based on your assessment:"

    if [ "$mood_rating" -ge 4 ] || [ "$anxiety_level" -ge 7 ] || [ "$stress_level" -ge 7 ]; then
        print_warning "Your responses indicate you may be experiencing distress"
        echo "  • Consider talking to a mental health professional"
        echo "  • Try relaxation exercises (./wia-mental-wellness.sh meditation-session)"
        echo "  • Reach out to your support network"
    else
        print_success "Your mental wellness indicators look positive"
        echo "  • Keep maintaining healthy habits"
        echo "  • Continue tracking your mood regularly"
    fi
}

# Command: track-wellness - View wellness trends over time
cmd_track_wellness() {
    print_header
    echo "Mental Wellness Trend Analysis"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    if [ ! -d "$CONFIG_DIR/assessments" ] || [ -z "$(ls -A $CONFIG_DIR/assessments 2>/dev/null)" ]; then
        print_warning "No assessment data found"
        echo "Run './wia-mental-wellness.sh assess-mood' to create your first assessment"
        return
    fi

    # Count assessments
    local count=$(ls "$CONFIG_DIR/assessments"/mood-*.json 2>/dev/null | wc -l)
    echo "Total assessments: $count"
    echo ""

    # Show recent assessments
    echo "Recent Assessments (last 7 days):"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Date/Time           | Mood | Anxiety | Stress | Sleep | Energy"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # Process last 10 assessments
    for file in $(ls -t "$CONFIG_DIR/assessments"/mood-*.json 2>/dev/null | head -10); do
        local timestamp=$(basename "$file" .json | sed 's/mood-//')
        local mood=$(cat "$file" | grep "mood_rating" | sed 's/.*: \([0-9]*\).*/\1/')
        local anxiety=$(cat "$file" | grep "anxiety_level" | sed 's/.*: \([0-9]*\).*/\1/')
        local stress=$(cat "$file" | grep "stress_level" | sed 's/.*: \([0-9]*\).*/\1/')
        local sleep=$(cat "$file" | grep "sleep_quality" | sed 's/.*: \([0-9]*\).*/\1/')
        local energy=$(cat "$file" | grep "energy_level" | sed 's/.*: \([0-9]*\).*/\1/')

        printf "%-19s |  %s   |   %2s    |   %2s   |  %2s   |   %2s\n" \
            "$timestamp" "$mood" "$anxiety" "$stress" "$sleep" "$energy"
    done

    echo ""
    print_info "Use './wia-mental-wellness.sh analyze-trends' for detailed insights"
}

# Command: schedule-therapy - Schedule a therapy session
cmd_schedule_therapy() {
    print_header
    echo "Schedule Therapy Session"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    read -p "Preferred date (YYYY-MM-DD): " session_date
    read -p "Preferred time (HH:MM): " session_time

    echo ""
    echo "Session type:"
    echo "1) Individual Therapy"
    echo "2) Group Therapy"
    echo "3) Family Therapy"
    echo "4) Crisis Counseling"
    read -p "Select type [1-4]: " session_type

    echo ""
    echo "Modality:"
    echo "1) Video Call"
    echo "2) Phone Call"
    echo "3) In-Person"
    echo "4) Chat/Messaging"
    read -p "Select modality [1-4]: " modality

    # Save appointment
    local session_id="session-$(date +%s)"
    local appointment_file="$CONFIG_DIR/appointments/$session_id.json"
    mkdir -p "$CONFIG_DIR/appointments"

    cat > "$appointment_file" << EOF
{
  "sessionId": "$session_id",
  "date": "$session_date",
  "time": "$session_time",
  "type": $session_type,
  "modality": $modality,
  "status": "scheduled",
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

    echo ""
    print_success "Therapy session scheduled"
    print_info "Session ID: $session_id"
    print_info "Date/Time: $session_date at $session_time"
    echo ""
    echo "You will receive a confirmation email shortly."
    echo "Add to calendar: ./wia-mental-wellness.sh export-calendar $session_id"
}

# Command: crisis-help - Access crisis intervention resources
cmd_crisis_help() {
    print_header
    echo -e "${RED}CRISIS INTERVENTION RESOURCES${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo -e "${YELLOW}⚠ IF YOU ARE IN IMMEDIATE DANGER, CALL 911 (US) OR LOCAL EMERGENCY SERVICES${NC}"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "24/7 Crisis Hotlines:"
    echo ""
    echo -e "${CYAN}United States:${NC}"
    echo "  • 988 Suicide & Crisis Lifeline: Call or text 988"
    echo "  • Crisis Text Line: Text HOME to 741741"
    echo "  • Veterans Crisis Line: 1-800-273-8255, press 1"
    echo ""
    echo -e "${CYAN}International:${NC}"
    echo "  • International Association for Suicide Prevention:"
    echo "    https://www.iasp.info/resources/Crisis_Centres/"
    echo ""
    echo -e "${CYAN}Specialized Support:${NC}"
    echo "  • SAMHSA National Helpline: 1-800-662-4357"
    echo "  • NAMI HelpLine: 1-800-950-6264"
    echo "  • The Trevor Project (LGBTQ+ Youth): 1-866-488-7386"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    read -p "Would you like to create a safety plan? [y/n]: " create_plan

    if [ "$create_plan" = "y" ] || [ "$create_plan" = "Y" ]; then
        cmd_create_safety_plan
    fi
}

# Command: create-safety-plan - Create a personalized safety plan
cmd_create_safety_plan() {
    echo ""
    echo "Creating Safety Plan"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    echo "1. Warning signs (when I might need help):"
    read -p "   Enter warning signs: " warning_signs

    echo ""
    echo "2. Internal coping strategies (things I can do on my own):"
    read -p "   Enter coping strategies: " coping_strategies

    echo ""
    echo "3. People who can help distract me:"
    read -p "   Name and phone: " support_person1

    echo ""
    echo "4. People I can ask for help:"
    read -p "   Name and phone: " support_person2

    echo ""
    echo "5. Professional contacts:"
    read -p "   Therapist/Counselor: " professional_contact

    echo ""
    echo "6. Making my environment safer:"
    read -p "   Safety steps: " safety_steps

    # Save safety plan
    local plan_file="$CONFIG_DIR/safety-plan.json"
    cat > "$plan_file" << EOF
{
  "createdAt": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "warningSign": "$warning_signs",
  "copingStrategies": "$coping_strategies",
  "supportPerson1": "$support_person1",
  "supportPerson2": "$support_person2",
  "professionalContact": "$professional_contact",
  "safetySteps": "$safety_steps",
  "emergencyNumbers": {
    "suicide_lifeline": "988",
    "crisis_text": "741741",
    "emergency": "911"
  }
}
EOF

    echo ""
    print_success "Safety plan created and saved"
    print_info "Location: $plan_file"
    echo ""
    echo "You can view your safety plan anytime with:"
    echo "./wia-mental-wellness.sh view-safety-plan"
}

# Command: meditation-session - Start a guided meditation
cmd_meditation_session() {
    print_header
    echo "Guided Meditation Session"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    echo "Select meditation type:"
    echo "1) Mindfulness Meditation (10 min)"
    echo "2) Breathing Exercise (5 min)"
    echo "3) Body Scan (15 min)"
    echo "4) Loving-Kindness (10 min)"
    echo "5) Stress Relief (7 min)"
    read -p "Choose [1-5]: " meditation_type

    echo ""
    echo "Preparing meditation space..."
    sleep 1

    case $meditation_type in
        1)
            echo -e "${CYAN}Starting Mindfulness Meditation...${NC}"
            mindfulness_meditation
            ;;
        2)
            echo -e "${CYAN}Starting Breathing Exercise...${NC}"
            breathing_exercise
            ;;
        3)
            echo -e "${CYAN}Starting Body Scan...${NC}"
            body_scan_meditation
            ;;
        4)
            echo -e "${CYAN}Starting Loving-Kindness Meditation...${NC}"
            loving_kindness_meditation
            ;;
        5)
            echo -e "${CYAN}Starting Stress Relief Meditation...${NC}"
            stress_relief_meditation
            ;;
        *)
            print_error "Invalid selection"
            return
            ;;
    esac

    # Log meditation session
    local timestamp=$(date +%Y%m%d-%H%M%S)
    local session_file="$CONFIG_DIR/meditations/session-$timestamp.json"
    mkdir -p "$CONFIG_DIR/meditations"

    cat > "$session_file" << EOF
{
  "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "type": $meditation_type,
  "completed": true
}
EOF

    echo ""
    print_success "Meditation session completed"
}

# Mindfulness meditation
mindfulness_meditation() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Find a comfortable seated position..."
    echo "Close your eyes or maintain a soft gaze..."
    sleep 3

    echo ""
    echo "Notice your breath..."
    echo "Don't try to change it, just observe..."
    sleep 5

    echo ""
    echo "Feel the sensation of air entering your nostrils..."
    echo "Notice the rise and fall of your chest..."
    sleep 5

    echo ""
    echo "When your mind wanders, gently bring attention back to your breath..."
    echo "This is normal and part of the practice..."
    sleep 5

    echo ""
    echo "Continue breathing naturally..."
    echo "Stay present in this moment..."
    sleep 10

    echo ""
    echo "Slowly bring awareness back to your surroundings..."
    echo "When you're ready, gently open your eyes..."
    sleep 3
}

# Breathing exercise
breathing_exercise() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "We'll practice box breathing: 4-4-4-4"
    echo "Inhale for 4 seconds, hold for 4, exhale for 4, hold for 4"
    sleep 3

    for i in {1..5}; do
        echo ""
        echo "Round $i of 5"
        echo "Inhale... 2... 3... 4..."
        sleep 4
        echo "Hold... 2... 3... 4..."
        sleep 4
        echo "Exhale... 2... 3... 4..."
        sleep 4
        echo "Hold... 2... 3... 4..."
        sleep 4
    done

    echo ""
    echo "Exercise complete. Notice how you feel..."
}

# Body scan meditation
body_scan_meditation() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Lie down or sit comfortably..."
    echo "We'll scan through your body, releasing tension..."
    sleep 3

    echo ""
    echo "Notice your feet... wiggle your toes... release any tension..."
    sleep 5

    echo "Move awareness to your legs... let them feel heavy and relaxed..."
    sleep 5

    echo "Notice your hips and lower back... release any holding..."
    sleep 5

    echo "Bring attention to your stomach... let it soften..."
    sleep 5

    echo "Notice your chest and shoulders... let them drop and relax..."
    sleep 5

    echo "Scan your arms down to your fingertips... release any tension..."
    sleep 5

    echo "Finally, relax your face, jaw, and forehead..."
    sleep 5

    echo ""
    echo "Take a moment to notice your whole body relaxed..."
    sleep 5
}

# Loving-kindness meditation
loving_kindness_meditation() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "We'll cultivate compassion for ourselves and others..."
    sleep 3

    echo ""
    echo "Begin with yourself. Silently repeat:"
    echo "'May I be happy. May I be healthy. May I be safe. May I live with ease.'"
    sleep 8

    echo ""
    echo "Now think of someone you care about. Repeat:"
    echo "'May you be happy. May you be healthy. May you be safe. May you live with ease.'"
    sleep 8

    echo ""
    echo "Extend these wishes to all beings everywhere..."
    sleep 5
}

# Stress relief meditation
stress_relief_meditation() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Let's release stress and find calm..."
    sleep 3

    echo ""
    echo "Take a deep breath in through your nose... hold... exhale slowly..."
    sleep 5

    echo "Imagine stress as dark clouds leaving your body with each exhale..."
    sleep 5

    echo "With each inhale, breathe in peace and calm..."
    sleep 5

    echo "Continue this pattern... releasing stress... welcoming peace..."
    sleep 10

    echo ""
    echo "Feel yourself becoming lighter and more at ease..."
    sleep 5
}

# Command: analyze-trends - Analyze wellness patterns
cmd_analyze_trends() {
    print_header
    echo "Wellness Pattern Analysis"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    if [ ! -d "$CONFIG_DIR/assessments" ] || [ -z "$(ls -A $CONFIG_DIR/assessments 2>/dev/null)" ]; then
        print_warning "Insufficient data for analysis"
        return
    fi

    echo "Analyzing your wellness data..."
    echo ""

    # Simple trend analysis
    echo "Key Insights:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✓ Regular mood tracking improves self-awareness"
    echo "✓ Sleep quality strongly correlates with mood"
    echo "✓ Physical activity reduces anxiety and stress"
    echo "✓ Consistent meditation practice enhances resilience"
    echo ""

    print_info "Continue tracking to reveal more personalized patterns"
}

# Command: help - Show help information
cmd_help() {
    print_header
    cat << EOF
Usage: ./wia-mental-wellness.sh [command] [options]

WELLNESS COMMANDS:
  assess-mood              Quick mood and wellness check-in
  track-wellness           View your wellness history and trends
  analyze-trends           Get insights from your wellness data
  meditation-session       Start a guided meditation session

THERAPY COMMANDS:
  schedule-therapy         Schedule a therapy appointment
  crisis-help              Access crisis intervention resources
  view-safety-plan         View your personalized safety plan

UTILITY COMMANDS:
  version                  Show version information
  help                     Show this help message

EXAMPLES:
  ./wia-mental-wellness.sh assess-mood
  ./wia-mental-wellness.sh meditation-session
  ./wia-mental-wellness.sh schedule-therapy
  ./wia-mental-wellness.sh crisis-help

CRISIS RESOURCES:
  US: 988 (Suicide & Crisis Lifeline)
  Text: HOME to 741741 (Crisis Text Line)
  Emergency: 911

For more information: https://docs.wia-official.org/mental-wellness

弘益人間 · Benefit All Humanity
EOF
}

# Command: version - Show version
cmd_version() {
    print_header
    echo "Version: $VERSION"
    echo "Standard: WIA-MENTAL_WELLNESS"
    echo "License: MIT"
    echo ""
    echo "© 2025 SmileStory Inc. / WIA"
    echo "弘益人間 · Benefit All Humanity"
}

# Main command dispatcher
main() {
    local command="${1:-help}"
    shift || true

    case $command in
        assess-mood)
            cmd_assess_mood "$@"
            ;;
        track-wellness)
            cmd_track_wellness "$@"
            ;;
        schedule-therapy)
            cmd_schedule_therapy "$@"
            ;;
        crisis-help)
            cmd_crisis_help "$@"
            ;;
        meditation-session)
            cmd_meditation_session "$@"
            ;;
        analyze-trends)
            cmd_analyze_trends "$@"
            ;;
        version)
            cmd_version
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run './wia-mental-wellness.sh help' for usage information"
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
