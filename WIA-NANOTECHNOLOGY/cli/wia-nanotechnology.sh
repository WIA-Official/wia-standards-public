#!/bin/bash
###############################################################################
# WIA-NANOTECHNOLOGY Command Line Interface
#
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
#
# Usage: wia-nanotechnology [command] [options]
###############################################################################

set -e

# Configuration
API_BASE_URL="${WIA_NANO_API_URL:-https://api.wia-nanotechnology.org/v1}"
API_KEY="${WIA_NANO_API_KEY:-}"
VERSION="1.0.0"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

###############################################################################
# Utility Functions
###############################################################################

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_NANO_API_KEY environment variable."
        exit 1
    fi
}

api_request() {
    local method=$1
    local endpoint=$2
    local data=${3:-""}

    local curl_opts=(
        -s
        -X "$method"
        -H "Authorization: Bearer $API_KEY"
        -H "Content-Type: application/json"
    )

    if [ -n "$data" ]; then
        curl_opts+=(-d "$data")
    fi

    response=$(curl "${curl_opts[@]}" "$API_BASE_URL$endpoint")
    echo "$response"
}

###############################################################################
# Command: material-search
###############################################################################

cmd_material_search() {
    check_api_key

    local type=""
    local elements=""
    local min_size=""
    local max_size=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type)
                type="$2"
                shift 2
                ;;
            --elements)
                elements="$2"
                shift 2
                ;;
            --min-size)
                min_size="$2"
                shift 2
                ;;
            --max-size)
                max_size="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    print_info "Searching nanomaterials..."

    local params=""
    [ -n "$type" ] && params="${params}&type=$type"
    [ -n "$elements" ] && params="${params}&elements=$elements"
    [ -n "$min_size" ] && params="${params}&minSize=$min_size"
    [ -n "$max_size" ] && params="${params}&maxSize=$max_size"

    response=$(api_request GET "/materials/search?${params#&}")

    if command -v jq &> /dev/null; then
        echo "$response" | jq -r '.materials[] | "\(.materialId): \(.name) (\(.type))"'
        total=$(echo "$response" | jq -r '.pagination.total')
        print_success "Found $total materials"
    else
        echo "$response"
        print_warning "Install jq for better output formatting"
    fi
}

###############################################################################
# Command: synthesis-plan
###############################################################################

cmd_synthesis_plan() {
    check_api_key

    local material_id=""
    local method=""
    local max_temp=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --material-id)
                material_id="$2"
                shift 2
                ;;
            --method)
                method="$2"
                shift 2
                ;;
            --max-temp)
                max_temp="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    if [ -z "$material_id" ] || [ -z "$method" ]; then
        print_error "Missing required arguments: --material-id and --method"
        return 1
    fi

    print_info "Creating synthesis plan..."

    local data="{\"targetMaterialId\":\"$material_id\",\"method\":\"$method\""
    if [ -n "$max_temp" ]; then
        data="$data,\"constraints\":{\"maxTemperature\":$max_temp}"
    fi
    data="$data}"

    response=$(api_request POST "/synthesis/plan" "$data")

    if command -v jq &> /dev/null; then
        echo "$response" | jq '.'
        plan_id=$(echo "$response" | jq -r '.planId')
        print_success "Synthesis plan created: $plan_id"
    else
        echo "$response"
    fi
}

###############################################################################
# Command: property-analyze
###############################################################################

cmd_property_analyze() {
    check_api_key

    local material_id=""
    local properties=""
    local method="ML"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --material-id)
                material_id="$2"
                shift 2
                ;;
            --properties)
                properties="$2"
                shift 2
                ;;
            --method)
                method="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    if [ -z "$material_id" ] || [ -z "$properties" ]; then
        print_error "Missing required arguments: --material-id and --properties"
        return 1
    fi

    print_info "Analyzing material properties..."

    # Convert comma-separated properties to JSON array
    IFS=',' read -ra PROPS <<< "$properties"
    local props_json="["
    for prop in "${PROPS[@]}"; do
        props_json="$props_json\"$prop\","
    done
    props_json="${props_json%,}]"

    local data="{\"materialId\":\"$material_id\",\"properties\":$props_json,\"method\":\"$method\"}"

    response=$(api_request POST "/properties/predict" "$data")

    if command -v jq &> /dev/null; then
        echo "$response" | jq -r '.predictions | to_entries[] | "\(.key): \(.value.value) \(.value.unit) (confidence: \(.value.confidence))"'
        print_success "Property analysis completed"
    else
        echo "$response"
    fi
}

###############################################################################
# Command: simulate
###############################################################################

cmd_simulate() {
    check_api_key

    local type=""
    local material_id=""
    local time_step=""
    local total_time=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type)
                type="$2"
                shift 2
                ;;
            --material-id)
                material_id="$2"
                shift 2
                ;;
            --time-step)
                time_step="$2"
                shift 2
                ;;
            --total-time)
                total_time="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    if [ -z "$type" ]; then
        print_error "Missing required argument: --type"
        return 1
    fi

    print_info "Submitting simulation job..."

    local data="{\"type\":\"$type\""
    [ -n "$material_id" ] && data="$data,\"materialId\":\"$material_id\""

    if [ -n "$time_step" ] && [ -n "$total_time" ]; then
        data="$data,\"parameters\":{\"timeStep\":$time_step,\"totalTime\":$total_time}"
    fi

    data="$data}"

    response=$(api_request POST "/simulation" "$data")

    if command -v jq &> /dev/null; then
        sim_id=$(echo "$response" | jq -r '.simulationId')
        status=$(echo "$response" | jq -r '.status')
        print_success "Simulation submitted: $sim_id (status: $status)"

        # Poll for completion
        print_info "Waiting for simulation to complete..."
        while true; do
            sleep 5
            status_response=$(api_request GET "/simulation/$sim_id")
            current_status=$(echo "$status_response" | jq -r '.status')

            if [ "$current_status" = "completed" ]; then
                print_success "Simulation completed!"
                echo "$status_response" | jq '.results'
                break
            elif [ "$current_status" = "failed" ]; then
                print_error "Simulation failed"
                break
            else
                progress=$(echo "$status_response" | jq -r '.progress // 0')
                print_info "Progress: ${progress}%"
            fi
        done
    else
        echo "$response"
    fi
}

###############################################################################
# Command: safety-check
###############################################################################

cmd_safety_check() {
    check_api_key

    local material_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --material-id)
                material_id="$2"
                shift 2
                ;;
            *)
                print_error "Unknown option: $1"
                return 1
                ;;
        esac
    done

    if [ -z "$material_id" ]; then
        print_error "Missing required argument: --material-id"
        return 1
    fi

    print_info "Checking safety information..."

    response=$(api_request GET "/materials/$material_id/safety")

    if command -v jq &> /dev/null; then
        echo ""
        echo "=== Hazard Classification ==="
        echo "$response" | jq -r '.hazardClassification.ghs.codes[]? // "No GHS codes"'

        echo ""
        echo "=== NFPA Rating ==="
        echo "$response" | jq -r '"Health: \(.hazardClassification.nfpa.health // "N/A"), Flammability: \(.hazardClassification.nfpa.flammability // "N/A"), Reactivity: \(.hazardClassification.nfpa.reactivity // "N/A")"'

        echo ""
        echo "=== PPE Required ==="
        echo "$response" | jq -r '.handling.ppe[]? // "No PPE information"'

        print_success "Safety check completed"
    else
        echo "$response"
    fi
}

###############################################################################
# Help
###############################################################################

show_help() {
    cat << EOF
WIA-NANOTECHNOLOGY CLI v${VERSION}
弘益人間 (Benefit All Humanity)

Usage: wia-nanotechnology [command] [options]

Commands:
  material-search       Search nanomaterial database
  synthesis-plan        Plan synthesis procedures
  property-analyze      Analyze material properties
  simulate              Run nanoscale simulations
  safety-check          Check safety protocols
  version               Show version information
  help                  Show this help message

Material Search Options:
  --type TYPE           Material type (CNT, graphene, quantum_dot, etc.)
  --elements ELEMENTS   Comma-separated element symbols
  --min-size SIZE       Minimum dimension in nm
  --max-size SIZE       Maximum dimension in nm

Synthesis Plan Options:
  --material-id ID      Target material ID
  --method METHOD       Synthesis method (CVD, sol_gel, etc.)
  --max-temp TEMP       Maximum temperature constraint

Property Analyze Options:
  --material-id ID      Material ID
  --properties PROPS    Comma-separated property names
  --method METHOD       Prediction method (ML, DFT, empirical)

Simulate Options:
  --type TYPE           Simulation type (molecular_dynamics, DFT, etc.)
  --material-id ID      Material ID (optional)
  --time-step STEP      Time step in fs
  --total-time TIME     Total simulation time in ps

Safety Check Options:
  --material-id ID      Material ID

Environment Variables:
  WIA_NANO_API_URL      API base URL (default: https://api.wia-nanotechnology.org/v1)
  WIA_NANO_API_KEY      API authentication key (required)

Examples:
  wia-nanotechnology material-search --type CNT --elements C
  wia-nanotechnology synthesis-plan --material-id abc-123 --method CVD
  wia-nanotechnology property-analyze --material-id abc-123 --properties bandgap,conductivity
  wia-nanotechnology simulate --type molecular_dynamics --time-step 1 --total-time 1000
  wia-nanotechnology safety-check --material-id abc-123

EOF
}

###############################################################################
# Main
###############################################################################

main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        material-search)
            cmd_material_search "$@"
            ;;
        synthesis-plan)
            cmd_synthesis_plan "$@"
            ;;
        property-analyze)
            cmd_property_analyze "$@"
            ;;
        simulate)
            cmd_simulate "$@"
            ;;
        safety-check)
            cmd_safety_check "$@"
            ;;
        version)
            echo "WIA-NANOTECHNOLOGY CLI v${VERSION}"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
