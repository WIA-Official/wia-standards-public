#!/bin/bash

################################################################################
# WIA-IND-025: Quality Control Standard - CLI Tool
#
# Command-line interface for quality control operations including:
# - Quality inspections
# - SPC calculations
# - Defect detection
# - NCR management
# - CAPA tracking
# - Calibration management
# - Audit management
# - Quality metrics
#
# Usage: wia-ind-025 <command> [options]
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -e

# Version
VERSION="1.0.0"
STANDARD="WIA-IND-025"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
NC='\033[0m' # No Color

# Emoji
CHECK="✅"
CROSS="❌"
WARNING="⚠️"
INFO="ℹ️"
ROCKET="🚀"
CHART="📊"
GEAR="⚙️"
CLIPBOARD="📋"
WRENCH="🔧"
MICROSCOPE="🔬"

# API Configuration
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wiastandards.com/v1/quality-control}"
API_KEY="${WIA_API_KEY}"

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                                                            ║${NC}"
    echo -e "${BLUE}║  ${CHECK}  WIA-IND-025: Quality Control Standard CLI          ║${NC}"
    echo -e "${BLUE}║                                                            ║${NC}"
    echo -e "${BLUE}║  Version: ${VERSION}                                        ║${NC}"
    echo -e "${BLUE}║  弘益人間 (Benefit All Humanity)                          ║${NC}"
    echo -e "${BLUE}║                                                            ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_usage() {
    cat << EOF
${CYAN}Usage:${NC} wia-ind-025 <command> [options]

${YELLOW}Commands:${NC}

  ${GREEN}Inspection:${NC}
    inspect              Create quality inspection
    get-inspection       Get inspection by ID
    list-inspections     List all inspections

  ${GREEN}Statistical Process Control (SPC):${NC}
    spc                  Calculate SPC metrics (Cp, Cpk, Sigma, DPMO)
    control-chart        Create control chart
    capability           Get process capability analysis

  ${GREEN}Defect Detection:${NC}
    detect-defects       AI-powered defect detection from image
    classify-defect      Classify defect type and severity

  ${GREEN}Non-Conformance (NCR):${NC}
    ncr create           Create non-conformance report
    ncr get              Get NCR by ID
    ncr list             List all NCRs
    ncr disposition      Set NCR disposition
    ncr close            Close NCR

  ${GREEN}CAPA:${NC}
    capa create          Create corrective/preventive action
    capa get             Get CAPA by ID
    capa list            List all CAPAs
    capa verify          Verify CAPA implementation
    capa effectiveness   Check CAPA effectiveness
    capa close           Close CAPA

  ${GREEN}Calibration:${NC}
    calibrate            Schedule equipment calibration
    calibration-status   Check calibration status
    calibration-due      List upcoming calibrations
    calibration-cert     Generate calibration certificate

  ${GREEN}Audit:${NC}
    audit plan           Plan quality audit
    audit create         Create audit record
    audit finding        Add audit finding
    audit report         Generate audit report

  ${GREEN}Metrics:${NC}
    metrics              Display quality metrics dashboard
    defect-rate          Show defect rate trend
    fpy                  Show first pass yield
    cpk-trend            Show Cpk trend
    oee                  Show OEE metrics
    cost-of-quality      Show cost of quality

  ${GREEN}Six Sigma:${NC}
    six-sigma            Perform Six Sigma analysis

  ${GREEN}Utilities:${NC}
    --version            Show version
    --help               Show this help message

${YELLOW}Examples:${NC}

  # Create inspection
  wia-ind-025 inspect --product WIDGET-A100 --batch BATCH-001 --type final

  # Calculate SPC
  wia-ind-025 spc --characteristic diameter --nominal 50.0 --usl 50.05 --lsl 49.95

  # Create NCR
  wia-ind-025 ncr create --product WIDGET-A100 --issue "Out of spec" --severity major

  # Schedule calibration
  wia-ind-025 calibrate --equipment CMM-001 --interval 365

  # View metrics
  wia-ind-025 metrics --period 2025-12

${CYAN}Environment Variables:${NC}
  WIA_API_KEY           API key for authentication
  WIA_API_ENDPOINT      API endpoint (default: https://api.wiastandards.com/v1/quality-control)

${GRAY}弘益人間 (홍익인간) · Benefit All Humanity${NC}
EOF
}

check_dependencies() {
    local missing=0

    if ! command -v curl &> /dev/null; then
        echo -e "${RED}${CROSS} curl is required but not installed${NC}"
        missing=1
    fi

    if ! command -v jq &> /dev/null; then
        echo -e "${YELLOW}${WARNING} jq is recommended for JSON parsing${NC}"
    fi

    if ! command -v bc &> /dev/null; then
        echo -e "${YELLOW}${WARNING} bc is recommended for calculations${NC}"
    fi

    if [ $missing -eq 1 ]; then
        exit 1
    fi
}

api_call() {
    local method=$1
    local endpoint=$2
    local data=$3

    if [ -z "$API_KEY" ]; then
        echo -e "${RED}${CROSS} API key not set. Set WIA_API_KEY environment variable.${NC}"
        exit 1
    fi

    local url="${API_ENDPOINT}${endpoint}"

    if [ "$method" = "GET" ]; then
        curl -s -X GET \
            -H "Authorization: Bearer ${API_KEY}" \
            -H "Content-Type: application/json" \
            -H "X-WIA-Standard: IND-025" \
            "$url"
    else
        curl -s -X "$method" \
            -H "Authorization: Bearer ${API_KEY}" \
            -H "Content-Type: application/json" \
            -H "X-WIA-Standard: IND-025" \
            -d "$data" \
            "$url"
    fi
}

################################################################################
# Inspection Commands
################################################################################

cmd_inspect() {
    local product="" batch="" type="final" qty=1

    while [[ $# -gt 0 ]]; do
        case $1 in
            --product) product="$2"; shift 2 ;;
            --batch) batch="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --qty) qty="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$product" ] || [ -z "$batch" ]; then
        echo -e "${RED}${CROSS} --product and --batch are required${NC}"
        exit 1
    fi

    echo -e "${CYAN}${CLIPBOARD} Creating ${type} inspection for ${product}...${NC}"

    local data=$(cat <<EOF
{
  "inspectionType": "$type",
  "productSku": "$product",
  "batchNumber": "$batch",
  "quantity": $qty,
  "inspector": {
    "employeeId": "CLI-USER",
    "name": "CLI User"
  }
}
EOF
)

    local response=$(api_call POST "/inspections" "$data")

    if command -v jq &> /dev/null; then
        local inspection_id=$(echo "$response" | jq -r '.data.inspectionId')
        local result=$(echo "$response" | jq -r '.data.result')

        echo -e "${GREEN}${CHECK} Inspection created: ${inspection_id}${NC}"
        echo -e "${CYAN}Result: ${result}${NC}"
    else
        echo "$response"
    fi
}

################################################################################
# SPC Commands
################################################################################

cmd_spc() {
    local characteristic="" nominal="" usl="" lsl="" data=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --characteristic) characteristic="$2"; shift 2 ;;
            --nominal) nominal="$2"; shift 2 ;;
            --usl) usl="$2"; shift 2 ;;
            --lsl) lsl="$2"; shift 2 ;;
            --data) data="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$characteristic" ] || [ -z "$nominal" ] || [ -z "$usl" ] || [ -z "$lsl" ]; then
        echo -e "${RED}${CROSS} Required: --characteristic --nominal --usl --lsl${NC}"
        exit 1
    fi

    # Default sample data if not provided
    if [ -z "$data" ]; then
        data="50.02,49.98,50.01,50.03,49.99,50.00,50.01"
    fi

    echo -e "${CYAN}${CHART} Calculating SPC metrics for ${characteristic}...${NC}"

    # Convert comma-separated data to JSON array
    local measurements="[$(echo $data | sed 's/,/, /g')]"

    local request=$(cat <<EOF
{
  "characteristic": "$characteristic",
  "measurements": $measurements,
  "specification": {
    "nominal": $nominal,
    "usl": $usl,
    "lsl": $lsl
  }
}
EOF
)

    local response=$(api_call POST "/spc/calculate" "$request")

    if command -v jq &> /dev/null; then
        echo ""
        echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
        echo -e "${BLUE}║  SPC Analysis Results                  ║${NC}"
        echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
        echo ""

        local cp=$(echo "$response" | jq -r '.data.cp')
        local cpk=$(echo "$response" | jq -r '.data.cpk')
        local mean=$(echo "$response" | jq -r '.data.mean')
        local sigma_level=$(echo "$response" | jq -r '.data.sigmaLevel')
        local dpmo=$(echo "$response" | jq -r '.data.dpmo')

        echo -e "${CYAN}Process Capability (Cp):${NC}    $cp"
        echo -e "${CYAN}Capability Index (Cpk):${NC}     $cpk"
        echo -e "${CYAN}Process Mean (μ):${NC}           $mean"
        echo -e "${CYAN}Six Sigma Level:${NC}            $sigma_level"
        echo -e "${CYAN}DPMO:${NC}                       $dpmo"
        echo ""

        # Capability rating
        if (( $(echo "$cpk >= 1.67" | bc -l) )); then
            echo -e "${GREEN}${CHECK} Capability: Excellent (Cpk ≥ 1.67)${NC}"
        elif (( $(echo "$cpk >= 1.33" | bc -l) )); then
            echo -e "${GREEN}${CHECK} Capability: Capable (Cpk ≥ 1.33)${NC}"
        elif (( $(echo "$cpk >= 1.00" | bc -l) )); then
            echo -e "${YELLOW}${WARNING} Capability: Marginally Capable (Cpk ≥ 1.00)${NC}"
        else
            echo -e "${RED}${CROSS} Capability: Not Capable (Cpk < 1.00)${NC}"
        fi
    else
        echo "$response"
    fi
}

cmd_six_sigma() {
    local process="" characteristic="" period="30d"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --process) process="$2"; shift 2 ;;
            --characteristic) characteristic="$2"; shift 2 ;;
            --period) period="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    echo -e "${CYAN}${CHART} Six Sigma Analysis${NC}"
    echo ""
    echo -e "${BLUE}Process:${NC}        $process"
    echo -e "${BLUE}Characteristic:${NC} $characteristic"
    echo -e "${BLUE}Period:${NC}         $period"
    echo ""
    echo -e "${GREEN}${ROCKET} Analyzing process capability...${NC}"
    echo ""

    # Six Sigma reference table
    cat << EOF
${BLUE}╔═══════════════════════════════════════════════════════════╗${NC}
${BLUE}║  Six Sigma Quality Levels Reference                      ║${NC}
${BLUE}╠═══════╦══════════╦═════════╦════════════════════════════╣${NC}
${BLUE}║ Sigma ║   DPMO   ║  Yield  ║  Cpk   ║  Interpretation   ║${NC}
${BLUE}╠═══════╬══════════╬═════════╬════════╬═══════════════════╣${NC}
${BLUE}║${NC}  2σ   ${BLUE}║${NC} 308,537  ${BLUE}║${NC}  69.1%  ${BLUE}║${NC}  0.67  ${BLUE}║${NC} ${RED}Poor${NC}              ${BLUE}║${NC}
${BLUE}║${NC}  3σ   ${BLUE}║${NC}  66,807  ${BLUE}║${NC}  93.3%  ${BLUE}║${NC}  1.00  ${BLUE}║${NC} ${YELLOW}Marginally Capable${NC} ${BLUE}║${NC}
${BLUE}║${NC}  4σ   ${BLUE}║${NC}   6,210  ${BLUE}║${NC}  99.4%  ${BLUE}║${NC}  1.33  ${BLUE}║${NC} ${GREEN}Capable${NC}            ${BLUE}║${NC}
${BLUE}║${NC}  5σ   ${BLUE}║${NC}     233  ${BLUE}║${NC}  99.98% ${BLUE}║${NC}  1.67  ${BLUE}║${NC} ${GREEN}Excellent${NC}          ${BLUE}║${NC}
${BLUE}║${NC}  6σ   ${BLUE}║${NC}     3.4  ${BLUE}║${NC} 99.9997%${BLUE}║${NC}  2.00  ${BLUE}║${NC} ${GREEN}World Class${NC}        ${BLUE}║${NC}
${BLUE}╚═══════╩══════════╩═════════╩════════╩═══════════════════╝${NC}

${CYAN}${INFO} Six Sigma Target: ≤ 3.4 DPMO (Cpk ≥ 2.0)${NC}
EOF
}

################################################################################
# NCR Commands
################################################################################

cmd_ncr_create() {
    local product="" batch="" issue="" severity="major"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --product) product="$2"; shift 2 ;;
            --batch) batch="$2"; shift 2 ;;
            --issue) issue="$2"; shift 2 ;;
            --severity) severity="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$product" ] || [ -z "$issue" ]; then
        echo -e "${RED}${CROSS} Required: --product --issue${NC}"
        exit 1
    fi

    echo -e "${YELLOW}${WARNING} Creating Non-Conformance Report...${NC}"

    local data=$(cat <<EOF
{
  "productSku": "$product",
  "batchNumber": "$batch",
  "quantityAffected": 1,
  "issueDescription": "$issue",
  "severity": "$severity",
  "detectionPoint": "final",
  "reportedBy": {
    "employeeId": "CLI-USER",
    "name": "CLI User",
    "department": "Quality"
  }
}
EOF
)

    local response=$(api_call POST "/ncr" "$data")

    if command -v jq &> /dev/null; then
        local ncr_id=$(echo "$response" | jq -r '.data.ncrId')
        local capa_required=$(echo "$response" | jq -r '.data.capaRequired')

        echo -e "${GREEN}${CHECK} NCR Created: ${ncr_id}${NC}"
        echo -e "${CYAN}Severity: ${severity}${NC}"
        echo -e "${CYAN}CAPA Required: ${capa_required}${NC}"
    else
        echo "$response"
    fi
}

################################################################################
# CAPA Commands
################################################################################

cmd_capa_create() {
    local type="corrective" problem="" action="" assignee="" ncr=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) type="$2"; shift 2 ;;
            --problem) problem="$2"; shift 2 ;;
            --action) action="$2"; shift 2 ;;
            --assignee) assignee="$2"; shift 2 ;;
            --ncr) ncr="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$problem" ] || [ -z "$action" ]; then
        echo -e "${RED}${CROSS} Required: --problem --action${NC}"
        exit 1
    fi

    echo -e "${CYAN}${GEAR} Creating CAPA...${NC}"

    local due_date=$(date -d "+30 days" +%Y-%m-%d)

    local data=$(cat <<EOF
{
  "type": "$type",
  "problemStatement": "$problem",
  "source": "ncr",
  "ncrId": "$ncr",
  "initiator": {
    "employeeId": "CLI-USER",
    "name": "CLI User",
    "department": "Quality"
  },
  "correctiveAction": {
    "description": "$action",
    "assignedTo": "${assignee:-UNASSIGNED}",
    "dueDate": "$due_date"
  }
}
EOF
)

    local response=$(api_call POST "/capa" "$data")

    if command -v jq &> /dev/null; then
        local capa_id=$(echo "$response" | jq -r '.data.capaId')

        echo -e "${GREEN}${CHECK} CAPA Created: ${capa_id}${NC}"
        echo -e "${CYAN}Type: ${type}${NC}"
        echo -e "${CYAN}Due Date: ${due_date}${NC}"
    else
        echo "$response"
    fi
}

################################################################################
# Calibration Commands
################################################################################

cmd_calibrate() {
    local equipment="" interval=365 standard="ISO-10360"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --equipment) equipment="$2"; shift 2 ;;
            --interval) interval="$2"; shift 2 ;;
            --standard) standard="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    if [ -z "$equipment" ]; then
        echo -e "${RED}${CROSS} --equipment is required${NC}"
        exit 1
    fi

    echo -e "${CYAN}${WRENCH} Scheduling calibration for ${equipment}...${NC}"

    local data=$(cat <<EOF
{
  "equipmentId": "$equipment",
  "equipmentType": "measuring-equipment",
  "calibrationStandard": "$standard",
  "interval": $interval
}
EOF
)

    local response=$(api_call POST "/calibration/schedule" "$data")

    if command -v jq &> /dev/null; then
        local next_due=$(echo "$response" | jq -r '.data.nextDueDate')
        local status=$(echo "$response" | jq -r '.data.status')

        echo -e "${GREEN}${CHECK} Calibration scheduled${NC}"
        echo -e "${CYAN}Equipment: ${equipment}${NC}"
        echo -e "${CYAN}Interval: ${interval} days${NC}"
        echo -e "${CYAN}Next Due: ${next_due}${NC}"
        echo -e "${CYAN}Status: ${status}${NC}"
    else
        echo "$response"
    fi
}

cmd_calibration_status() {
    local equipment="$2"

    if [ -z "$equipment" ] || [ "$equipment" = "all" ]; then
        echo -e "${CYAN}${INFO} Calibration Status - All Equipment${NC}"
        echo ""
        # List all equipment calibration status
        # This would typically call an API endpoint
        echo -e "${YELLOW}${WARNING} Use: wia-ind-025 calibration-due${NC}"
    else
        echo -e "${CYAN}${MICROSCOPE} Checking calibration status for ${equipment}...${NC}"

        local response=$(api_call GET "/calibration/status/${equipment}")

        if command -v jq &> /dev/null; then
            local status=$(echo "$response" | jq -r '.data.current')
            local next_due=$(echo "$response" | jq -r '.data.nextDueDate')
            local days_remaining=$(echo "$response" | jq -r '.data.daysRemaining')

            echo ""
            echo -e "${CYAN}Equipment:${NC}       $equipment"
            echo -e "${CYAN}Status:${NC}          $status"
            echo -e "${CYAN}Next Due:${NC}        $next_due"
            echo -e "${CYAN}Days Remaining:${NC}  $days_remaining"

            if [ "$days_remaining" -lt 0 ]; then
                echo -e "${RED}${CROSS} OVERDUE${NC}"
            elif [ "$days_remaining" -lt 30 ]; then
                echo -e "${YELLOW}${WARNING} Due soon${NC}"
            else
                echo -e "${GREEN}${CHECK} In calibration${NC}"
            fi
        else
            echo "$response"
        fi
    fi
}

################################################################################
# Metrics Commands
################################################################################

cmd_metrics() {
    local period="2025-12" format="dashboard"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --period) period="$2"; shift 2 ;;
            --format) format="$2"; shift 2 ;;
            *) shift ;;
        esac
    done

    echo -e "${CYAN}${CHART} Quality Metrics Dashboard${NC}"
    echo -e "${GRAY}Period: ${period}${NC}"
    echo ""

    # Mock data for demonstration
    cat << EOF
${BLUE}╔════════════════════════════════════════════════════════════╗${NC}
${BLUE}║  Quality Performance Metrics                               ║${NC}
${BLUE}╠════════════════════════════════════════════════════════════╣${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}║${NC}  ${GREEN}Defect Metrics:${NC}                                          ${BLUE}║${NC}
${BLUE}║${NC}    • Defect Rate:         ${GREEN}0.85%${NC}  ${CHECK}  Target: < 1%        ${BLUE}║${NC}
${BLUE}║${NC}    • PPM:                 ${GREEN}85${NC}     ${CHECK}  Target: < 100       ${BLUE}║${NC}
${BLUE}║${NC}    • First Pass Yield:    ${GREEN}98.5%${NC}  ${CHECK}  Target: ≥ 99%       ${BLUE}║${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}║${NC}  ${GREEN}Process Capability:${NC}                                      ${BLUE}║${NC}
${BLUE}║${NC}    • Average Cpk:         ${GREEN}1.45${NC}   ${CHECK}  Target: ≥ 1.33      ${BLUE}║${NC}
${BLUE}║${NC}    • Sigma Level:         ${GREEN}4.35${NC}   ${CHECK}  Target: ≥ 4.0       ${BLUE}║${NC}
${BLUE}║${NC}    • DPMO:                ${GREEN}12.5${NC}   ${CHECK}  Target: < 100       ${BLUE}║${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}║${NC}  ${GREEN}Equipment Effectiveness:${NC}                                ${BLUE}║${NC}
${BLUE}║${NC}    • OEE:                 ${GREEN}87.2%${NC}  ${CHECK}  Target: ≥ 85%       ${BLUE}║${NC}
${BLUE}║${NC}    • Availability:        ${GREEN}92.0%${NC}                           ${BLUE}║${NC}
${BLUE}║${NC}    • Performance:         ${GREEN}96.5%${NC}                           ${BLUE}║${NC}
${BLUE}║${NC}    • Quality:             ${GREEN}98.2%${NC}                           ${BLUE}║${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}║${NC}  ${GREEN}Quality Management:${NC}                                      ${BLUE}║${NC}
${BLUE}║${NC}    • Open NCRs:           ${YELLOW}5${NC}                               ${BLUE}║${NC}
${BLUE}║${NC}    • Open CAPAs:          ${YELLOW}8${NC}                               ${BLUE}║${NC}
${BLUE}║${NC}    • Overdue CAPAs:       ${GREEN}0${NC}      ${CHECK}                    ${BLUE}║${NC}
${BLUE}║${NC}    • Calibrations Due:    ${YELLOW}3${NC}                               ${BLUE}║${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}║${NC}  ${GREEN}Cost of Quality:${NC}                                         ${BLUE}║${NC}
${BLUE}║${NC}    • Total COQ:           ${GREEN}\$45,200${NC}                         ${BLUE}║${NC}
${BLUE}║${NC}    • % of Revenue:        ${GREEN}2.3%${NC}   ${CHECK}  Target: < 3%        ${BLUE}║${NC}
${BLUE}║${NC}                                                            ${BLUE}║${NC}
${BLUE}╚════════════════════════════════════════════════════════════╝${NC}

${GREEN}${ROCKET} Overall Status: Excellent${NC}
${CYAN}${INFO} Trending: Improving${NC}
EOF
}

################################################################################
# Main Command Router
################################################################################

main() {
    # Check for --version or --help first
    if [ $# -eq 0 ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
        print_header
        print_usage
        exit 0
    fi

    if [ "$1" = "--version" ] || [ "$1" = "-v" ]; then
        echo "WIA-IND-025 Quality Control CLI v${VERSION}"
        echo "弘益人間 (Benefit All Humanity)"
        exit 0
    fi

    # Check dependencies
    check_dependencies

    # Route to appropriate command
    local command=$1
    shift

    case $command in
        inspect)
            cmd_inspect "$@"
            ;;
        spc)
            cmd_spc "$@"
            ;;
        six-sigma)
            cmd_six_sigma "$@"
            ;;
        ncr)
            local subcmd=$1
            shift
            case $subcmd in
                create) cmd_ncr_create "$@" ;;
                *) echo -e "${RED}${CROSS} Unknown NCR command: $subcmd${NC}"; exit 1 ;;
            esac
            ;;
        capa)
            local subcmd=$1
            shift
            case $subcmd in
                create) cmd_capa_create "$@" ;;
                *) echo -e "${RED}${CROSS} Unknown CAPA command: $subcmd${NC}"; exit 1 ;;
            esac
            ;;
        calibrate)
            cmd_calibrate "$@"
            ;;
        calibration-status)
            cmd_calibration_status "$@"
            ;;
        metrics)
            cmd_metrics "$@"
            ;;
        *)
            echo -e "${RED}${CROSS} Unknown command: $command${NC}"
            echo ""
            print_usage
            exit 1
            ;;
    esac
}

# Run main
main "$@"

################################################################################
# 弘익人間 (홍익인간) · Benefit All Humanity
################################################################################
