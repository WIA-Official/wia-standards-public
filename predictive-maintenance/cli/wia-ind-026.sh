#!/bin/bash

###############################################################################
# WIA-IND-026: Predictive Maintenance CLI
# World Certification Industry Association
#
# Usage: wia-ind-026 [command] [options]
#
# Version: 1.0.0
###############################################################################

set -e

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
WRENCH="🔧"
CHECK="✅"
CROSS="❌"
WARNING="⚠️"
INFO="ℹ️"
ROCKET="🚀"
CHART="📊"
GEAR="⚙️"
BELL="🔔"
THERMOMETER="🌡️"
WAVE="〰️"
OIL="🛢️"
SOUND="🔊"
CLOCK="⏰"
TOOLS="🛠️"
PACKAGE="📦"

# Configuration
API_ENDPOINT="${WIA_IND_026_ENDPOINT:-https://api.wia.org/ind-026/v1}"
API_KEY="${WIA_IND_026_API_KEY:-}"
ORG_ID="${WIA_IND_026_ORG_ID:-}"
CONFIG_FILE="${HOME}/.wia/ind-026/config/config.json"

# Version
VERSION="1.0.0"

###############################################################################
# Utility Functions
###############################################################################

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                                                            ║${NC}"
    echo -e "${BLUE}║      ${WRENCH}  WIA-IND-026: Predictive Maintenance CLI  ${WRENCH}      ║${NC}"
    echo -e "${BLUE}║                      Version ${VERSION}                        ║${NC}"
    echo -e "${BLUE}║                                                            ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${CHECK} ${GREEN}$1${NC}"
}

print_error() {
    echo -e "${CROSS} ${RED}$1${NC}"
}

print_warning() {
    echo -e "${WARNING} ${YELLOW}$1${NC}"
}

print_info() {
    echo -e "${INFO} ${CYAN}$1${NC}"
}

check_config() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_IND_026_API_KEY environment variable."
        echo ""
        echo "  export WIA_IND_026_API_KEY=your-api-key"
        echo ""
        exit 1
    fi

    if [ -z "$ORG_ID" ]; then
        print_error "Organization ID not set. Please set WIA_IND_026_ORG_ID environment variable."
        echo ""
        echo "  export WIA_IND_026_ORG_ID=your-org-id"
        echo ""
        exit 1
    fi
}

api_call() {
    local method="$1"
    local endpoint="$2"
    local data="$3"

    local curl_opts=(
        -X "$method"
        -H "Content-Type: application/json"
        -H "X-API-Key: $API_KEY"
        -H "X-Organization-ID: $ORG_ID"
        -s
    )

    if [ -n "$data" ]; then
        curl_opts+=(-d "$data")
    fi

    local response=$(curl "${curl_opts[@]}" "${API_ENDPOINT}${endpoint}")
    echo "$response"
}

###############################################################################
# Asset Management
###############################################################################

asset_register() {
    local asset_id="$1"
    local asset_type="$2"
    local location="$3"
    shift 3
    local specs="$@"

    print_info "Registering asset: $asset_id"

    local spec_json="{"
    for spec in $specs; do
        IFS='=' read -r key value <<< "$spec"
        spec_json+="\"$key\":\"$value\","
    done
    spec_json="${spec_json%,}}"

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "assetType": "$asset_type",
  "location": "$location",
  "specifications": $spec_json
}
EOF
)

    local response=$(api_call "POST" "/assets" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Asset registered successfully!"
        echo ""
        echo -e "${CYAN}Asset Details:${NC}"
        echo "$response" | jq -r '.data | "  ID: \(.assetId)\n  Type: \(.assetType)\n  Location: \(.location)"'
    else
        print_error "Failed to register asset"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

asset_get() {
    local asset_id="$1"

    local response=$(api_call "GET" "/assets/$asset_id" "")

    if echo "$response" | grep -q '"status":"success"'; then
        echo -e "${CYAN}${GEAR} Asset Information:${NC}"
        echo ""
        echo "$response" | jq -r '.data |
            "  Asset ID:      \(.assetId)\n" +
            "  Type:          \(.assetType)\n" +
            "  Location:      \(.location)\n" +
            "  Status:        \(.status // "N/A")\n" +
            "  Criticality:   \(.criticality // "N/A")\n" +
            "  Created:       \(.createdAt // "N/A")"'
        echo ""

        if echo "$response" | jq -e '.data.specifications' > /dev/null 2>&1; then
            echo -e "${CYAN}  Specifications:${NC}"
            echo "$response" | jq -r '.data.specifications | to_entries[] | "    \(.key): \(.value)"'
        fi
    else
        print_error "Asset not found: $asset_id"
        exit 1
    fi
}

asset_list() {
    local type="$1"
    local location="$2"

    local params=""
    [ -n "$type" ] && params="?type=$type"
    [ -n "$location" ] && params="${params:+${params}&}location=$location"

    local response=$(api_call "GET" "/assets$params" "")

    if echo "$response" | grep -q '"status":"success"'; then
        echo -e "${CYAN}${GEAR} Assets:${NC}"
        echo ""
        echo "$response" | jq -r '.data.items[] |
            "  [\(.assetId)] \(.assetType) - \(.location) (\(.status // "UNKNOWN"))"'
        echo ""
        local total=$(echo "$response" | jq -r '.data.total')
        print_info "Total: $total assets"
    else
        print_error "Failed to list assets"
        exit 1
    fi
}

###############################################################################
# Sensor Management
###############################################################################

sensor_attach() {
    local asset_id="$1"
    local sensor_id="$2"
    local sensor_type="$3"
    local location="$4"
    local sampling_rate="$5"

    print_info "Attaching sensor $sensor_id to asset $asset_id"

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "sensors": [{
    "sensorId": "$sensor_id",
    "type": "$sensor_type",
    "location": "$location",
    "samplingRate": $sampling_rate
  }]
}
EOF
)

    local response=$(api_call "POST" "/assets/$asset_id/sensors" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Sensor attached successfully!"
        echo ""
        echo -e "${CYAN}Sensor Details:${NC}"
        echo "$response" | jq -r '.data[0] | "  ID: \(.sensorId)\n  Type: \(.type)\n  Location: \(.location)\n  Sampling Rate: \(.samplingRate) Hz"'
    else
        print_error "Failed to attach sensor"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

###############################################################################
# Data Ingestion
###############################################################################

data_ingest() {
    local sensor_id="$1"
    local file="$2"
    local format="${3:-csv}"

    print_info "Ingesting data from $file for sensor $sensor_id"

    if [ ! -f "$file" ]; then
        print_error "File not found: $file"
        exit 1
    fi

    local count=0
    if [ "$format" = "csv" ]; then
        while IFS=',' read -r timestamp value; do
            [ "$timestamp" = "timestamp" ] && continue  # Skip header

            local data=$(cat <<EOF
{
  "sensorId": "$sensor_id",
  "timestamp": $timestamp,
  "values": $value,
  "unit": "auto"
}
EOF
)
            api_call "POST" "/data/ingest" "$data" > /dev/null
            ((count++))

            if [ $((count % 100)) -eq 0 ]; then
                echo -ne "\r  ${GRAY}Ingested $count records...${NC}"
            fi
        done < "$file"
    fi

    echo ""
    print_success "Ingested $count records"
}

###############################################################################
# Analysis
###############################################################################

analyze_vibration() {
    local asset_id="$1"
    local sensor_id="$2"
    local analysis_types="$3"
    local output="${4:-json}"

    print_info "Performing vibration analysis on $asset_id..."

    local types_array="["
    IFS=',' read -ra TYPES <<< "$analysis_types"
    for type in "${TYPES[@]}"; do
        types_array+="\"${type^^}\","
    done
    types_array="${types_array%,}]"

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "sensorId": "$sensor_id",
  "analysisTypes": $types_array
}
EOF
)

    local response=$(api_call "POST" "/analysis/vibration" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Analysis completed!"
        echo ""

        if [ "$output" = "json" ]; then
            echo "$response" | jq '.data'
        else
            echo -e "${CYAN}${WAVE} Vibration Analysis Results:${NC}"
            echo ""

            # Time domain
            if echo "$response" | jq -e '.data.timeDomain' > /dev/null 2>&1; then
                echo -e "${CYAN}  Time Domain:${NC}"
                echo "$response" | jq -r '.data.timeDomain |
                    "    RMS:           \(.rms // "N/A") g\n" +
                    "    Peak:          \(.peak // "N/A") g\n" +
                    "    Crest Factor:  \(.crestFactor // "N/A")\n" +
                    "    Kurtosis:      \(.kurtosis // "N/A")"'
                echo ""
            fi

            # Severity
            if echo "$response" | jq -e '.data.severity' > /dev/null 2>&1; then
                local zone=$(echo "$response" | jq -r '.data.severity.zone')
                local desc=$(echo "$response" | jq -r '.data.severity.description')
                local rms=$(echo "$response" | jq -r '.data.severity.rmsVelocity')

                echo -e "${CYAN}  Severity (ISO 20816):${NC}"
                echo -e "    Zone: ${YELLOW}$zone${NC} - $desc"
                echo -e "    RMS Velocity: $rms mm/s"
                echo ""
            fi

            # Bearing condition
            if echo "$response" | jq -e '.data.bearingCondition' > /dev/null 2>&1; then
                local condition=$(echo "$response" | jq -r '.data.bearingCondition.overall')
                echo -e "${CYAN}  Bearing Condition:${NC}"
                echo -e "    Overall: ${GREEN}$condition${NC}"
                echo ""
            fi

            # RUL
            if echo "$response" | jq -e '.data.remainingUsefulLife' > /dev/null 2>&1; then
                local rul=$(echo "$response" | jq -r '.data.remainingUsefulLife.days')
                local conf=$(echo "$response" | jq -r '.data.remainingUsefulLife.confidence')
                echo -e "${CYAN}  Remaining Useful Life:${NC}"
                echo -e "    ${CLOCK} $rul days (confidence: $(awk "BEGIN {printf \"%.0f\", $conf*100}")%)"
                echo ""
            fi

            # Recommendations
            if echo "$response" | jq -e '.data.recommendations' > /dev/null 2>&1; then
                echo -e "${CYAN}  ${TOOLS} Recommendations:${NC}"
                echo "$response" | jq -r '.data.recommendations[] | "    • \(.)"'
            fi
        fi

        # Save to file if requested
        if [ -n "$output" ] && [ "$output" != "table" ] && [ "$output" != "json" ]; then
            echo "$response" | jq '.data' > "$output"
            print_info "Results saved to $output"
        fi
    else
        print_error "Analysis failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

analyze_thermal() {
    local asset_id="$1"
    local image="$2"
    local reference_temp="${3:-25}"
    local output="${4:-thermal-report.json}"

    print_info "Performing thermal analysis on $asset_id..."

    if [ ! -f "$image" ]; then
        print_error "Image file not found: $image"
        exit 1
    fi

    local image_data=$(base64 "$image")

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "imageData": "$image_data",
  "referenceTemperature": $reference_temp,
  "algorithm": "HOTSPOT_DETECTION"
}
EOF
)

    local response=$(api_call "POST" "/analysis/thermal" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Thermal analysis completed!"
        echo ""

        echo -e "${CYAN}${THERMOMETER} Thermal Analysis Results:${NC}"
        echo ""

        local assessment=$(echo "$response" | jq -r '.data.assessment')
        echo -e "  Assessment: ${YELLOW}$assessment${NC}"
        echo ""

        # Hotspots
        local hotspot_count=$(echo "$response" | jq -r '.data.hotspots | length')
        if [ "$hotspot_count" -gt 0 ]; then
            echo -e "${CYAN}  ${WARNING} Hotspots Detected ($hotspot_count):${NC}"
            echo "$response" | jq -r '.data.hotspots[] |
                "    • Temperature: \(.temperature)°C (ΔT: +\(.delta)°C) - \(.severity)"'
            echo ""
        else
            print_success "  No hotspots detected"
            echo ""
        fi

        # Statistics
        echo -e "${CYAN}  Temperature Statistics:${NC}"
        echo "$response" | jq -r '.data.statistics |
            "    Min: \(.min)°C\n" +
            "    Max: \(.max)°C\n" +
            "    Mean: \(.mean)°C\n" +
            "    Std Dev: \(.stdDev)°C"'
        echo ""

        # Save results
        echo "$response" | jq '.data' > "$output"
        print_info "Results saved to $output"
    else
        print_error "Thermal analysis failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

analyze_oil() {
    local asset_id="$1"
    local sample_id="$2"
    shift 2
    local tests="$@"

    print_info "Performing oil analysis for $asset_id (Sample: $sample_id)..."

    # Parse test parameters
    local tests_json="{"
    for test in $tests; do
        IFS='=' read -r key value <<< "$test"
        if [[ "$key" == "particle-count" ]]; then
            tests_json+="\"particleCount\":{"
            IFS=',' read -ra COUNTS <<< "$value"
            for count in "${COUNTS[@]}"; do
                IFS=':' read -r size num <<< "$count"
                tests_json+="\"$size\":$num,"
            done
            tests_json="${tests_json%,}},"
        else
            tests_json+="\"${key/-/}\": $value,"
        fi
    done
    tests_json="${tests_json%,}}"

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "sampleId": "$sample_id",
  "tests": $tests_json
}
EOF
)

    local response=$(api_call "POST" "/analysis/oil" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Oil analysis completed!"
        echo ""

        echo -e "${CYAN}${OIL} Oil Analysis Results:${NC}"
        echo ""

        local condition=$(echo "$response" | jq -r '.data.condition')
        echo -e "  Overall Condition: ${GREEN}$condition${NC}"
        echo ""

        if echo "$response" | jq -e '.data.isoCode' > /dev/null 2>&1; then
            local iso=$(echo "$response" | jq -r '.data.isoCode')
            echo -e "  ISO Cleanliness Code: ${YELLOW}$iso${NC}"
            echo ""
        fi

        echo -e "${CYAN}  ${TOOLS} Recommendations:${NC}"
        echo "$response" | jq -r '.data.recommendations[] | "    • \(.)"'

    else
        print_error "Oil analysis failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

analyze_acoustic() {
    local asset_id="$1"
    local audio="$2"
    local analysis_types="$3"

    print_info "Performing acoustic analysis on $asset_id..."

    if [ ! -f "$audio" ]; then
        print_error "Audio file not found: $audio"
        exit 1
    fi

    local audio_data=$(base64 "$audio")

    local types_array="["
    IFS=',' read -ra TYPES <<< "$analysis_types"
    for type in "${TYPES[@]}"; do
        types_array+="\"${type^^}\","
    done
    types_array="${types_array%,}]"

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "audioData": "$audio_data",
  "samplingRate": 44100,
  "analysisTypes": $types_array
}
EOF
)

    local response=$(api_call "POST" "/analysis/acoustic" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Acoustic analysis completed!"
        echo ""

        echo -e "${CYAN}${SOUND} Acoustic Analysis Results:${NC}"
        echo ""

        local assessment=$(echo "$response" | jq -r '.data.assessment')
        echo -e "  Assessment: ${YELLOW}$assessment${NC}"
        echo ""

        local anomaly_count=$(echo "$response" | jq -r '.data.anomalies | length')
        if [ "$anomaly_count" -gt 0 ]; then
            echo -e "${CYAN}  ${WARNING} Anomalies Detected ($anomaly_count):${NC}"
            echo "$response" | jq -r '.data.anomalies[] |
                "    • \(.type) at \(.frequency) Hz - \(.severity) (\(.amplitude) dB)"'
        else
            print_success "  No anomalies detected"
        fi
    else
        print_error "Acoustic analysis failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

###############################################################################
# Predictions
###############################################################################

predict() {
    local asset_id="$1"
    local horizon="${2:-30}"
    local threshold="${3:-70}"
    local format="${4:-table}"

    print_info "Analyzing failure predictions for $asset_id..."

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "timeHorizon": $horizon,
  "threshold": $threshold,
  "includeRecommendations": true
}
EOF
)

    local response=$(api_call "POST" "/predictions/analyze" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Prediction analysis completed!"
        echo ""

        local overall_risk=$(echo "$response" | jq -r '.data.overallRisk')
        echo -e "${CYAN}${CHART} Prediction Results:${NC}"
        echo ""
        echo -e "  Overall Risk: ${YELLOW}$overall_risk%${NC}"
        echo ""

        local pred_count=$(echo "$response" | jq -r '.data.predictions | length')
        if [ "$pred_count" -gt 0 ]; then
            echo -e "${CYAN}  ${BELL} Predicted Failures:${NC}"
            echo ""

            if [ "$format" = "table" ]; then
                printf "  %-25s %-15s %-12s %-12s\n" "Failure Mode" "Probability" "Days" "Severity"
                printf "  %-25s %-15s %-12s %-12s\n" "-------------------------" "---------------" "------------" "------------"

                echo "$response" | jq -r '.data.predictions[] |
                    "\(.failureMode)|\(.probability)%|\(.daysToFailure)|\(.severity)"' | while IFS='|' read -r mode prob days sev; do
                    printf "  %-25s %-15s %-12s %-12s\n" "$mode" "$prob" "$days" "$sev"
                done
            else
                echo "$response" | jq -r '.data.predictions[] |
                    "  • \(.failureMode)\n" +
                    "    Probability: \(.probability)%\n" +
                    "    Days to Failure: \(.daysToFailure)\n" +
                    "    Severity: \(.severity)\n" +
                    "    Recommended Maintenance: \(.recommendedMaintenanceDate)\n"'
            fi
        else
            print_success "  No failures predicted in the next $horizon days"
        fi

        echo ""

        # RUL
        if echo "$response" | jq -e '.data.remainingUsefulLife' > /dev/null 2>&1; then
            local rul=$(echo "$response" | jq -r '.data.remainingUsefulLife.days')
            local conf=$(echo "$response" | jq -r '.data.remainingUsefulLife.confidence')
            echo -e "${CYAN}  Remaining Useful Life:${NC}"
            echo -e "    ${CLOCK} $rul days (confidence: $(awk "BEGIN {printf \"%.0f\", $conf*100}")%)"
        fi
    else
        print_error "Prediction failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

###############################################################################
# Maintenance
###############################################################################

maintenance_create_wo() {
    local asset_id="$1"
    local priority="$2"
    local failure_mode="$3"
    local scheduled_date="$4"

    print_info "Creating work order for $asset_id..."

    local data=$(cat <<EOF
{
  "assetId": "$asset_id",
  "priority": "$priority",
  "predictedFailure": "$failure_mode",
  "recommendedActions": ["Inspect and replace if necessary"],
  "scheduledDate": "$scheduled_date"
}
EOF
)

    local response=$(api_call "POST" "/maintenance/work-orders" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Work order created successfully!"
        echo ""

        local wo_id=$(echo "$response" | jq -r '.data.workOrderId')
        echo -e "${CYAN}${TOOLS} Work Order Details:${NC}"
        echo "$response" | jq -r '.data |
            "  ID: \(.workOrderId)\n" +
            "  Asset: \(.assetId)\n" +
            "  Priority: \(.priority)\n" +
            "  Status: \(.status)\n" +
            "  Scheduled: \(.scheduledDate // "Not scheduled")"'
    else
        print_error "Failed to create work order"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

maintenance_optimize() {
    local assets="$1"
    local window="$2"
    local crews="${3:-2}"

    print_info "Optimizing maintenance schedule..."

    local assets_array="["
    IFS=',' read -ra ASSETS <<< "$assets"
    for asset in "${ASSETS[@]}"; do
        assets_array+="\"$asset\","
    done
    assets_array="${assets_array%,}]"

    IFS='/' read -r start end <<< "$window"

    local data=$(cat <<EOF
{
  "assets": $assets_array,
  "constraints": {
    "maintenanceWindows": [{
      "start": "$start",
      "end": "$end"
    }],
    "availableCrews": $crews
  },
  "objectives": ["MINIMIZE_DOWNTIME", "MINIMIZE_COST"]
}
EOF
)

    local response=$(api_call "POST" "/maintenance/optimize" "$data")

    if echo "$response" | grep -q '"status":"success"'; then
        print_success "Optimization completed!"
        echo ""

        echo -e "${CYAN}${ROCKET} Optimized Schedule:${NC}"
        echo ""

        local total_downtime=$(echo "$response" | jq -r '.data.totalDowntime')
        local total_cost=$(echo "$response" | jq -r '.data.totalCost')
        local savings=$(echo "$response" | jq -r '.data.totalSavings')

        echo -e "  Total Downtime: ${YELLOW}$total_downtime hours${NC}"
        echo -e "  Total Cost: ${YELLOW}\$$total_cost${NC}"
        echo -e "  Savings: ${GREEN}\$$savings${NC}"
        echo ""

        local bundle_count=$(echo "$response" | jq -r '.data.bundles | length')
        echo -e "${CYAN}  Maintenance Bundles ($bundle_count):${NC}"
        echo "$response" | jq -r '.data.bundles[] |
            "  • Bundle \(.bundleId): \(.assets | join(", "))\n" +
            "    Window: \(.window.start) - \(.window.end)\n" +
            "    Downtime: \(.totalDowntime) hours\n"'
    else
        print_error "Optimization failed"
        echo "$response" | jq -r '.error.message'
        exit 1
    fi
}

###############################################################################
# Monitoring
###############################################################################

monitor_dashboard() {
    local assets="$1"
    local refresh="${2:-5}"

    print_info "Starting real-time monitoring dashboard..."
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""

    IFS=',' read -ra ASSETS <<< "$assets"

    while true; do
        clear
        print_header

        for asset in "${ASSETS[@]}"; do
            echo -e "${CYAN}${GEAR} $asset${NC}"

            # Get latest predictions
            local pred=$(api_call "POST" "/predictions/analyze" "{\"assetId\":\"$asset\",\"timeHorizon\":7}")

            if echo "$pred" | grep -q '"status":"success"'; then
                local risk=$(echo "$pred" | jq -r '.data.overallRisk')
                local rul=$(echo "$pred" | jq -r '.data.remainingUsefulLife.days // "N/A"')

                echo -e "  Risk: ${YELLOW}$risk%${NC} | RUL: ${CLOCK} $rul days"

                local pred_count=$(echo "$pred" | jq -r '.data.predictions | length')
                if [ "$pred_count" -gt 0 ]; then
                    echo "$pred" | jq -r '.data.predictions[0] | "  ⚠️  \(.failureMode): \(.probability)% in \(.daysToFailure) days"'
                fi
            fi

            echo ""
        done

        echo -e "${GRAY}Last updated: $(date)${NC}"
        echo -e "${GRAY}Refreshing in ${refresh}s...${NC}"

        sleep "$refresh"
    done
}

###############################################################################
# Help and Usage
###############################################################################

show_help() {
    print_header

    echo -e "${CYAN}USAGE:${NC}"
    echo "  wia-ind-026 [command] [options]"
    echo ""

    echo -e "${CYAN}COMMANDS:${NC}"
    echo ""

    echo -e "${YELLOW}Asset Management:${NC}"
    echo "  asset register --id ID --type TYPE --location LOC [--specs KEY=VAL,...]"
    echo "  asset get --id ID"
    echo "  asset list [--type TYPE] [--location LOC]"
    echo ""

    echo -e "${YELLOW}Sensor Management:${NC}"
    echo "  sensor attach --asset ID --sensor ID --type TYPE --location LOC --sampling-rate RATE"
    echo ""

    echo -e "${YELLOW}Data Ingestion:${NC}"
    echo "  data ingest --sensor ID --file FILE [--format csv]"
    echo ""

    echo -e "${YELLOW}Analysis:${NC}"
    echo "  analyze vibration --asset ID --sensor ID --analysis TYPE,... [--output FILE]"
    echo "  analyze thermal --asset ID --image FILE [--reference-temp TEMP] [--output FILE]"
    echo "  analyze oil --asset ID --sample-id ID --viscosity VAL --water-content VAL [...]"
    echo "  analyze acoustic --asset ID --audio FILE --analysis TYPE,..."
    echo ""

    echo -e "${YELLOW}Predictions:${NC}"
    echo "  predict --asset ID [--horizon DAYS] [--threshold PCT] [--format table|json]"
    echo ""

    echo -e "${YELLOW}Maintenance:${NC}"
    echo "  maintenance create-wo --asset ID --priority LEVEL --failure-mode MODE --scheduled-date DATE"
    echo "  maintenance optimize --assets ID,ID,... --window START/END [--crews N]"
    echo ""

    echo -e "${YELLOW}Monitoring:${NC}"
    echo "  monitor dashboard --assets ID,ID,... [--refresh SECONDS]"
    echo ""

    echo -e "${YELLOW}Other:${NC}"
    echo "  --version           Show version"
    echo "  --help              Show this help"
    echo ""

    echo -e "${CYAN}EXAMPLES:${NC}"
    echo "  wia-ind-026 asset register --id MOTOR-001 --type ROTATING_MACHINERY --location Plant-A"
    echo "  wia-ind-026 predict --asset MOTOR-001 --horizon 30 --threshold 70"
    echo "  wia-ind-026 analyze vibration --asset MOTOR-001 --sensor VIB-001 --analysis fft,envelope"
    echo ""

    echo -e "${CYAN}ENVIRONMENT:${NC}"
    echo "  WIA_IND_026_API_KEY      API authentication key"
    echo "  WIA_IND_026_ORG_ID       Organization ID"
    echo "  WIA_IND_026_ENDPOINT     API endpoint (default: https://api.wia.org/ind-026/v1)"
    echo ""

    echo -e "${BLUE}弘익人間 (홍익인간) · Benefit All Humanity${NC}"
    echo ""
}

###############################################################################
# Main
###############################################################################

main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command="$1"
    shift

    case "$command" in
        --version|-v)
            echo "WIA-IND-026 CLI v${VERSION}"
            ;;
        --help|-h)
            show_help
            ;;
        asset)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                register)
                    # Parse arguments
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --id) ASSET_ID="$2"; shift 2 ;;
                            --type) ASSET_TYPE="$2"; shift 2 ;;
                            --location) LOCATION="$2"; shift 2 ;;
                            --specs) SPECS="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    asset_register "$ASSET_ID" "$ASSET_TYPE" "$LOCATION" "$SPECS"
                    ;;
                get)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --id) ASSET_ID="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    asset_get "$ASSET_ID"
                    ;;
                list)
                    TYPE=""
                    LOCATION=""
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --type) TYPE="$2"; shift 2 ;;
                            --location) LOCATION="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    asset_list "$TYPE" "$LOCATION"
                    ;;
                *)
                    print_error "Unknown asset command: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        sensor)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                attach)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --sensor) SENSOR_ID="$2"; shift 2 ;;
                            --type) SENSOR_TYPE="$2"; shift 2 ;;
                            --location) LOCATION="$2"; shift 2 ;;
                            --sampling-rate) SAMPLING_RATE="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    sensor_attach "$ASSET_ID" "$SENSOR_ID" "$SENSOR_TYPE" "$LOCATION" "$SAMPLING_RATE"
                    ;;
                *)
                    print_error "Unknown sensor command: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        data)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                ingest)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --sensor) SENSOR_ID="$2"; shift 2 ;;
                            --file) FILE="$2"; shift 2 ;;
                            --format) FORMAT="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    data_ingest "$SENSOR_ID" "$FILE" "${FORMAT:-csv}"
                    ;;
                *)
                    print_error "Unknown data command: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        analyze)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                vibration)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --sensor) SENSOR_ID="$2"; shift 2 ;;
                            --analysis) ANALYSIS="$2"; shift 2 ;;
                            --output) OUTPUT="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    analyze_vibration "$ASSET_ID" "$SENSOR_ID" "$ANALYSIS" "${OUTPUT:-table}"
                    ;;
                thermal)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --image) IMAGE="$2"; shift 2 ;;
                            --reference-temp) REF_TEMP="$2"; shift 2 ;;
                            --output) OUTPUT="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    analyze_thermal "$ASSET_ID" "$IMAGE" "${REF_TEMP:-25}" "${OUTPUT:-thermal-report.json}"
                    ;;
                oil)
                    ASSET_ID=""
                    SAMPLE_ID=""
                    TESTS=""
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --sample-id) SAMPLE_ID="$2"; shift 2 ;;
                            --*) TESTS="$TESTS ${1#--}=$2"; shift 2 ;;
                        esac
                    done
                    analyze_oil "$ASSET_ID" "$SAMPLE_ID" $TESTS
                    ;;
                acoustic)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --audio) AUDIO="$2"; shift 2 ;;
                            --analysis) ANALYSIS="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    analyze_acoustic "$ASSET_ID" "$AUDIO" "$ANALYSIS"
                    ;;
                *)
                    print_error "Unknown analysis type: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        predict)
            check_config
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --asset) ASSET_ID="$2"; shift 2 ;;
                    --horizon) HORIZON="$2"; shift 2 ;;
                    --threshold) THRESHOLD="$2"; shift 2 ;;
                    --format) FORMAT="$2"; shift 2 ;;
                    *) shift ;;
                esac
            done
            predict "$ASSET_ID" "${HORIZON:-30}" "${THRESHOLD:-70}" "${FORMAT:-table}"
            ;;
        maintenance)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                create-wo)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --asset) ASSET_ID="$2"; shift 2 ;;
                            --priority) PRIORITY="$2"; shift 2 ;;
                            --failure-mode) FAILURE_MODE="$2"; shift 2 ;;
                            --scheduled-date) SCHEDULED_DATE="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    maintenance_create_wo "$ASSET_ID" "$PRIORITY" "$FAILURE_MODE" "$SCHEDULED_DATE"
                    ;;
                optimize)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --assets) ASSETS="$2"; shift 2 ;;
                            --window) WINDOW="$2"; shift 2 ;;
                            --crews) CREWS="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    maintenance_optimize "$ASSETS" "$WINDOW" "${CREWS:-2}"
                    ;;
                *)
                    print_error "Unknown maintenance command: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        monitor)
            check_config
            local subcommand="$1"
            shift
            case "$subcommand" in
                dashboard)
                    while [[ $# -gt 0 ]]; do
                        case $1 in
                            --assets) ASSETS="$2"; shift 2 ;;
                            --refresh) REFRESH="$2"; shift 2 ;;
                            *) shift ;;
                        esac
                    done
                    monitor_dashboard "$ASSETS" "${REFRESH:-5}"
                    ;;
                *)
                    print_error "Unknown monitor command: $subcommand"
                    exit 1
                    ;;
            esac
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            echo "Run 'wia-ind-026 --help' for usage information"
            exit 1
            ;;
    esac
}

# Run main
main "$@"

###############################################################################
# 弘익人間 (홍익인간) · Benefit All Humanity
###############################################################################
