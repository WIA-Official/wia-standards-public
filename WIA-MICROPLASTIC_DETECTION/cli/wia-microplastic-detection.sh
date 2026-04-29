#!/usr/bin/env bash

# WIA-MICROPLASTIC_DETECTION CLI
# Command-line interface for microplastic detection and analysis
#
# Version: 1.0.0
# Philosophy: 弘益人間 (Benefit All Humanity)
#
# Usage:
#   wia-microplastic-detection sample-analyze <sample_id>
#   wia-microplastic-detection particle-count <image_path>
#   wia-microplastic-detection spectrum-identify <spectrum_file>
#   wia-microplastic-detection report-generate <sample_id>
#   wia-microplastic-detection sensor-read <sensor_id>
#   wia-microplastic-detection --help

set -euo pipefail

# ============================================================================
# Configuration
# ============================================================================

VERSION="1.0.0"
API_BASE_URL="${WIA_API_URL:-https://api.wia.org/microplastic-detection/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/microplastic-detection.conf"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# Helper Functions
# ============================================================================

# Print colored output
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

# Check if API key is configured
check_api_key() {
    if [[ -z "$API_KEY" ]]; then
        if [[ -f "$CONFIG_FILE" ]]; then
            # shellcheck source=/dev/null
            source "$CONFIG_FILE"
        fi
    fi

    if [[ -z "$API_KEY" ]]; then
        print_error "API key not configured"
        print_info "Set WIA_API_KEY environment variable or create $CONFIG_FILE"
        print_info "Example: export WIA_API_KEY='your_api_key'"
        exit 1
    fi
}

# Make API request
api_request() {
    local method=$1
    local endpoint=$2
    local data=${3:-}

    local url="${API_BASE_URL}${endpoint}"
    local response
    local http_code

    if [[ -n "$data" ]]; then
        response=$(curl -s -w "\n%{http_code}" -X "$method" "$url" \
            -H "Authorization: Bearer $API_KEY" \
            -H "Content-Type: application/json" \
            -d "$data")
    else
        response=$(curl -s -w "\n%{http_code}" -X "$method" "$url" \
            -H "Authorization: Bearer $API_KEY")
    fi

    http_code=$(echo "$response" | tail -n1)
    body=$(echo "$response" | sed '$d')

    if [[ $http_code -ge 200 && $http_code -lt 300 ]]; then
        echo "$body"
        return 0
    else
        print_error "API request failed (HTTP $http_code)"
        echo "$body" | jq -r '.error.message // .message // "Unknown error"' >&2
        return 1
    fi
}

# Wait for job completion
wait_for_job() {
    local job_id=$1
    local timeout=${2:-3600}
    local elapsed=0

    print_info "Waiting for job $job_id to complete..."

    while [[ $elapsed -lt $timeout ]]; do
        local status_response
        status_response=$(api_request GET "/jobs/$job_id")

        local status
        status=$(echo "$status_response" | jq -r '.status')

        local progress
        progress=$(echo "$status_response" | jq -r '.progress // 0')

        case "$status" in
            COMPLETED)
                print_success "Job completed successfully"
                return 0
                ;;
            FAILED)
                local error_msg
                error_msg=$(echo "$status_response" | jq -r '.errorMessage // "Unknown error"')
                print_error "Job failed: $error_msg"
                return 1
                ;;
            CANCELLED)
                print_warning "Job was cancelled"
                return 1
                ;;
            IN_PROGRESS|QUEUED)
                printf "\rProgress: %d%%" "$progress"
                sleep 5
                elapsed=$((elapsed + 5))
                ;;
        esac
    done

    print_error "Job timeout after ${timeout}s"
    return 1
}

# ============================================================================
# Command Functions
# ============================================================================

# Analyze a sample
cmd_sample_analyze() {
    local sample_id=$1
    local techniques=${2:-"RAMAN,FTIR"}

    check_api_key

    print_info "Submitting sample $sample_id for analysis..."

    local job_data
    job_data=$(cat <<EOF
{
  "analysisMethod": "AUTOMATED_IMAGING",
  "techniques": $(echo "$techniques" | jq -R 'split(",")'),
  "minParticleSize": 10,
  "maxParticleSize": 5000,
  "minPolymerConfidence": 0.80,
  "priority": "NORMAL"
}
EOF
)

    local job_response
    job_response=$(api_request POST "/samples/$sample_id/analyze" "$job_data")

    local job_id
    job_id=$(echo "$job_response" | jq -r '.jobId')

    print_success "Analysis job created: $job_id"

    # Wait for completion
    if wait_for_job "$job_id"; then
        echo ""
        print_info "Retrieving results..."

        local results
        results=$(api_request GET "/samples/$sample_id/results")

        echo "$results" | jq '{
            totalParticles,
            concentration,
            polymerDistribution,
            sizeDistribution: {
                meanSize: .sizeDistribution.meanSize,
                medianSize: .sizeDistribution.medianSize
            }
        }'

        print_success "Analysis complete"
    fi
}

# Count particles in image
cmd_particle_count() {
    local image_path=$1
    local sample_id=${2:-"temp-sample"}

    check_api_key

    if [[ ! -f "$image_path" ]]; then
        print_error "Image file not found: $image_path"
        exit 1
    fi

    print_info "Uploading image for particle detection..."

    local job_response
    job_response=$(curl -s -X POST "${API_BASE_URL}/images/analyze" \
        -H "Authorization: Bearer $API_KEY" \
        -F "image=@$image_path" \
        -F "sampleId=$sample_id" \
        -F "imageType=optical")

    local job_id
    job_id=$(echo "$job_response" | jq -r '.jobId')

    print_success "Image analysis job created: $job_id"

    sleep 5

    print_info "Retrieving particle count..."

    local results
    results=$(api_request GET "/images/analyze/$job_id/results")

    local particle_count
    particle_count=$(echo "$results" | jq -r '.particlesDetected')

    print_success "Detected $particle_count particles"

    echo "$results" | jq '.particles[] | {
        size: .size,
        shape: .shape,
        confidence: .confidence
    }'
}

# Identify polymer from spectrum
cmd_spectrum_identify() {
    local spectrum_file=$1
    local spectrum_type=${2:-"RAMAN"}

    check_api_key

    if [[ ! -f "$spectrum_file" ]]; then
        print_error "Spectrum file not found: $spectrum_file"
        exit 1
    fi

    print_info "Reading spectrum data from $spectrum_file..."

    # Assume spectrum file is JSON with wavenumbers and intensities
    local spectrum_data
    spectrum_data=$(cat "$spectrum_file")

    local identify_data
    identify_data=$(cat <<EOF
{
  "spectrumType": "$spectrum_type",
  "wavenumbers": $(echo "$spectrum_data" | jq '.wavenumbers'),
  "intensities": $(echo "$spectrum_data" | jq '.intensities'),
  "spectralLibrary": "WILEY_KNOWITALL"
}
EOF
)

    print_info "Identifying polymer..."

    local results
    results=$(api_request POST "/spectra/identify" "$identify_data")

    local best_match
    best_match=$(echo "$results" | jq -r '.bestMatch.polymerType')
    local confidence
    confidence=$(echo "$results" | jq -r '.bestMatch.confidence')

    print_success "Best match: $best_match (confidence: $confidence)"

    echo "$results" | jq '.matches[] | {
        polymer: .polymerType,
        confidence,
        hitQuality,
        libraryEntry
    }'
}

# Generate report
cmd_report_generate() {
    local sample_id=$1
    local format=${2:-"PDF"}
    local output_file=${3:-"report.$format"}

    check_api_key

    print_info "Generating $format report for sample $sample_id..."

    local report_data
    report_data=$(cat <<EOF
{
  "reportType": "SAMPLE_ANALYSIS",
  "sampleIds": ["$sample_id"],
  "format": "$format",
  "includeCharts": true,
  "includeRawData": false,
  "language": "en"
}
EOF
)

    local report_response
    report_response=$(api_request POST "/reports/generate" "$report_data")

    local report_id
    report_id=$(echo "$report_response" | jq -r '.reportId')

    print_success "Report generation started: $report_id"

    sleep 3

    print_info "Waiting for report generation..."

    local max_wait=60
    local elapsed=0
    while [[ $elapsed -lt $max_wait ]]; do
        local status_response
        status_response=$(api_request GET "/reports/$report_id")

        local status
        status=$(echo "$status_response" | jq -r '.status')

        if [[ "$status" == "COMPLETED" ]]; then
            print_success "Report ready"

            print_info "Downloading report to $output_file..."

            curl -s -o "$output_file" \
                -H "Authorization: Bearer $API_KEY" \
                "${API_BASE_URL}/reports/$report_id/download"

            print_success "Report saved to $output_file"
            return 0
        elif [[ "$status" == "FAILED" ]]; then
            print_error "Report generation failed"
            return 1
        fi

        sleep 2
        elapsed=$((elapsed + 2))
    done

    print_error "Report generation timeout"
    return 1
}

# Read sensor data
cmd_sensor_read() {
    local sensor_id=$1
    local days=${2:-7}

    check_api_key

    print_info "Retrieving sensor readings for $sensor_id (last $days days)..."

    local end_date
    end_date=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
    local start_date
    start_date=$(date -u -d "$days days ago" +"%Y-%m-%dT%H:%M:%SZ" 2>/dev/null || date -u -v-"${days}d" +"%Y-%m-%dT%H:%M:%SZ")

    local readings
    readings=$(api_request GET "/sensors/$sensor_id/readings?startDate=$start_date&endDate=$end_date")

    local total_readings
    total_readings=$(echo "$readings" | jq -r '.summary.totalReadings')

    print_success "Retrieved $total_readings readings"

    echo "$readings" | jq '{
        sensor: .sensorName,
        summary: .summary,
        recentReadings: .readings[-10:] | .[] | {
            timestamp,
            particleCount,
            temperature: .waterQuality.temperature
        }
    }'
}

# List samples
cmd_sample_list() {
    local limit=${1:-50}
    local status=${2:-""}

    check_api_key

    print_info "Listing samples..."

    local params="limit=$limit"
    if [[ -n "$status" ]]; then
        params="$params&status=$status"
    fi

    local response
    response=$(api_request GET "/samples?$params")

    local total
    total=$(echo "$response" | jq -r '.total')

    print_success "Found $total samples"

    echo "$response" | jq -r '.samples[] | "\(.sampleId)\t\(.sampleName)\t\(.status)\t\(.totalParticleCount // 0) particles"' | column -t -s $'\t'
}

# Get sample details
cmd_sample_get() {
    local sample_id=$1

    check_api_key

    print_info "Retrieving sample $sample_id..."

    local sample
    sample=$(api_request GET "/samples/$sample_id")

    echo "$sample" | jq '{
        sampleId,
        sampleName,
        collectedAt,
        location: {
            siteName: .location.siteName,
            latitude: .location.latitude,
            longitude: .location.longitude
        },
        environmentType,
        status,
        totalParticleCount,
        particleConcentration,
        dominantPolymer
    }'
}

# ============================================================================
# Help Function
# ============================================================================

show_help() {
    cat <<EOF
WIA-MICROPLASTIC_DETECTION CLI v$VERSION
弘益人間 (Benefit All Humanity)

USAGE:
    wia-microplastic-detection <command> [arguments]

COMMANDS:
    sample-analyze <sample_id> [techniques]
        Analyze a microplastic sample
        techniques: Comma-separated list (default: RAMAN,FTIR)

    particle-count <image_path> [sample_id]
        Count particles in an image using AI detection
        image_path: Path to microscopy image

    spectrum-identify <spectrum_file> [type]
        Identify polymer from spectroscopic data
        spectrum_file: JSON file with wavenumbers and intensities
        type: RAMAN or FTIR (default: RAMAN)

    report-generate <sample_id> [format] [output_file]
        Generate analysis report
        format: PDF, HTML, or DOCX (default: PDF)
        output_file: Output filename (default: report.FORMAT)

    sensor-read <sensor_id> [days]
        Read sensor monitoring data
        days: Number of days to retrieve (default: 7)

    sample-list [limit] [status]
        List samples with optional filtering
        limit: Maximum results (default: 50)
        status: Filter by status (ANALYZED, PENDING_ANALYSIS, etc.)

    sample-get <sample_id>
        Get detailed sample information

    --help, -h
        Show this help message

    --version, -v
        Show version information

CONFIGURATION:
    Set API key via environment variable:
        export WIA_API_KEY='your_api_key'

    Or create configuration file:
        mkdir -p ~/.wia
        echo 'API_KEY="your_api_key"' > ~/.wia/microplastic-detection.conf

    Set custom API URL (optional):
        export WIA_API_URL='https://custom-api.example.com/v1'

EXAMPLES:
    # Analyze a sample
    wia-microplastic-detection sample-analyze sample-2026-001

    # Count particles in an image
    wia-microplastic-detection particle-count microscopy-image.jpg

    # Identify polymer from Raman spectrum
    wia-microplastic-detection spectrum-identify spectrum.json RAMAN

    # Generate PDF report
    wia-microplastic-detection report-generate sample-2026-001 PDF report.pdf

    # Read sensor data for the last 14 days
    wia-microplastic-detection sensor-read sensor-smb-001 14

    # List analyzed samples
    wia-microplastic-detection sample-list 100 ANALYZED

DOCUMENTATION:
    https://wia.org/standards/microplastic-detection

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
EOF
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    if [[ $# -eq 0 ]]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case "$command" in
        sample-analyze)
            if [[ $# -lt 1 ]]; then
                print_error "Missing sample_id argument"
                echo "Usage: wia-microplastic-detection sample-analyze <sample_id> [techniques]"
                exit 1
            fi
            cmd_sample_analyze "$@"
            ;;
        particle-count)
            if [[ $# -lt 1 ]]; then
                print_error "Missing image_path argument"
                echo "Usage: wia-microplastic-detection particle-count <image_path> [sample_id]"
                exit 1
            fi
            cmd_particle_count "$@"
            ;;
        spectrum-identify)
            if [[ $# -lt 1 ]]; then
                print_error "Missing spectrum_file argument"
                echo "Usage: wia-microplastic-detection spectrum-identify <spectrum_file> [type]"
                exit 1
            fi
            cmd_spectrum_identify "$@"
            ;;
        report-generate)
            if [[ $# -lt 1 ]]; then
                print_error "Missing sample_id argument"
                echo "Usage: wia-microplastic-detection report-generate <sample_id> [format] [output_file]"
                exit 1
            fi
            cmd_report_generate "$@"
            ;;
        sensor-read)
            if [[ $# -lt 1 ]]; then
                print_error "Missing sensor_id argument"
                echo "Usage: wia-microplastic-detection sensor-read <sensor_id> [days]"
                exit 1
            fi
            cmd_sensor_read "$@"
            ;;
        sample-list)
            cmd_sample_list "$@"
            ;;
        sample-get)
            if [[ $# -lt 1 ]]; then
                print_error "Missing sample_id argument"
                echo "Usage: wia-microplastic-detection sample-get <sample_id>"
                exit 1
            fi
            cmd_sample_get "$@"
            ;;
        --help|-h)
            show_help
            ;;
        --version|-v)
            echo "WIA-MICROPLASTIC_DETECTION CLI v$VERSION"
            echo "弘益人間 (Benefit All Humanity)"
            ;;
        *)
            print_error "Unknown command: $command"
            echo "Run 'wia-microplastic-detection --help' for usage information"
            exit 1
            ;;
    esac
}

main "$@"
