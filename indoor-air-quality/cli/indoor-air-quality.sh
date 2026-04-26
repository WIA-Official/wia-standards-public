#!/bin/bash

# WIA-ENE-027: Indoor Air Quality Standard - CLI Tool
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-027 API

set -e

# Configuration
API_ENDPOINT="${WIA_ENE027_ENDPOINT:-https://api.wia.org/ene-027/v1}"
API_KEY="${WIA_ENE027_API_KEY:-}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

print_json() {
    if command -v jq &> /dev/null; then
        echo "$1" | jq .
    else
        echo "$1"
    fi
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API 키가 설정되지 않았습니다."
        print_info "환경 변수 WIA_ENE027_API_KEY를 설정하거나 --api-key 옵션을 사용하세요."
        exit 1
    fi
}

api_request() {
    local method="$1"
    local endpoint="$2"
    local data="$3"

    check_api_key

    local curl_opts=(
        -X "$method"
        -H "Content-Type: application/json"
        -H "Authorization: Bearer $API_KEY"
        -s
    )

    if [ -n "$data" ]; then
        curl_opts+=(-d "$data")
    fi

    local response=$(curl "${curl_opts[@]}" "${API_ENDPOINT}${endpoint}")
    echo "$response"
}

# Commands
cmd_register() {
    print_header "공간 등록"

    local space_id=""
    local name=""
    local type="OFFICE"
    local area="100"
    local height="2.7"
    local max_occupancy="20"
    local lat=""
    local lon=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --space-id) space_id="$2"; shift 2 ;;
            --name) name="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --area) area="$2"; shift 2 ;;
            --height) height="$2"; shift 2 ;;
            --occupancy) max_occupancy="$2"; shift 2 ;;
            --lat) lat="$2"; shift 2 ;;
            --lon) lon="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    # Validate required fields
    if [ -z "$space_id" ] || [ -z "$name" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 register --space-id ID --name NAME"
        echo "        [--type TYPE] [--area M2] [--height M] [--occupancy N]"
        echo "        [--lat LAT] [--lon LON]"
        exit 1
    fi

    # Calculate volume
    local volume=$(echo "$area * $height" | bc)

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "space": {
    "spaceId": "$space_id",
    "name": "$name",
    "type": "$type",
    "location": {
      "lat": ${lat:-37.5665},
      "lon": ${lon:-126.9780}
    },
    "floorArea_m2": $area,
    "ceilingHeight_m": $height,
    "volume_m3": $volume,
    "occupancy": {
      "maxOccupancy": $max_occupancy
    }
  },
  "sensors": []
}
EOF
)

    print_info "공간 등록 중..."
    response=$(api_request POST "/spaces/register" "$payload")

    if echo "$response" | grep -q "spaceId"; then
        print_success "공간이 성공적으로 등록되었습니다!"
        echo ""
        print_json "$response"

        # Extract spaceId
        if command -v jq &> /dev/null; then
            dashboard_url=$(echo "$response" | jq -r '.data.dashboardUrl')
            echo ""
            print_info "대시보드 URL: $dashboard_url"
        fi
    else
        print_error "공간 등록 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_status() {
    local space_id="$1"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 status SPACE_ID"
        exit 1
    fi

    print_header "공기질 현황: $space_id"
    print_info "현재 상태 조회 중..."

    response=$(api_request GET "/spaces/$space_id/status" "")

    if echo "$response" | grep -q "reading"; then
        print_success "공기질 정보를 찾았습니다!"
        echo ""

        if command -v jq &> /dev/null; then
            # Parse and display key metrics
            iaq=$(echo "$response" | jq -r '.data.reading.iaq.overall // "N/A"')
            category=$(echo "$response" | jq -r '.data.reading.iaq.category // "N/A"')
            temp=$(echo "$response" | jq -r '.data.reading.environmental.temperature_c // "N/A"')
            humidity=$(echo "$response" | jq -r '.data.reading.environmental.humidity_percent // "N/A"')
            co2=$(echo "$response" | jq -r '.data.reading.gasConcentrations.co2_ppm // "N/A"')
            pm25=$(echo "$response" | jq -r '.data.reading.particulateMatter.pm25_ugm3 // "N/A"')

            echo -e "${CYAN}실내 공기질 지수 (IAQ):${NC} $iaq ($category)"
            echo -e "${CYAN}온도:${NC} ${temp}°C"
            echo -e "${CYAN}습도:${NC} ${humidity}%"
            echo -e "${CYAN}CO₂:${NC} ${co2} ppm"
            echo -e "${CYAN}PM2.5:${NC} ${pm25} μg/m³"
            echo ""
        fi

        print_json "$response"
    else
        print_error "공기질 정보를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_submit() {
    print_header "측정값 제출"

    local space_id=""
    local temp=""
    local humidity=""
    local co2=""
    local pm25=""
    local pm10=""
    local tvoc=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --space-id) space_id="$2"; shift 2 ;;
            --temp) temp="$2"; shift 2 ;;
            --humidity) humidity="$2"; shift 2 ;;
            --co2) co2="$2"; shift 2 ;;
            --pm25) pm25="$2"; shift 2 ;;
            --pm10) pm10="$2"; shift 2 ;;
            --tvoc) tvoc="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$space_id" ]; then
        print_error "공간 ID가 필요합니다."
        echo "사용법: $0 submit --space-id SPACE_ID [--temp C] [--humidity %] [--co2 PPM] [--pm25 UG/M3]"
        exit 1
    fi

    # Build reading object
    local timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    local reading_id="${space_id}-$(date +%s)"

    local payload=$(cat <<EOF
{
  "readings": [
    {
      "readingId": "$reading_id",
      "spaceId": "$space_id",
      "timestamp": "$timestamp",
      "environmental": {
        "temperature_c": ${temp:-22},
        "humidity_percent": ${humidity:-50},
        "timestamp": "$timestamp"
      }
EOF
)

    # Add optional measurements
    if [ -n "$co2" ]; then
        payload+=",
      \"gasConcentrations\": {
        \"co2_ppm\": $co2,
        \"timestamp\": \"$timestamp\",
        \"qc_flag\": \"VALID\"
      }"
    fi

    if [ -n "$pm25" ] || [ -n "$pm10" ]; then
        payload+=",
      \"particulateMatter\": {
        \"pm25_ugm3\": ${pm25:-0},
        \"pm10_ugm3\": ${pm10:-0},
        \"timestamp\": \"$timestamp\",
        \"qc_flag\": \"VALID\"
      }"
    fi

    if [ -n "$tvoc" ]; then
        payload+=",
      \"vocMeasurements\": {
        \"tvoc_ppb\": $tvoc,
        \"timestamp\": \"$timestamp\",
        \"qc_flag\": \"VALID\"
      }"
    fi

    payload+=",
      \"sensorId\": \"CLI-SENSOR\",
      \"dataQuality\": {
        \"completeness_percent\": 100,
        \"validityScore\": 1.0
      }
    }
  ]
}"

    print_info "측정값 제출 중..."
    response=$(api_request POST "/readings/submit" "$payload")

    if echo "$response" | grep -q "accepted"; then
        print_success "측정값이 성공적으로 제출되었습니다!"
        echo ""
        print_json "$response"
    else
        print_error "측정값 제출 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_alerts() {
    local space_id="$1"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 alerts SPACE_ID"
        exit 1
    fi

    print_header "활성 경보: $space_id"
    print_info "경보 조회 중..."

    response=$(api_request GET "/spaces/$space_id/alerts/active" "")

    if echo "$response" | grep -q "data"; then
        if command -v jq &> /dev/null; then
            alert_count=$(echo "$response" | jq '.data | length')
            echo -e "${CYAN}활성 경보 수:${NC} $alert_count"
            echo ""
        fi
        print_json "$response"
    else
        print_error "경보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_report() {
    local space_id="$1"
    local report_type="${2:-DAILY}"
    local start_date="${3:-$(date -d '7 days ago' -u +%Y-%m-%dT00:00:00Z)}"
    local end_date="${4:-$(date -u +%Y-%m-%dT23:59:59Z)}"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 report SPACE_ID [REPORT_TYPE] [START_DATE] [END_DATE]"
        echo "보고서 유형: DAILY, WEEKLY, MONTHLY"
        exit 1
    fi

    print_header "보고서 생성: $space_id"
    print_info "보고서 생성 중 ($report_type)..."

    local payload=$(cat <<EOF
{
  "spaceId": "$space_id",
  "reportType": "$report_type",
  "startDate": "$start_date",
  "endDate": "$end_date"
}
EOF
)

    response=$(api_request POST "/reports/generate" "$payload")

    if echo "$response" | grep -q "reportId"; then
        print_success "보고서가 성공적으로 생성되었습니다!"
        echo ""
        print_json "$response"

        if command -v jq &> /dev/null; then
            report_id=$(echo "$response" | jq -r '.data.reportId')
            echo ""
            print_info "보고서 ID: $report_id"
            print_info "PDF 다운로드: ${API_ENDPOINT}/reports/${report_id}/export/pdf"
        fi
    else
        print_error "보고서 생성 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_hvac() {
    local space_id="$1"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 hvac SPACE_ID"
        exit 1
    fi

    print_header "HVAC 현황: $space_id"
    print_info "HVAC 상태 조회 중..."

    response=$(api_request GET "/spaces/$space_id/hvac/status" "")

    if echo "$response" | grep -q "data"; then
        print_success "HVAC 정보를 찾았습니다!"
        echo ""
        print_json "$response"
    else
        print_error "HVAC 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_ventilation() {
    local space_id="$1"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 ventilation SPACE_ID"
        exit 1
    fi

    print_header "환기 권장사항: $space_id"
    print_info "환기 권장사항 조회 중..."

    response=$(api_request GET "/spaces/$space_id/ventilation/recommendations" "")

    if echo "$response" | grep -q "currentACH"; then
        print_success "환기 권장사항을 찾았습니다!"
        echo ""

        if command -v jq &> /dev/null; then
            current_ach=$(echo "$response" | jq -r '.data.currentACH')
            recommended_ach=$(echo "$response" | jq -r '.data.recommendedACH')
            reason=$(echo "$response" | jq -r '.data.reason')

            echo -e "${CYAN}현재 환기율:${NC} $current_ach ACH"
            echo -e "${CYAN}권장 환기율:${NC} $recommended_ach ACH"
            echo -e "${CYAN}사유:${NC} $reason"
            echo ""
        fi

        print_json "$response"
    else
        print_error "환기 권장사항 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_compliance() {
    local space_id="$1"
    local standard="${2:-WHO_2021}"

    if [ -z "$space_id" ]; then
        print_error "공간 ID를 입력하세요."
        echo "사용법: $0 compliance SPACE_ID [STANDARD]"
        echo "표준: WHO_2021, ASHRAE_62.1, KR_IAQ"
        exit 1
    fi

    print_header "기준 준수 확인: $space_id"
    print_info "기준 준수 상태 확인 중 ($standard)..."

    response=$(api_request GET "/spaces/$space_id/compliance?standard=$standard" "")

    if echo "$response" | grep -q "compliant"; then
        if command -v jq &> /dev/null; then
            compliant=$(echo "$response" | jq -r '.data.compliant')
            if [ "$compliant" == "true" ]; then
                print_success "기준을 준수하고 있습니다!"
            else
                print_error "기준 미준수"
            fi
        fi
        echo ""
        print_json "$response"
    else
        print_error "기준 준수 확인 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_help() {
    cat <<EOF
${BLUE}WIA-ENE-027: 실내 공기질 표준 CLI${NC}
${CYAN}弘益人間 (홍익인간) - Benefit All Humanity${NC}

사용법: $0 <명령어> [옵션]

${YELLOW}명령어:${NC}
  register      새 공간 등록
  status        공기질 현황 조회
  submit        측정값 제출
  alerts        활성 경보 조회
  report        보고서 생성
  hvac          HVAC 현황 조회
  ventilation   환기 권장사항 조회
  compliance    기준 준수 확인
  help          도움말 표시

${YELLOW}환경 변수:${NC}
  WIA_ENE027_API_KEY      API 키 (필수)
  WIA_ENE027_ENDPOINT     API 엔드포인트 (기본값: https://api.wia.org/ene-027/v1)

${YELLOW}예제:${NC}
  # 공간 등록
  $0 register --space-id OFFICE-SEL-001 --name "서울 본사 3층" \\
     --type OFFICE --area 150 --height 2.7 --occupancy 20

  # 현재 상태 조회
  $0 status OFFICE-SEL-001

  # 측정값 제출
  $0 submit --space-id OFFICE-SEL-001 --temp 23.5 --humidity 55 \\
     --co2 850 --pm25 12 --pm10 28

  # 활성 경보 조회
  $0 alerts OFFICE-SEL-001

  # 보고서 생성 (주간)
  $0 report OFFICE-SEL-001 WEEKLY

  # HVAC 현황
  $0 hvac OFFICE-SEL-001

  # 환기 권장사항
  $0 ventilation OFFICE-SEL-001

  # 기준 준수 확인
  $0 compliance OFFICE-SEL-001 WHO_2021

${CYAN}🏠 건강한 실내 공기, 행복한 생활!${NC}
EOF
}

# Main
main() {
    if [ $# -eq 0 ]; then
        cmd_help
        exit 0
    fi

    local command="$1"
    shift

    case "$command" in
        register) cmd_register "$@" ;;
        status) cmd_status "$@" ;;
        submit) cmd_submit "$@" ;;
        alerts) cmd_alerts "$@" ;;
        report) cmd_report "$@" ;;
        hvac) cmd_hvac "$@" ;;
        ventilation) cmd_ventilation "$@" ;;
        compliance) cmd_compliance "$@" ;;
        help|--help|-h) cmd_help ;;
        *)
            print_error "알 수 없는 명령어: $command"
            echo ""
            cmd_help
            exit 1
            ;;
    esac
}

main "$@"
