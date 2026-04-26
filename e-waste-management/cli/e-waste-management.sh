#!/bin/bash

# WIA-ENE-025: E-Waste Management Standard - CLI Tool
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-025 API

set -e

# Configuration
API_ENDPOINT="${WIA_ENE025_ENDPOINT:-https://api.wia.org/ene-025/v1}"
API_KEY="${WIA_ENE025_API_KEY:-}"

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
        print_info "환경 변수 WIA_ENE025_API_KEY를 설정하거나 --api-key 옵션을 사용하세요."
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
    print_header "기기 등록"

    local category=""
    local type=""
    local brand=""
    local model=""
    local weight=""
    local condition="NON_FUNCTIONAL"
    local owner_type="INDIVIDUAL"
    local country="KR"
    local facility_id=""
    local facility_name=""
    local lat=""
    local lon=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --category) category="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --brand) brand="$2"; shift 2 ;;
            --model) model="$2"; shift 2 ;;
            --weight) weight="$2"; shift 2 ;;
            --condition) condition="$2"; shift 2 ;;
            --owner-type) owner_type="$2"; shift 2 ;;
            --country) country="$2"; shift 2 ;;
            --facility-id) facility_id="$2"; shift 2 ;;
            --facility-name) facility_name="$2"; shift 2 ;;
            --lat) lat="$2"; shift 2 ;;
            --lon) lon="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    # Validate required fields
    if [ -z "$type" ] || [ -z "$brand" ] || [ -z "$model" ] || [ -z "$weight" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 register --type TYPE --brand BRAND --model MODEL --weight WEIGHT"
        echo "        [--category CATEGORY] [--condition CONDITION]"
        echo "        [--facility-id ID] [--facility-name NAME] [--lat LAT] [--lon LON]"
        exit 1
    fi

    # Auto-detect category if not provided
    if [ -z "$category" ]; then
        case $type in
            SMARTPHONE|TABLET|LAPTOP|DESKTOP|SERVER) category="IT_EQUIPMENT" ;;
            REFRIGERATOR|WASHING_MACHINE|AIR_CONDITIONER) category="LARGE_HOUSEHOLD" ;;
            LCD_TV|LED_MONITOR|OLED_DISPLAY|CRT_MONITOR) category="DISPLAY_EQUIPMENT" ;;
            *) category="OTHER" ;;
        esac
    fi

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "device": {
    "category": "$category",
    "type": "$type",
    "brand": "$brand",
    "model": "$model",
    "weight_kg": $weight,
    "condition": "$condition"
  },
  "owner": {
    "type": "$owner_type",
    "country": "$country"
  },
  "collectionPoint": {
    "facilityId": "${facility_id:-CP-DEFAULT-001}",
    "name": "${facility_name:-기본 수거센터}",
    "location": {
      "lat": ${lat:-37.5665},
      "lon": ${lon:-126.9780}
    }
  }
}
EOF
)

    print_info "기기 등록 중..."
    response=$(api_request POST "/devices/register" "$payload")

    if echo "$response" | grep -q "deviceId"; then
        print_success "기기가 성공적으로 등록되었습니다!"
        echo ""
        print_json "$response"

        # Extract deviceId
        if command -v jq &> /dev/null; then
            device_id=$(echo "$response" | jq -r '.data.deviceId')
            tracking_url=$(echo "$response" | jq -r '.data.trackingUrl')
            echo ""
            print_info "기기 ID: $device_id"
            print_info "추적 URL: $tracking_url"
        fi
    else
        print_error "기기 등록 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_track() {
    local device_id="$1"

    if [ -z "$device_id" ]; then
        print_error "기기 ID를 입력하세요."
        echo "사용법: $0 track DEVICE_ID"
        exit 1
    fi

    print_header "기기 추적: $device_id"
    print_info "추적 정보 조회 중..."

    response=$(api_request GET "/devices/$device_id/tracking" "")

    if echo "$response" | grep -q "currentStatus"; then
        print_success "추적 정보를 찾았습니다!"
        echo ""
        print_json "$response"
    else
        print_error "추적 정보를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_process() {
    print_header "처리 기록 제출"

    local device_id=""
    local facility_id=""
    local gold_mg=0
    local silver_mg=0
    local copper_g=0
    local co2_kg=0
    local energy_kwh=0

    while [[ $# -gt 0 ]]; do
        case $1 in
            --device-id) device_id="$2"; shift 2 ;;
            --facility-id) facility_id="$2"; shift 2 ;;
            --gold) gold_mg="$2"; shift 2 ;;
            --silver) silver_mg="$2"; shift 2 ;;
            --copper) copper_g="$2"; shift 2 ;;
            --co2-avoided) co2_kg="$2"; shift 2 ;;
            --energy-saved) energy_kwh="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$device_id" ] || [ -z "$facility_id" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 process --device-id ID --facility-id FACILITY_ID"
        echo "        [--gold MG] [--silver MG] [--copper G]"
        echo "        [--co2-avoided KG] [--energy-saved KWH]"
        exit 1
    fi

    local payload=$(cat <<EOF
{
  "deviceId": "$device_id",
  "facility": {
    "facilityId": "$facility_id",
    "name": "재활용 시설",
    "license": "ENV-2024-1234",
    "location": { "lat": 37.5665, "lon": 126.9780 },
    "certifications": ["R2", "ISO14001"]
  },
  "steps": [
    {
      "step": "DATA_DESTRUCTION",
      "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
      "dataDestructionMethod": "NIST_800-88",
      "verified": true
    },
    {
      "step": "PCB_EXTRACTION",
      "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
      "weight_g": ${copper_g}
    }
  ],
  "materialsRecovered": {
    "gold_mg": $gold_mg,
    "silver_mg": $silver_mg,
    "copper_g": $copper_g
  },
  "environmentalImpact": {
    "co2Avoided_kg": $co2_kg,
    "energySaved_kWh": $energy_kwh
  }
}
EOF
)

    print_info "처리 기록 제출 중..."
    response=$(api_request POST "/processing/submit" "$payload")

    if echo "$response" | grep -q "processingId"; then
        print_success "처리 기록이 성공적으로 제출되었습니다!"
        echo ""
        print_json "$response"
    else
        print_error "처리 기록 제출 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_facility() {
    local facility_id="$1"

    if [ -z "$facility_id" ]; then
        print_error "시설 ID를 입력하세요."
        echo "사용법: $0 facility FACILITY_ID"
        exit 1
    fi

    print_header "시설 현황: $facility_id"
    print_info "시설 정보 조회 중..."

    response=$(api_request GET "/facilities/$facility_id/metrics" "")

    if echo "$response" | grep -q "metrics"; then
        print_success "시설 정보를 찾았습니다!"
        echo ""
        print_json "$response"
    else
        print_error "시설 정보를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_nearby() {
    local lat="${1:-37.5665}"
    local lon="${2:-126.9780}"
    local radius="${3:-10}"

    print_header "주변 수거센터 찾기"
    print_info "위치: $lat, $lon (반경 ${radius}km)"

    response=$(api_request GET "/utilities/nearby?lat=$lat&lon=$lon&radius=$radius" "")

    if echo "$response" | grep -q "facilityId"; then
        print_success "주변 수거센터를 찾았습니다!"
        echo ""
        print_json "$response"
    else
        print_error "주변 수거센터를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_estimate() {
    local type="${1:-SMARTPHONE}"
    local weight="${2:-0.168}"

    print_header "가치 추정"
    print_info "기기: $type, 무게: ${weight}kg"

    response=$(api_request POST "/utilities/estimate-value" "{\"deviceType\":\"$type\",\"weight_kg\":$weight}")

    if echo "$response" | grep -q "totalValue_USD"; then
        print_success "가치를 추정했습니다!"
        echo ""
        print_json "$response"
    else
        print_error "가치 추정 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_help() {
    cat <<EOF
${BLUE}WIA-ENE-025: 전자폐기물 관리 표준 CLI${NC}
${CYAN}弘益人間 (홍익인간) - Benefit All Humanity${NC}

사용법: $0 <명령어> [옵션]

${YELLOW}명령어:${NC}
  register      새 기기 등록
  track         기기 추적
  process       처리 기록 제출
  facility      시설 현황 조회
  nearby        주변 수거센터 찾기
  estimate      가치 추정
  help          도움말 표시

${YELLOW}환경 변수:${NC}
  WIA_ENE025_API_KEY      API 키 (필수)
  WIA_ENE025_ENDPOINT     API 엔드포인트 (기본값: https://api.wia.org/ene-025/v1)

${YELLOW}예제:${NC}
  # 기기 등록
  $0 register --type SMARTPHONE --brand Samsung --model "Galaxy S23" --weight 0.168

  # 기기 추적
  $0 track EWASTE-2025-KR-000123456

  # 처리 기록 제출
  $0 process --device-id EWASTE-2025-KR-000123456 --facility-id RF-SEL-007 \\
     --gold 34 --silver 340 --copper 16 --co2-avoided 0.42 --energy-saved 1.85

  # 시설 현황
  $0 facility RF-SEL-007

  # 주변 수거센터 (위도, 경도, 반경km)
  $0 nearby 37.5665 126.9780 10

  # 가치 추정 (기기 타입, 무게kg)
  $0 estimate SMARTPHONE 0.168

${CYAN}🖥️  전자폐기물을 자원으로! 순환경제를 함께 만들어갑니다.${NC}
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
        track) cmd_track "$@" ;;
        process) cmd_process "$@" ;;
        facility) cmd_facility "$@" ;;
        nearby) cmd_nearby "$@" ;;
        estimate) cmd_estimate "$@" ;;
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
