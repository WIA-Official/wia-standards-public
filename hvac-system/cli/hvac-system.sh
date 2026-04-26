#!/bin/bash

# WIA-CITY-010: HVAC System Standard - CLI Tool
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-010 API

set -e

# Configuration
API_ENDPOINT="${WIA_CITY010_ENDPOINT:-https://api.wia.org/city-010/v1}"
API_KEY="${WIA_CITY010_API_KEY:-}"

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
        print_info "환경 변수 WIA_CITY010_API_KEY를 설정하거나 --api-key 옵션을 사용하세요."
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
    print_header "HVAC 시스템 등록"

    local system_id=""
    local name=""
    local type="VRF"
    local building=""
    local cooling_kw="50.0"
    local heating_kw="55.0"
    local lat=""
    local lon=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --system-id) system_id="$2"; shift 2 ;;
            --name) name="$2"; shift 2 ;;
            --type) type="$2"; shift 2 ;;
            --building) building="$2"; shift 2 ;;
            --cooling-kw) cooling_kw="$2"; shift 2 ;;
            --heating-kw) heating_kw="$2"; shift 2 ;;
            --lat) lat="$2"; shift 2 ;;
            --lon) lon="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    # Validate required fields
    if [ -z "$system_id" ] || [ -z "$name" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 register --system-id ID --name NAME"
        echo "        [--type TYPE] [--building BUILDING]"
        echo "        [--cooling-kw KW] [--heating-kw KW]"
        echo "        [--lat LAT] [--lon LON]"
        exit 1
    fi

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "system": {
    "system_id": "$system_id",
    "name": "$name",
    "system_type": "$type",
    "building": "${building:-본관}",
    "location": {
      "lat": ${lat:-37.5665},
      "lon": ${lon:-126.9780}
    },
    "total_cooling_capacity_kw": $cooling_kw,
    "total_heating_capacity_kw": $heating_kw,
    "equipment": [],
    "zones": [],
    "status": "RUNNING"
  }
}
EOF
)

    print_info "시스템 등록 중..."
    response=$(api_request POST "/hvac/systems/register" "$payload")

    if echo "$response" | grep -q "system_id"; then
        print_success "시스템이 성공적으로 등록되었습니다!"
        echo ""
        print_json "$response"

        # Extract system_id
        if command -v jq &> /dev/null; then
            dashboard_url=$(echo "$response" | jq -r '.data.dashboard_url')
            echo ""
            print_info "대시보드 URL: $dashboard_url"
        fi
    else
        print_error "시스템 등록 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_status() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 status SYSTEM_ID"
        exit 1
    fi

    print_header "HVAC 시스템 현황: $system_id"
    print_info "현재 상태 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/status" "")

    if echo "$response" | grep -q "overall_status"; then
        print_success "시스템 정보를 찾았습니다!"
        echo ""

        if command -v jq &> /dev/null; then
            # Parse and display key metrics
            status=$(echo "$response" | jq -r '.data.overall_status // "N/A"')
            power_kw=$(echo "$response" | jq -r '.data.energy.current_power_kw // "N/A"')
            energy_kwh=$(echo "$response" | jq -r '.data.energy.daily_energy_kwh // "N/A"')

            echo -e "${CYAN}전체 상태:${NC} $status"
            echo -e "${CYAN}현재 전력:${NC} ${power_kw} kW"
            echo -e "${CYAN}금일 에너지:${NC} ${energy_kwh} kWh"
            echo ""

            # Display zone information
            zone_count=$(echo "$response" | jq '.data.zones | length')
            echo -e "${CYAN}존 정보:${NC} (총 $zone_count 개)"
            echo "$response" | jq -r '.data.zones[] | "  - \(.name): \(.temperature_c)°C (설정: \(.setpoint_c)°C) - \(.comfort_level)"'
            echo ""
        fi

        print_json "$response"
    else
        print_error "시스템 정보를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_set_temp() {
    local system_id=""
    local zone_id=""
    local temp=""
    local mode="AUTO"
    local fan="AUTO"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --system-id) system_id="$2"; shift 2 ;;
            --zone-id) zone_id="$2"; shift 2 ;;
            --temp) temp="$2"; shift 2 ;;
            --mode) mode="$2"; shift 2 ;;
            --fan) fan="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$system_id" ] || [ -z "$zone_id" ] || [ -z "$temp" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 set-temp --system-id SYSTEM_ID --zone-id ZONE_ID --temp TEMP"
        echo "        [--mode MODE] [--fan FAN]"
        exit 1
    fi

    print_header "온도 설정"

    local payload=$(cat <<EOF
{
  "zone_id": "$zone_id",
  "temperature_c": $temp,
  "mode": "$mode",
  "fan_speed": "$fan"
}
EOF
)

    print_info "온도 설정 중: ${temp}°C ..."
    response=$(api_request PUT "/hvac/systems/$system_id/zones/$zone_id/setpoint" "$payload")

    if echo "$response" | grep -q "setpoint_updated"; then
        print_success "온도가 성공적으로 설정되었습니다!"
        echo ""
        print_json "$response"
    else
        print_error "온도 설정 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_set_mode() {
    local system_id=""
    local zone_id=""
    local mode="AUTO"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --system-id) system_id="$2"; shift 2 ;;
            --zone-id) zone_id="$2"; shift 2 ;;
            --mode) mode="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$system_id" ] || [ -z "$zone_id" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 set-mode --system-id SYSTEM_ID --zone-id ZONE_ID --mode MODE"
        echo "모드: OFF, COOLING, HEATING, AUTO, DRY, FAN_ONLY, VENTILATION"
        exit 1
    fi

    print_header "운전 모드 설정"

    local payload=$(cat <<EOF
{
  "mode": "$mode"
}
EOF
)

    print_info "모드 설정 중: $mode ..."
    response=$(api_request PUT "/hvac/systems/$system_id/zones/$zone_id/mode" "$payload")

    if echo "$response" | grep -q "mode"; then
        print_success "모드가 성공적으로 설정되었습니다!"
        echo ""
        print_json "$response"
    else
        print_error "모드 설정 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_energy() {
    local system_id="$1"
    local start_date="${2:-$(date -d '7 days ago' -u +%Y-%m-%dT00:00:00Z)}"
    local end_date="${3:-$(date -u +%Y-%m-%dT23:59:59Z)}"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 energy SYSTEM_ID [START_DATE] [END_DATE]"
        exit 1
    fi

    print_header "에너지 사용량: $system_id"
    print_info "에너지 데이터 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/energy?start=$start_date&end=$end_date" "")

    if echo "$response" | grep -q "energy_kwh"; then
        print_success "에너지 데이터를 찾았습니다!"
        echo ""

        if command -v jq &> /dev/null; then
            energy=$(echo "$response" | jq -r '.data.energy_kwh // "N/A"')
            power=$(echo "$response" | jq -r '.data.current_power_kw // "N/A"')
            cop=$(echo "$response" | jq -r '.data.cop // "N/A"')

            echo -e "${CYAN}에너지 소비:${NC} ${energy} kWh"
            echo -e "${CYAN}현재 전력:${NC} ${power} kW"
            echo -e "${CYAN}COP:${NC} ${cop}"
            echo ""
        fi

        print_json "$response"
    else
        print_error "에너지 데이터 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_alarms() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 alarms SYSTEM_ID"
        exit 1
    fi

    print_header "활성 경보: $system_id"
    print_info "경보 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/alarms/active" "")

    if echo "$response" | grep -q "data"; then
        if command -v jq &> /dev/null; then
            alarm_count=$(echo "$response" | jq '.data | length')
            echo -e "${CYAN}활성 경보 수:${NC} $alarm_count"
            echo ""

            if [ "$alarm_count" -gt 0 ]; then
                echo "$response" | jq -r '.data[] | "[\(.severity)] \(.type): \(.message) (\(.timestamp))"'
                echo ""
            fi
        fi
        print_json "$response"
    else
        print_error "경보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_filters() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 filters SYSTEM_ID"
        exit 1
    fi

    print_header "필터 상태: $system_id"
    print_info "필터 상태 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/filters" "")

    if echo "$response" | grep -q "data"; then
        if command -v jq &> /dev/null; then
            filter_count=$(echo "$response" | jq '.data | length')
            echo -e "${CYAN}필터 수:${NC} $filter_count"
            echo ""

            echo "$response" | jq -r '.data[] | "[\(.filter_type)] \(.location): 압력강하 \(.pressure_drop_pa)Pa (교체 \(if .needs_replacement then "필요" else "불필요" end))"'
            echo ""
        fi
        print_json "$response"
    else
        print_error "필터 상태 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_ventilation() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 ventilation SYSTEM_ID"
        exit 1
    fi

    print_header "환기 현황: $system_id"
    print_info "환기 상태 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/ventilation" "")

    if echo "$response" | grep -q "mode"; then
        print_success "환기 정보를 찾았습니다!"
        echo ""

        if command -v jq &> /dev/null; then
            mode=$(echo "$response" | jq -r '.data.mode // "N/A"')
            oa_percent=$(echo "$response" | jq -r '.data.outdoor_air_percent // "N/A"')
            ach=$(echo "$response" | jq -r '.data.air_change_rate_ach // "N/A"')
            eco=$(echo "$response" | jq -r '.data.economizer_enabled // "N/A"')

            echo -e "${CYAN}환기 모드:${NC} $mode"
            echo -e "${CYAN}외기 비율:${NC} ${oa_percent}%"
            echo -e "${CYAN}환기율:${NC} ${ach} ACH"
            echo -e "${CYAN}이코노마이저:${NC} $eco"
            echo ""
        fi

        print_json "$response"
    else
        print_error "환기 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_maintenance() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 maintenance SYSTEM_ID"
        exit 1
    fi

    print_header "정비 이력: $system_id"
    print_info "정비 이력 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/maintenance?limit=10" "")

    if echo "$response" | grep -q "data"; then
        if command -v jq &> /dev/null; then
            record_count=$(echo "$response" | jq '.data | length')
            echo -e "${CYAN}정비 기록 수:${NC} $record_count"
            echo ""

            echo "$response" | jq -r '.data[] | "[\(.type)] \(.performed_date): \(.description)"'
            echo ""
        fi
        print_json "$response"
    else
        print_error "정비 이력 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_predictions() {
    local system_id="$1"

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력하세요."
        echo "사용법: $0 predictions SYSTEM_ID"
        exit 1
    fi

    print_header "고장 예측: $system_id"
    print_info "예측 정보 조회 중..."

    response=$(api_request GET "/hvac/systems/$system_id/predictions" "")

    if echo "$response" | grep -q "data"; then
        if command -v jq &> /dev/null; then
            pred_count=$(echo "$response" | jq '.data | length')
            echo -e "${CYAN}예측 수:${NC} $pred_count"
            echo ""

            if [ "$pred_count" -gt 0 ]; then
                echo "$response" | jq -r '.data[] | "[\(.urgency)] \(.component): \(.failure_mode) (확률: \(.probability_percent)%)"'
                echo ""
            fi
        fi
        print_json "$response"
    else
        print_error "예측 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_help() {
    cat <<EOF
${BLUE}WIA-CITY-010: 냉난방 시스템 표준 CLI${NC}
${CYAN}弘益人間 (홍익인간) - Benefit All Humanity${NC}

사용법: $0 <명령어> [옵션]

${YELLOW}명령어:${NC}
  register        새 HVAC 시스템 등록
  status          시스템 현황 조회
  set-temp        존 온도 설정
  set-mode        운전 모드 설정
  energy          에너지 사용량 조회
  alarms          활성 경보 조회
  filters         필터 상태 조회
  ventilation     환기 현황 조회
  maintenance     정비 이력 조회
  predictions     고장 예측 조회
  help            도움말 표시

${YELLOW}환경 변수:${NC}
  WIA_CITY010_API_KEY     API 키 (필수)
  WIA_CITY010_ENDPOINT    API 엔드포인트 (기본값: https://api.wia.org/city-010/v1)

${YELLOW}예제:${NC}
  # 시스템 등록
  $0 register --system-id HVAC-BLDG-A-01 --name "본관 VRF 시스템" \\
     --type VRF --building "본관" --cooling-kw 50 --heating-kw 55

  # 현재 상태 조회
  $0 status HVAC-BLDG-A-01

  # 온도 설정
  $0 set-temp --system-id HVAC-BLDG-A-01 --zone-id ZONE-301 \\
     --temp 22 --mode AUTO --fan AUTO

  # 운전 모드 설정
  $0 set-mode --system-id HVAC-BLDG-A-01 --zone-id ZONE-301 --mode COOLING

  # 에너지 사용량
  $0 energy HVAC-BLDG-A-01

  # 활성 경보
  $0 alarms HVAC-BLDG-A-01

  # 필터 상태
  $0 filters HVAC-BLDG-A-01

  # 환기 현황
  $0 ventilation HVAC-BLDG-A-01

  # 정비 이력
  $0 maintenance HVAC-BLDG-A-01

  # 고장 예측
  $0 predictions HVAC-BLDG-A-01

${CYAN}❄️ 효율적인 냉난방, 쾌적한 환경!${NC}
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
        set-temp) cmd_set_temp "$@" ;;
        set-mode) cmd_set_mode "$@" ;;
        energy) cmd_energy "$@" ;;
        alarms) cmd_alarms "$@" ;;
        filters) cmd_filters "$@" ;;
        ventilation) cmd_ventilation "$@" ;;
        maintenance) cmd_maintenance "$@" ;;
        predictions) cmd_predictions "$@" ;;
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
