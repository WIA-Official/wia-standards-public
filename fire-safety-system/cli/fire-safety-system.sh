#!/bin/bash

# WIA-CITY-013: Fire Safety System Standard - CLI Tool
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-013 API

set -e

# Configuration
API_ENDPOINT="${WIA_CITY013_ENDPOINT:-https://api.wia.org/city-013/v1}"
API_KEY="${WIA_CITY013_API_KEY:-}"

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

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
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
        print_info "환경 변수 WIA_CITY013_API_KEY를 설정하거나 --api-key 옵션을 사용하세요."
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

cmd_detect() {
    print_header "화재 감지 보고"

    local sensor_id=""
    local sensor_type="SMOKE"
    local floor=""
    local zone=""
    local smoke_level=""
    local temperature=""
    local co_level=""
    local flame=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --sensor-id) sensor_id="$2"; shift 2 ;;
            --sensor-type) sensor_type="$2"; shift 2 ;;
            --floor) floor="$2"; shift 2 ;;
            --zone) zone="$2"; shift 2 ;;
            --smoke-level) smoke_level="$2"; shift 2 ;;
            --temperature) temperature="$2"; shift 2 ;;
            --co-level) co_level="$2"; shift 2 ;;
            --flame) flame="true"; shift 1 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$sensor_id" ]; then
        print_error "센서 ID를 입력해주세요 (--sensor-id)"
        exit 1
    fi

    print_info "화재 감지 이벤트 생성 중..."

    local data=$(cat <<EOF
{
  "sensorId": "$sensor_id",
  "sensorType": "$sensor_type",
  "location": {
    "floor": "$floor",
    "zone": "$zone"
  },
  "readings": {
    $([ -n "$smoke_level" ] && echo "\"smokeLevel_ppm\": $smoke_level,")
    $([ -n "$temperature" ] && echo "\"temperature_C\": $temperature,")
    $([ -n "$co_level" ] && echo "\"coLevel_ppm\": $co_level,")
    $([ -n "$flame" ] && echo "\"flameDetected\": true")
  },
  "alarmTriggered": true
}
EOF
)

    local response=$(api_request "POST" "/fire/detect" "$data")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "화재 감지 이벤트가 생성되었습니다"
        print_json "$response"
    else
        print_error "화재 감지 이벤트 생성 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_activate_sprinkler() {
    print_header "스프링클러 작동"

    local system_id=""
    local zone_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --system-id) system_id="$2"; shift 2 ;;
            --zone-id) zone_id="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$system_id" ]; then
        print_error "시스템 ID를 입력해주세요 (--system-id)"
        exit 1
    fi

    print_warning "스프링클러를 작동합니다..."

    local data=$(cat <<EOF
{
  "systemId": "$system_id"
  $([ -n "$zone_id" ] && echo ", \"zoneId\": \"$zone_id\"")
}
EOF
)

    local response=$(api_request "POST" "/sprinklers/activate" "$data")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "스프링클러가 작동되었습니다"
        print_json "$response"
    else
        print_error "스프링클러 작동 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_trigger_alarm() {
    print_header "화재 경보 발생"

    local alarm_type="COMBINED"
    local severity="EMERGENCY"
    local floor=""
    local zone=""
    local message="화재가 감지되었습니다. 즉시 대피하세요."
    local triggered_by=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --type) alarm_type="$2"; shift 2 ;;
            --severity) severity="$2"; shift 2 ;;
            --floor) floor="$2"; shift 2 ;;
            --zone) zone="$2"; shift 2 ;;
            --message) message="$2"; shift 2 ;;
            --triggered-by) triggered_by="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$triggered_by" ]; then
        print_error "트리거 센서 ID를 입력해주세요 (--triggered-by)"
        exit 1
    fi

    print_warning "화재 경보를 발생합니다..."

    local data=$(cat <<EOF
{
  "type": "$alarm_type",
  "severity": "$severity",
  "location": {
    "floor": "$floor",
    "zone": "$zone"
  },
  "triggeredBy": "$triggered_by",
  "message": "$message"
}
EOF
)

    local response=$(api_request "POST" "/alarms/trigger" "$data")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "화재 경보가 발생되었습니다"
        print_json "$response"
    else
        print_error "화재 경보 발생 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_evacuation_route() {
    print_header "대피 경로 조회"

    local floor=""
    local zone=""
    local accessible="false"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --floor) floor="$2"; shift 2 ;;
            --zone) zone="$2"; shift 2 ;;
            --accessible) accessible="true"; shift 1 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$floor" ]; then
        print_error "층 정보를 입력해주세요 (--floor)"
        exit 1
    fi

    print_info "대피 경로를 조회합니다..."

    local data=$(cat <<EOF
{
  "startLocation": {
    "floor": "$floor",
    "zone": "$zone"
  },
  "accessible": $accessible
}
EOF
)

    local response=$(api_request "POST" "/evacuation/route" "$data")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "대피 경로를 찾았습니다"
        print_json "$response"
    else
        print_error "대피 경로 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_check_extinguisher() {
    print_header "소화기 점검"

    local extinguisher_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) extinguisher_id="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$extinguisher_id" ]; then
        print_error "소화기 ID를 입력해주세요 (--id)"
        exit 1
    fi

    print_info "소화기 정보를 조회합니다..."

    local response=$(api_request "GET" "/equipment/extinguishers/$extinguisher_id" "")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "소화기 정보"
        print_json "$response"
    else
        print_error "소화기 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_health_check() {
    print_header "시스템 상태 점검"

    print_info "시스템 상태를 확인합니다..."

    local response=$(api_request "GET" "/system/health" "")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        local overall=$(echo "$response" | jq -r '.data.overall')

        case $overall in
            "HEALTHY")
                print_success "시스템 정상"
                ;;
            "WARNING")
                print_warning "시스템 주의"
                ;;
            "CRITICAL")
                print_error "시스템 심각"
                ;;
        esac

        print_json "$response"
    else
        print_error "시스템 상태 확인 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_sensor_status() {
    print_header "센서 상태 조회"

    local sensor_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) sensor_id="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$sensor_id" ]; then
        print_error "센서 ID를 입력해주세요 (--id)"
        exit 1
    fi

    print_info "센서 상태를 조회합니다..."

    local response=$(api_request "GET" "/sensors/$sensor_id" "")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "센서 정보"
        print_json "$response"
    else
        print_error "센서 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_zone_info() {
    print_header "구역 정보 조회"

    local zone_id=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            --id) zone_id="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$zone_id" ]; then
        print_error "구역 ID를 입력해주세요 (--id)"
        exit 1
    fi

    print_info "구역 정보를 조회합니다..."

    local response=$(api_request "GET" "/zones/$zone_id" "")

    if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
        print_success "구역 정보"
        print_json "$response"
    else
        print_error "구역 정보 조회 실패"
        print_json "$response"
        exit 1
    fi
}

show_help() {
    cat << EOF
${BLUE}WIA-CITY-013: 화재 안전 시스템 표준 CLI${NC}
${CYAN}弘益人間 (홍익인간) - 널리 인간을 이롭게 하라${NC}

사용법: $0 <명령어> [옵션]

명령어:
  detect              화재 감지 보고
  activate-sprinkler  스프링클러 작동
  trigger-alarm       화재 경보 발생
  evacuation-route    대피 경로 조회
  check-extinguisher  소화기 점검
  sensor-status       센서 상태 조회
  zone-info           구역 정보 조회
  health-check        시스템 상태 점검
  help                이 도움말 표시

화재 감지 보고 옵션:
  --sensor-id <ID>         센서 ID (필수)
  --sensor-type <TYPE>     센서 타입 (SMOKE, HEAT, FLAME, CO)
  --floor <FLOOR>          층
  --zone <ZONE>            구역
  --smoke-level <PPM>      연기 농도 (ppm)
  --temperature <C>        온도 (°C)
  --co-level <PPM>         CO 농도 (ppm)
  --flame                  화염 감지

스프링클러 작동 옵션:
  --system-id <ID>         시스템 ID (필수)
  --zone-id <ID>           구역 ID (선택)

화재 경보 발생 옵션:
  --type <TYPE>            경보 타입 (AUDIBLE, VISUAL, VOICE, COMBINED)
  --severity <LEVEL>       심각도 (INFO, WARNING, CRITICAL, EMERGENCY)
  --floor <FLOOR>          층
  --zone <ZONE>            구역
  --message <MSG>          경보 메시지
  --triggered-by <ID>      트리거 센서 ID (필수)

대피 경로 조회 옵션:
  --floor <FLOOR>          층 (필수)
  --zone <ZONE>            구역
  --accessible             장애인 접근 가능 경로

소화기/센서/구역 조회 옵션:
  --id <ID>                장비/센서/구역 ID (필수)

환경 변수:
  WIA_CITY013_API_KEY      API 키
  WIA_CITY013_ENDPOINT     API 엔드포인트

예제:
  # 연기 감지 보고
  $0 detect --sensor-id SMOKE-FL2-A001 --floor 2F --zone A --smoke-level 250

  # 스프링클러 작동
  $0 activate-sprinkler --system-id SPR-001 --zone-id ZONE-A

  # 화재 경보 발생
  $0 trigger-alarm --severity EMERGENCY --floor 3F --zone B --triggered-by SMOKE-FL3-B001

  # 대피 경로 조회
  $0 evacuation-route --floor 2F --zone A --accessible

  # 시스템 상태 점검
  $0 health-check

© 2025 WIA - World Certification Industry Association
EOF
}

# Main
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    local command=$1
    shift

    case $command in
        detect)
            cmd_detect "$@"
            ;;
        activate-sprinkler)
            cmd_activate_sprinkler "$@"
            ;;
        trigger-alarm)
            cmd_trigger_alarm "$@"
            ;;
        evacuation-route)
            cmd_evacuation_route "$@"
            ;;
        check-extinguisher)
            cmd_check_extinguisher "$@"
            ;;
        sensor-status)
            cmd_sensor_status "$@"
            ;;
        zone-info)
            cmd_zone_info "$@"
            ;;
        health-check)
            cmd_health_check "$@"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "알 수 없는 명령어: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
