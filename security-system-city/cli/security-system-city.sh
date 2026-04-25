#!/bin/bash

##############################################################################
# WIA-CITY-014: Security System Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Version: 1.0.0
# License: MIT
##############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_SECURITY_SYSTEM_ENDPOINT:-https://api.wia.org/city-014/v1}"
API_KEY="${WIA_SECURITY_SYSTEM_API_KEY}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Helper functions
print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_alert() {
    echo -e "${RED}🚨${NC} $1"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_SECURITY_SYSTEM_API_KEY environment variable."
        exit 1
    fi
}

api_request() {
    local method="$1"
    local path="$2"
    local data="$3"

    check_api_key

    if [ -n "$data" ]; then
        curl -s -X "$method" \
            -H "Authorization: Bearer $API_KEY" \
            -H "Content-Type: application/json" \
            -d "$data" \
            "$API_ENDPOINT$path"
    else
        curl -s -X "$method" \
            -H "Authorization: Bearer $API_KEY" \
            "$API_ENDPOINT$path"
    fi
}

# ============================================================================
# Camera Commands
# ============================================================================

cmd_list_cameras() {
    print_info "카메라 목록 조회 중..."

    local zone="${1:-}"
    local query=""

    if [ -n "$zone" ]; then
        query="?zone=$zone"
    fi

    local response=$(api_request GET "/api/v1/cameras$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.camera_id)\t\(.name)\t\(.type)\t\(.status.online)\t\(.status.recording)"' | \
            column -t -s $'\t' -N "ID,이름,유형,온라인,녹화중"
        print_success "카메라 목록 조회 완료"
    else
        print_error "카메라 목록 조회 실패"
        exit 1
    fi
}

cmd_get_camera() {
    local camera_id="$1"

    if [ -z "$camera_id" ]; then
        print_error "사용법: $0 get-camera <camera-id>"
        exit 1
    fi

    print_info "카메라 정보 조회: $camera_id"

    local response=$(api_request GET "/api/v1/cameras/$camera_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "카메라 정보 조회 완료"
    else
        print_error "카메라 정보 조회 실패"
        exit 1
    fi
}

cmd_get_stream() {
    local camera_id="$1"
    local protocol="${2:-rtsp}"

    if [ -z "$camera_id" ]; then
        print_error "사용법: $0 get-stream <camera-id> [rtsp|hls|webrtc]"
        exit 1
    fi

    print_info "스트림 URL 조회: $camera_id (프로토콜: $protocol)"

    local response=$(api_request GET "/api/v1/cameras/$camera_id/stream?protocol=$protocol")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.stream_url'
        print_success "스트림 URL 조회 완료"
    else
        print_error "스트림 URL 조회 실패"
        exit 1
    fi
}

cmd_snapshot() {
    local camera_id="$1"

    if [ -z "$camera_id" ]; then
        print_error "사용법: $0 snapshot <camera-id>"
        exit 1
    fi

    print_info "스냅샷 캡처 중: $camera_id"

    local response=$(api_request POST "/api/v1/cameras/$camera_id/snapshot")

    if [ $? -eq 0 ]; then
        local image_url=$(echo "$response" | jq -r '.data.image_url')
        echo "스냅샷 URL: $image_url"
        print_success "스냅샷 캡처 완료"
    else
        print_error "스냅샷 캡처 실패"
        exit 1
    fi
}

# ============================================================================
# Sensor Commands
# ============================================================================

cmd_list_sensors() {
    print_info "센서 목록 조회 중..."

    local zone="${1:-}"
    local query=""

    if [ -n "$zone" ]; then
        query="?zone=$zone"
    fi

    local response=$(api_request GET "/api/v1/sensors$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.sensor_id)\t\(.name)\t\(.type)\t\(.state.armed)\t\(.state.triggered)"' | \
            column -t -s $'\t' -N "ID,이름,유형,무장,감지"
        print_success "센서 목록 조회 완료"
    else
        print_error "센서 목록 조회 실패"
        exit 1
    fi
}

cmd_arm_sensor() {
    local sensor_id="$1"

    if [ -z "$sensor_id" ]; then
        print_error "사용법: $0 arm-sensor <sensor-id>"
        exit 1
    fi

    print_info "센서 무장 중: $sensor_id"

    local response=$(api_request POST "/api/v1/sensors/$sensor_id/arm")

    if [ $? -eq 0 ]; then
        print_success "센서 무장 완료"
    else
        print_error "센서 무장 실패"
        exit 1
    fi
}

cmd_disarm_sensor() {
    local sensor_id="$1"

    if [ -z "$sensor_id" ]; then
        print_error "사용법: $0 disarm-sensor <sensor-id>"
        exit 1
    fi

    print_info "센서 해제 중: $sensor_id"

    local response=$(api_request POST "/api/v1/sensors/$sensor_id/disarm")

    if [ $? -eq 0 ]; then
        print_success "센서 해제 완료"
    else
        print_error "센서 해제 실패"
        exit 1
    fi
}

cmd_test_sensor() {
    local sensor_id="$1"

    if [ -z "$sensor_id" ]; then
        print_error "사용법: $0 test-sensor <sensor-id>"
        exit 1
    fi

    print_info "센서 테스트 중: $sensor_id"

    local response=$(api_request POST "/api/v1/sensors/$sensor_id/test")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "센서 테스트 완료"
    else
        print_error "센서 테스트 실패"
        exit 1
    fi
}

# ============================================================================
# Zone Commands
# ============================================================================

cmd_list_zones() {
    print_info "보안 존 목록 조회 중..."

    local response=$(api_request GET "/api/v1/zones")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.zone_id)\t\(.zone_name)\t\(.zone_type)\t\(.state.armed)\t\(.state.status)"' | \
            column -t -s $'\t' -N "ID,이름,유형,무장,상태"
        print_success "보안 존 목록 조회 완료"
    else
        print_error "보안 존 목록 조회 실패"
        exit 1
    fi
}

cmd_arm_zone() {
    local zone_id="$1"
    local mode="${2:-away}"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 arm-zone <zone-id> [away|stay|night]"
        exit 1
    fi

    print_info "보안 존 무장 중: $zone_id (모드: $mode)"

    local data="{\"mode\":\"$mode\"}"
    local response=$(api_request POST "/api/v1/zones/$zone_id/arm" "$data")

    if [ $? -eq 0 ]; then
        print_success "보안 존 무장 완료"
    else
        print_error "보안 존 무장 실패"
        exit 1
    fi
}

cmd_disarm_zone() {
    local zone_id="$1"
    local code="${2:-}"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 disarm-zone <zone-id> [code]"
        exit 1
    fi

    print_info "보안 존 해제 중: $zone_id"

    local data="{}"
    if [ -n "$code" ]; then
        data="{\"code\":\"$code\"}"
    fi

    local response=$(api_request POST "/api/v1/zones/$zone_id/disarm" "$data")

    if [ $? -eq 0 ]; then
        print_success "보안 존 해제 완료"
    else
        print_error "보안 존 해제 실패"
        exit 1
    fi
}

# ============================================================================
# Alert Commands
# ============================================================================

cmd_list_alerts() {
    print_info "경보 목록 조회 중..."

    local severity="${1:-}"
    local query=""

    if [ -n "$severity" ]; then
        query="?severity=$severity"
    fi

    local response=$(api_request GET "/api/v1/alerts$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.alert_id)\t\(.severity)\t\(.type)\t\(.status)\t\(.message)"' | \
            column -t -s $'\t' -N "ID,심각도,유형,상태,메시지"
        print_success "경보 목록 조회 완료"
    else
        print_error "경보 목록 조회 실패"
        exit 1
    fi
}

cmd_acknowledge_alert() {
    local alert_id="$1"
    local user="${2:-system}"

    if [ -z "$alert_id" ]; then
        print_error "사용법: $0 ack-alert <alert-id> [user]"
        exit 1
    fi

    print_info "경보 확인 중: $alert_id"

    local data="{\"acknowledged_by\":\"$user\"}"
    local response=$(api_request POST "/api/v1/alerts/$alert_id/acknowledge" "$data")

    if [ $? -eq 0 ]; then
        print_success "경보 확인 완료"
    else
        print_error "경보 확인 실패"
        exit 1
    fi
}

cmd_resolve_alert() {
    local alert_id="$1"
    local user="${2:-system}"

    if [ -z "$alert_id" ]; then
        print_error "사용법: $0 resolve-alert <alert-id> [user]"
        exit 1
    fi

    print_info "경보 해결 중: $alert_id"

    local data="{\"resolved_by\":\"$user\"}"
    local response=$(api_request POST "/api/v1/alerts/$alert_id/resolve" "$data")

    if [ $? -eq 0 ]; then
        print_success "경보 해결 완료"
    else
        print_error "경보 해결 실패"
        exit 1
    fi
}

# ============================================================================
# Event Commands
# ============================================================================

cmd_list_events() {
    print_info "이벤트 목록 조회 중..."

    local type="${1:-}"
    local query=""

    if [ -n "$type" ]; then
        query="?type=$type"
    fi

    local response=$(api_request GET "/api/v1/events$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.event_id)\t\(.timestamp)\t\(.event_type)\t\(.severity)\t\(.details.description)"' | \
            column -t -s $'\t' -N "ID,시간,유형,심각도,설명"
        print_success "이벤트 목록 조회 완료"
    else
        print_error "이벤트 목록 조회 실패"
        exit 1
    fi
}

# ============================================================================
# Guard Commands
# ============================================================================

cmd_list_guards() {
    print_info "경비원 목록 조회 중..."

    local response=$(api_request GET "/api/v1/guards")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.guard_id)\t\(.name)\t\(.shift)\t\(.status)"' | \
            column -t -s $'\t' -N "ID,이름,근무조,상태"
        print_success "경비원 목록 조회 완료"
    else
        print_error "경비원 목록 조회 실패"
        exit 1
    fi
}

cmd_update_guard_status() {
    local guard_id="$1"
    local status="$2"

    if [ -z "$guard_id" ] || [ -z "$status" ]; then
        print_error "사용법: $0 update-guard-status <guard-id> <status>"
        echo "  상태: on_patrol, at_post, break, responding, off_duty"
        exit 1
    fi

    print_info "경비원 상태 업데이트 중: $guard_id -> $status"

    local data="{\"status\":\"$status\"}"
    local response=$(api_request POST "/api/v1/guards/$guard_id/status" "$data")

    if [ $? -eq 0 ]; then
        print_success "경비원 상태 업데이트 완료"
    else
        print_error "경비원 상태 업데이트 실패"
        exit 1
    fi
}

# ============================================================================
# Emergency Commands
# ============================================================================

cmd_trigger_emergency() {
    local type="$1"
    local location="$2"
    local description="${3:-긴급 상황}"

    if [ -z "$type" ] || [ -z "$location" ]; then
        print_error "사용법: $0 trigger-emergency <type> <location> [description]"
        echo "  유형: medical, fire, intrusion, assault, natural_disaster, bomb_threat, active_shooter, hazmat, evacuation, other"
        exit 1
    fi

    print_alert "긴급 상황 알림 발송 중: $type at $location"

    local data="{\"type\":\"$type\",\"priority\":\"critical\",\"location\":{\"building\":\"$location\"},\"details\":{\"description\":\"$description\"},\"reporter\":{\"type\":\"system\"}}"
    local response=$(api_request POST "/api/v1/emergency/alert" "$data")

    if [ $? -eq 0 ]; then
        local alert_id=$(echo "$response" | jq -r '.data.alert_id')
        print_alert "긴급 상황 알림 발송 완료! Alert ID: $alert_id"
    else
        print_error "긴급 상황 알림 발송 실패"
        exit 1
    fi
}

cmd_list_active_emergencies() {
    print_info "활성 긴급 상황 조회 중..."

    local response=$(api_request GET "/api/v1/emergency/active")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.emergencies[] | "\(.alert_id)\t\(.type)\t\(.priority)\t\(.response.status)\t\(.details.description)"' | \
            column -t -s $'\t' -N "ID,유형,우선순위,대응상태,설명"
        print_success "활성 긴급 상황 조회 완료"
    else
        print_error "활성 긴급 상황 조회 실패"
        exit 1
    fi
}

# ============================================================================
# Dashboard Command
# ============================================================================

cmd_dashboard() {
    print_info "통합 대시보드 조회 중..."

    local response=$(api_request GET "/api/v1/control/dashboard")

    if [ $? -eq 0 ]; then
        echo -e "${CYAN}=== 보안 시스템 대시보드 ===${NC}"
        echo "$response" | jq -r '
            "활성 경보: " + (.data.active_alerts | tostring) + "개",
            "온라인 카메라: " + (.data.cameras_online | tostring) + "대",
            "온라인 센서: " + (.data.sensors_online | tostring) + "개",
            "근무 중 경비원: " + (.data.guards_on_duty | tostring) + "명"
        '
        echo ""
        echo -e "${CYAN}최근 이벤트:${NC}"
        echo "$response" | jq -r '.data.recent_events[] | "  - [\(.severity)] \(.event_type): \(.details.description)"'
        print_success "대시보드 조회 완료"
    else
        print_error "대시보드 조회 실패"
        exit 1
    fi
}

# ============================================================================
# Help Command
# ============================================================================

cmd_help() {
    cat <<EOF
WIA-CITY-014 보안 시스템 CLI 도구

사용법: $0 <명령어> [옵션]

카메라 명령어:
  list-cameras [zone]                     카메라 목록 조회
  get-camera <camera-id>                  카메라 정보 조회
  get-stream <camera-id> [protocol]       스트림 URL 조회 (rtsp/hls/webrtc)
  snapshot <camera-id>                    스냅샷 캡처

센서 명령어:
  list-sensors [zone]                     센서 목록 조회
  arm-sensor <sensor-id>                  센서 무장
  disarm-sensor <sensor-id>               센서 해제
  test-sensor <sensor-id>                 센서 테스트

보안 존 명령어:
  list-zones                              보안 존 목록 조회
  arm-zone <zone-id> [mode]               보안 존 무장 (away/stay/night)
  disarm-zone <zone-id> [code]            보안 존 해제

경보 명령어:
  list-alerts [severity]                  경보 목록 조회
  ack-alert <alert-id> [user]             경보 확인
  resolve-alert <alert-id> [user]         경보 해결

이벤트 명령어:
  list-events [type]                      이벤트 목록 조회

경비원 명령어:
  list-guards                             경비원 목록 조회
  update-guard-status <id> <status>       경비원 상태 업데이트

긴급 상황 명령어:
  trigger-emergency <type> <loc> [desc]   긴급 상황 알림
  list-active-emergencies                 활성 긴급 상황 조회

통합 관제:
  dashboard                               통합 대시보드 조회

환경 변수:
  WIA_SECURITY_SYSTEM_ENDPOINT            API 엔드포인트 (기본: https://api.wia.org/city-014/v1)
  WIA_SECURITY_SYSTEM_API_KEY             API 키 (필수)

사용 예시:
  # 카메라 목록 조회
  $0 list-cameras

  # 특정 존의 카메라만 조회
  $0 list-cameras zone-entrance

  # 카메라 스트림 URL 가져오기
  $0 get-stream camera-001 rtsp

  # 보안 존 무장
  $0 arm-zone zone-floor-1 away

  # 긴급 상황 알림
  $0 trigger-emergency fire "Building A" "화재 감지됨"

  # 통합 대시보드 확인
  $0 dashboard

弘益人間 (홍익인간) - Benefit All Humanity
© 2025 WIA (World Certification Industry Association)
EOF
}

# ============================================================================
# Main
# ============================================================================

COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
    # Camera commands
    list-cameras)
        cmd_list_cameras "$@"
        ;;
    get-camera)
        cmd_get_camera "$@"
        ;;
    get-stream)
        cmd_get_stream "$@"
        ;;
    snapshot)
        cmd_snapshot "$@"
        ;;

    # Sensor commands
    list-sensors)
        cmd_list_sensors "$@"
        ;;
    arm-sensor)
        cmd_arm_sensor "$@"
        ;;
    disarm-sensor)
        cmd_disarm_sensor "$@"
        ;;
    test-sensor)
        cmd_test_sensor "$@"
        ;;

    # Zone commands
    list-zones)
        cmd_list_zones "$@"
        ;;
    arm-zone)
        cmd_arm_zone "$@"
        ;;
    disarm-zone)
        cmd_disarm_zone "$@"
        ;;

    # Alert commands
    list-alerts)
        cmd_list_alerts "$@"
        ;;
    ack-alert)
        cmd_acknowledge_alert "$@"
        ;;
    resolve-alert)
        cmd_resolve_alert "$@"
        ;;

    # Event commands
    list-events)
        cmd_list_events "$@"
        ;;

    # Guard commands
    list-guards)
        cmd_list_guards "$@"
        ;;
    update-guard-status)
        cmd_update_guard_status "$@"
        ;;

    # Emergency commands
    trigger-emergency)
        cmd_trigger_emergency "$@"
        ;;
    list-active-emergencies)
        cmd_list_active_emergencies "$@"
        ;;

    # Dashboard
    dashboard)
        cmd_dashboard "$@"
        ;;

    # Help
    help|--help|-h)
        cmd_help
        ;;

    *)
        print_error "알 수 없는 명령어: $COMMAND"
        echo
        cmd_help
        exit 1
        ;;
esac
