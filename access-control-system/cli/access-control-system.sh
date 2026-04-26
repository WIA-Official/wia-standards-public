#!/bin/bash

################################################################################
# WIA-CITY-015: Access Control System Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-015
# Access Control System Standard API.
#
# 사용법:
#   access-control-system.sh <명령어> [옵션]
#
# 명령어:
#   dashboard           실시간 대시보드 보기
#   users               사용자 목록/조회/생성
#   grant-access        접근 권한 부여
#   revoke-access       접근 권한 철회
#   credentials         크리덴셜 관리
#   access-points       출입구 목록/조회/제어
#   unlock              출입구 원격 개방
#   lock                출입구 원격 잠금
#   access-logs         출입 로그 조회
#   visitors            방문자 관리
#   check-in            방문자 체크인
#   check-out           방문자 체크아웃
#   parking-status      주차장 현황
#   parking-entry       입차 기록
#   parking-exit        출차 기록
#   emergency-activate  비상 모드 활성화
#   emergency-clear     비상 모드 해제
#   alerts              활성 알람 보기
#   config              설정
#   help                도움말 표시
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/city-015/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/access-control-system.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║         🔐  WIA-CITY-015: 출입 통제 시스템 CLI               ║"
  echo "║                         v${VERSION}                              ║"
  echo "║                                                                ║"
  echo "║              弘益人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
}

print_success() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1" >&2
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

print_metric() {
  local label="$1"
  local value="$2"
  local unit="$3"
  echo -e "${CYAN}${label}:${NC} ${value} ${unit}"
}

# Check if jq is available
has_jq() {
  command -v jq &> /dev/null
}

# Pretty print JSON
pretty_json() {
  local json="$1"

  if has_jq; then
    echo "$json" | jq '.'
  else
    echo "$json"
  fi
}

# Load configuration
load_config() {
  if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    if [ -n "${WIA_API_KEY_CONF:-}" ]; then
      API_KEY="${WIA_API_KEY_CONF}"
    fi
    if [ -n "${WIA_API_ENDPOINT_CONF:-}" ]; then
      API_ENDPOINT="${WIA_API_ENDPOINT_CONF}"
    fi
  fi
}

# Save configuration
save_config() {
  mkdir -p "$(dirname "$CONFIG_FILE")"
  cat > "$CONFIG_FILE" <<EOF
# WIA-CITY-015 Access Control System CLI Configuration
WIA_API_KEY_CONF="${API_KEY}"
WIA_API_ENDPOINT_CONF="${API_ENDPOINT}"
EOF
  chmod 600 "$CONFIG_FILE"
  print_success "설정이 저장되었습니다: $CONFIG_FILE"
}

# Make API request
api_request() {
  local method="$1"
  local path="$2"
  local data="${3:-}"

  if [ -z "$API_KEY" ]; then
    print_error "API 키가 설정되지 않았습니다. 사용법: export WIA_API_KEY=your-key 또는 'config' 명령 실행"
    exit 1
  fi

  local url="${API_ENDPOINT}${path}"
  local response

  if [ "$method" = "GET" ]; then
    response=$(curl -s -X GET "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: CITY-015" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: CITY-015" \
      -H "X-WIA-Version: 1.0.0" \
      -d "$data")
  fi

  echo "$response"
}

################################################################################
# Commands
################################################################################

cmd_config() {
  print_header
  echo "설정 구성"
  echo ""

  read -p "API 엔드포인트 [${API_ENDPOINT}]: " input_endpoint
  if [ -n "$input_endpoint" ]; then
    API_ENDPOINT="$input_endpoint"
  fi

  read -sp "API 키: " input_key
  echo ""
  if [ -n "$input_key" ]; then
    API_KEY="$input_key"
  fi

  save_config
  echo ""
  print_success "설정 완료!"
}

cmd_dashboard() {
  print_header
  local building_id="${1:-BLD-001}"

  print_info "실시간 대시보드 로드 중... (빌딩 ID: $building_id)"
  echo ""

  local response=$(api_request GET "/api/v1/dashboard/${building_id}")

  if has_jq; then
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}📊 재실 현황${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  현재 재실 인원" "$(echo "$response" | jq -r '.data.occupancy.current')" "명"
    print_metric "  최대 수용 인원" "$(echo "$response" | jq -r '.data.occupancy.max')" "명"
    print_metric "  재실률" "$(echo "$response" | jq -r '.data.occupancy.rate')" "%"
    echo ""

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}🚪 출입 활동${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  오늘 총 출입" "$(echo "$response" | jq -r '.data.accessActivity.totalToday')" "회"
    print_metric "  입실" "$(echo "$response" | jq -r '.data.accessActivity.entries')" "회"
    print_metric "  퇴실" "$(echo "$response" | jq -r '.data.accessActivity.exits')" "회"
    print_metric "  거부됨" "$(echo "$response" | jq -r '.data.accessActivity.denied')" "회"
    echo ""

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}👥 방문자${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  현재 방문자" "$(echo "$response" | jq -r '.data.visitors.current')" "명"
    print_metric "  체크인 완료" "$(echo "$response" | jq -r '.data.visitors.checkedIn')" "명"
    print_metric "  체크아웃 완료" "$(echo "$response" | jq -r '.data.visitors.checkedOut')" "명"
    echo ""

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}🚗 주차 현황${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  총 주차 공간" "$(echo "$response" | jq -r '.data.parking.totalSlots')" "면"
    print_metric "  사용 중" "$(echo "$response" | jq -r '.data.parking.occupied')" "면"
    print_metric "  이용 가능" "$(echo "$response" | jq -r '.data.parking.available')" "면"
    print_metric "  주차율" "$(echo "$response" | jq -r '.data.parking.occupancyRate')" "%"
    echo ""
  else
    pretty_json "$response"
  fi

  print_success "대시보드 로드 완료"
}

cmd_users() {
  print_header
  local action="${1:-list}"

  case $action in
    list)
      print_info "사용자 목록 조회 중..."
      local response=$(api_request GET "/api/v1/users?limit=20")
      pretty_json "$response"
      ;;
    get)
      local user_id="${2:-}"
      if [ -z "$user_id" ]; then
        print_error "사용자 ID를 입력하세요"
        exit 1
      fi
      print_info "사용자 정보 조회 중... (ID: $user_id)"
      local response=$(api_request GET "/api/v1/users/${user_id}")
      pretty_json "$response"
      ;;
    *)
      print_error "알 수 없는 작업: $action"
      exit 1
      ;;
  esac
}

cmd_grant_access() {
  print_header
  local user_id="${1:-}"
  local zone="${2:-}"

  if [ -z "$user_id" ] || [ -z "$zone" ]; then
    print_error "사용법: grant-access <사용자ID> <구역>"
    exit 1
  fi

  print_info "접근 권한 부여 중... (사용자: $user_id, 구역: $zone)"

  local data=$(cat <<EOF
{
  "userId": "$user_id",
  "accessZones": ["$zone"],
  "validFrom": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
  "reason": "CLI를 통한 권한 부여"
}
EOF
)

  local response=$(api_request POST "/api/v1/access/grant" "$data")
  pretty_json "$response"
  print_success "접근 권한이 부여되었습니다"
}

cmd_revoke_access() {
  print_header
  local user_id="${1:-}"
  local zone="${2:-}"

  if [ -z "$user_id" ]; then
    print_error "사용법: revoke-access <사용자ID> [구역]"
    exit 1
  fi

  print_warning "접근 권한 철회 중... (사용자: $user_id)"

  local zones_json="[]"
  if [ -n "$zone" ]; then
    zones_json="[\"$zone\"]"
  fi

  local data=$(cat <<EOF
{
  "userId": "$user_id",
  "accessZones": $zones_json,
  "reason": "CLI를 통한 권한 철회",
  "immediate": true
}
EOF
)

  local response=$(api_request POST "/api/v1/access/revoke" "$data")
  pretty_json "$response"
  print_success "접근 권한이 철회되었습니다"
}

cmd_unlock() {
  print_header
  local access_point_id="${1:-}"
  local duration="${2:-5}"

  if [ -z "$access_point_id" ]; then
    print_error "사용법: unlock <출입구ID> [지속시간(초)]"
    exit 1
  fi

  print_warning "출입구 원격 개방 중... (ID: $access_point_id, 지속시간: ${duration}초)"

  local data=$(cat <<EOF
{
  "accessPointId": "$access_point_id",
  "duration": $duration,
  "reason": "CLI를 통한 원격 개방",
  "authorizedBy": "admin"
}
EOF
)

  local response=$(api_request POST "/api/v1/access-points/unlock" "$data")
  pretty_json "$response"
  print_success "출입구가 개방되었습니다"
}

cmd_access_logs() {
  print_header
  local user_id="${1:-}"

  print_info "출입 로그 조회 중..."

  local query="?limit=20"
  if [ -n "$user_id" ]; then
    query="${query}&userId=${user_id}"
  fi

  local response=$(api_request GET "/api/v1/access-logs${query}")
  pretty_json "$response"
}

cmd_visitors() {
  print_header
  local action="${1:-list}"

  case $action in
    list)
      print_info "방문자 목록 조회 중..."
      local response=$(api_request GET "/api/v1/visitors?limit=20")
      pretty_json "$response"
      ;;
    get)
      local visitor_id="${2:-}"
      if [ -z "$visitor_id" ]; then
        print_error "방문자 ID를 입력하세요"
        exit 1
      fi
      print_info "방문자 정보 조회 중... (ID: $visitor_id)"
      local response=$(api_request GET "/api/v1/visitors/${visitor_id}")
      pretty_json "$response"
      ;;
    *)
      print_error "알 수 없는 작업: $action"
      exit 1
      ;;
  esac
}

cmd_check_in() {
  print_header
  local visitor_id="${1:-}"

  if [ -z "$visitor_id" ]; then
    print_error "사용법: check-in <방문자ID>"
    exit 1
  fi

  print_info "방문자 체크인 중... (ID: $visitor_id)"

  local response=$(api_request POST "/api/v1/visitors/${visitor_id}/check-in" "{}")
  pretty_json "$response"
  print_success "체크인 완료"
}

cmd_check_out() {
  print_header
  local visitor_id="${1:-}"

  if [ -z "$visitor_id" ]; then
    print_error "사용법: check-out <방문자ID>"
    exit 1
  fi

  print_info "방문자 체크아웃 중... (ID: $visitor_id)"

  local response=$(api_request POST "/api/v1/visitors/${visitor_id}/check-out" "{}")
  pretty_json "$response"
  print_success "체크아웃 완료"
}

cmd_parking_status() {
  print_header
  local parking_lot_id="${1:-PL-001}"

  print_info "주차장 현황 조회 중... (ID: $parking_lot_id)"

  local response=$(api_request GET "/api/v1/parking/lots/${parking_lot_id}/status")

  if has_jq; then
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${GREEN}🚗 주차장 현황${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    print_metric "  총 주차면" "$(echo "$response" | jq -r '.data.totalSlots')" "면"
    print_metric "  사용 중" "$(echo "$response" | jq -r '.data.occupiedSlots')" "면"
    print_metric "  이용 가능" "$(echo "$response" | jq -r '.data.availableSlots')" "면"
    print_metric "  주차율" "$(echo "$response" | jq -r '.data.occupancyRate')" "%"
    echo ""
  else
    pretty_json "$response"
  fi

  print_success "주차장 현황 조회 완료"
}

cmd_emergency_activate() {
  print_header
  local emergency_type="${1:-fire}"

  print_warning "비상 모드 활성화 중... (유형: $emergency_type)"

  local data=$(cat <<EOF
{
  "emergencyType": "$emergency_type",
  "actions": ["unlock-all"],
  "reason": "CLI를 통한 비상 모드 활성화",
  "activatedBy": "admin"
}
EOF
)

  local response=$(api_request POST "/api/v1/emergency/activate" "$data")
  pretty_json "$response"
  print_success "비상 모드가 활성화되었습니다"
}

cmd_emergency_clear() {
  print_header
  local emergency_id="${1:-}"

  if [ -z "$emergency_id" ]; then
    print_error "사용법: emergency-clear <비상ID>"
    exit 1
  fi

  print_info "비상 모드 해제 중... (ID: $emergency_id)"

  local data=$(cat <<EOF
{
  "emergencyId": "$emergency_id",
  "deactivatedBy": "admin",
  "notes": "CLI를 통한 해제"
}
EOF
)

  local response=$(api_request POST "/api/v1/emergency/deactivate" "$data")
  pretty_json "$response"
  print_success "비상 모드가 해제되었습니다"
}

cmd_alerts() {
  print_header
  local building_id="${1:-BLD-001}"

  print_info "활성 알람 조회 중... (빌딩 ID: $building_id)"

  local response=$(api_request GET "/api/v1/alerts?buildingId=${building_id}&limit=10")
  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}사용법:${NC}
  access-control-system.sh <명령어> [옵션]

${CYAN}명령어:${NC}
  ${GREEN}config${NC}                        API 설정
  ${GREEN}dashboard${NC} [빌딩ID]             실시간 대시보드
  ${GREEN}users${NC} list|get [사용자ID]     사용자 관리
  ${GREEN}grant-access${NC} <사용자ID> <구역>   접근 권한 부여
  ${GREEN}revoke-access${NC} <사용자ID> [구역]  접근 권한 철회
  ${GREEN}unlock${NC} <출입구ID> [지속시간]     원격 개방
  ${GREEN}access-logs${NC} [사용자ID]          출입 로그 조회
  ${GREEN}visitors${NC} list|get [방문자ID]   방문자 목록
  ${GREEN}check-in${NC} <방문자ID>             방문자 체크인
  ${GREEN}check-out${NC} <방문자ID>            방문자 체크아웃
  ${GREEN}parking-status${NC} [주차장ID]       주차 현황
  ${GREEN}emergency-activate${NC} [유형]       비상 모드 활성화
  ${GREEN}emergency-clear${NC} <비상ID>        비상 모드 해제
  ${GREEN}alerts${NC} [빌딩ID]                 활성 알람
  ${GREEN}help${NC}                          도움말

${CYAN}예시:${NC}
  # 대시보드 보기
  ./access-control-system.sh dashboard BLD-001

  # 접근 권한 부여
  ./access-control-system.sh grant-access EMP-001 SERVER-ROOM

  # 출입구 원격 개방
  ./access-control-system.sh unlock AP-MAIN-001 10

  # 방문자 체크인
  ./access-control-system.sh check-in VIS-20251225-001

${CYAN}환경 변수:${NC}
  WIA_API_KEY          API 키
  WIA_API_ENDPOINT     API 엔드포인트

${CYAN}버전:${NC} ${VERSION}
${CYAN}라이선스:${NC} MIT

${BLUE}弘익人間 (홍익인간) - Benefit All Humanity${NC}
EOF
}

################################################################################
# Main
################################################################################

main() {
  load_config

  local command="${1:-help}"
  shift || true

  case $command in
    config)
      cmd_config "$@"
      ;;
    dashboard)
      cmd_dashboard "$@"
      ;;
    users)
      cmd_users "$@"
      ;;
    grant-access)
      cmd_grant_access "$@"
      ;;
    revoke-access)
      cmd_revoke_access "$@"
      ;;
    unlock)
      cmd_unlock "$@"
      ;;
    access-logs)
      cmd_access_logs "$@"
      ;;
    visitors)
      cmd_visitors "$@"
      ;;
    check-in)
      cmd_check_in "$@"
      ;;
    check-out)
      cmd_check_out "$@"
      ;;
    parking-status)
      cmd_parking_status "$@"
      ;;
    emergency-activate)
      cmd_emergency_activate "$@"
      ;;
    emergency-clear)
      cmd_emergency_clear "$@"
      ;;
    alerts)
      cmd_alerts "$@"
      ;;
    help|--help|-h)
      cmd_help
      ;;
    *)
      print_error "알 수 없는 명령어: $command"
      echo ""
      cmd_help
      exit 1
      ;;
  esac
}

main "$@"
