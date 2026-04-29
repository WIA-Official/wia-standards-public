#!/bin/bash

################################################################################
# WIA-FARMING_ROBOT CLI Tool
#
# 농업 로봇 제어 및 모니터링 명령줄 인터페이스
# Command-line interface for agricultural robot control and monitoring
#
# Version: 1.0.0
# Author: WIA (World Industry Association)
# License: MIT
#
# 弘益人間 (Benefit All Humanity)
################################################################################

set -euo pipefail

# 설정 / Configuration
readonly VERSION="1.0.0"
readonly API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.farming-robot.wia.org/v1}"
readonly CONFIG_DIR="${HOME}/.wia/farming-robot"
readonly CONFIG_FILE="${CONFIG_DIR}/config.json"
readonly LOG_DIR="${CONFIG_DIR}/logs"
readonly LOG_FILE="${LOG_DIR}/wia-farming-robot.log"

# 색상 코드 / Color codes
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly MAGENTA='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

################################################################################
# 헬퍼 함수 / Helper Functions
################################################################################

# 로그 기록 / Logging
log() {
    local level="$1"
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[${timestamp}] [${level}] ${message}" >> "${LOG_FILE}"
}

# 정보 메시지 / Info message
info() {
    echo -e "${CYAN}ℹ${NC} $@"
    log "INFO" "$@"
}

# 성공 메시지 / Success message
success() {
    echo -e "${GREEN}✓${NC} $@"
    log "SUCCESS" "$@"
}

# 경고 메시지 / Warning message
warn() {
    echo -e "${YELLOW}⚠${NC} $@" >&2
    log "WARN" "$@"
}

# 에러 메시지 / Error message
error() {
    echo -e "${RED}✗${NC} $@" >&2
    log "ERROR" "$@"
}

# 디버그 메시지 / Debug message
debug() {
    if [[ "${DEBUG:-false}" == "true" ]]; then
        echo -e "${MAGENTA}[DEBUG]${NC} $@" >&2
        log "DEBUG" "$@"
    fi
}

# 초기화 / Initialize
init_config() {
    mkdir -p "${CONFIG_DIR}" "${LOG_DIR}"

    if [[ ! -f "${CONFIG_FILE}" ]]; then
        cat > "${CONFIG_FILE}" <<EOF
{
  "apiKey": "",
  "robotId": "",
  "farmId": "",
  "apiEndpoint": "${API_ENDPOINT}",
  "defaultTimeout": 30,
  "telemetryInterval": 10
}
EOF
        info "설정 파일이 생성되었습니다 / Configuration file created: ${CONFIG_FILE}"
    fi
}

# API 호출 / API call
api_call() {
    local method="$1"
    local endpoint="$2"
    local data="${3:-}"

    local api_key=$(jq -r '.apiKey' "${CONFIG_FILE}")
    local robot_id=$(jq -r '.robotId' "${CONFIG_FILE}")

    if [[ -z "${api_key}" ]]; then
        error "API 키가 설정되지 않았습니다 / API key not configured"
        error "다음 명령을 실행하세요 / Run: wia-farming-robot config set apiKey YOUR_API_KEY"
        return 1
    fi

    local curl_opts=(
        -s
        -X "${method}"
        -H "Authorization: Bearer ${api_key}"
        -H "X-Robot-ID: ${robot_id}"
        -H "Content-Type: application/json"
    )

    if [[ -n "${data}" ]]; then
        curl_opts+=(-d "${data}")
    fi

    debug "API Call: ${method} ${API_ENDPOINT}${endpoint}"

    local response=$(curl "${curl_opts[@]}" "${API_ENDPOINT}${endpoint}")
    echo "${response}"
}

################################################################################
# 주요 기능 / Main Functions
################################################################################

# 로봇 상태 조회 / Get robot status
function cmd_status() {
    info "로봇 상태를 조회합니다... / Fetching robot status..."

    local robot_id=$(jq -r '.robotId' "${CONFIG_FILE}")
    local response=$(api_call "GET" "/robots/${robot_id}/status")

    if [[ -z "${response}" ]]; then
        error "응답을 받지 못했습니다 / No response received"
        return 1
    fi

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${GREEN}로봇 상태 / Robot Status${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"

    echo "${response}" | jq -r '
        "로봇 ID / Robot ID: \(.robotId)",
        "상태 / Status: \(.status)",
        "위치 / Position: \(.position.latitude), \(.position.longitude)",
        "정확도 / Accuracy: \(.position.accuracy)cm",
        "배터리 / Battery: \(.battery)%",
        "현재 작업 / Current Task: \(.currentTask // "없음 / None")",
        "전체 상태 / Health: \(.health.overall)"
    '

    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo ""

    success "상태 조회 완료 / Status retrieved successfully"
}

# 작업 생성 / Create task
function cmd_create_task() {
    local task_type="${1:-SEEDING}"
    local field_id="${2:-}"

    if [[ -z "${field_id}" ]]; then
        error "필드 ID가 필요합니다 / Field ID required"
        echo "사용법 / Usage: wia-farming-robot create-task <TASK_TYPE> <FIELD_ID>"
        return 1
    fi

    info "작업을 생성합니다... / Creating task..."

    local robot_id=$(jq -r '.robotId' "${CONFIG_FILE}")
    local task_data=$(cat <<EOF
{
  "robotId": "${robot_id}",
  "taskType": "${task_type}",
  "fieldId": "${field_id}",
  "parameters": {
    "seedType": "corn",
    "seedingRate": 75000,
    "rowSpacing": 0.76,
    "depth": 5.0
  },
  "schedule": {
    "startTime": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "priority": "HIGH"
  }
}
EOF
)

    local response=$(api_call "POST" "/tasks" "${task_data}")

    local task_id=$(echo "${response}" | jq -r '.taskId')
    success "작업이 생성되었습니다 / Task created: ${task_id}"

    echo "${response}" | jq '.'
}

# 작업 목록 조회 / List tasks
function cmd_list_tasks() {
    info "작업 목록을 조회합니다... / Fetching task list..."

    local response=$(api_call "GET" "/tasks?robotId=$(jq -r '.robotId' "${CONFIG_FILE}")")

    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${GREEN}작업 목록 / Task List${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"

    echo "${response}" | jq -r '.tasks[] |
        "ID: \(.taskId)\n" +
        "  타입 / Type: \(.taskType)\n" +
        "  상태 / Status: \(.status)\n" +
        "  진행률 / Progress: \(.progress)%\n" +
        "  시작 시간 / Started: \(.startedAt // "N/A")\n"
    '

    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo ""
}

# 작업 일시 정지 / Pause task
function cmd_pause_task() {
    local task_id="${1:-}"

    if [[ -z "${task_id}" ]]; then
        error "작업 ID가 필요합니다 / Task ID required"
        echo "사용법 / Usage: wia-farming-robot pause-task <TASK_ID>"
        return 1
    fi

    info "작업을 일시 정지합니다... / Pausing task: ${task_id}"

    local response=$(api_call "POST" "/tasks/${task_id}/pause")

    success "작업이 일시 정지되었습니다 / Task paused successfully"
    echo "${response}" | jq '.'
}

# 작업 재개 / Resume task
function cmd_resume_task() {
    local task_id="${1:-}"

    if [[ -z "${task_id}" ]]; then
        error "작업 ID가 필요합니다 / Task ID required"
        echo "사용법 / Usage: wia-farming-robot resume-task <TASK_ID>"
        return 1
    fi

    info "작업을 재개합니다... / Resuming task: ${task_id}"

    local response=$(api_call "POST" "/tasks/${task_id}/resume")

    success "작업이 재개되었습니다 / Task resumed successfully"
    echo "${response}" | jq '.'
}

# 경로 계획 생성 / Generate path plan
function cmd_plan_path() {
    local field_id="${1:-}"

    if [[ -z "${field_id}" ]]; then
        error "필드 ID가 필요합니다 / Field ID required"
        echo "사용법 / Usage: wia-farming-robot plan-path <FIELD_ID>"
        return 1
    fi

    info "경로를 계획합니다... / Planning path for field: ${field_id}"

    local plan_data=$(cat <<EOF
{
  "fieldId": "${field_id}",
  "taskType": "SEEDING",
  "workingWidth": 12.2,
  "overlap": 0.15,
  "pattern": "AB_LINE",
  "constraints": {
    "maxSlope": 15,
    "minTurningRadius": 8.0
  }
}
EOF
)

    local response=$(api_call "POST" "/pathplanning/generate" "${plan_data}")

    local plan_id=$(echo "${response}" | jq -r '.planId')
    success "경로 계획이 생성되었습니다 / Path plan created: ${plan_id}"

    echo "${response}" | jq '.'
}

# 센서 데이터 조회 / Query sensor data
function cmd_sensor_data() {
    local sensor_type="${1:-lidar}"
    local hours="${2:-1}"

    info "센서 데이터를 조회합니다... / Querying sensor data (${sensor_type}, last ${hours}h)"

    local robot_id=$(jq -r '.robotId' "${CONFIG_FILE}")
    local end_time=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    local start_time=$(date -u -d "${hours} hours ago" +%Y-%m-%dT%H:%M:%SZ)

    local response=$(api_call "GET" "/sensors/data?robotId=${robot_id}&type=${sensor_type}&start=${start_time}&end=${end_time}")

    echo "${response}" | jq '.'
}

# 긴급 정지 / Emergency stop
function cmd_emergency_stop() {
    warn "긴급 정지를 실행합니다! / EMERGENCY STOP INITIATED!"

    local robot_id=$(jq -r '.robotId' "${CONFIG_FILE}")
    local stop_data=$(cat <<EOF
{
  "type": "EMERGENCY_STOP",
  "robotId": "${robot_id}",
  "reason": "USER_REQUESTED",
  "timestamp": "$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
}
EOF
)

    local response=$(api_call "POST" "/robots/${robot_id}/emergency-stop" "${stop_data}")

    success "긴급 정지가 실행되었습니다 / Emergency stop executed"
    warn "로봇을 수동으로 검사한 후 재시작하세요 / Manual inspection required before restart"

    echo "${response}" | jq '.'
}

# 설정 관리 / Configuration management
function cmd_config() {
    local action="${1:-show}"
    local key="${2:-}"
    local value="${3:-}"

    case "${action}" in
        show)
            info "현재 설정 / Current configuration:"
            cat "${CONFIG_FILE}" | jq '.'
            ;;
        set)
            if [[ -z "${key}" ]] || [[ -z "${value}" ]]; then
                error "키와 값이 필요합니다 / Key and value required"
                echo "사용법 / Usage: wia-farming-robot config set <KEY> <VALUE>"
                return 1
            fi

            jq ".${key} = \"${value}\"" "${CONFIG_FILE}" > "${CONFIG_FILE}.tmp"
            mv "${CONFIG_FILE}.tmp" "${CONFIG_FILE}"
            success "설정이 업데이트되었습니다 / Configuration updated: ${key} = ${value}"
            ;;
        get)
            if [[ -z "${key}" ]]; then
                error "키가 필요합니다 / Key required"
                return 1
            fi

            jq -r ".${key}" "${CONFIG_FILE}"
            ;;
        *)
            error "알 수 없는 동작입니다 / Unknown action: ${action}"
            echo "사용 가능한 동작 / Available actions: show, set, get"
            return 1
            ;;
    esac
}

################################################################################
# 도움말 / Help
################################################################################

function cmd_help() {
    cat <<EOF
${GREEN}WIA-FARMING_ROBOT CLI v${VERSION}${NC}
농업 로봇 제어 및 모니터링 도구 / Agricultural Robot Control & Monitoring Tool

${BLUE}사용법 / Usage:${NC}
  wia-farming-robot <command> [options]

${BLUE}명령어 / Commands:${NC}
  ${CYAN}status${NC}                          로봇 상태 조회 / Get robot status
  ${CYAN}create-task${NC} <TYPE> <FIELD_ID>  작업 생성 / Create new task
  ${CYAN}list-tasks${NC}                     작업 목록 조회 / List all tasks
  ${CYAN}pause-task${NC} <TASK_ID>           작업 일시 정지 / Pause task
  ${CYAN}resume-task${NC} <TASK_ID>          작업 재개 / Resume task
  ${CYAN}plan-path${NC} <FIELD_ID>           경로 계획 생성 / Generate path plan
  ${CYAN}sensor-data${NC} [TYPE] [HOURS]     센서 데이터 조회 / Query sensor data
  ${CYAN}emergency-stop${NC}                 긴급 정지 / Emergency stop
  ${CYAN}config${NC} <show|set|get>          설정 관리 / Manage configuration
  ${CYAN}version${NC}                        버전 정보 / Show version
  ${CYAN}help${NC}                           도움말 표시 / Show this help

${BLUE}예제 / Examples:${NC}
  # 로봇 상태 조회
  wia-farming-robot status

  # 파종 작업 생성
  wia-farming-robot create-task SEEDING field-001

  # 작업 목록 조회
  wia-farming-robot list-tasks

  # 경로 계획 생성
  wia-farming-robot plan-path field-001

  # API 키 설정
  wia-farming-robot config set apiKey "your-api-key"

${BLUE}환경 변수 / Environment Variables:${NC}
  WIA_API_ENDPOINT    API 엔드포인트 / API endpoint
  DEBUG               디버그 모드 활성화 / Enable debug mode

${GREEN}弘益人間 (Benefit All Humanity)${NC}
© 2026 WIA (World Industry Association)
EOF
}

################################################################################
# 메인 / Main
################################################################################

function main() {
    # 초기화
    init_config

    # 명령어 처리
    local command="${1:-help}"
    shift || true

    case "${command}" in
        status)
            cmd_status "$@"
            ;;
        create-task)
            cmd_create_task "$@"
            ;;
        list-tasks)
            cmd_list_tasks "$@"
            ;;
        pause-task)
            cmd_pause_task "$@"
            ;;
        resume-task)
            cmd_resume_task "$@"
            ;;
        plan-path)
            cmd_plan_path "$@"
            ;;
        sensor-data)
            cmd_sensor_data "$@"
            ;;
        emergency-stop)
            cmd_emergency_stop "$@"
            ;;
        config)
            cmd_config "$@"
            ;;
        version)
            echo "WIA-FARMING_ROBOT CLI v${VERSION}"
            ;;
        help|--help|-h)
            cmd_help
            ;;
        *)
            error "알 수 없는 명령어입니다 / Unknown command: ${command}"
            echo ""
            cmd_help
            return 1
            ;;
    esac
}

# 스크립트 실행
main "$@"
