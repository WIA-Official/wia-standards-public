#!/bin/bash

# WIA-CITY-008: 3D Printing Construction Standard - CLI Tool
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-CITY-008 API

set -e

# Configuration
API_ENDPOINT="${WIA_CITY008_ENDPOINT:-https://api.wia.org/city-008/v1}"
API_KEY="${WIA_CITY008_API_KEY:-}"

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
    echo -e "${MAGENTA}⚠${NC} $1"
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
        print_info "환경 변수 WIA_CITY008_API_KEY를 설정하거나 --api-key 옵션을 사용하세요."
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
cmd_register_project() {
    print_header "프로젝트 등록"

    local name=""
    local building_type="RESIDENTIAL"
    local address=""
    local city=""
    local country=""
    local lat=""
    local lon=""
    local length_m=""
    local width_m=""
    local height_m=""
    local wall_thickness_mm=200

    while [[ $# -gt 0 ]]; do
        case $1 in
            --name) name="$2"; shift 2 ;;
            --type) building_type="$2"; shift 2 ;;
            --address) address="$2"; shift 2 ;;
            --city) city="$2"; shift 2 ;;
            --country) country="$2"; shift 2 ;;
            --lat) lat="$2"; shift 2 ;;
            --lon) lon="$2"; shift 2 ;;
            --length) length_m="$2"; shift 2 ;;
            --width) width_m="$2"; shift 2 ;;
            --height) height_m="$2"; shift 2 ;;
            --wall-thickness) wall_thickness_mm="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    # Validate required fields
    if [ -z "$name" ] || [ -z "$address" ] || [ -z "$city" ] || [ -z "$country" ] || \
       [ -z "$lat" ] || [ -z "$lon" ] || [ -z "$length_m" ] || [ -z "$width_m" ] || [ -z "$height_m" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 register-project --name NAME --address ADDRESS --city CITY --country COUNTRY"
        echo "        --lat LAT --lon LON --length M --width M --height M"
        echo "        [--type TYPE] [--wall-thickness MM]"
        exit 1
    fi

    # Calculate area
    local area=$(echo "$length_m * $width_m" | bc)

    # Build JSON payload
    local payload=$(cat <<EOF
{
  "project": {
    "name": "$name",
    "design": {
      "name": "$name Design",
      "type": "$building_type",
      "dimensions": {
        "length_m": $length_m,
        "width_m": $width_m,
        "height_m": $height_m,
        "total_area_m2": $area
      },
      "walls": {
        "thickness_mm": $wall_thickness_mm,
        "material": "CONCRETE"
      },
      "roof": {
        "type": "FLAT"
      },
      "foundation": {
        "type": "SLAB"
      },
      "rooms": 4,
      "floors": 1,
      "windows": 6,
      "doors": 2,
      "model_file_url": "https://storage.wia.org/models/placeholder.stl",
      "building_code": "IBC",
      "estimated_print_time_hours": 24,
      "estimated_material_kg": 5000
    },
    "location": {
      "lat": $lat,
      "lon": $lon,
      "address": "$address",
      "city": "$city",
      "country": "$country"
    },
    "owner": "Default Owner",
    "contractor": "Default Contractor",
    "printer": {
      "printerId": "PRINTER-001",
      "model": "ICON Vulcan II",
      "manufacturer": "ICON",
      "type": "GANTRY"
    },
    "materials": [],
    "start_date": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "estimated_completion_date": "$(date -u -d '+30 days' +%Y-%m-%dT%H:%M:%SZ)",
    "completion_percent": 0,
    "building_permit": "PERMIT-$(date +%Y%m%d)-001"
  }
}
EOF
)

    print_info "프로젝트 등록 중..."
    response=$(api_request POST "/projects/register" "$payload")

    if echo "$response" | grep -q "projectId"; then
        print_success "프로젝트가 성공적으로 등록되었습니다!"
        echo ""
        print_json "$response"

        # Extract projectId
        if command -v jq &> /dev/null; then
            project_id=$(echo "$response" | jq -r '.data.projectId')
            tracking_url=$(echo "$response" | jq -r '.data.trackingUrl')
            echo ""
            print_info "프로젝트 ID: $project_id"
            print_info "추적 URL: $tracking_url"
        fi
    else
        print_error "프로젝트 등록 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_submit_print() {
    print_header "프린트 작업 제출"

    local project_id=""
    local gcode_url=""
    local layer_height=10
    local print_speed=100
    local extrusion_rate=150

    while [[ $# -gt 0 ]]; do
        case $1 in
            --project-id) project_id="$2"; shift 2 ;;
            --gcode) gcode_url="$2"; shift 2 ;;
            --layer-height) layer_height="$2"; shift 2 ;;
            --print-speed) print_speed="$2"; shift 2 ;;
            --extrusion-rate) extrusion_rate="$2"; shift 2 ;;
            *) print_error "알 수 없는 옵션: $1"; exit 1 ;;
        esac
    done

    if [ -z "$project_id" ] || [ -z "$gcode_url" ]; then
        print_error "필수 항목이 누락되었습니다."
        echo "사용법: $0 submit-print --project-id ID --gcode URL"
        echo "        [--layer-height MM] [--print-speed MM/S] [--extrusion-rate KG/H]"
        exit 1
    fi

    local payload=$(cat <<EOF
{
  "projectId": "$project_id",
  "gcode_url": "$gcode_url",
  "parameters": {
    "layer_height_mm": $layer_height,
    "layer_width_mm": 50,
    "print_speed_mm_s": $print_speed,
    "travel_speed_mm_s": 200,
    "extrusion_rate_kg_h": $extrusion_rate,
    "extrusion_multiplier": 1.0,
    "wall_thickness_mm": 200,
    "infill_density_percent": 20,
    "support_required": false
  }
}
EOF
)

    print_info "프린트 작업 제출 중..."
    response=$(api_request POST "/print-jobs/submit" "$payload")

    if echo "$response" | grep -q "jobId"; then
        print_success "프린트 작업이 성공적으로 제출되었습니다!"
        echo ""
        print_json "$response"
    else
        print_error "프린트 작업 제출 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_status() {
    local job_id="$1"

    if [ -z "$job_id" ]; then
        print_error "작업 ID를 입력하세요."
        echo "사용법: $0 status JOB_ID"
        exit 1
    fi

    print_header "프린트 작업 상태: $job_id"
    print_info "상태 조회 중..."

    response=$(api_request GET "/print-jobs/$job_id" "")

    if echo "$response" | grep -q "status"; then
        print_success "상태 정보를 찾았습니다!"
        echo ""
        print_json "$response"

        # Extract key info
        if command -v jq &> /dev/null; then
            status=$(echo "$response" | jq -r '.data.status')
            progress=$(echo "$response" | jq -r '.data.progress_percent')
            echo ""
            print_info "상태: $status"
            print_info "진행률: ${progress}%"
        fi
    else
        print_error "상태 정보를 찾을 수 없습니다."
        print_json "$response"
        exit 1
    fi
}

cmd_list_projects() {
    print_header "프로젝트 목록"
    print_info "프로젝트 조회 중..."

    response=$(api_request GET "/projects?limit=10" "")

    if echo "$response" | grep -q "data"; then
        print_success "프로젝트 목록을 조회했습니다!"
        echo ""
        print_json "$response"
    else
        print_error "프로젝트 목록 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_list_printers() {
    print_header "프린터 목록"
    print_info "프린터 조회 중..."

    response=$(api_request GET "/printers" "")

    if echo "$response" | grep -q "data"; then
        print_success "프린터 목록을 조회했습니다!"
        echo ""
        print_json "$response"
    else
        print_error "프린터 목록 조회 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_estimate_cost() {
    local length_m="${1:-10}"
    local width_m="${2:-8}"
    local height_m="${3:-3}"

    print_header "비용 추정"
    print_info "크기: ${length_m}m × ${width_m}m × ${height_m}m"

    local area=$(echo "$length_m * $width_m" | bc)

    local payload=$(cat <<EOF
{
  "design": {
    "dimensions": {
      "length_m": $length_m,
      "width_m": $width_m,
      "height_m": $height_m,
      "total_area_m2": $area
    },
    "walls": {
      "thickness_mm": 200,
      "material": "CONCRETE"
    }
  }
}
EOF
)

    response=$(api_request POST "/utilities/estimate-cost" "$payload")

    if echo "$response" | grep -q "total_cost_USD"; then
        print_success "비용을 추정했습니다!"
        echo ""
        print_json "$response"
    else
        print_error "비용 추정 실패"
        print_json "$response"
        exit 1
    fi
}

cmd_help() {
    cat <<EOF
${BLUE}WIA-CITY-008: 3D 프린팅 건설 표준 CLI${NC}
${CYAN}弘益人間 (홍익인간) - Benefit All Humanity${NC}

사용법: $0 <명령어> [옵션]

${YELLOW}명령어:${NC}
  register-project    새 건설 프로젝트 등록
  submit-print        프린트 작업 제출
  status              프린트 작업 상태 조회
  list-projects       프로젝트 목록 조회
  list-printers       프린터 목록 조회
  estimate-cost       건설 비용 추정
  help                도움말 표시

${YELLOW}환경 변수:${NC}
  WIA_CITY008_API_KEY      API 키 (필수)
  WIA_CITY008_ENDPOINT     API 엔드포인트 (기본값: https://api.wia.org/city-008/v1)

${YELLOW}예제:${NC}
  # 프로젝트 등록
  $0 register-project --name "Eco House" --type RESIDENTIAL \\
     --address "123 Main St" --city "Seoul" --country "Korea" \\
     --lat 37.5665 --lon 126.9780 --length 10 --width 8 --height 3

  # 프린트 작업 제출
  $0 submit-print --project-id CITY3DP-2025-001 \\
     --gcode https://storage.wia.org/gcode/house.gcode \\
     --layer-height 10 --print-speed 100

  # 작업 상태 조회
  $0 status PRINTJOB-2025-001

  # 프로젝트 목록
  $0 list-projects

  # 프린터 목록
  $0 list-printers

  # 비용 추정 (length, width, height in meters)
  $0 estimate-cost 10 8 3

${CYAN}🖨️ 3D 프린팅으로 더 빠르고 저렴하게! 지속가능한 건설의 미래를 함께 만들어갑니다.${NC}
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
        register-project) cmd_register_project "$@" ;;
        submit-print) cmd_submit_print "$@" ;;
        status) cmd_status "$@" ;;
        list-projects) cmd_list_projects ;;
        list-printers) cmd_list_printers ;;
        estimate-cost) cmd_estimate_cost "$@" ;;
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
