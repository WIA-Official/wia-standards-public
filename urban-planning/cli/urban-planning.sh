#!/bin/bash

##############################################################################
# WIA-CITY-016: Urban Planning Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Version: 1.0.0
# License: MIT
##############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_URBAN_PLANNING_ENDPOINT:-https://api.wia.org/city-016/v1}"
API_KEY="${WIA_URBAN_PLANNING_API_KEY}"

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

print_header() {
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}$1${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API 키가 설정되지 않았습니다. WIA_URBAN_PLANNING_API_KEY 환경 변수를 설정하세요."
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

# Commands

##############################################################################
# 토지 이용 관리
##############################################################################

cmd_list_land_use() {
    print_header "🏞️  토지 이용 목록"

    local district="${1:-}"
    local query=""

    if [ -n "$district" ]; then
        query="?district=$district"
    fi

    local response=$(api_request GET "/api/v1/land-use$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.parcelId)\t\(.zoning)\t\(.area)m²\t\(.buildingRegulation.far)\t\(.buildingRegulation.bcr)"' | \
            column -t -s $'\t' -N "필지ID,용도지역,면적,용적률,건폐율"
        print_success "토지 이용 목록을 성공적으로 조회했습니다"
    else
        print_error "토지 이용 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_get_land_use() {
    local parcel_id="$1"

    if [ -z "$parcel_id" ]; then
        print_error "사용법: $0 get-land-use <필지ID>"
        exit 1
    fi

    print_header "🏞️  토지 이용 조회: $parcel_id"

    local response=$(api_request GET "/api/v1/land-use/$parcel_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "토지 이용 정보를 성공적으로 조회했습니다"
    else
        print_error "토지 이용 정보 조회에 실패했습니다"
        exit 1
    fi
}

cmd_analyze_land_use() {
    local district="${1:-}"

    print_header "📊 토지 이용 분석"

    local data="{}"
    if [ -n "$district" ]; then
        data="{\"district\": \"$district\"}"
    fi

    local response=$(api_request POST "/api/v1/land-use/analyze" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "토지 이용 분석을 완료했습니다"
    else
        print_error "토지 이용 분석에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 용도 지역 관리
##############################################################################

cmd_list_zoning() {
    print_header "🏙️  용도 지역 목록"

    local response=$(api_request GET "/api/v1/zoning")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.zoneId)\t\(.name)\t\(.type)\t\(.totalArea)m²"' | \
            column -t -s $'\t' -N "구역ID,이름,유형,면적"
        print_success "용도 지역 목록을 성공적으로 조회했습니다"
    else
        print_error "용도 지역 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_get_zoning() {
    local zone_id="$1"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 get-zoning <구역ID>"
        exit 1
    fi

    print_header "🏙️  용도 지역 조회: $zone_id"

    local response=$(api_request GET "/api/v1/zoning/$zone_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "용도 지역 정보를 성공적으로 조회했습니다"
    else
        print_error "용도 지역 정보 조회에 실패했습니다"
        exit 1
    fi
}

cmd_check_compatibility() {
    local zone1="$1"
    local zone2="$2"

    if [ -z "$zone1" ] || [ -z "$zone2" ]; then
        print_error "사용법: $0 check-compatibility <용도지역1> <용도지역2>"
        exit 1
    fi

    print_header "🔍 용도 지역 호환성 분석"

    local data="{\"zone1\": \"$zone1\", \"zone2\": \"$zone2\"}"
    local response=$(api_request POST "/api/v1/zoning/compatibility" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "호환성 분석을 완료했습니다"
    else
        print_error "호환성 분석에 실패했습니다"
        exit 1
    fi
}

cmd_calculate_capacity() {
    local zone_id="$1"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 calculate-capacity <구역ID>"
        exit 1
    fi

    print_header "📐 개발 용량 계산: $zone_id"

    local response=$(api_request POST "/api/v1/zoning/$zone_id/capacity")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "개발 용량 계산을 완료했습니다"
    else
        print_error "개발 용량 계산에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 녹지 및 공원 관리
##############################################################################

cmd_list_green_space() {
    print_header "🌳 녹지 및 공원 목록"

    local response=$(api_request GET "/api/v1/green-space")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.greenSpaceId)\t\(.name)\t\(.type)\t\(.area)m²"' | \
            column -t -s $'\t' -N "녹지ID,이름,유형,면적"
        print_success "녹지 및 공원 목록을 성공적으로 조회했습니다"
    else
        print_error "녹지 및 공원 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_green_space_per_capita() {
    local zone_id="${1:-}"

    print_header "📊 1인당 녹지 면적 계산"

    local data="{}"
    if [ -n "$zone_id" ]; then
        data="{\"zoneId\": \"$zone_id\"}"
    fi

    local response=$(api_request POST "/api/v1/green-space/per-capita" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "1인당 녹지 면적 계산을 완료했습니다"
    else
        print_error "1인당 녹지 면적 계산에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 인프라 관리
##############################################################################

cmd_list_roads() {
    print_header "🛣️  도로 네트워크 목록"

    local response=$(api_request GET "/api/v1/infrastructure/roads")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.roadId)\t\(.name)\t\(.type)\t\(.length)m\t\(.lanes)차선"' | \
            column -t -s $'\t' -N "도로ID,이름,유형,길이,차선"
        print_success "도로 네트워크 목록을 성공적으로 조회했습니다"
    else
        print_error "도로 네트워크 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_list_transit() {
    print_header "🚇 대중교통 목록"

    local response=$(api_request GET "/api/v1/infrastructure/transit")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.transitId)\t\(.name)\t\(.type)\t\(.stations | length)개 역"' | \
            column -t -s $'\t' -N "노선ID,이름,유형,역수"
        print_success "대중교통 목록을 성공적으로 조회했습니다"
    else
        print_error "대중교통 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_plan_infrastructure() {
    local plan_id="$1"
    local population="$2"
    local area="$3"

    if [ -z "$plan_id" ] || [ -z "$population" ] || [ -z "$area" ]; then
        print_error "사용법: $0 plan-infrastructure <계획ID> <인구수> <면적(m²)>"
        exit 1
    fi

    print_header "🏗️  인프라 계획 수립"

    local data="{\"planId\": \"$plan_id\", \"population\": $population, \"area\": $area, \"landUse\": {}}"
    local response=$(api_request POST "/api/v1/infrastructure/plan" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "인프라 계획 수립을 완료했습니다"
    else
        print_error "인프라 계획 수립에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 인구 및 밀도 분석
##############################################################################

cmd_get_population() {
    local zone_id="$1"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 get-population <구역ID>"
        exit 1
    fi

    print_header "👥 인구 데이터 조회: $zone_id"

    local response=$(api_request GET "/api/v1/population/$zone_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "인구 데이터를 성공적으로 조회했습니다"
    else
        print_error "인구 데이터 조회에 실패했습니다"
        exit 1
    fi
}

cmd_calculate_density() {
    local zone_id="$1"
    local type="${2:-gross}"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 calculate-density <구역ID> [gross|net]"
        exit 1
    fi

    print_header "📊 밀도 계산: $zone_id"

    local data="{\"zoneId\": \"$zone_id\", \"type\": \"$type\"}"
    local response=$(api_request POST "/api/v1/population/density" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "밀도 계산을 완료했습니다"
    else
        print_error "밀도 계산에 실패했습니다"
        exit 1
    fi
}

cmd_project_population() {
    local zone_id="$1"
    local target_year="$2"
    local growth_rate="${3:-}"

    if [ -z "$zone_id" ] || [ -z "$target_year" ]; then
        print_error "사용법: $0 project-population <구역ID> <목표년도> [성장률(%)]"
        exit 1
    fi

    print_header "📈 인구 예측: $zone_id → $target_year"

    local data="{\"zoneId\": \"$zone_id\", \"targetYear\": $target_year"
    if [ -n "$growth_rate" ]; then
        data="$data, \"growthRate\": $growth_rate"
    fi
    data="$data}"

    local response=$(api_request POST "/api/v1/population/project" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "인구 예측을 완료했습니다"
    else
        print_error "인구 예측에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 도시 계획 관리
##############################################################################

cmd_list_plans() {
    print_header "📋 도시 계획 목록"

    local response=$(api_request GET "/api/v1/plans")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.planId)\t\(.name)\t\(.type)\t\(.status)"' | \
            column -t -s $'\t' -N "계획ID,이름,유형,상태"
        print_success "도시 계획 목록을 성공적으로 조회했습니다"
    else
        print_error "도시 계획 목록 조회에 실패했습니다"
        exit 1
    fi
}

cmd_get_plan() {
    local plan_id="$1"

    if [ -z "$plan_id" ]; then
        print_error "사용법: $0 get-plan <계획ID>"
        exit 1
    fi

    print_header "📋 도시 계획 조회: $plan_id"

    local response=$(api_request GET "/api/v1/plans/$plan_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "도시 계획 정보를 성공적으로 조회했습니다"
    else
        print_error "도시 계획 정보 조회에 실패했습니다"
        exit 1
    fi
}

cmd_plan_status() {
    local plan_id="$1"

    if [ -z "$plan_id" ]; then
        print_error "사용법: $0 plan-status <계획ID>"
        exit 1
    fi

    print_header "📊 계획 진행 상황: $plan_id"

    local response=$(api_request GET "/api/v1/plans/$plan_id/status")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "계획 진행 상황을 성공적으로 조회했습니다"
    else
        print_error "계획 진행 상황 조회에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 시뮬레이션
##############################################################################

cmd_simulate_growth() {
    local start_year="$1"
    local end_year="$2"
    local scenario="${3:-bau}"

    if [ -z "$start_year" ] || [ -z "$end_year" ]; then
        print_error "사용법: $0 simulate-growth <시작년도> <종료년도> [시나리오: bau|compact|sprawl]"
        exit 1
    fi

    print_header "🌆 도시 성장 시뮬레이션: $start_year → $end_year ($scenario)"

    local data="{\"startYear\": $start_year, \"endYear\": $end_year, \"timeStep\": 1, \"scenario\": \"$scenario\", \"populationGrowthRate\": 1.5, \"economicGrowthRate\": 2.0, \"constraints\": {}}"
    local response=$(api_request POST "/api/v1/simulation/growth" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "도시 성장 시뮬레이션을 완료했습니다"
    else
        print_error "도시 성장 시뮬레이션에 실패했습니다"
        exit 1
    fi
}

cmd_get_simulation() {
    local simulation_id="$1"

    if [ -z "$simulation_id" ]; then
        print_error "사용법: $0 get-simulation <시뮬레이션ID>"
        exit 1
    fi

    print_header "📊 시뮬레이션 결과 조회: $simulation_id"

    local response=$(api_request GET "/api/v1/simulation/$simulation_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "시뮬레이션 결과를 성공적으로 조회했습니다"
    else
        print_error "시뮬레이션 결과 조회에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# 분석
##############################################################################

cmd_analyze_accessibility() {
    local zone_id="$1"

    if [ -z "$zone_id" ]; then
        print_error "사용법: $0 analyze-accessibility <구역ID>"
        exit 1
    fi

    print_header "🚶 접근성 분석: $zone_id"

    local data="{\"zoneId\": \"$zone_id\"}"
    local response=$(api_request POST "/api/v1/analysis/accessibility" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "접근성 분석을 완료했습니다"
    else
        print_error "접근성 분석에 실패했습니다"
        exit 1
    fi
}

cmd_analyze_sustainability() {
    local plan_id="$1"

    if [ -z "$plan_id" ]; then
        print_error "사용법: $0 analyze-sustainability <계획ID>"
        exit 1
    fi

    print_header "🌱 지속가능성 분석: $plan_id"

    local data="{\"planId\": \"$plan_id\"}"
    local response=$(api_request POST "/api/v1/analysis/sustainability" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "지속가능성 분석을 완료했습니다"
    else
        print_error "지속가능성 분석에 실패했습니다"
        exit 1
    fi
}

##############################################################################
# Help
##############################################################################

cmd_help() {
    cat << EOF
${MAGENTA}╔════════════════════════════════════════════════════════════════╗
║  WIA-CITY-016: 도시 계획 표준 CLI                              ║
║  弘益人間 (홍익인간) - 널리 인간을 이롭게 하라                    ║
╚════════════════════════════════════════════════════════════════╝${NC}

${CYAN}토지 이용 관리:${NC}
  list-land-use [지구]                토지 이용 목록 조회
  get-land-use <필지ID>               토지 이용 정보 조회
  analyze-land-use [지구]             토지 이용 분석

${CYAN}용도 지역 관리:${NC}
  list-zoning                         용도 지역 목록 조회
  get-zoning <구역ID>                 용도 지역 정보 조회
  check-compatibility <유형1> <유형2> 용도 지역 호환성 분석
  calculate-capacity <구역ID>         개발 용량 계산

${CYAN}녹지 및 공원 관리:${NC}
  list-green-space                    녹지 및 공원 목록 조회
  green-space-per-capita [구역ID]     1인당 녹지 면적 계산

${CYAN}인프라 관리:${NC}
  list-roads                          도로 네트워크 목록 조회
  list-transit                        대중교통 목록 조회
  plan-infrastructure <계획ID> <인구> <면적>  인프라 계획 수립

${CYAN}인구 및 밀도:${NC}
  get-population <구역ID>             인구 데이터 조회
  calculate-density <구역ID> [유형]   밀도 계산
  project-population <구역ID> <년도> [성장률]  인구 예측

${CYAN}도시 계획:${NC}
  list-plans                          도시 계획 목록 조회
  get-plan <계획ID>                   도시 계획 정보 조회
  plan-status <계획ID>                계획 진행 상황 조회

${CYAN}시뮬레이션:${NC}
  simulate-growth <시작> <종료> [시나리오]  도시 성장 시뮬레이션
  get-simulation <시뮬레이션ID>       시뮬레이션 결과 조회

${CYAN}분석:${NC}
  analyze-accessibility <구역ID>      접근성 분석
  analyze-sustainability <계획ID>     지속가능성 분석

${CYAN}환경 변수:${NC}
  WIA_URBAN_PLANNING_API_KEY          API 키
  WIA_URBAN_PLANNING_ENDPOINT         API 엔드포인트

${CYAN}예제:${NC}
  # 토지 이용 목록 조회
  ./urban-planning.sh list-land-use

  # 용도 지역 호환성 확인
  ./urban-planning.sh check-compatibility residential_low commercial_central

  # 인구 밀도 계산
  ./urban-planning.sh calculate-density zone-001

  # 도시 성장 시뮬레이션
  ./urban-planning.sh simulate-growth 2025 2050 compact

EOF
}

##############################################################################
# Main
##############################################################################

main() {
    local command="${1:-help}"
    shift || true

    case "$command" in
        # 토지 이용
        list-land-use)
            cmd_list_land_use "$@"
            ;;
        get-land-use)
            cmd_get_land_use "$@"
            ;;
        analyze-land-use)
            cmd_analyze_land_use "$@"
            ;;

        # 용도 지역
        list-zoning)
            cmd_list_zoning "$@"
            ;;
        get-zoning)
            cmd_get_zoning "$@"
            ;;
        check-compatibility)
            cmd_check_compatibility "$@"
            ;;
        calculate-capacity)
            cmd_calculate_capacity "$@"
            ;;

        # 녹지 및 공원
        list-green-space)
            cmd_list_green_space "$@"
            ;;
        green-space-per-capita)
            cmd_green_space_per_capita "$@"
            ;;

        # 인프라
        list-roads)
            cmd_list_roads "$@"
            ;;
        list-transit)
            cmd_list_transit "$@"
            ;;
        plan-infrastructure)
            cmd_plan_infrastructure "$@"
            ;;

        # 인구
        get-population)
            cmd_get_population "$@"
            ;;
        calculate-density)
            cmd_calculate_density "$@"
            ;;
        project-population)
            cmd_project_population "$@"
            ;;

        # 계획
        list-plans)
            cmd_list_plans "$@"
            ;;
        get-plan)
            cmd_get_plan "$@"
            ;;
        plan-status)
            cmd_plan_status "$@"
            ;;

        # 시뮬레이션
        simulate-growth)
            cmd_simulate_growth "$@"
            ;;
        get-simulation)
            cmd_get_simulation "$@"
            ;;

        # 분석
        analyze-accessibility)
            cmd_analyze_accessibility "$@"
            ;;
        analyze-sustainability)
            cmd_analyze_sustainability "$@"
            ;;

        # 도움말
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
