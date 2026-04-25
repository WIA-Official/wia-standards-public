#!/bin/bash

##############################################################################
# WIA-CITY-017: Traffic Simulation Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# Version: 1.0.0
# License: MIT
##############################################################################

set -e

# Configuration
API_ENDPOINT="${WIA_TRAFFIC_SIM_ENDPOINT:-https://api.wia.org/city-017/v1}"
API_KEY="${WIA_TRAFFIC_SIM_API_KEY}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

check_api_key() {
    if [ -z "$API_KEY" ]; then
        print_error "API key not set. Please set WIA_TRAFFIC_SIM_API_KEY environment variable."
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
# Simulation Commands
# ============================================================================

cmd_create_simulation() {
    local name="$1"
    local network_id="$2"
    local start_time="$3"
    local duration="${4:-7200}"

    if [ -z "$name" ] || [ -z "$network_id" ] || [ -z "$start_time" ]; then
        print_error "Usage: $0 create-simulation <name> <network-id> <start-time> [duration]"
        exit 1
    fi

    print_info "Creating simulation: $name"

    local data=$(cat <<EOF
{
  "name": "$name",
  "network": {
    "networkId": "$network_id"
  },
  "timeSettings": {
    "startTime": "$start_time",
    "endTime": "$(date -d "$start_time + $duration seconds" -Iseconds)",
    "timeStep": 0.5,
    "warmupPeriod": 900
  },
  "demand": {
    "scalingFactor": 1.0
  },
  "models": {
    "carFollowing": {
      "model": "idm",
      "minGap": 2.0,
      "timeHeadway": 1.5
    },
    "laneChanging": {
      "model": "mobil",
      "anticipationDistance": 200
    },
    "routeChoice": "dynamic"
  },
  "output": {
    "interval": 300,
    "metrics": ["speed", "volume", "density", "delay"],
    "aggregationLevel": "link"
  }
}
EOF
)

    local response=$(api_request POST "/api/v1/simulations" "$data")

    if [ $? -eq 0 ]; then
        local sim_id=$(echo "$response" | jq -r '.data.simulationId')
        print_success "Simulation created: $sim_id"
        echo "$sim_id"
    else
        print_error "Failed to create simulation"
        exit 1
    fi
}

cmd_start_simulation() {
    local sim_id="$1"

    if [ -z "$sim_id" ]; then
        print_error "Usage: $0 start-simulation <simulation-id>"
        exit 1
    fi

    print_info "Starting simulation: $sim_id"

    local response=$(api_request POST "/api/v1/simulations/$sim_id/start")

    if [ $? -eq 0 ]; then
        print_success "Simulation started"
    else
        print_error "Failed to start simulation"
        exit 1
    fi
}

cmd_get_simulation_status() {
    local sim_id="$1"

    if [ -z "$sim_id" ]; then
        print_error "Usage: $0 simulation-status <simulation-id>"
        exit 1
    fi

    print_info "Fetching simulation status: $sim_id"

    local response=$(api_request GET "/api/v1/simulations/$sim_id/status")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Status retrieved"
    else
        print_error "Failed to get status"
        exit 1
    fi
}

cmd_get_simulation_results() {
    local sim_id="$1"

    if [ -z "$sim_id" ]; then
        print_error "Usage: $0 simulation-results <simulation-id>"
        exit 1
    fi

    print_info "Fetching simulation results: $sim_id"

    local response=$(api_request GET "/api/v1/simulations/$sim_id/results")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Results retrieved"
    else
        print_error "Failed to get results"
        exit 1
    fi
}

cmd_list_simulations() {
    print_info "Fetching simulations..."

    local response=$(api_request GET "/api/v1/simulations")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.simulationId)\t\(.name)\t\(.status // "N/A")"' | \
            column -t -s $'\t' -N "ID,Name,Status"
        print_success "Simulations listed successfully"
    else
        print_error "Failed to fetch simulations"
        exit 1
    fi
}

# ============================================================================
# Traffic Commands
# ============================================================================

cmd_get_traffic_realtime() {
    local link_id="${1:-}"

    print_info "Fetching real-time traffic data..."

    local query=""
    if [ -n "$link_id" ]; then
        query="?linkIds=$link_id"
    fi

    local response=$(api_request GET "/api/v1/traffic/realtime$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.states[] | "\(.linkId)\t\(.flow.speed)\t\(.flow.volume)\t\(.flow.density)\t\(.quality.los)"' | \
            column -t -s $'\t' -N "Link,Speed(km/h),Volume(veh/h),Density(veh/km),LOS"
        print_success "Real-time traffic data retrieved"
    else
        print_error "Failed to fetch traffic data"
        exit 1
    fi
}

cmd_get_link_traffic() {
    local link_id="$1"
    local start="${2:-}"
    local end="${3:-}"

    if [ -z "$link_id" ]; then
        print_error "Usage: $0 link-traffic <link-id> [start-time] [end-time]"
        exit 1
    fi

    print_info "Fetching traffic data for link: $link_id"

    local query=""
    if [ -n "$start" ]; then
        query="?start=$start"
        if [ -n "$end" ]; then
            query="$query&end=$end"
        fi
    fi

    local response=$(api_request GET "/api/v1/traffic/links/$link_id$query")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Traffic data retrieved"
    else
        print_error "Failed to fetch traffic data"
        exit 1
    fi
}

# ============================================================================
# Signal Commands
# ============================================================================

cmd_list_signals() {
    print_info "Fetching traffic signals..."

    local response=$(api_request GET "/api/v1/signals")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.signalId)\t\(.nodeId)\t\(.controlType)\t\(.status.operational)"' | \
            column -t -s $'\t' -N "Signal-ID,Node-ID,Type,Operational"
        print_success "Signals listed successfully"
    else
        print_error "Failed to fetch signals"
        exit 1
    fi
}

cmd_get_signal() {
    local signal_id="$1"

    if [ -z "$signal_id" ]; then
        print_error "Usage: $0 get-signal <signal-id>"
        exit 1
    fi

    print_info "Fetching signal: $signal_id"

    local response=$(api_request GET "/api/v1/signals/$signal_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Signal retrieved"
    else
        print_error "Failed to fetch signal"
        exit 1
    fi
}

cmd_optimize_signals() {
    local signal_ids="$1"

    if [ -z "$signal_ids" ]; then
        print_error "Usage: $0 optimize-signals <signal-id1,signal-id2,...>"
        exit 1
    fi

    print_info "Optimizing signals: $signal_ids"

    local data=$(cat <<EOF
{
  "signalIds": ["$(echo $signal_ids | sed 's/,/","/g')"],
  "objective": "minimize_delay"
}
EOF
)

    local response=$(api_request POST "/api/v1/signals/optimize" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Signals optimized"
    else
        print_error "Failed to optimize signals"
        exit 1
    fi
}

# ============================================================================
# Incident Commands
# ============================================================================

cmd_list_incidents() {
    print_info "Fetching traffic incidents..."

    local response=$(api_request GET "/api/v1/incidents?active=true")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.incidentId)\t\(.type)\t\(.severity)\t\(.location.linkId)"' | \
            column -t -s $'\t' -N "ID,Type,Severity,Link"
        print_success "Incidents listed successfully"
    else
        print_error "Failed to fetch incidents"
        exit 1
    fi
}

cmd_create_incident() {
    local type="$1"
    local link_id="$2"
    local position="$3"
    local lanes_blocked="$4"

    if [ -z "$type" ] || [ -z "$link_id" ] || [ -z "$position" ]; then
        print_error "Usage: $0 create-incident <type> <link-id> <position-m> <lanes-blocked>"
        exit 1
    fi

    print_info "Creating incident..."

    local data=$(cat <<EOF
{
  "type": "$type",
  "location": {
    "linkId": "$link_id",
    "position": $position,
    "coordinates": {"latitude": 0, "longitude": 0}
  },
  "impact": {
    "lanesBlocked": [$(echo $lanes_blocked | sed 's/,/, /g')],
    "capacityReduction": 50
  },
  "time": {
    "startTime": "$(date -Iseconds)"
  },
  "severity": "moderate"
}
EOF
)

    local response=$(api_request POST "/api/v1/incidents" "$data")

    if [ $? -eq 0 ]; then
        local incident_id=$(echo "$response" | jq -r '.data.incidentId')
        print_success "Incident created: $incident_id"
        echo "$incident_id"
    else
        print_error "Failed to create incident"
        exit 1
    fi
}

cmd_analyze_incident() {
    local incident_id="$1"

    if [ -z "$incident_id" ]; then
        print_error "Usage: $0 analyze-incident <incident-id>"
        exit 1
    fi

    print_info "Analyzing incident impact: $incident_id"

    local response=$(api_request POST "/api/v1/incidents/$incident_id/analyze")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Impact analysis completed"
    else
        print_error "Failed to analyze incident"
        exit 1
    fi
}

# ============================================================================
# Forecasting Commands
# ============================================================================

cmd_forecast_traffic() {
    local link_id="$1"
    local horizon="${2:-short}"

    if [ -z "$link_id" ]; then
        print_error "Usage: $0 forecast-traffic <link-id> [short|medium|long]"
        exit 1
    fi

    print_info "Forecasting traffic for link: $link_id (horizon: $horizon)"

    local response=$(api_request GET "/api/v1/forecast/links/$link_id?horizon=$horizon")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Forecast retrieved"
    else
        print_error "Failed to get forecast"
        exit 1
    fi
}

cmd_predict_congestion() {
    local time="$1"
    local duration="${2:-3600}"

    if [ -z "$time" ]; then
        print_error "Usage: $0 predict-congestion <time> [duration-seconds]"
        exit 1
    fi

    print_info "Predicting congestion at: $time"

    local data=$(cat <<EOF
{
  "time": "$time",
  "duration": $duration
}
EOF
)

    local response=$(api_request POST "/api/v1/forecast/congestion" "$data")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Congestion prediction completed"
    else
        print_error "Failed to predict congestion"
        exit 1
    fi
}

# ============================================================================
# Network Commands
# ============================================================================

cmd_list_links() {
    print_info "Fetching road links..."

    local response=$(api_request GET "/api/v1/network/links")

    if [ $? -eq 0 ]; then
        echo "$response" | jq -r '.data.items[] | "\(.linkId)\t\(.name)\t\(.characteristics.linkType)\t\(.geometry.length)\t\(.geometry.lanes)"' | \
            column -t -s $'\t' -N "Link-ID,Name,Type,Length(m),Lanes"
        print_success "Links listed successfully"
    else
        print_error "Failed to fetch links"
        exit 1
    fi
}

cmd_get_link() {
    local link_id="$1"

    if [ -z "$link_id" ]; then
        print_error "Usage: $0 get-link <link-id>"
        exit 1
    fi

    print_info "Fetching link: $link_id"

    local response=$(api_request GET "/api/v1/network/links/$link_id")

    if [ $? -eq 0 ]; then
        echo "$response" | jq '.data'
        print_success "Link retrieved"
    else
        print_error "Failed to fetch link"
        exit 1
    fi
}

# ============================================================================
# Help Command
# ============================================================================

cmd_help() {
    cat <<EOF
WIA-CITY-017 교통 시뮬레이션 CLI 도구

사용법: $0 <명령어> [옵션]

시뮬레이션 명령어:
  create-simulation <이름> <네트워크ID> <시작시간> [기간]
                                      새 시뮬레이션 생성
  start-simulation <시뮬레이션ID>    시뮬레이션 시작
  simulation-status <시뮬레이션ID>   시뮬레이션 상태 조회
  simulation-results <시뮬레이션ID>  시뮬레이션 결과 조회
  list-simulations                    시뮬레이션 목록 조회

교통 데이터 명령어:
  traffic-realtime [링크ID]          실시간 교통 데이터 조회
  link-traffic <링크ID> [시작] [종료] 링크 교통 데이터 조회

신호등 명령어:
  list-signals                        신호등 목록 조회
  get-signal <신호ID>                 신호등 정보 조회
  optimize-signals <신호ID1,ID2,...>  신호 최적화

사고/사건 명령어:
  list-incidents                      사고 목록 조회
  create-incident <유형> <링크ID> <위치> <차단차로>
                                      사고 생성
  analyze-incident <사고ID>           사고 영향 분석

예측 명령어:
  forecast-traffic <링크ID> [단기|중기|장기]
                                      교통량 예측
  predict-congestion <시간> [기간]    혼잡 예측

네트워크 명령어:
  list-links                          도로 링크 목록 조회
  get-link <링크ID>                   링크 정보 조회

환경 변수:
  WIA_TRAFFIC_SIM_ENDPOINT            API 엔드포인트 (기본값: https://api.wia.org/city-017/v1)
  WIA_TRAFFIC_SIM_API_KEY             API 키 (필수)

예시:
  # 시뮬레이션 생성
  $0 create-simulation "강남 출근 시간대" network-gangnam "2025-12-26T07:00:00" 7200

  # 실시간 교통 데이터 조회
  $0 traffic-realtime

  # 특정 링크 교통 데이터
  $0 link-traffic link-001

  # 신호 최적화
  $0 optimize-signals signal-001,signal-002,signal-003

  # 교통량 예측
  $0 forecast-traffic link-001 short

  # 혼잡 예측
  $0 predict-congestion "2025-12-26T18:00:00" 3600

弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
© 2025 WIA (World Certification Industry Association)
EOF
}

# ============================================================================
# Main
# ============================================================================

COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
    create-simulation)
        cmd_create_simulation "$@"
        ;;
    start-simulation)
        cmd_start_simulation "$@"
        ;;
    simulation-status)
        cmd_get_simulation_status "$@"
        ;;
    simulation-results)
        cmd_get_simulation_results "$@"
        ;;
    list-simulations)
        cmd_list_simulations "$@"
        ;;
    traffic-realtime)
        cmd_get_traffic_realtime "$@"
        ;;
    link-traffic)
        cmd_get_link_traffic "$@"
        ;;
    list-signals)
        cmd_list_signals "$@"
        ;;
    get-signal)
        cmd_get_signal "$@"
        ;;
    optimize-signals)
        cmd_optimize_signals "$@"
        ;;
    list-incidents)
        cmd_list_incidents "$@"
        ;;
    create-incident)
        cmd_create_incident "$@"
        ;;
    analyze-incident)
        cmd_analyze_incident "$@"
        ;;
    forecast-traffic)
        cmd_forecast_traffic "$@"
        ;;
    predict-congestion)
        cmd_predict_congestion "$@"
        ;;
    list-links)
        cmd_list_links "$@"
        ;;
    get-link)
        cmd_get_link "$@"
        ;;
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
