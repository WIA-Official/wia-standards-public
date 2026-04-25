#!/bin/bash

################################################################################
# WIA-ENE-030: Biodiversity Index Standard - CLI Tool
#
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This CLI tool provides command-line access to the WIA-ENE-030
# Biodiversity Index Standard API.
#
# Usage:
#   biodiversity-index.sh <command> [options]
#
# Commands:
#   create-observation   Create species observation
#   get-observation      Get observation by ID
#   list-observations    List observations
#   create-survey        Create biodiversity survey
#   calculate-diversity  Calculate diversity indices
#   assess-habitat       Assess habitat quality
#   list-endangered      List endangered species
#   list-endemic         List endemic species
#   search-species       Search for species
#   submit-edna          Submit eDNA sample
#   generate-report      Generate biodiversity report
#   kpi                  View KPI dashboard
#   help                 Show this help
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Configuration
VERSION="1.0.0"
API_ENDPOINT="${WIA_API_ENDPOINT:-https://api.wia.org/ene-030/v1}"
API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/biodiversity-index.conf"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║      🦋  WIA-ENE-030: Biodiversity Index CLI v${VERSION}         ║"
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

# Check if jq is available
has_jq() {
  command -v jq &> /dev/null
}

# Pretty print JSON (uses jq if available)
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
# WIA-ENE-030 Biodiversity Index CLI Configuration
WIA_API_KEY_CONF="${API_KEY}"
WIA_API_ENDPOINT_CONF="${API_ENDPOINT}"
EOF
  chmod 600 "$CONFIG_FILE"
  print_success "Configuration saved to $CONFIG_FILE"
}

# Make API request
api_request() {
  local method="$1"
  local path="$2"
  local data="${3:-}"

  if [ -z "$API_KEY" ]; then
    print_error "API key not set. Use: export WIA_API_KEY=your-key or run 'config' command"
    exit 1
  fi

  local url="${API_ENDPOINT}${path}"
  local response

  if [ "$method" = "GET" ]; then
    response=$(curl -s -X GET "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-030" \
      -H "X-WIA-Version: 1.0.0")
  else
    response=$(curl -s -X "$method" "$url" \
      -H "Authorization: Bearer $API_KEY" \
      -H "Content-Type: application/json" \
      -H "X-WIA-Standard: ENE-030" \
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
  echo "Configuration Setup"
  echo ""

  read -p "API Endpoint [${API_ENDPOINT}]: " input_endpoint
  if [ -n "$input_endpoint" ]; then
    API_ENDPOINT="$input_endpoint"
  fi

  read -sp "API Key: " input_key
  echo ""
  if [ -n "$input_key" ]; then
    API_KEY="$input_key"
  fi

  save_config
  echo ""
  print_success "Configuration complete!"
}

cmd_create_observation() {
  local species="${1:-}"
  local common_name="${2:-}"
  local count="${3:-1}"
  local location="${4:-Unknown}"

  if [ -z "$species" ]; then
    echo "Usage: $0 create-observation --species SCIENTIFIC_NAME --common-name NAME --count N --location SITE"
    echo ""
    echo "Example:"
    echo "  $0 create-observation --species 'Parus major' --common-name '박새' --count 3 --location '한라산 국립공원'"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "location": {
    "siteName": "${location}",
    "coordinates": {
      "latitude": 0,
      "longitude": 0
    },
    "habitat": "MONTANE_FOREST",
    "ecosystem": "TERRESTRIAL"
  },
  "species": {
    "scientificName": "${species}",
    "commonName": "${common_name}",
    "taxonomicRank": "SPECIES",
    "taxonomy": {
      "kingdom": "Animalia",
      "phylum": "Chordata",
      "class": "Aves",
      "order": "Passeriformes",
      "family": "Paridae",
      "genus": "Parus",
      "species": "major"
    },
    "isEndemic": false
  },
  "individual": {
    "count": ${count}
  },
  "method": {
    "surveyType": "VISUAL_ENCOUNTER",
    "samplingEffort": 1,
    "detectionMethod": "시각",
    "identificationMethod": "야외 관찰"
  },
  "evidence": {},
  "observerId": "CLI-USER"
}
EOF
)

  print_info "Creating species observation..."
  local response=$(api_request "POST" "/api/v1/observation/create" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local obs_id=$(echo "$response" | jq -r '.data.observationId')
    print_success "Observation created: $obs_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create observation"
    echo "$response"
    exit 1
  fi
}

cmd_get_observation() {
  local obs_id="${1:-}"

  if [ -z "$obs_id" ]; then
    echo "Usage: $0 get-observation OBSERVATION_ID"
    exit 1
  fi

  print_info "Fetching observation: $obs_id"
  local response=$(api_request "GET" "/api/v1/observation/$obs_id")
  pretty_json "$response"
}

cmd_list_observations() {
  local page="${1:-1}"
  local limit="${2:-10}"

  local path="/api/v1/observations?page=$page&limit=$limit"

  print_info "Listing observations..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_create_survey() {
  local name="${1:-}"
  local site="${2:-}"
  local start_date="${3:-$(date +%Y-%m-%d)}"
  local end_date="${4:-}"

  if [ -z "$name" ] || [ -z "$site" ]; then
    echo "Usage: $0 create-survey --name NAME --site SITE --start-date DATE [--end-date DATE]"
    echo ""
    echo "Example:"
    echo "  $0 create-survey --name '한라산 생물다양성 조사' --site '한라산 국립공원' --start-date 2025-01-01 --end-date 2025-12-31"
    exit 1
  fi

  if [ -z "$end_date" ]; then
    end_date=$(date -d "+1 year" +%Y-%m-%d)
  fi

  local data=$(cat <<EOF
{
  "projectName": "${name}",
  "period": {
    "startDate": "${start_date}T00:00:00Z",
    "endDate": "${end_date}T23:59:59Z"
  },
  "area": {
    "siteName": "${site}",
    "boundary": {
      "type": "Polygon",
      "coordinates": [[[126.5, 33.3], [126.6, 33.3], [126.6, 33.4], [126.5, 33.4], [126.5, 33.3]]]
    },
    "areaSize": 100
  },
  "team": {
    "leader": "CLI-USER",
    "members": [],
    "organization": "WIA"
  },
  "targets": {
    "taxonomicGroups": ["Aves", "Mammalia"],
    "habitatTypes": ["MONTANE_FOREST"]
  },
  "results": {
    "totalSpecies": 0,
    "totalIndividuals": 0,
    "endemicSpecies": 0,
    "endangeredSpecies": 0,
    "observations": []
  }
}
EOF
)

  print_info "Creating biodiversity survey..."
  local response=$(api_request "POST" "/api/v1/survey/create" "$data")

  if has_jq && echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    local survey_id=$(echo "$response" | jq -r '.data.surveyId')
    print_success "Survey created: $survey_id"
    echo ""
    pretty_json "$response"
  else
    print_error "Failed to create survey"
    echo "$response"
    exit 1
  fi
}

cmd_calculate_diversity() {
  local survey_id="${1:-}"

  if [ -z "$survey_id" ]; then
    echo "Usage: $0 calculate-diversity --survey-id SURVEY_ID"
    exit 1
  fi

  print_info "Calculating diversity indices..."
  local data=$(cat <<EOF
{
  "surveyId": "${survey_id}"
}
EOF
)

  local response=$(api_request "POST" "/api/v1/diversity/calculate" "$data")
  pretty_json "$response"
}

cmd_assess_habitat() {
  local site="${1:-}"
  local habitat_type="${2:-MONTANE_FOREST}"

  if [ -z "$site" ]; then
    echo "Usage: $0 assess-habitat --site SITE_NAME [--habitat-type TYPE]"
    echo ""
    echo "Habitat Types: MONTANE_FOREST, LOWLAND_FOREST, GRASSLAND, WETLAND, RIVER, LAKE, CORAL_REEF, etc."
    exit 1
  fi

  local data=$(cat <<EOF
{
  "siteName": "${site}",
  "physical": {
    "habitatType": "${habitat_type}",
    "areaSize": 100,
    "elevation": 1000,
    "slope": 15,
    "aspect": "N"
  },
  "environmental": {
    "temperature": {
      "mean": 15,
      "min": 5,
      "max": 25
    },
    "precipitation": 1500,
    "humidity": 70
  },
  "vegetation": {
    "canopyCover": 80,
    "treeHeight": 15,
    "understory": "dense",
    "invasiveSpecies": []
  },
  "disturbances": [],
  "threats": [],
  "quality": {
    "overallGrade": "B",
    "integrityScore": 85,
    "conditionScore": 80,
    "managementNeeds": []
  }
}
EOF
)

  print_info "Assessing habitat..."
  local response=$(api_request "POST" "/api/v1/habitat/assess" "$data")
  pretty_json "$response"
}

cmd_list_endangered() {
  local region="${1:-}"

  local path="/api/v1/species/endangered"
  if [ -n "$region" ]; then
    path="$path?region=$region"
  fi

  print_info "Listing endangered species..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_list_endemic() {
  local region="${1:-}"

  if [ -z "$region" ]; then
    echo "Usage: $0 list-endemic --region REGION"
    exit 1
  fi

  print_info "Listing endemic species in: $region"
  local response=$(api_request "GET" "/api/v1/species/endemic?region=$region")
  pretty_json "$response"
}

cmd_search_species() {
  local query="${1:-}"

  if [ -z "$query" ]; then
    echo "Usage: $0 search-species QUERY"
    exit 1
  fi

  print_info "Searching for: $query"
  local response=$(api_request "GET" "/api/v1/species/search?q=$(echo "$query" | jq -sRr @uri)")
  pretty_json "$response"
}

cmd_submit_edna() {
  local sample_id="${1:-}"
  local location="${2:-Unknown}"

  if [ -z "$sample_id" ]; then
    echo "Usage: $0 submit-edna --sample-id ID --location SITE"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "sampleId": "${sample_id}",
  "collectionDate": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "location": {
    "siteName": "${location}",
    "coordinates": {
      "latitude": 0,
      "longitude": 0
    },
    "habitat": "RIVER",
    "ecosystem": "FRESHWATER"
  },
  "sampleType": "water",
  "volume": 1.5,
  "preservationMethod": "frozen",
  "storageConditions": "-20C"
}
EOF
)

  print_info "Submitting eDNA sample..."
  local response=$(api_request "POST" "/api/v1/edna/submit" "$data")
  pretty_json "$response"
}

cmd_generate_report() {
  local type="${1:-annual}"
  local region="${2:-}"
  local start_date="${3:-}"
  local end_date="${4:-}"

  if [ -z "$region" ] || [ -z "$start_date" ] || [ -z "$end_date" ]; then
    echo "Usage: $0 generate-report --type TYPE --region REGION --start-date DATE --end-date DATE"
    echo ""
    echo "Report Types: quarterly, annual, comprehensive"
    exit 1
  fi

  local data=$(cat <<EOF
{
  "reportType": "$type",
  "region": "$region",
  "period": {
    "startDate": "${start_date}T00:00:00Z",
    "endDate": "${end_date}T23:59:59Z"
  }
}
EOF
)

  print_info "Generating $type report for: $region"
  local response=$(api_request "POST" "/api/v1/report/generate" "$data")
  pretty_json "$response"
}

cmd_kpi() {
  local region="${1:-}"
  local date="${2:-$(date -u +%Y-%m-%d)}"

  local path="/api/v1/analytics/kpi"
  local params=""
  if [ -n "$region" ]; then
    params="region=$region"
  fi
  if [ -n "$date" ]; then
    if [ -n "$params" ]; then
      params="$params&date=$date"
    else
      params="date=$date"
    fi
  fi
  if [ -n "$params" ]; then
    path="$path?$params"
  fi

  print_info "Fetching KPI dashboard..."
  local response=$(api_request "GET" "$path")
  pretty_json "$response"
}

cmd_help() {
  print_header
  cat <<EOF
${CYAN}Commands:${NC}

  ${GREEN}config${NC}
    Configure API endpoint and key

  ${GREEN}create-observation${NC} --species NAME --common-name NAME --count N --location SITE
    Create species observation
    Example: create-observation --species 'Parus major' --common-name '박새' --count 3

  ${GREEN}get-observation${NC} OBSERVATION_ID
    Get observation by ID

  ${GREEN}list-observations${NC} [--page PAGE] [--limit LIMIT]
    List observations with pagination

  ${GREEN}create-survey${NC} --name NAME --site SITE --start-date DATE [--end-date DATE]
    Create biodiversity survey project

  ${GREEN}calculate-diversity${NC} --survey-id SURVEY_ID
    Calculate diversity indices for a survey

  ${GREEN}assess-habitat${NC} --site SITE_NAME [--habitat-type TYPE]
    Assess habitat quality

  ${GREEN}list-endangered${NC} [--region REGION]
    List endangered species

  ${GREEN}list-endemic${NC} --region REGION
    List endemic species for a region

  ${GREEN}search-species${NC} QUERY
    Search for species by name

  ${GREEN}submit-edna${NC} --sample-id ID --location SITE
    Submit eDNA sample

  ${GREEN}generate-report${NC} --type TYPE --region REGION --start-date DATE --end-date DATE
    Generate biodiversity report

  ${GREEN}kpi${NC} [--region REGION] [--date DATE]
    View KPI dashboard

  ${GREEN}help${NC}
    Show this help

${CYAN}Environment Variables:${NC}

  ${YELLOW}WIA_API_KEY${NC}         API authentication key
  ${YELLOW}WIA_API_ENDPOINT${NC}    API endpoint (default: https://api.wia.org/ene-030/v1)

${CYAN}Examples:${NC}

  # Configure CLI
  $0 config

  # Create observation
  $0 create-observation --species 'Parus major' --common-name '박새' --count 3 --location '한라산'

  # Create survey
  $0 create-survey --name '한라산 조사' --site '한라산 국립공원' --start-date 2025-01-01

  # Calculate diversity
  $0 calculate-diversity --survey-id SURVEY-2025-001

  # List endangered species
  $0 list-endangered --region '제주도'

${CYAN}More Information:${NC}

  Documentation: https://docs.wia.org/ene-030
  GitHub: https://github.com/WIA-Official/wia-standards
  Website: https://wia.org/standards/ene-030

${BLUE}════════════════════════════════════════════════════════════════${NC}
${GREEN}         弘益人間 (홍익인간) · Benefit All Humanity${NC}
${BLUE}════════════════════════════════════════════════════════════════${NC}
EOF
}

################################################################################
# Main
################################################################################

# Load configuration
load_config

# Parse command
COMMAND="${1:-help}"
shift || true

case "$COMMAND" in
  config)
    cmd_config
    ;;
  create-observation)
    SPECIES=""
    COMMON_NAME=""
    COUNT="1"
    LOCATION="Unknown"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --species) SPECIES="$2"; shift 2 ;;
        --common-name) COMMON_NAME="$2"; shift 2 ;;
        --count) COUNT="$2"; shift 2 ;;
        --location) LOCATION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_observation "$SPECIES" "$COMMON_NAME" "$COUNT" "$LOCATION"
    ;;
  get-observation)
    cmd_get_observation "$@"
    ;;
  list-observations)
    PAGE="1"
    LIMIT="10"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --page) PAGE="$2"; shift 2 ;;
        --limit) LIMIT="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_observations "$PAGE" "$LIMIT"
    ;;
  create-survey)
    NAME=""
    SITE=""
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --name) NAME="$2"; shift 2 ;;
        --site) SITE="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_create_survey "$NAME" "$SITE" "$START_DATE" "$END_DATE"
    ;;
  calculate-diversity)
    SURVEY_ID=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --survey-id) SURVEY_ID="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_calculate_diversity "$SURVEY_ID"
    ;;
  assess-habitat)
    SITE=""
    HABITAT_TYPE="MONTANE_FOREST"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --site) SITE="$2"; shift 2 ;;
        --habitat-type) HABITAT_TYPE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_assess_habitat "$SITE" "$HABITAT_TYPE"
    ;;
  list-endangered)
    REGION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_endangered "$REGION"
    ;;
  list-endemic)
    REGION=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_list_endemic "$REGION"
    ;;
  search-species)
    cmd_search_species "$@"
    ;;
  submit-edna)
    SAMPLE_ID=""
    LOCATION="Unknown"
    while [[ $# -gt 0 ]]; do
      case $1 in
        --sample-id) SAMPLE_ID="$2"; shift 2 ;;
        --location) LOCATION="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_submit_edna "$SAMPLE_ID" "$LOCATION"
    ;;
  generate-report)
    TYPE="annual"
    REGION=""
    START_DATE=""
    END_DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --type) TYPE="$2"; shift 2 ;;
        --region) REGION="$2"; shift 2 ;;
        --start-date) START_DATE="$2"; shift 2 ;;
        --end-date) END_DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_generate_report "$TYPE" "$REGION" "$START_DATE" "$END_DATE"
    ;;
  kpi)
    REGION=""
    DATE=""
    while [[ $# -gt 0 ]]; do
      case $1 in
        --region) REGION="$2"; shift 2 ;;
        --date) DATE="$2"; shift 2 ;;
        *) shift ;;
      esac
    done
    cmd_kpi "$REGION" "$DATE"
    ;;
  help|--help|-h)
    cmd_help
    ;;
  version|--version|-v)
    echo "WIA-ENE-030 Biodiversity Index CLI v${VERSION}"
    ;;
  *)
    print_error "Unknown command: $COMMAND"
    echo ""
    echo "Run '$0 help' for usage information"
    exit 1
    ;;
esac
