#!/bin/bash
#
# WIA-DDOS_PROTECTION CLI
# Distributed Denial of Service Protection Standard
#
# Version: 1.0.0
# 弘益人間 (홍익인간) - Benefit All Humanity
#

set -e

# Configuration
WIA_API_URL="${WIA_API_URL:-https://api.wia.live/ddos-protection/v1}"
WIA_API_KEY="${WIA_API_KEY:-}"
CONFIG_FILE="${HOME}/.wia/ddos-protection.conf"
CACHE_DIR="${HOME}/.wia/ddos-cache"
LOG_FILE="${HOME}/.wia/ddos-protection.log"

# Colors - Red/Orange Theme
RED='\033[38;5;203m'
DARK_RED='\033[38;5;196m'
ORANGE='\033[38;5;208m'
BRIGHT_RED='\033[1;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
NC='\033[0m' # No Color

# DDoS Attack Types
ATTACK_TYPE_VOLUMETRIC="volumetric"
ATTACK_TYPE_PROTOCOL="protocol"
ATTACK_TYPE_APPLICATION="application"

# Mitigation Strategies
STRATEGY_RATE_LIMIT="rate_limit"
STRATEGY_SCRUBBING="scrubbing"
STRATEGY_CDN="cdn"
STRATEGY_WAF="waf"
STRATEGY_BLACKHOLE="blackhole"

# ============================================================================
# Helper Functions
# ============================================================================

print_banner() {
    echo -e "${RED}"
    cat << "EOF"
╔═══════════════════════════════════════════════════════════════════════════╗
║                      WIA-DDOS_PROTECTION CLI                              ║
║              Distributed Denial of Service Protection                     ║
║                                                                           ║
║                    弘益人間 - Benefit All Humanity                         ║
╚═══════════════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

print_help() {
    print_banner
    cat << EOF
${ORANGE}Usage:${NC} wia-ddos-protection <command> [options]

${ORANGE}Commands:${NC}
  ${BRIGHT_RED}init${NC}                        Initialize DDoS protection system
  ${BRIGHT_RED}monitor${NC} <target>            Monitor target for DDoS attacks
  ${BRIGHT_RED}analyze${NC} <traffic_id>        Analyze traffic patterns
  ${BRIGHT_RED}mitigate${NC} <attack_id>        Apply mitigation strategies
  ${BRIGHT_RED}whitelist${NC} <operation>       Manage IP whitelist
  ${BRIGHT_RED}blacklist${NC} <operation>       Manage IP blacklist
  ${BRIGHT_RED}rate-limit${NC} <operation>      Configure rate limiting
  ${BRIGHT_RED}report${NC} <type>               Generate protection reports
  ${BRIGHT_RED}status${NC}                      Show protection status
  ${BRIGHT_RED}config${NC}                      Configure API credentials
  ${BRIGHT_RED}validate${NC} <file>             Validate configuration file
  ${BRIGHT_RED}threshold${NC} <operation>       Manage detection thresholds
  ${BRIGHT_RED}scrubbing${NC} <operation>       Manage scrubbing centers
  ${BRIGHT_RED}cdn${NC} <operation>             Manage CDN protection
  ${BRIGHT_RED}waf${NC} <operation>             Manage WAF rules
  ${BRIGHT_RED}alerts${NC}                      Show active alerts
  ${BRIGHT_RED}stats${NC}                       Show statistics
  ${BRIGHT_RED}history${NC}                     Show attack history
  ${BRIGHT_RED}simulate${NC} <type>             Simulate DDoS attack (testing)
  ${BRIGHT_RED}help${NC}                        Show this help message
  ${BRIGHT_RED}version${NC}                     Show version information

${ORANGE}Options:${NC}
  --target <url>          Target URL or IP address
  --type <type>           Attack type (volumetric|protocol|application)
  --strategy <strategy>   Mitigation strategy (rate_limit|scrubbing|cdn|waf|blackhole)
  --severity <level>      Severity level (low|medium|high|critical)
  --duration <seconds>    Duration for monitoring or mitigation
  --threshold <value>     Traffic threshold (requests/second)
  --ip <address>          IP address or CIDR block
  --port <number>         Port number
  --protocol <proto>      Protocol (tcp|udp|icmp|http|https)
  --format <fmt>          Output format (json|table|summary|detailed)
  --output <file>         Write output to file
  --auto                  Enable automatic mitigation
  --verbose               Verbose output
  --dry-run               Simulate without applying changes
  --force                 Force operation without confirmation

${ORANGE}Environment Variables:${NC}
  WIA_API_URL             API base URL (default: https://api.wia.live/ddos-protection/v1)
  WIA_API_KEY             API authentication key
  WIA_DDOS_AUTO_MITIGATE  Enable auto-mitigation (true|false)

${ORANGE}Examples:${NC}
  ${GRAY}# Initialize protection system${NC}
  wia-ddos-protection init --target https://myapp.com

  ${GRAY}# Monitor for attacks${NC}
  wia-ddos-protection monitor https://myapp.com --duration 3600

  ${GRAY}# Analyze traffic${NC}
  wia-ddos-protection analyze traffic-123 --format detailed

  ${GRAY}# Apply mitigation${NC}
  wia-ddos-protection mitigate attack-456 --strategy rate_limit --auto

  ${GRAY}# Add IP to whitelist${NC}
  wia-ddos-protection whitelist add --ip 192.168.1.100

  ${GRAY}# Configure rate limiting${NC}
  wia-ddos-protection rate-limit set --threshold 1000 --port 80

  ${GRAY}# Generate attack report${NC}
  wia-ddos-protection report attacks --output report.json

  ${GRAY}# Show current status${NC}
  wia-ddos-protection status --verbose

${ORANGE}Attack Types:${NC}
  ${RED}Volumetric${NC}    - UDP floods, ICMP floods, DNS amplification, NTP amplification
  ${RED}Protocol${NC}      - SYN floods, fragmented packet attacks, Ping of Death
  ${RED}Application${NC}   - HTTP floods, Slowloris, RUDY, low-and-slow attacks

${ORANGE}Mitigation Strategies:${NC}
  ${ORANGE}Rate Limiting${NC}  - Limit requests per IP/second
  ${ORANGE}Scrubbing${NC}      - Filter malicious traffic through scrubbing centers
  ${ORANGE}CDN${NC}            - Distribute traffic across global CDN
  ${ORANGE}WAF${NC}            - Web Application Firewall rules
  ${ORANGE}Blackhole${NC}      - Drop all traffic from malicious sources

${ORANGE}For detailed command help:${NC}
  wia-ddos-protection <command> --help

EOF
}

print_version() {
    print_banner
    echo -e "${ORANGE}Version:${NC} 1.0.0"
    echo -e "${ORANGE}Build:${NC} 2025.01.08"
    echo -e "${ORANGE}API Version:${NC} v1"
    echo -e "${ORANGE}Protocol:${NC} WIA-DDOS_PROTECTION-v1.0"
    echo ""
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [INFO] $1" >> "$LOG_FILE" 2>/dev/null || true
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [SUCCESS] $1" >> "$LOG_FILE" 2>/dev/null || true
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [WARNING] $1" >> "$LOG_FILE" 2>/dev/null || true
}

log_error() {
    echo -e "${BRIGHT_RED}[ERROR]${NC} $1" >&2
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ERROR] $1" >> "$LOG_FILE" 2>/dev/null || true
}

log_attack() {
    echo -e "${DARK_RED}[ATTACK DETECTED]${NC} $1"
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [ATTACK] $1" >> "$LOG_FILE" 2>/dev/null || true
}

check_dependencies() {
    local deps=("curl" "jq")
    for dep in "${deps[@]}"; do
        if ! command -v "$dep" &> /dev/null; then
            log_error "Required dependency not found: $dep"
            log_info "Please install: $dep"
            exit 1
        fi
    done
}

check_api_key() {
    if [ -z "$WIA_API_KEY" ]; then
        if [ -f "$CONFIG_FILE" ]; then
            source "$CONFIG_FILE"
        fi
    fi

    if [ -z "$WIA_API_KEY" ]; then
        log_error "API key not configured. Run 'wia-ddos-protection config' first."
        exit 1
    fi
}

api_request() {
    local method="$1"
    local endpoint="$2"
    local data="$3"

    local curl_opts=(-s -X "$method")
    curl_opts+=(-H "Authorization: Bearer $WIA_API_KEY")
    curl_opts+=(-H "Content-Type: application/json")
    curl_opts+=(-H "Accept: application/json")
    curl_opts+=(-H "X-WIA-Client: cli/1.0.0")

    if [ -n "$data" ]; then
        curl_opts+=(-d "$data")
    fi

    local response
    response=$(curl "${curl_opts[@]}" "${WIA_API_URL}${endpoint}" 2>/dev/null)

    if [ $? -ne 0 ]; then
        log_error "API request failed"
        exit 1
    fi

    echo "$response"
}

format_output() {
    local data="$1"
    local format="${2:-json}"

    case "$format" in
        json)
            echo "$data" | jq .
            ;;
        table)
            echo "$data" | jq -r 'to_entries | .[] | "\(.key): \(.value)"'
            ;;
        summary)
            format_summary "$data"
            ;;
        detailed)
            format_detailed "$data"
            ;;
        *)
            echo "$data" | jq .
            ;;
    esac
}

format_summary() {
    local data="$1"

    echo ""
    echo -e "${RED}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${RED}                    DDOS PROTECTION SUMMARY                ${NC}"
    echo -e "${RED}═══════════════════════════════════════════════════════════${NC}"
    echo ""

    local status=$(echo "$data" | jq -r '.status // "unknown"')
    local threats=$(echo "$data" | jq -r '.active_threats // 0')
    local mitigations=$(echo "$data" | jq -r '.active_mitigations // 0')
    local traffic=$(echo "$data" | jq -r '.traffic_rate // "N/A"')

    echo -e "  ${ORANGE}Protection Status:${NC} $status"
    echo -e "  ${ORANGE}Active Threats:${NC} $threats"
    echo -e "  ${ORANGE}Active Mitigations:${NC} $mitigations"
    echo -e "  ${ORANGE}Traffic Rate:${NC} $traffic req/s"
    echo ""
    echo -e "${RED}═══════════════════════════════════════════════════════════${NC}"
}

format_detailed() {
    local data="$1"

    echo ""
    echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║              DDOS PROTECTION DETAILED REPORT              ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Traffic Analysis
    echo -e "${ORANGE}Traffic Analysis:${NC}"
    echo "  Total Requests: $(echo "$data" | jq -r '.traffic.total_requests // 0')"
    echo "  Suspicious Requests: $(echo "$data" | jq -r '.traffic.suspicious_requests // 0')"
    echo "  Blocked Requests: $(echo "$data" | jq -r '.traffic.blocked_requests // 0')"
    echo "  Average Rate: $(echo "$data" | jq -r '.traffic.avg_rate // 0') req/s"
    echo "  Peak Rate: $(echo "$data" | jq -r '.traffic.peak_rate // 0') req/s"
    echo ""

    # Attack Detection
    echo -e "${ORANGE}Attack Detection:${NC}"
    echo "  Volumetric Attacks: $(echo "$data" | jq -r '.attacks.volumetric // 0')"
    echo "  Protocol Attacks: $(echo "$data" | jq -r '.attacks.protocol // 0')"
    echo "  Application Attacks: $(echo "$data" | jq -r '.attacks.application // 0')"
    echo ""

    # Mitigation Status
    echo -e "${ORANGE}Mitigation Status:${NC}"
    echo "  Rate Limiting: $(echo "$data" | jq -r '.mitigation.rate_limit // "disabled"')"
    echo "  Scrubbing Centers: $(echo "$data" | jq -r '.mitigation.scrubbing // "disabled"')"
    echo "  CDN Protection: $(echo "$data" | jq -r '.mitigation.cdn // "disabled"')"
    echo "  WAF Rules: $(echo "$data" | jq -r '.mitigation.waf // "disabled"')"
    echo ""
}

confirm_action() {
    local message="$1"
    if [ "$FORCE" = true ]; then
        return 0
    fi

    echo -e "${YELLOW}$message${NC}"
    read -p "Continue? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Operation cancelled"
        exit 0
    fi
}

# ============================================================================
# Command: init
# ============================================================================

cmd_init() {
    print_banner
    log_info "Initializing DDoS Protection System"

    local target="$TARGET_URL"
    if [ -z "$target" ]; then
        log_error "Target URL required. Use --target <url>"
        exit 1
    fi

    # Create directories
    mkdir -p "$CACHE_DIR"
    mkdir -p "$(dirname "$LOG_FILE")"

    log_info "Creating directories..."
    log_success "Cache directory: $CACHE_DIR"
    log_success "Log file: $LOG_FILE"

    # Initialize configuration
    if [ ! -f "$CONFIG_FILE" ]; then
        log_warning "Configuration not found. Please run 'wia-ddos-protection config'"
    fi

    check_api_key

    # Register target with API
    log_info "Registering target: $target"

    local payload=$(cat <<EOF
{
  "target": "$target",
  "auto_mitigation": ${AUTO_MITIGATION:-false},
  "protection_level": "${SEVERITY:-medium}",
  "protocols": ["http", "https", "tcp", "udp"]
}
EOF
)

    local response=$(api_request "POST" "/targets/register" "$payload")
    local target_id=$(echo "$response" | jq -r '.target_id // "unknown"')

    log_success "Target registered successfully"
    log_info "Target ID: $target_id"

    # Initialize default thresholds
    log_info "Setting default thresholds..."

    local thresholds=$(cat <<EOF
{
  "target_id": "$target_id",
  "volumetric_threshold": 10000,
  "request_rate_threshold": 1000,
  "concurrent_connections": 5000,
  "new_connections_per_second": 100
}
EOF
)

    api_request "POST" "/thresholds" "$thresholds" > /dev/null
    log_success "Default thresholds configured"

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║           DDOS PROTECTION INITIALIZED                     ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  ${ORANGE}Target:${NC} $target"
    echo -e "  ${ORANGE}Target ID:${NC} $target_id"
    echo -e "  ${ORANGE}Protection Level:${NC} ${SEVERITY:-medium}"
    echo -e "  ${ORANGE}Auto Mitigation:${NC} ${AUTO_MITIGATION:-false}"
    echo ""
    echo -e "${GRAY}Next steps:${NC}"
    echo "  1. Start monitoring: wia-ddos-protection monitor $target"
    echo "  2. Configure rate limits: wia-ddos-protection rate-limit set --threshold 1000"
    echo "  3. View status: wia-ddos-protection status"
    echo ""
}

# ============================================================================
# Command: monitor
# ============================================================================

cmd_monitor() {
    local target="$1"

    if [ -z "$target" ]; then
        target="$TARGET_URL"
    fi

    if [ -z "$target" ]; then
        log_error "Target required. Provide as argument or use --target <url>"
        exit 1
    fi

    check_api_key

    local duration="${DURATION:-3600}"

    log_info "Starting DDoS monitoring for: $target"
    log_info "Duration: $duration seconds ($(($duration / 60)) minutes)"

    local payload=$(cat <<EOF
{
  "target": "$target",
  "duration": $duration,
  "detection_modes": ["volumetric", "protocol", "application"],
  "real_time": true,
  "alert_on_anomaly": true
}
EOF
)

    local response=$(api_request "POST" "/monitor/start" "$payload")
    local monitor_id=$(echo "$response" | jq -r '.monitor_id // "unknown"')

    log_success "Monitoring session started"
    log_info "Monitor ID: $monitor_id"

    echo ""
    echo -e "${ORANGE}Monitoring in progress...${NC}"
    echo ""

    # Simulate real-time monitoring display
    local elapsed=0
    local check_interval=5

    while [ $elapsed -lt $duration ]; do
        sleep $check_interval
        elapsed=$((elapsed + check_interval))

        # Get current stats
        local stats=$(api_request "GET" "/monitor/$monitor_id/stats")

        local current_rate=$(echo "$stats" | jq -r '.current_rate // 0')
        local suspicious=$(echo "$stats" | jq -r '.suspicious_count // 0')
        local attacks=$(echo "$stats" | jq -r '.attacks_detected // 0')

        # Clear line and print status
        echo -ne "\r${GRAY}[$(date +'%H:%M:%S')]${NC} Rate: ${current_rate} req/s | Suspicious: ${suspicious} | Attacks: "

        if [ "$attacks" -gt 0 ]; then
            echo -ne "${BRIGHT_RED}${attacks}${NC}  "
        else
            echo -ne "${GREEN}${attacks}${NC}  "
        fi

        # Check for attacks
        if [ "$attacks" -gt 0 ]; then
            echo ""
            log_attack "DDoS attack detected! Attack count: $attacks"

            if [ "$AUTO_MITIGATION" = true ]; then
                log_info "Auto-mitigation enabled, applying countermeasures..."
                cmd_mitigate "$monitor_id"
            fi
        fi

        if [ "$VERBOSE" = true ] && [ $((elapsed % 30)) -eq 0 ]; then
            echo ""
            format_summary "$stats"
        fi
    done

    echo ""
    echo ""
    log_success "Monitoring session completed"

    local final_report=$(api_request "GET" "/monitor/$monitor_id/report")
    format_detailed "$final_report"
}

# ============================================================================
# Command: analyze
# ============================================================================

cmd_analyze() {
    local traffic_id="$1"

    if [ -z "$traffic_id" ]; then
        log_error "Traffic ID required"
        exit 1
    fi

    check_api_key

    log_info "Analyzing traffic: $traffic_id"

    local response=$(api_request "GET" "/analysis/$traffic_id?deep=true")

    echo ""
    echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║                   TRAFFIC ANALYSIS REPORT                 ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Attack Classification
    local attack_type=$(echo "$response" | jq -r '.classification.attack_type // "none"')
    local confidence=$(echo "$response" | jq -r '.classification.confidence // 0')
    local severity=$(echo "$response" | jq -r '.classification.severity // "unknown"')

    echo -e "${ORANGE}Attack Classification:${NC}"
    echo "  Type: $attack_type"
    echo "  Confidence: ${confidence}%"
    echo "  Severity: $severity"
    echo ""

    # Source Analysis
    echo -e "${ORANGE}Source Analysis:${NC}"
    echo "  Unique IPs: $(echo "$response" | jq -r '.sources.unique_ips // 0')"
    echo "  Geographic Distribution: $(echo "$response" | jq -r '.sources.countries // 0') countries"
    echo "  Botnet Indicators: $(echo "$response" | jq -r '.sources.botnet_score // 0')/100"
    echo ""

    # Traffic Patterns
    echo -e "${ORANGE}Traffic Patterns:${NC}"
    echo "  Request Rate: $(echo "$response" | jq -r '.patterns.request_rate // 0') req/s"
    echo "  Packet Size Avg: $(echo "$response" | jq -r '.patterns.avg_packet_size // 0') bytes"
    echo "  Connection Duration: $(echo "$response" | jq -r '.patterns.avg_duration // 0')s"
    echo "  Abnormal Patterns: $(echo "$response" | jq -r '.patterns.anomalies // 0')"
    echo ""

    # Protocol Analysis
    echo -e "${ORANGE}Protocol Distribution:${NC}"
    echo "  HTTP/HTTPS: $(echo "$response" | jq -r '.protocols.http_percentage // 0')%"
    echo "  TCP: $(echo "$response" | jq -r '.protocols.tcp_percentage // 0')%"
    echo "  UDP: $(echo "$response" | jq -r '.protocols.udp_percentage // 0')%"
    echo "  ICMP: $(echo "$response" | jq -r '.protocols.icmp_percentage // 0')%"
    echo ""

    # Recommendations
    echo -e "${ORANGE}Mitigation Recommendations:${NC}"
    local recommendations=$(echo "$response" | jq -r '.recommendations[]' 2>/dev/null)
    if [ -n "$recommendations" ]; then
        echo "$recommendations" | while read -r rec; do
            echo "  • $rec"
        done
    else
        echo "  • No specific recommendations"
    fi
    echo ""

    if [ "$OUTPUT_FILE" ]; then
        echo "$response" | jq . > "$OUTPUT_FILE"
        log_success "Full analysis saved to: $OUTPUT_FILE"
    fi
}

# ============================================================================
# Command: mitigate
# ============================================================================

cmd_mitigate() {
    local attack_id="$1"

    if [ -z "$attack_id" ]; then
        log_error "Attack ID required"
        exit 1
    fi

    check_api_key

    local strategy="${STRATEGY:-rate_limit}"

    log_info "Applying mitigation for attack: $attack_id"
    log_info "Strategy: $strategy"

    if [ "$DRY_RUN" = true ]; then
        log_warning "DRY RUN MODE - No changes will be applied"
    fi

    confirm_action "This will apply $strategy mitigation strategy."

    local payload=$(cat <<EOF
{
  "attack_id": "$attack_id",
  "strategy": "$strategy",
  "auto_scale": true,
  "dry_run": ${DRY_RUN:-false}
}
EOF
)

    local response=$(api_request "POST" "/mitigation/apply" "$payload")
    local mitigation_id=$(echo "$response" | jq -r '.mitigation_id // "unknown"')
    local status=$(echo "$response" | jq -r '.status // "unknown"')

    if [ "$status" = "active" ]; then
        log_success "Mitigation applied successfully"
    else
        log_warning "Mitigation status: $status"
    fi

    echo ""
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                MITIGATION ACTIVATED                       ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  ${ORANGE}Mitigation ID:${NC} $mitigation_id"
    echo -e "  ${ORANGE}Strategy:${NC} $strategy"
    echo -e "  ${ORANGE}Status:${NC} $status"
    echo ""

    # Show mitigation details
    case "$strategy" in
        rate_limit)
            echo -e "${ORANGE}Rate Limiting Configuration:${NC}"
            echo "  Threshold: $(echo "$response" | jq -r '.config.threshold // "N/A"') req/s"
            echo "  Window: $(echo "$response" | jq -r '.config.window // "N/A"')s"
            echo "  Action: $(echo "$response" | jq -r '.config.action // "N/A"')"
            ;;
        scrubbing)
            echo -e "${ORANGE}Scrubbing Center Configuration:${NC}"
            echo "  Location: $(echo "$response" | jq -r '.config.location // "N/A"')"
            echo "  Capacity: $(echo "$response" | jq -r '.config.capacity // "N/A"') Gbps"
            echo "  Latency: $(echo "$response" | jq -r '.config.latency // "N/A"')ms"
            ;;
        cdn)
            echo -e "${ORANGE}CDN Protection Configuration:${NC}"
            echo "  Provider: $(echo "$response" | jq -r '.config.provider // "N/A"')"
            echo "  Edge Locations: $(echo "$response" | jq -r '.config.edge_locations // "N/A"')"
            echo "  Cache Mode: $(echo "$response" | jq -r '.config.cache_mode // "N/A"')"
            ;;
        waf)
            echo -e "${ORANGE}WAF Configuration:${NC}"
            echo "  Rules Applied: $(echo "$response" | jq -r '.config.rules_count // "N/A"')"
            echo "  Mode: $(echo "$response" | jq -r '.config.mode // "N/A"')"
            echo "  Custom Rules: $(echo "$response" | jq -r '.config.custom_rules // "N/A"')"
            ;;
    esac
    echo ""
}

# ============================================================================
# Command: whitelist
# ============================================================================

cmd_whitelist() {
    local operation="${1:-list}"

    check_api_key

    case "$operation" in
        add)
            local ip="$IP_ADDRESS"
            if [ -z "$ip" ]; then
                log_error "IP address required. Use --ip <address>"
                exit 1
            fi

            log_info "Adding IP to whitelist: $ip"

            local payload=$(cat <<EOF
{
  "ip": "$ip",
  "comment": "Added via CLI",
  "expiry": null
}
EOF
)

            local response=$(api_request "POST" "/whitelist" "$payload")
            log_success "IP added to whitelist: $ip"
            ;;

        remove)
            local ip="$IP_ADDRESS"
            if [ -z "$ip" ]; then
                log_error "IP address required. Use --ip <address>"
                exit 1
            fi

            confirm_action "Remove $ip from whitelist?"

            api_request "DELETE" "/whitelist/$ip" ""
            log_success "IP removed from whitelist: $ip"
            ;;

        list)
            log_info "Fetching whitelist..."

            local response=$(api_request "GET" "/whitelist")

            echo ""
            echo -e "${ORANGE}IP Whitelist:${NC}"
            echo "$response" | jq -r '.entries[] | "  \(.ip) - \(.comment // "No comment")"'
            echo ""
            ;;

        clear)
            confirm_action "Clear entire whitelist?"

            api_request "DELETE" "/whitelist" ""
            log_success "Whitelist cleared"
            ;;

        *)
            log_error "Unknown whitelist operation: $operation"
            echo "Valid operations: add, remove, list, clear"
            exit 1
            ;;
    esac
}

# ============================================================================
# Command: blacklist
# ============================================================================

cmd_blacklist() {
    local operation="${1:-list}"

    check_api_key

    case "$operation" in
        add)
            local ip="$IP_ADDRESS"
            if [ -z "$ip" ]; then
                log_error "IP address required. Use --ip <address>"
                exit 1
            fi

            log_info "Adding IP to blacklist: $ip"

            local payload=$(cat <<EOF
{
  "ip": "$ip",
  "reason": "Manual block via CLI",
  "duration": ${DURATION:-0}
}
EOF
)

            local response=$(api_request "POST" "/blacklist" "$payload")
            log_success "IP added to blacklist: $ip"
            ;;

        remove)
            local ip="$IP_ADDRESS"
            if [ -z "$ip" ]; then
                log_error "IP address required. Use --ip <address>"
                exit 1
            fi

            confirm_action "Remove $ip from blacklist?"

            api_request "DELETE" "/blacklist/$ip" ""
            log_success "IP removed from blacklist: $ip"
            ;;

        list)
            log_info "Fetching blacklist..."

            local response=$(api_request "GET" "/blacklist")

            echo ""
            echo -e "${RED}IP Blacklist:${NC}"
            echo "$response" | jq -r '.entries[] | "  \(.ip) - \(.reason // "No reason") [\(.added_at)]"'
            echo ""
            ;;

        clear)
            confirm_action "Clear entire blacklist?"

            api_request "DELETE" "/blacklist" ""
            log_success "Blacklist cleared"
            ;;

        auto-populate)
            log_info "Auto-populating blacklist from threat intelligence..."

            local response=$(api_request "POST" "/blacklist/auto-populate" "{}")
            local count=$(echo "$response" | jq -r '.added_count // 0')

            log_success "Added $count IPs to blacklist from threat feeds"
            ;;

        *)
            log_error "Unknown blacklist operation: $operation"
            echo "Valid operations: add, remove, list, clear, auto-populate"
            exit 1
            ;;
    esac
}

# ============================================================================
# Command: rate-limit
# ============================================================================

cmd_rate_limit() {
    local operation="${1:-show}"

    check_api_key

    case "$operation" in
        set)
            local threshold="$THRESHOLD"
            if [ -z "$threshold" ]; then
                log_error "Threshold required. Use --threshold <value>"
                exit 1
            fi

            log_info "Setting rate limit: $threshold req/s"

            local payload=$(cat <<EOF
{
  "threshold": $threshold,
  "window": 60,
  "port": ${PORT:-80},
  "protocol": "${PROTOCOL:-http}",
  "action": "reject"
}
EOF
)

            local response=$(api_request "POST" "/rate-limit/rules" "$payload")
            log_success "Rate limit configured"
            ;;

        show)
            log_info "Fetching rate limit configuration..."

            local response=$(api_request "GET" "/rate-limit/rules")

            echo ""
            echo -e "${ORANGE}╔═══════════════════════════════════════════════════════════╗${NC}"
            echo -e "${ORANGE}║                 RATE LIMIT CONFIGURATION                  ║${NC}"
            echo -e "${ORANGE}╚═══════════════════════════════════════════════════════════╝${NC}"
            echo ""

            format_output "$response" "$OUTPUT_FORMAT"
            ;;

        disable)
            confirm_action "Disable rate limiting?"

            api_request "DELETE" "/rate-limit/rules" ""
            log_success "Rate limiting disabled"
            ;;

        test)
            local ip="${IP_ADDRESS:-127.0.0.1}"
            log_info "Testing rate limit for IP: $ip"

            local response=$(api_request "GET" "/rate-limit/test?ip=$ip")

            local allowed=$(echo "$response" | jq -r '.allowed // false')
            local current=$(echo "$response" | jq -r '.current_rate // 0')
            local limit=$(echo "$response" | jq -r '.limit // 0')

            echo ""
            echo "  IP: $ip"
            echo "  Current Rate: $current req/s"
            echo "  Limit: $limit req/s"
            echo "  Status: $([ "$allowed" = "true" ] && echo -e "${GREEN}Allowed${NC}" || echo -e "${RED}Blocked${NC}")"
            echo ""
            ;;

        *)
            log_error "Unknown rate-limit operation: $operation"
            echo "Valid operations: set, show, disable, test"
            exit 1
            ;;
    esac
}

# ============================================================================
# Command: report
# ============================================================================

cmd_report() {
    local report_type="${1:-summary}"

    check_api_key

    log_info "Generating $report_type report..."

    local response=$(api_request "GET" "/reports/$report_type")

    echo ""
    echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║                    DDOS PROTECTION REPORT                 ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    case "$report_type" in
        attacks)
            echo -e "${ORANGE}Attack Summary (Last 30 Days):${NC}"
            echo ""
            echo "  Total Attacks: $(echo "$response" | jq -r '.total_attacks // 0')"
            echo "  Volumetric: $(echo "$response" | jq -r '.by_type.volumetric // 0')"
            echo "  Protocol: $(echo "$response" | jq -r '.by_type.protocol // 0')"
            echo "  Application: $(echo "$response" | jq -r '.by_type.application // 0')"
            echo ""
            echo "  Successfully Mitigated: $(echo "$response" | jq -r '.mitigated // 0')"
            echo "  Average Duration: $(echo "$response" | jq -r '.avg_duration // 0')s"
            echo "  Peak Traffic: $(echo "$response" | jq -r '.peak_traffic // 0') req/s"
            ;;

        mitigation)
            echo -e "${ORANGE}Mitigation Effectiveness:${NC}"
            echo ""
            echo "  Total Mitigations: $(echo "$response" | jq -r '.total_mitigations // 0')"
            echo "  Success Rate: $(echo "$response" | jq -r '.success_rate // 0')%"
            echo "  Average Response Time: $(echo "$response" | jq -r '.avg_response_time // 0')s"
            echo "  False Positives: $(echo "$response" | jq -r '.false_positives // 0')"
            ;;

        traffic)
            echo -e "${ORANGE}Traffic Analysis:${NC}"
            echo ""
            echo "  Total Requests: $(echo "$response" | jq -r '.total_requests // 0')"
            echo "  Legitimate Traffic: $(echo "$response" | jq -r '.legitimate_percentage // 0')%"
            echo "  Blocked Traffic: $(echo "$response" | jq -r '.blocked_percentage // 0')%"
            echo "  Average Bandwidth: $(echo "$response" | jq -r '.avg_bandwidth // 0') Mbps"
            ;;

        summary)
            format_detailed "$response"
            ;;
    esac

    echo ""

    if [ "$OUTPUT_FILE" ]; then
        echo "$response" | jq . > "$OUTPUT_FILE"
        log_success "Report saved to: $OUTPUT_FILE"
    fi
}

# ============================================================================
# Command: status
# ============================================================================

cmd_status() {
    check_api_key

    log_info "Fetching protection status..."

    local response=$(api_request "GET" "/status")

    echo ""
    echo -e "${RED}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║               DDOS PROTECTION STATUS                      ║${NC}"
    echo -e "${RED}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    local protection_status=$(echo "$response" | jq -r '.protection_status // "unknown"')
    local health=$(echo "$response" | jq -r '.health // "unknown"')

    # Status indicator
    case "$protection_status" in
        active)
            echo -e "  Status: ${GREEN}●${NC} Active"
            ;;
        warning)
            echo -e "  Status: ${YELLOW}●${NC} Warning"
            ;;
        critical)
            echo -e "  Status: ${BRIGHT_RED}●${NC} Critical"
            ;;
        *)
            echo -e "  Status: ${GRAY}●${NC} Unknown"
            ;;
    esac

    echo ""
    echo -e "${ORANGE}System Health:${NC}"
    echo "  Overall: $health"
    echo "  Uptime: $(echo "$response" | jq -r '.uptime // "N/A"')"
    echo "  Last Check: $(echo "$response" | jq -r '.last_check // "N/A"')"
    echo ""

    echo -e "${ORANGE}Active Protection:${NC}"
    echo "  Monitored Targets: $(echo "$response" | jq -r '.monitored_targets // 0')"
    echo "  Active Mitigations: $(echo "$response" | jq -r '.active_mitigations // 0')"
    echo "  Rate Limiting: $(echo "$response" | jq -r '.rate_limiting_enabled // false')"
    echo "  WAF Status: $(echo "$response" | jq -r '.waf_status // "disabled"')"
    echo ""

    echo -e "${ORANGE}Current Traffic:${NC}"
    echo "  Incoming Rate: $(echo "$response" | jq -r '.traffic.incoming_rate // 0') req/s"
    echo "  Blocked Rate: $(echo "$response" | jq -r '.traffic.blocked_rate // 0') req/s"
    echo "  Bandwidth Usage: $(echo "$response" | jq -r '.traffic.bandwidth // 0') Mbps"
    echo ""

    if [ "$VERBOSE" = true ]; then
        echo -e "${ORANGE}Detailed Metrics:${NC}"
        echo "$response" | jq -r '.metrics'
        echo ""
    fi
}

# ============================================================================
# Command: config
# ============================================================================

cmd_config() {
    print_banner

    mkdir -p "$(dirname "$CONFIG_FILE")"

    echo "WIA-DDOS_PROTECTION Configuration"
    echo ""
    read -p "API Key: " api_key
    read -p "API URL [${WIA_API_URL}]: " api_url

    if [ -z "$api_url" ]; then
        api_url="$WIA_API_URL"
    fi

    cat > "$CONFIG_FILE" << EOF
WIA_API_KEY="$api_key"
WIA_API_URL="$api_url"
EOF

    chmod 600 "$CONFIG_FILE"
    log_success "Configuration saved to $CONFIG_FILE"
}

# ============================================================================
# Command: validate
# ============================================================================

cmd_validate() {
    local file="$1"

    if [ -z "$file" ]; then
        log_error "File path required"
        exit 1
    fi

    if [ ! -f "$file" ]; then
        log_error "File not found: $file"
        exit 1
    fi

    log_info "Validating configuration file: $file"

    # Basic JSON validation
    if ! jq . "$file" > /dev/null 2>&1; then
        log_error "Invalid JSON format"
        exit 1
    fi

    # Check required fields
    local required_fields=("target" "thresholds")
    for field in "${required_fields[@]}"; do
        if ! jq -e ".$field" "$file" > /dev/null 2>&1; then
            log_error "Missing required field: $field"
            exit 1
        fi
    done

    log_success "Configuration file is valid"
}

# ============================================================================
# Additional Commands
# ============================================================================

cmd_stats() {
    check_api_key
    local response=$(api_request "GET" "/statistics")
    format_output "$response" "${OUTPUT_FORMAT:-summary}"
}

cmd_alerts() {
    check_api_key
    local response=$(api_request "GET" "/alerts/active")

    echo ""
    echo -e "${BRIGHT_RED}Active Alerts:${NC}"
    echo ""
    echo "$response" | jq -r '.alerts[] | "  [\(.severity)] \(.message) - \(.timestamp)"'
    echo ""
}

cmd_history() {
    check_api_key
    local response=$(api_request "GET" "/attacks/history?limit=20")

    echo ""
    echo -e "${ORANGE}Recent Attack History:${NC}"
    echo ""
    echo "$response" | jq -r '.attacks[] | "  \(.timestamp) | \(.type) | \(.severity) | \(.status)"'
    echo ""
}

# ============================================================================
# Main
# ============================================================================

# Check dependencies
check_dependencies

# Parse global options
OUTPUT_FORMAT="json"
OUTPUT_FILE=""
VERBOSE=false
DRY_RUN=false
FORCE=false
TARGET_URL=""
ATTACK_TYPE=""
STRATEGY=""
SEVERITY=""
DURATION=""
THRESHOLD=""
IP_ADDRESS=""
PORT=""
PROTOCOL=""
AUTO_MITIGATION=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --target)
            TARGET_URL="$2"
            shift 2
            ;;
        --type)
            ATTACK_TYPE="$2"
            shift 2
            ;;
        --strategy)
            STRATEGY="$2"
            shift 2
            ;;
        --severity)
            SEVERITY="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --threshold)
            THRESHOLD="$2"
            shift 2
            ;;
        --ip)
            IP_ADDRESS="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --protocol)
            PROTOCOL="$2"
            shift 2
            ;;
        --format)
            OUTPUT_FORMAT="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --auto)
            AUTO_MITIGATION=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --force)
            FORCE=true
            shift
            ;;
        *)
            break
            ;;
    esac
done

# Get command
COMMAND="${1:-help}"
shift || true

# Execute command
case "$COMMAND" in
    init)
        cmd_init "$@"
        ;;
    monitor)
        cmd_monitor "$@"
        ;;
    analyze)
        cmd_analyze "$@"
        ;;
    mitigate)
        cmd_mitigate "$@"
        ;;
    whitelist)
        cmd_whitelist "$@"
        ;;
    blacklist)
        cmd_blacklist "$@"
        ;;
    rate-limit)
        cmd_rate_limit "$@"
        ;;
    report)
        cmd_report "$@"
        ;;
    status)
        cmd_status "$@"
        ;;
    config)
        cmd_config
        ;;
    validate)
        cmd_validate "$@"
        ;;
    stats)
        cmd_stats
        ;;
    alerts)
        cmd_alerts
        ;;
    history)
        cmd_history
        ;;
    version)
        print_version
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        log_error "Unknown command: $COMMAND"
        echo "Run 'wia-ddos-protection help' for usage"
        exit 1
        ;;
esac
