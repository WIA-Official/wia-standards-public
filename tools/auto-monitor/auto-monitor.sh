#!/bin/bash
#
# AUTO-MONITOR v1.0
# One-Click Server Monitoring Solution
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: auto-monitor [OPTIONS]
#

set -e

# ============================================================
# Version & Constants
# ============================================================
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/auto-monitor.conf"
DATA_DIR="${SCRIPT_DIR}/data"
LOG_FILE="${SCRIPT_DIR}/logs/auto-monitor.log"

# ============================================================
# Colors & Formatting
# ============================================================
if [[ -t 1 ]]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    MAGENTA='\033[0;35m'
    BOLD='\033[1m'
    DIM='\033[2m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' CYAN='' MAGENTA='' BOLD='' DIM='' NC=''
fi

# ============================================================
# Default Configuration
# ============================================================
CHECK_INTERVAL=300
HOSTNAME=$(hostname -f 2>/dev/null || hostname)

# Thresholds
CPU_WARNING=70
CPU_CRITICAL=90
MEMORY_WARNING=80
MEMORY_CRITICAL=95
SWAP_WARNING=50
SWAP_CRITICAL=80
DISK_WARNING=80
DISK_CRITICAL=90
LOAD_WARNING=4.0
LOAD_CRITICAL=8.0

# Services
SERVICES_TO_MONITOR="httpd apache2 nginx mysqld mariadb postgresql php-fpm redis-server memcached docker"
CUSTOM_PROCESSES=""
PORTS_TO_CHECK="80 443 3306"

# Web monitoring
URLS_TO_MONITOR=""

# SSL monitoring
SSL_DOMAINS=""
SSL_WARNING_DAYS=30
SSL_CRITICAL_DAYS=7

# Log monitoring
LOG_FILES_TO_MONITOR=""

# Notifications
EMAIL_ENABLED=false
EMAIL_TO=""
EMAIL_FROM="monitor@${HOSTNAME}"
SLACK_ENABLED=false
SLACK_WEBHOOK=""
SLACK_CHANNEL="#alerts"
TELEGRAM_ENABLED=false
TELEGRAM_BOT_TOKEN=""
TELEGRAM_CHAT_ID=""
DISCORD_ENABLED=false
DISCORD_WEBHOOK=""

# Alert rules
ALERT_ON_WARNING=true
ALERT_ON_CRITICAL=true
ALERT_ON_RECOVERY=true
ALERT_COOLDOWN=1800
QUIET_HOURS=""

# Report
DAILY_REPORT_ENABLED=false
DAILY_REPORT_TIME="08:00"

# Runtime
VERBOSE=false
WATCH_MODE=false
CHECK_ONLY=""

# State tracking
declare -A LAST_ALERT_TIME
declare -A PREVIOUS_STATE
ALERTS=()
WARNINGS=()
CRITICALS=()

# ============================================================
# Load Configuration
# ============================================================
load_config() {
    [[ -f "$CONFIG_FILE" ]] && source "$CONFIG_FILE"
    mkdir -p "$DATA_DIR" "$(dirname "$LOG_FILE")"
}

# ============================================================
# Logging
# ============================================================
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")

    echo "[$timestamp] [$level] $message" >> "$LOG_FILE" 2>/dev/null || true
}

log_info()  { log "INFO" "$@"; }
log_warn()  { log "WARN" "$@"; WARNINGS+=("$*"); }
log_error() { log "ERROR" "$@"; CRITICALS+=("$*"); }

# ============================================================
# Banner
# ============================================================
print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   █████╗ ██╗   ██╗████████╗ ██████╗                          ║
║  ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗                         ║
║  ███████║██║   ██║   ██║   ██║   ██║                         ║
║  ██╔══██║██║   ██║   ██║   ██║   ██║                         ║
║  ██║  ██║╚██████╔╝   ██║   ╚██████╔╝                         ║
║  ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝                          ║
║                                                              ║
║  ███╗   ███╗ ██████╗ ███╗   ██╗██╗████████╗ ██████╗ ██████╗  ║
║  ████╗ ████║██╔═══██╗████╗  ██║██║╚══██╔══╝██╔═══██╗██╔══██╗ ║
║  ██╔████╔██║██║   ██║██╔██╗ ██║██║   ██║   ██║   ██║██████╔╝ ║
║  ██║╚██╔╝██║██║   ██║██║╚██╗██║██║   ██║   ██║   ██║██╔══██╗ ║
║  ██║ ╚═╝ ██║╚██████╔╝██║ ╚████║██║   ██║   ╚██████╔╝██║  ██║ ║
║  ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝ ║
║                                                              ║
║  AUTO-MONITOR v1.0 · Server Monitoring Solution              ║
║  弘益人間 (홍익인간) · Benefit All Humanity                  ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# ============================================================
# Help
# ============================================================
print_help() {
    cat << EOF
${BOLD}AUTO-MONITOR v${VERSION}${NC} - One-Click Server Monitoring

${BOLD}USAGE:${NC}
    auto-monitor [OPTIONS]

${BOLD}MODES:${NC}
    (none)              Single check with output
    --watch             Real-time dashboard (like top)
    --daemon            Run as background daemon
    --install           Install cron job (every 5 min)
    --uninstall         Remove cron job

${BOLD}SETUP:${NC}
    --setup             Interactive setup wizard
    --setup-slack URL   Setup Slack webhook
    --setup-telegram TOKEN CHAT_ID
    --setup-discord URL
    --setup-email EMAIL

${BOLD}THRESHOLDS:${NC}
    --set KEY=VALUE     Set threshold (e.g., cpu_warning=80)
    --preset NAME       Apply preset (production/development/minimal)

${BOLD}ADD MONITORS:${NC}
    --add-url URL       Add URL to monitor
    --add-ssl DOMAIN    Add SSL certificate to monitor
    --add-service NAME  Add service to monitor

${BOLD}CHECKS:${NC}
    --check-only TYPE   Check only specific type
                        (cpu, memory, disk, services, web, ssl)
    --test-alert        Send test alert

${BOLD}REPORTS:${NC}
    --report [TYPE]     Generate report (daily/weekly/monthly)
    --history PERIOD    Show history (1h, 24h, 7d, 30d)
    --export FORMAT     Export data (csv, json)

${BOLD}MANAGEMENT:${NC}
    --status            Show daemon status
    --incidents         Show incident history
    --cleanup           Remove old data

${BOLD}OTHER:${NC}
    -v, --verbose       Verbose output
    -h, --help          Show this help
    --version           Show version

${BOLD}EXAMPLES:${NC}
    auto-monitor                     # Single check
    auto-monitor --watch             # Real-time dashboard
    auto-monitor --install           # Setup cron
    auto-monitor --setup-slack URL   # Add Slack alerts
    auto-monitor --set cpu_warning=80

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utility Functions
# ============================================================
check_command() {
    command -v "$1" &>/dev/null
}

get_cpu_cores() {
    nproc 2>/dev/null || grep -c ^processor /proc/cpuinfo 2>/dev/null || echo "1"
}

# Progress bar
progress_bar() {
    local percent=$1
    local width=20
    local filled=$((percent * width / 100))
    local empty=$((width - filled))

    printf "["
    printf "%0.s█" $(seq 1 $filled 2>/dev/null) || true
    printf "%0.s░" $(seq 1 $empty 2>/dev/null) || true
    printf "]"
}

# Status indicator
status_indicator() {
    local value=$1
    local warning=$2
    local critical=$3

    if [[ $value -ge $critical ]]; then
        echo -e "${RED}✗${NC}"
    elif [[ $value -ge $warning ]]; then
        echo -e "${YELLOW}!${NC}"
    else
        echo -e "${GREEN}✓${NC}"
    fi
}

# ============================================================
# System Checks
# ============================================================
check_cpu() {
    local cpu_usage
    cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print int($2 + $4)}' 2>/dev/null || echo "0")

    local status="ok"
    if [[ $cpu_usage -ge $CPU_CRITICAL ]]; then
        status="critical"
        log_error "CPU usage critical: ${cpu_usage}%"
    elif [[ $cpu_usage -ge $CPU_WARNING ]]; then
        status="warning"
        log_warn "CPU usage warning: ${cpu_usage}%"
    fi

    echo "${cpu_usage}|${status}"
}

check_memory() {
    local mem_info
    mem_info=$(free | grep Mem)
    local total=$(echo "$mem_info" | awk '{print $2}')
    local used=$(echo "$mem_info" | awk '{print $3}')
    local mem_usage=$((used * 100 / total))

    local status="ok"
    if [[ $mem_usage -ge $MEMORY_CRITICAL ]]; then
        status="critical"
        log_error "Memory usage critical: ${mem_usage}%"
    elif [[ $mem_usage -ge $MEMORY_WARNING ]]; then
        status="warning"
        log_warn "Memory usage warning: ${mem_usage}%"
    fi

    echo "${mem_usage}|${status}"
}

check_swap() {
    local swap_info
    swap_info=$(free | grep Swap)
    local total=$(echo "$swap_info" | awk '{print $2}')
    local used=$(echo "$swap_info" | awk '{print $3}')

    if [[ $total -eq 0 ]]; then
        echo "0|ok"
        return
    fi

    local swap_usage=$((used * 100 / total))

    local status="ok"
    if [[ $swap_usage -ge $SWAP_CRITICAL ]]; then
        status="critical"
        log_error "Swap usage critical: ${swap_usage}%"
    elif [[ $swap_usage -ge $SWAP_WARNING ]]; then
        status="warning"
        log_warn "Swap usage warning: ${swap_usage}%"
    fi

    echo "${swap_usage}|${status}"
}

check_disk() {
    local worst_usage=0
    local worst_mount=""
    local status="ok"
    local results=""

    while read -r line; do
        local usage=$(echo "$line" | awk '{print $5}' | tr -d '%')
        local mount=$(echo "$line" | awk '{print $6}')

        if [[ $usage -gt $worst_usage ]]; then
            worst_usage=$usage
            worst_mount=$mount
        fi

        if [[ $usage -ge $DISK_CRITICAL ]]; then
            status="critical"
            log_error "Disk usage critical on ${mount}: ${usage}%"
        elif [[ $usage -ge $DISK_WARNING && "$status" != "critical" ]]; then
            status="warning"
            log_warn "Disk usage warning on ${mount}: ${usage}%"
        fi

        results+="${mount}:${usage},"
    done < <(df -h | grep -E '^/dev/' | awk '{print $5, $6}' | while read pct mnt; do echo "$pct $mnt"; done)

    echo "${worst_usage}|${status}|${results}"
}

check_load() {
    local load
    load=$(cat /proc/loadavg | awk '{print $1}')
    local cores=$(get_cpu_cores)
    local load_int=$(echo "$load" | cut -d. -f1)

    local status="ok"
    local load_critical=$(echo "$LOAD_CRITICAL" | cut -d. -f1)
    local load_warning=$(echo "$LOAD_WARNING" | cut -d. -f1)

    if [[ $load_int -ge $load_critical ]]; then
        status="critical"
        log_error "Load average critical: ${load}"
    elif [[ $load_int -ge $load_warning ]]; then
        status="warning"
        log_warn "Load average warning: ${load}"
    fi

    echo "${load}|${cores}|${status}"
}

check_network() {
    local rx_bytes=$(cat /sys/class/net/*/statistics/rx_bytes 2>/dev/null | awk '{sum+=$1} END {print sum}')
    local tx_bytes=$(cat /sys/class/net/*/statistics/tx_bytes 2>/dev/null | awk '{sum+=$1} END {print sum}')

    echo "${rx_bytes}|${tx_bytes}"
}

# ============================================================
# Service Checks
# ============================================================
check_services() {
    local results=""

    for service in $SERVICES_TO_MONITOR; do
        local status="unknown"
        local pid=""

        if systemctl is-active --quiet "$service" 2>/dev/null; then
            status="running"
            pid=$(systemctl show -p MainPID "$service" 2>/dev/null | cut -d= -f2)
        elif pgrep -x "$service" >/dev/null 2>&1; then
            status="running"
            pid=$(pgrep -x "$service" | head -1)
        elif pgrep -f "$service" >/dev/null 2>&1; then
            status="running"
            pid=$(pgrep -f "$service" | head -1)
        else
            # Service might not exist, skip
            continue
        fi

        if [[ "$status" != "running" ]]; then
            log_error "Service ${service} is not running"
        fi

        results+="${service}:${status}:${pid},"
    done

    echo "$results"
}

check_ports() {
    local results=""

    for port in $PORTS_TO_CHECK; do
        local status="closed"

        if ss -tuln 2>/dev/null | grep -q ":${port} " || \
           netstat -tuln 2>/dev/null | grep -q ":${port} "; then
            status="open"
        fi

        results+="${port}:${status},"
    done

    echo "$results"
}

# ============================================================
# Web Checks
# ============================================================
check_url() {
    local url="$1"
    local expected_status="${2:-200}"
    local timeout="${3:-5}"

    local start_time=$(date +%s%N)
    local response
    response=$(curl -s -o /dev/null -w "%{http_code}" --max-time "$timeout" "$url" 2>/dev/null || echo "000")
    local end_time=$(date +%s%N)

    local response_time=$(( (end_time - start_time) / 1000000 ))

    local status="ok"
    if [[ "$response" != "$expected_status" ]]; then
        status="critical"
        log_error "URL ${url} returned ${response}, expected ${expected_status}"
    fi

    echo "${url}|${response}|${response_time}|${status}"
}

check_urls() {
    local results=""

    if [[ -z "$URLS_TO_MONITOR" ]]; then
        return
    fi

    while IFS='|' read -r url expected timeout; do
        [[ -z "$url" ]] && continue
        local result=$(check_url "$url" "${expected:-200}" "${timeout:-5}")
        results+="${result};"
    done <<< "$URLS_TO_MONITOR"

    echo "$results"
}

# ============================================================
# SSL Checks
# ============================================================
check_ssl() {
    local domain="$1"

    if ! check_command openssl; then
        echo "${domain}|0|error|openssl not found"
        return
    fi

    local expiry_date
    expiry_date=$(echo | openssl s_client -servername "$domain" -connect "${domain}:443" 2>/dev/null | \
        openssl x509 -noout -enddate 2>/dev/null | cut -d= -f2)

    if [[ -z "$expiry_date" ]]; then
        echo "${domain}|0|error|could not connect"
        return
    fi

    local expiry_epoch=$(date -d "$expiry_date" +%s 2>/dev/null)
    local now_epoch=$(date +%s)
    local days_left=$(( (expiry_epoch - now_epoch) / 86400 ))

    local status="ok"
    if [[ $days_left -le $SSL_CRITICAL_DAYS ]]; then
        status="critical"
        log_error "SSL certificate for ${domain} expires in ${days_left} days"
    elif [[ $days_left -le $SSL_WARNING_DAYS ]]; then
        status="warning"
        log_warn "SSL certificate for ${domain} expires in ${days_left} days"
    fi

    echo "${domain}|${days_left}|${status}"
}

check_all_ssl() {
    local results=""

    for domain in $SSL_DOMAINS; do
        [[ -z "$domain" ]] && continue
        local result=$(check_ssl "$domain")
        results+="${result};"
    done

    echo "$results"
}

# ============================================================
# Log Checks
# ============================================================
check_logs() {
    local results=""
    local last_check_file="${DATA_DIR}/last-log-check"
    local last_check_time=0

    [[ -f "$last_check_file" ]] && last_check_time=$(cat "$last_check_file")

    while IFS='|' read -r log_file patterns; do
        [[ -z "$log_file" || ! -f "$log_file" ]] && continue

        local new_errors=0
        local pattern_regex=$(echo "$patterns" | tr '|' '|')

        if [[ -n "$pattern_regex" ]]; then
            new_errors=$(find "$log_file" -newermt "@${last_check_time}" 2>/dev/null | \
                xargs grep -c -E "$pattern_regex" 2>/dev/null || echo "0")
        fi

        if [[ $new_errors -gt 0 ]]; then
            log_warn "Found ${new_errors} new errors in ${log_file}"
            results+="${log_file}:${new_errors},"
        fi
    done <<< "$LOG_FILES_TO_MONITOR"

    date +%s > "$last_check_file"

    echo "$results"
}

# ============================================================
# Alerts
# ============================================================
should_alert() {
    local key="$1"
    local current_time=$(date +%s)
    local last_time=${LAST_ALERT_TIME[$key]:-0}

    if [[ $((current_time - last_time)) -lt $ALERT_COOLDOWN ]]; then
        return 1
    fi

    LAST_ALERT_TIME[$key]=$current_time
    return 0
}

send_alert() {
    local level="$1"
    local title="$2"
    local message="$3"

    local icon="🔔"
    [[ "$level" == "warning" ]] && icon="⚠️"
    [[ "$level" == "critical" ]] && icon="🚨"
    [[ "$level" == "recovery" ]] && icon="✅"

    # Slack
    if [[ "$SLACK_ENABLED" == "true" && -n "$SLACK_WEBHOOK" ]]; then
        local color="good"
        [[ "$level" == "warning" ]] && color="warning"
        [[ "$level" == "critical" ]] && color="danger"

        local payload=$(cat << EOF
{
    "channel": "${SLACK_CHANNEL}",
    "username": "AUTO-MONITOR",
    "icon_emoji": ":chart_with_upwards_trend:",
    "attachments": [{
        "color": "${color}",
        "title": "${icon} ${title}",
        "text": "${message}",
        "footer": "AUTO-MONITOR v${VERSION} | ${HOSTNAME}",
        "ts": $(date +%s)
    }]
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$SLACK_WEBHOOK" >/dev/null 2>&1 || true
    fi

    # Telegram
    if [[ "$TELEGRAM_ENABLED" == "true" && -n "$TELEGRAM_BOT_TOKEN" ]]; then
        local text="${icon} *${title}*%0A%0A${message}%0A%0A_${HOSTNAME}_"
        curl -s "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
            -d "chat_id=${TELEGRAM_CHAT_ID}&text=${text}&parse_mode=Markdown" \
            >/dev/null 2>&1 || true
    fi

    # Discord
    if [[ "$DISCORD_ENABLED" == "true" && -n "$DISCORD_WEBHOOK" ]]; then
        local payload=$(cat << EOF
{
    "content": "${icon} **${title}**\n\n${message}\n\n_${HOSTNAME}_"
}
EOF
)
        curl -s -X POST -H 'Content-type: application/json' \
            --data "$payload" "$DISCORD_WEBHOOK" >/dev/null 2>&1 || true
    fi

    # Email
    if [[ "$EMAIL_ENABLED" == "true" && -n "$EMAIL_TO" ]]; then
        echo -e "Subject: ${icon} ${title}\n\n${message}\n\n-- AUTO-MONITOR v${VERSION}\n${HOSTNAME}" | \
            sendmail "$EMAIL_TO" 2>/dev/null || true
    fi

    log_info "Alert sent: ${level} - ${title}"
}

test_alert() {
    echo -e "${CYAN}Sending test alert...${NC}"

    send_alert "info" "AUTO-MONITOR Test Alert" \
        "This is a test alert from AUTO-MONITOR v${VERSION}.\nServer: ${HOSTNAME}\nTime: $(date)"

    echo -e "${GREEN}✓${NC} Test alert sent to all configured channels"
}

# ============================================================
# Display Functions
# ============================================================
print_single_check() {
    print_banner

    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  📊 AUTO-MONITOR v${VERSION}                                        ║${NC}"
    echo -e "${BOLD}║  🤟 WIA AUTO-MONITOR · https://wiastandards.com/auto-monitor  ║${NC}"
    echo -e "${BOLD}║  弘益人間 · Benefit All Humanity                              ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📍 서버: %-49s║${NC}\n" "$HOSTNAME"
    printf "${BOLD}║  📅 체크: %-49s║${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BOLD}║                                                              ║${NC}"

    # System resources
    echo -e "${BOLD}║  💻 시스템 리소스                                            ║${NC}"

    # CPU
    IFS='|' read -r cpu_val cpu_status <<< "$(check_cpu)"
    local cpu_bar=$(progress_bar $cpu_val)
    local cpu_ind=$(status_indicator $cpu_val $CPU_WARNING $CPU_CRITICAL)
    printf "║  ├── CPU:      %3d%% ${cpu_bar} ${cpu_ind}                  ║\n" "$cpu_val"

    # Memory
    IFS='|' read -r mem_val mem_status <<< "$(check_memory)"
    local mem_bar=$(progress_bar $mem_val)
    local mem_ind=$(status_indicator $mem_val $MEMORY_WARNING $MEMORY_CRITICAL)
    printf "║  ├── 메모리:   %3d%% ${mem_bar} ${mem_ind}                  ║\n" "$mem_val"

    # Swap
    IFS='|' read -r swap_val swap_status <<< "$(check_swap)"
    local swap_bar=$(progress_bar $swap_val)
    local swap_ind=$(status_indicator $swap_val $SWAP_WARNING $SWAP_CRITICAL)
    printf "║  ├── 스왑:     %3d%% ${swap_bar} ${swap_ind}                  ║\n" "$swap_val"

    # Disk
    IFS='|' read -r disk_val disk_status disk_details <<< "$(check_disk)"
    local disk_bar=$(progress_bar $disk_val)
    local disk_ind=$(status_indicator $disk_val $DISK_WARNING $DISK_CRITICAL)
    printf "║  ├── 디스크:   %3d%% ${disk_bar} ${disk_ind}                  ║\n" "$disk_val"

    # Load
    IFS='|' read -r load_val cores load_status <<< "$(check_load)"
    local load_ind="${GREEN}✓${NC}"
    [[ "$load_status" == "warning" ]] && load_ind="${YELLOW}!${NC}"
    [[ "$load_status" == "critical" ]] && load_ind="${RED}✗${NC}"
    printf "║  └── 부하:    %s (%s cores) ${load_ind}                              ║\n" "$load_val" "$cores"

    echo -e "${BOLD}║                                                              ║${NC}"

    # Services
    echo -e "${BOLD}║  ⚙️ 서비스 상태                                              ║${NC}"
    local services=$(check_services)
    local service_count=0
    IFS=',' read -ra service_list <<< "$services"
    for svc in "${service_list[@]}"; do
        [[ -z "$svc" ]] && continue
        IFS=':' read -r name status pid <<< "$svc"
        local svc_ind="${GREEN}✓${NC}"
        [[ "$status" != "running" ]] && svc_ind="${RED}✗${NC}"
        if [[ $service_count -lt 4 ]]; then
            printf "║  ├── %-9s Running ${svc_ind}                                    ║\n" "$name:"
        fi
        ((service_count++))
    done
    if [[ $service_count -eq 0 ]]; then
        echo -e "║  └── (감지된 서비스 없음)                                    ║"
    fi

    echo -e "${BOLD}║                                                              ║${NC}"

    # Web monitoring
    if [[ -n "$URLS_TO_MONITOR" ]]; then
        echo -e "${BOLD}║  🌐 웹 모니터링                                              ║${NC}"
        local url_results=$(check_urls)
        IFS=';' read -ra url_list <<< "$url_results"
        for url_result in "${url_list[@]}"; do
            [[ -z "$url_result" ]] && continue
            IFS='|' read -r url code time status <<< "$url_result"
            local url_short=$(echo "$url" | sed 's|https://||' | cut -c1-20)
            local url_ind="${GREEN}✓${NC}"
            [[ "$status" != "ok" ]] && url_ind="${RED}✗${NC}"
            printf "║  ├── %-20s %s OK   (%.2fs) ${url_ind}                ║\n" "$url_short" "$code" "$(echo "scale=2; $time/1000" | bc)"
        done
        echo -e "${BOLD}║                                                              ║${NC}"
    fi

    # SSL certificates
    if [[ -n "$SSL_DOMAINS" ]]; then
        echo -e "${BOLD}║  🔐 SSL 인증서                                               ║${NC}"
        local ssl_results=$(check_all_ssl)
        IFS=';' read -ra ssl_list <<< "$ssl_results"
        for ssl_result in "${ssl_list[@]}"; do
            [[ -z "$ssl_result" ]] && continue
            IFS='|' read -r domain days status <<< "$ssl_result"
            local ssl_ind="${GREEN}✓${NC}"
            [[ "$status" == "warning" ]] && ssl_ind="${YELLOW}!${NC}"
            [[ "$status" == "critical" ]] && ssl_ind="${RED}✗${NC}"
            printf "║  ├── %-20s %3d일 남음 ${ssl_ind}                       ║\n" "$domain" "$days"
        done
        echo -e "${BOLD}║                                                              ║${NC}"
    fi

    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"

    # Overall status
    if [[ ${#CRITICALS[@]} -gt 0 ]]; then
        echo -e "║  ${RED}🚨 심각한 문제 발견: ${#CRITICALS[@]}개${NC}                                    ║"
    elif [[ ${#WARNINGS[@]} -gt 0 ]]; then
        echo -e "║  ${YELLOW}⚠️ 경고: ${#WARNINGS[@]}개${NC}                                                ║"
    else
        echo -e "║  ${GREEN}✅ 모든 시스템 정상!${NC}                                        ║"
    fi

    printf "║  🕐 다음 체크: %d분 후                                        ║\n" "$((CHECK_INTERVAL / 60))"
    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo
}

# ============================================================
# Watch Mode (Real-time Dashboard)
# ============================================================
run_watch_mode() {
    local interval=2

    # Hide cursor
    tput civis 2>/dev/null || true
    trap 'tput cnorm 2>/dev/null; exit' INT TERM

    while true; do
        clear

        local cpu=$(check_cpu | cut -d'|' -f1)
        local mem=$(check_memory | cut -d'|' -f1)
        local swap=$(check_swap | cut -d'|' -f1)
        local disk=$(check_disk | cut -d'|' -f1)
        IFS='|' read -r load cores _ <<< "$(check_load)"

        echo "┌─ AUTO-MONITOR v${VERSION} ─────────────────────── ${HOSTNAME} ─┐"
        echo "│  Updated: $(date '+%Y-%m-%d %H:%M:%S')                    [q] Quit    │"
        echo "├──────────────────────────────────────────────────────────────────┤"

        local cpu_bar=$(progress_bar $cpu)
        local mem_bar=$(progress_bar $mem)
        local swap_bar=$(progress_bar $swap)
        local disk_bar=$(progress_bar $disk)

        printf "│  CPU  ${cpu_bar} %3d%%    LOAD  %-5s / %s cores        │\n" "$cpu" "$load" "$cores"
        printf "│  MEM  ${mem_bar} %3d%%    SWAP  ${swap_bar} %3d%%│\n" "$mem" "$swap"
        printf "│  DISK ${disk_bar} %3d%%                                      │\n" "$disk"

        echo "├──────────────────────────────────────────────────────────────────┤"

        echo "│  SERVICES                          │  WEBSITES                   │"

        local services=$(check_services)
        local count=0
        IFS=',' read -ra svc_array <<< "$services"
        for svc in "${svc_array[@]}"; do
            [[ -z "$svc" ]] && continue
            IFS=':' read -r name status pid <<< "$svc"
            local ind="✅"
            [[ "$status" != "running" ]] && ind="❌"
            if [[ $count -lt 4 ]]; then
                printf "│  %s %-10s (pid: %-6s)         │                             │\n" "$ind" "$name" "${pid:-N/A}"
            fi
            ((count++))
        done

        for ((i=count; i<4; i++)); do
            echo "│                                    │                             │"
        done

        echo "├──────────────────────────────────────────────────────────────────┤"
        echo "│  SSL CERTIFICATES                  │  RECENT ALERTS              │"

        if [[ -n "$SSL_DOMAINS" ]]; then
            for domain in $SSL_DOMAINS; do
                IFS='|' read -r _ days status <<< "$(check_ssl "$domain")"
                local ind="✅"
                [[ "$status" != "ok" ]] && ind="⚠️"
                printf "│  %-24s %3d days %s   │  (none)                     │\n" "$domain" "$days" "$ind"
                break
            done
        else
            echo "│  (no SSL domains configured)       │  (none)                     │"
        fi

        echo "└──────────────────────────────────────────────────────────────────┘"

        # Check for quit
        read -t $interval -n 1 key 2>/dev/null && [[ "$key" == "q" ]] && break
    done

    tput cnorm 2>/dev/null || true
}

# ============================================================
# Cron Installation
# ============================================================
install_cron() {
    local cron_file="/etc/cron.d/auto-monitor"
    local script_path=$(realpath "$0")

    cat > "$cron_file" << EOF
# AUTO-MONITOR v${VERSION}
# 弘益人間 (홍익인간) · Benefit All Humanity

SHELL=/bin/bash
PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin

*/5 * * * * root $script_path --quiet 2>&1 | logger -t auto-monitor
EOF

    chmod 644 "$cron_file"

    echo -e "${GREEN}✓${NC} Cron job installed (every 5 minutes)"
    echo -e "  File: ${cron_file}"
}

uninstall_cron() {
    rm -f /etc/cron.d/auto-monitor
    echo -e "${GREEN}✓${NC} Cron job removed"
}

# ============================================================
# Setup Wizard
# ============================================================
run_setup() {
    print_banner

    echo -e "${BOLD}AUTO-MONITOR 설정 마법사${NC}\n"

    # Slack
    read -p "Slack Webhook URL (비워두면 건너뜀): " SLACK_WEBHOOK
    if [[ -n "$SLACK_WEBHOOK" ]]; then
        SLACK_ENABLED=true
        echo -e "${GREEN}✓${NC} Slack 알림 활성화"
    fi

    # Telegram
    read -p "Telegram Bot Token (비워두면 건너뜀): " TELEGRAM_BOT_TOKEN
    if [[ -n "$TELEGRAM_BOT_TOKEN" ]]; then
        read -p "Telegram Chat ID: " TELEGRAM_CHAT_ID
        TELEGRAM_ENABLED=true
        echo -e "${GREEN}✓${NC} Telegram 알림 활성화"
    fi

    # URLs
    read -p "모니터링할 URL (예: https://example.com, 비워두면 건너뜀): " url
    if [[ -n "$url" ]]; then
        URLS_TO_MONITOR="${url}|200|5"
        echo -e "${GREEN}✓${NC} URL 추가됨: $url"
    fi

    # SSL
    read -p "SSL 모니터링할 도메인 (예: example.com, 비워두면 건너뜀): " domain
    if [[ -n "$domain" ]]; then
        SSL_DOMAINS="$domain"
        echo -e "${GREEN}✓${NC} SSL 도메인 추가됨: $domain"
    fi

    # Save config
    save_config

    echo -e "\n${GREEN}✓${NC} 설정 저장 완료: ${CONFIG_FILE}"
    echo -e "\n사용법:"
    echo -e "  auto-monitor           # 한 번 체크"
    echo -e "  auto-monitor --watch   # 실시간 대시보드"
    echo -e "  auto-monitor --install # cron 등록"
}

save_config() {
    cat > "$CONFIG_FILE" << EOF
#!/bin/bash
# AUTO-MONITOR v${VERSION} Configuration
# Generated: $(date)

CHECK_INTERVAL=${CHECK_INTERVAL}

# Thresholds
CPU_WARNING=${CPU_WARNING}
CPU_CRITICAL=${CPU_CRITICAL}
MEMORY_WARNING=${MEMORY_WARNING}
MEMORY_CRITICAL=${MEMORY_CRITICAL}
SWAP_WARNING=${SWAP_WARNING}
SWAP_CRITICAL=${SWAP_CRITICAL}
DISK_WARNING=${DISK_WARNING}
DISK_CRITICAL=${DISK_CRITICAL}
LOAD_WARNING=${LOAD_WARNING}
LOAD_CRITICAL=${LOAD_CRITICAL}

# Services
SERVICES_TO_MONITOR="${SERVICES_TO_MONITOR}"
PORTS_TO_CHECK="${PORTS_TO_CHECK}"

# URLs
URLS_TO_MONITOR="${URLS_TO_MONITOR}"

# SSL
SSL_DOMAINS="${SSL_DOMAINS}"
SSL_WARNING_DAYS=${SSL_WARNING_DAYS}
SSL_CRITICAL_DAYS=${SSL_CRITICAL_DAYS}

# Slack
SLACK_ENABLED=${SLACK_ENABLED}
SLACK_WEBHOOK="${SLACK_WEBHOOK}"
SLACK_CHANNEL="${SLACK_CHANNEL}"

# Telegram
TELEGRAM_ENABLED=${TELEGRAM_ENABLED}
TELEGRAM_BOT_TOKEN="${TELEGRAM_BOT_TOKEN}"
TELEGRAM_CHAT_ID="${TELEGRAM_CHAT_ID}"

# Discord
DISCORD_ENABLED=${DISCORD_ENABLED}
DISCORD_WEBHOOK="${DISCORD_WEBHOOK}"

# Email
EMAIL_ENABLED=${EMAIL_ENABLED}
EMAIL_TO="${EMAIL_TO}"

# Alert rules
ALERT_ON_WARNING=${ALERT_ON_WARNING}
ALERT_ON_CRITICAL=${ALERT_ON_CRITICAL}
ALERT_ON_RECOVERY=${ALERT_ON_RECOVERY}
ALERT_COOLDOWN=${ALERT_COOLDOWN}
EOF

    chmod 600 "$CONFIG_FILE"
}

# ============================================================
# Preset Configurations
# ============================================================
apply_preset() {
    local preset="$1"

    case "$preset" in
        production)
            CPU_WARNING=60
            CPU_CRITICAL=80
            MEMORY_WARNING=70
            MEMORY_CRITICAL=85
            DISK_WARNING=70
            DISK_CRITICAL=85
            echo -e "${GREEN}✓${NC} Applied 'production' preset (strict thresholds)"
            ;;
        development)
            CPU_WARNING=80
            CPU_CRITICAL=95
            MEMORY_WARNING=85
            MEMORY_CRITICAL=98
            DISK_WARNING=85
            DISK_CRITICAL=95
            echo -e "${GREEN}✓${NC} Applied 'development' preset (relaxed thresholds)"
            ;;
        minimal)
            CPU_WARNING=90
            CPU_CRITICAL=99
            MEMORY_WARNING=95
            MEMORY_CRITICAL=99
            DISK_WARNING=90
            DISK_CRITICAL=98
            echo -e "${GREEN}✓${NC} Applied 'minimal' preset (alerts only on critical)"
            ;;
        *)
            echo -e "${RED}✗${NC} Unknown preset: $preset"
            echo "Available: production, development, minimal"
            return 1
            ;;
    esac

    save_config
}

# ============================================================
# Main Entry Point
# ============================================================
main() {
    load_config

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --watch)
                WATCH_MODE=true
                ;;
            --daemon)
                # Would implement daemon mode
                echo "Daemon mode: use --install for cron-based monitoring"
                exit 0
                ;;
            --install)
                install_cron
                exit 0
                ;;
            --uninstall)
                uninstall_cron
                exit 0
                ;;
            --setup)
                run_setup
                exit 0
                ;;
            --setup-slack)
                shift
                SLACK_WEBHOOK="$1"
                SLACK_ENABLED=true
                save_config
                echo -e "${GREEN}✓${NC} Slack webhook configured"
                exit 0
                ;;
            --setup-telegram)
                shift
                TELEGRAM_BOT_TOKEN="$1"
                shift
                TELEGRAM_CHAT_ID="$1"
                TELEGRAM_ENABLED=true
                save_config
                echo -e "${GREEN}✓${NC} Telegram configured"
                exit 0
                ;;
            --setup-discord)
                shift
                DISCORD_WEBHOOK="$1"
                DISCORD_ENABLED=true
                save_config
                echo -e "${GREEN}✓${NC} Discord webhook configured"
                exit 0
                ;;
            --setup-email)
                shift
                EMAIL_TO="$1"
                EMAIL_ENABLED=true
                save_config
                echo -e "${GREEN}✓${NC} Email configured"
                exit 0
                ;;
            --set)
                shift
                for kv in "$@"; do
                    IFS='=' read -r key val <<< "$kv"
                    key=$(echo "$key" | tr '[:lower:]' '[:upper:]')
                    eval "${key}=${val}"
                done
                save_config
                echo -e "${GREEN}✓${NC} Settings updated"
                exit 0
                ;;
            --preset)
                shift
                apply_preset "$1"
                exit 0
                ;;
            --add-url)
                shift
                if [[ -n "$URLS_TO_MONITOR" ]]; then
                    URLS_TO_MONITOR="${URLS_TO_MONITOR}\n${1}|200|5"
                else
                    URLS_TO_MONITOR="${1}|200|5"
                fi
                save_config
                echo -e "${GREEN}✓${NC} URL added: $1"
                exit 0
                ;;
            --add-ssl)
                shift
                SSL_DOMAINS="${SSL_DOMAINS} $1"
                save_config
                echo -e "${GREEN}✓${NC} SSL domain added: $1"
                exit 0
                ;;
            --test-alert)
                test_alert
                exit 0
                ;;
            --check-only)
                shift
                CHECK_ONLY="$1"
                ;;
            --quiet|-q)
                exec 1>/dev/null
                ;;
            -v|--verbose)
                VERBOSE=true
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            --version)
                echo "AUTO-MONITOR v${VERSION}"
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
        shift
    done

    # Execute
    if [[ "$WATCH_MODE" == "true" ]]; then
        run_watch_mode
    else
        print_single_check

        # Send alerts if needed
        if [[ ${#CRITICALS[@]} -gt 0 && "$ALERT_ON_CRITICAL" == "true" ]]; then
            send_alert "critical" "Critical Issues Detected" "${CRITICALS[*]}"
        elif [[ ${#WARNINGS[@]} -gt 0 && "$ALERT_ON_WARNING" == "true" ]]; then
            send_alert "warning" "Warnings Detected" "${WARNINGS[*]}"
        fi
    fi
}

main "$@"
