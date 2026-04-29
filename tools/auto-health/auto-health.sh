#!/bin/bash
#
# AUTO-HEALTH v1.0
# One-Click Server Health Diagnostics
#
# World Certification Industry Association (WIA)
# https://wia.family
#
# 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
# Usage: auto-health [OPTIONS]
#

set -e

# ============================================================
# Version & Constants
# ============================================================
VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/auto-health.conf"
REPORT_DIR="/var/log/auto-health/reports"
LOG_FILE="${SCRIPT_DIR}/logs/auto-health.log"
DATA_DIR="${SCRIPT_DIR}/data"

# ============================================================
# Colors
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
CHECK_SECURITY=true
CHECK_PERFORMANCE=true
CHECK_SERVICES=true
CHECK_FILESYSTEM=true

# Security thresholds
SSH_PORT_DEFAULT=22
ROOT_LOGIN_WARN=true
PASSWORD_AUTH_WARN=true

# Performance thresholds
CPU_HIGH_THRESHOLD=80
MEMORY_HIGH_THRESHOLD=90
DISK_HIGH_THRESHOLD=85
SWAP_HIGH_THRESHOLD=50

# Filesystem
LARGE_FILE_THRESHOLD="100M"
OLD_LOG_DAYS=30

# Auto-fix
AUTO_FIX_ENABLED=false
AUTO_FIX_CONFIRM=true
BACKUP_BEFORE_FIX=true
DRY_RUN=false

# Report
REPORT_FORMAT="terminal"
REPORT_EMAIL=""

# Mode
QUICK_MODE=false
FULL_MODE=false
FIX_MODE=false
FIX_TARGET=""

# Runtime
HOSTNAME=$(hostname -f 2>/dev/null || hostname)
declare -a RECOMMENDATIONS
declare -a FIXABLE_ITEMS
SECURITY_SCORE=100
PERFORMANCE_SCORE=100
SERVICES_SCORE=100
FILESYSTEM_SCORE=100

# ============================================================
# Load Configuration
# ============================================================
load_config() {
    [[ -f "$CONFIG_FILE" ]] && source "$CONFIG_FILE"
    mkdir -p "$REPORT_DIR" "$DATA_DIR" "$(dirname "$LOG_FILE")"
}

# ============================================================
# Logging
# ============================================================
log() {
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    echo "[$timestamp] $*" >> "$LOG_FILE" 2>/dev/null || true
}

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
║  ██╗  ██╗███████╗ █████╗ ██╗  ████████╗██╗  ██╗              ║
║  ██║  ██║██╔════╝██╔══██╗██║  ╚══██╔══╝██║  ██║              ║
║  ███████║█████╗  ███████║██║     ██║   ███████║              ║
║  ██╔══██║██╔══╝  ██╔══██║██║     ██║   ██╔══██║              ║
║  ██║  ██║███████╗██║  ██║███████╗██║   ██║  ██║              ║
║  ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚══════╝╚═╝   ╚═╝  ╚═╝              ║
║                                                              ║
║  AUTO-HEALTH v1.0 · Server Health Diagnostics                ║
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
${BOLD}AUTO-HEALTH v${VERSION}${NC} - Server Health Diagnostics

${BOLD}USAGE:${NC}
    auto-health [OPTIONS]

${BOLD}DIAGNOSTIC MODES:${NC}
    (none)              Standard diagnosis
    --quick             Quick diagnosis (essential only)
    --full              Full comprehensive diagnosis
    --security          Security audit only
    --performance       Performance analysis only
    --services          Service optimization only
    --filesystem        Filesystem analysis only

${BOLD}AUTO-FIX:${NC}
    --fix               Auto-fix all fixable issues
    --fix --dry-run     Simulate fixes without changes
    --fix ssh           Fix SSH configuration only
    --fix firewall      Fix firewall only
    --fix permissions   Fix file permissions only

${BOLD}REPORTS:${NC}
    --report            Terminal report (default)
    --report html       Generate HTML report
    --report json       Output as JSON
    --email ADDRESS     Send report via email

${BOLD}SCHEDULING:${NC}
    --schedule weekly   Weekly health check
    --schedule monthly  Monthly health check
    --unschedule        Remove scheduled checks

${BOLD}OTHER:${NC}
    --cleanup logs      Clean old log files
    --cleanup temp      Clean temp files
    -v, --verbose       Verbose output
    -h, --help          Show this help
    --version           Show version

${BOLD}EXAMPLES:${NC}
    auto-health                    # Standard check
    auto-health --quick            # Quick check
    auto-health --security         # Security audit
    auto-health --fix --dry-run    # Preview fixes
    auto-health --fix              # Apply fixes
    auto-health --report html > report.html

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utility Functions
# ============================================================
check_command() {
    command -v "$1" &>/dev/null
}

add_recommendation() {
    local category="$1"
    local message="$2"
    local fixable="${3:-false}"

    RECOMMENDATIONS+=("[$category] $message")
    [[ "$fixable" == "true" ]] && FIXABLE_ITEMS+=("$message")
}

deduct_score() {
    local category="$1"
    local points="$2"

    case "$category" in
        security)    SECURITY_SCORE=$((SECURITY_SCORE - points)) ;;
        performance) PERFORMANCE_SCORE=$((PERFORMANCE_SCORE - points)) ;;
        services)    SERVICES_SCORE=$((SERVICES_SCORE - points)) ;;
        filesystem)  FILESYSTEM_SCORE=$((FILESYSTEM_SCORE - points)) ;;
    esac
}

# ============================================================
# Security Checks
# ============================================================
check_ssh_config() {
    local sshd_config="/etc/ssh/sshd_config"
    [[ ! -f "$sshd_config" ]] && return

    echo -e "  ├── SSH 설정 분석"

    # Check port
    local ssh_port=$(grep -E "^Port " "$sshd_config" 2>/dev/null | awk '{print $2}' || echo "22")
    ssh_port=${ssh_port:-22}

    if [[ "$ssh_port" == "22" ]]; then
        echo -e "  │   ├── 포트: ${ssh_port} ${YELLOW}⚠️ 기본 포트 사용 중${NC}"
        echo -e "  │   │   └── ${DIM}💡 권장: 비표준 포트로 변경${NC}"
        add_recommendation "보안" "SSH 포트를 비표준 포트로 변경" true
        deduct_score security 5
    else
        echo -e "  │   ├── 포트: ${ssh_port} ${GREEN}✅${NC}"
    fi

    # Check root login
    local root_login=$(grep -E "^PermitRootLogin" "$sshd_config" 2>/dev/null | awk '{print $2}' || echo "yes")
    if [[ "$root_login" != "no" ]]; then
        echo -e "  │   ├── 루트 로그인: 허용 ${YELLOW}⚠️${NC}"
        echo -e "  │   │   └── ${DIM}💡 권장: PermitRootLogin no${NC}"
        add_recommendation "보안" "SSH 루트 로그인 비활성화" true
        deduct_score security 10
    else
        echo -e "  │   ├── 루트 로그인: 비활성 ${GREEN}✅${NC}"
    fi

    # Check password auth
    local pass_auth=$(grep -E "^PasswordAuthentication" "$sshd_config" 2>/dev/null | awk '{print $2}' || echo "yes")
    if [[ "$pass_auth" != "no" ]]; then
        echo -e "  │   └── 패스워드 인증: 허용 ${YELLOW}⚠️${NC}"
        echo -e "  │       └── ${DIM}💡 권장: 키 인증만 사용${NC}"
        add_recommendation "보안" "SSH 키 인증만 사용하도록 변경"
        deduct_score security 5
    else
        echo -e "  │   └── 패스워드 인증: 비활성 ${GREEN}✅${NC}"
    fi
}

check_firewall() {
    echo -e "  ├── 방화벽 상태"

    local fw_status="inactive"
    local fw_type="none"

    if systemctl is-active --quiet firewalld 2>/dev/null; then
        fw_status="active"
        fw_type="firewalld"
    elif systemctl is-active --quiet ufw 2>/dev/null; then
        fw_status="active"
        fw_type="ufw"
    elif iptables -L -n 2>/dev/null | grep -q "Chain INPUT"; then
        local rule_count=$(iptables -L -n 2>/dev/null | grep -c "^[A-Z]" || echo "0")
        if [[ $rule_count -gt 3 ]]; then
            fw_status="active"
            fw_type="iptables"
        fi
    fi

    if [[ "$fw_status" == "active" ]]; then
        echo -e "  │   └── ${fw_type}: 활성 ${GREEN}✅${NC}"
    else
        echo -e "  │   └── 방화벽: 비활성 ${RED}✗${NC}"
        add_recommendation "보안" "방화벽 활성화 필요" true
        deduct_score security 15
    fi
}

check_fail2ban() {
    echo -e "  ├── fail2ban"

    if systemctl is-active --quiet fail2ban 2>/dev/null; then
        local jail_count=$(fail2ban-client status 2>/dev/null | grep "Number of jail" | awk '{print $NF}' || echo "0")
        echo -e "  │   └── 활성 (${jail_count} jails) ${GREEN}✅${NC}"
    else
        echo -e "  │   └── 비활성 ${YELLOW}⚠️${NC}"
        add_recommendation "보안" "fail2ban 설치 및 활성화 권장"
        deduct_score security 5
    fi
}

check_selinux() {
    echo -e "  ├── SELinux/AppArmor"

    if check_command getenforce; then
        local selinux_status=$(getenforce 2>/dev/null || echo "Disabled")
        if [[ "$selinux_status" == "Enforcing" ]]; then
            echo -e "  │   └── SELinux: Enforcing ${GREEN}✅${NC}"
        elif [[ "$selinux_status" == "Permissive" ]]; then
            echo -e "  │   └── SELinux: Permissive ${YELLOW}⚠️${NC}"
            add_recommendation "보안" "SELinux를 Enforcing 모드로 변경 권장"
            deduct_score security 5
        else
            echo -e "  │   └── SELinux: Disabled ${YELLOW}⚠️${NC}"
            deduct_score security 5
        fi
    elif check_command aa-status; then
        if aa-status --enabled 2>/dev/null; then
            echo -e "  │   └── AppArmor: 활성 ${GREEN}✅${NC}"
        else
            echo -e "  │   └── AppArmor: 비활성 ${YELLOW}⚠️${NC}"
        fi
    else
        echo -e "  │   └── SELinux/AppArmor: 미설치 ${DIM}(선택사항)${NC}"
    fi
}

check_security_updates() {
    echo -e "  ├── 보안 업데이트"

    local updates=0

    if check_command dnf; then
        updates=$(dnf check-update --security 2>/dev/null | grep -c "^[a-zA-Z]" || echo "0")
    elif check_command yum; then
        updates=$(yum check-update --security 2>/dev/null | grep -c "^[a-zA-Z]" || echo "0")
    elif check_command apt; then
        updates=$(apt list --upgradable 2>/dev/null | grep -c security || echo "0")
    fi

    if [[ $updates -gt 0 ]]; then
        echo -e "  │   └── ${updates}개 보안 업데이트 대기 ${YELLOW}⚠️${NC}"
        echo -e "  │       └── ${DIM}💡 권장: 보안 업데이트 적용${NC}"
        add_recommendation "보안" "보안 업데이트 ${updates}개 적용 필요"
        deduct_score security $((updates > 5 ? 10 : updates * 2))
    else
        echo -e "  │   └── 최신 상태 ${GREEN}✅${NC}"
    fi
}

check_open_ports() {
    echo -e "  └── 열린 포트"

    local ports=$(ss -tuln 2>/dev/null | grep LISTEN | awk '{print $5}' | grep -oE '[0-9]+$' | sort -nu | tr '\n' ', ' | sed 's/,$//')

    echo -e "      └── ${ports:-없음}"

    # Check for potentially dangerous exposed ports
    if echo "$ports" | grep -qE "\b3306\b"; then
        echo -e "          └── 3306 ${YELLOW}⚠️ MySQL 외부 노출 주의${NC}"
        add_recommendation "보안" "MySQL 포트(3306) 외부 접근 제한 권장"
        deduct_score security 5
    fi

    if echo "$ports" | grep -qE "\b6379\b"; then
        echo -e "          └── 6379 ${YELLOW}⚠️ Redis 외부 노출 주의${NC}"
        add_recommendation "보안" "Redis 포트(6379) 외부 접근 제한 권장"
        deduct_score security 5
    fi
}

run_security_checks() {
    echo -e "\n${BOLD}  🔒 보안 진단${NC}"
    echo -e "  ────────────────────────────────────────────────────────────"

    check_ssh_config
    check_firewall
    check_fail2ban
    check_selinux
    check_security_updates
    check_open_ports
}

# ============================================================
# Performance Checks
# ============================================================
check_cpu_performance() {
    local cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print int($2 + $4)}' 2>/dev/null || echo "0")

    if [[ $cpu_usage -ge $CPU_HIGH_THRESHOLD ]]; then
        echo -e "  ├── CPU 사용률: ${cpu_usage}% ${RED}✗${NC}"
        add_recommendation "성능" "CPU 사용률이 높음 - 프로세스 확인 필요"
        deduct_score performance 10
    else
        echo -e "  ├── CPU 사용률: ${cpu_usage}% ${GREEN}✅${NC}"
    fi
}

check_memory_performance() {
    local mem_info=$(free | grep Mem)
    local total=$(echo "$mem_info" | awk '{print $2}')
    local used=$(echo "$mem_info" | awk '{print $3}')
    local mem_usage=$((used * 100 / total))

    if [[ $mem_usage -ge $MEMORY_HIGH_THRESHOLD ]]; then
        echo -e "  ├── 메모리: ${mem_usage}% ${RED}✗${NC}"
        add_recommendation "성능" "메모리 사용률이 높음"
        deduct_score performance 10
    else
        echo -e "  ├── 메모리: ${mem_usage}% ${GREEN}✅${NC}"
    fi
}

check_swap_performance() {
    local swap_info=$(free | grep Swap)
    local total=$(echo "$swap_info" | awk '{print $2}')
    local used=$(echo "$swap_info" | awk '{print $3}')

    if [[ $total -eq 0 ]]; then
        echo -e "  ├── 스왑: 없음 ${DIM}(정상)${NC}"
        return
    fi

    local swap_usage=$((used * 100 / total))

    if [[ $swap_usage -ge $SWAP_HIGH_THRESHOLD ]]; then
        echo -e "  ├── 스왑: ${swap_usage}% ${YELLOW}⚠️${NC}"
        add_recommendation "성능" "스왑 사용률이 높음 - 메모리 증설 고려"
        deduct_score performance 5
    else
        echo -e "  ├── 스왑: ${swap_usage}% ${GREEN}✅${NC}"
    fi
}

check_disk_performance() {
    local worst_usage=0
    local worst_mount=""

    while read -r line; do
        local usage=$(echo "$line" | awk '{print $5}' | tr -d '%')
        local mount=$(echo "$line" | awk '{print $6}')

        if [[ $usage -gt $worst_usage ]]; then
            worst_usage=$usage
            worst_mount=$mount
        fi
    done < <(df -h | grep -E '^/dev/')

    if [[ $worst_usage -ge $DISK_HIGH_THRESHOLD ]]; then
        echo -e "  ├── 디스크: ${worst_usage}% (${worst_mount}) ${YELLOW}⚠️${NC}"
        add_recommendation "성능" "디스크 사용률이 높음 (${worst_mount})"
        deduct_score performance 10
    else
        echo -e "  ├── 디스크: ${worst_usage}% ${GREEN}✅${NC}"
    fi
}

check_load_average() {
    local load=$(cat /proc/loadavg | awk '{print $1}')
    local cores=$(nproc 2>/dev/null || echo "1")

    echo -e "  ├── 시스템 부하: ${load} (${cores} cores) ${GREEN}✅${NC}"
}

check_boot_time() {
    if check_command systemd-analyze; then
        local boot_time=$(systemd-analyze 2>/dev/null | head -1 | grep -oE '[0-9.]+s' | head -1 || echo "N/A")
        echo -e "  └── 부팅 시간: ${boot_time} ${GREEN}✅${NC}"
    else
        echo -e "  └── 부팅 시간: N/A"
    fi
}

run_performance_checks() {
    echo -e "\n${BOLD}  💻 성능 진단${NC}"
    echo -e "  ────────────────────────────────────────────────────────────"

    check_cpu_performance
    check_memory_performance
    check_swap_performance
    check_disk_performance
    check_load_average
    check_boot_time
}

# ============================================================
# Service Checks
# ============================================================
check_apache() {
    if ! systemctl is-active --quiet httpd 2>/dev/null && \
       ! systemctl is-active --quiet apache2 2>/dev/null; then
        return
    fi

    echo -e "  ├── Apache: 실행 중 ${GREEN}✅${NC}"

    local apache_conf=""
    [[ -f /etc/httpd/conf/httpd.conf ]] && apache_conf="/etc/httpd/conf/httpd.conf"
    [[ -f /etc/apache2/apache2.conf ]] && apache_conf="/etc/apache2/apache2.conf"

    if [[ -n "$apache_conf" ]]; then
        # Check KeepAlive
        local keepalive=$(grep -E "^KeepAlive " "$apache_conf" 2>/dev/null | awk '{print $2}' || echo "On")
        if [[ "$keepalive" == "On" ]]; then
            echo -e "  │   ├── KeepAlive: On ${GREEN}✅${NC}"
        else
            echo -e "  │   ├── KeepAlive: Off ${YELLOW}⚠️${NC}"
            add_recommendation "서비스" "Apache KeepAlive 활성화 권장"
            deduct_score services 3
        fi

        # Check MPM
        local mpm=$(apachectl -V 2>/dev/null | grep "MPM" | awk '{print $3}' || echo "prefork")
        if [[ "$mpm" == "prefork" ]]; then
            echo -e "  │   └── MPM: prefork ${YELLOW}⚠️ event 권장${NC}"
            add_recommendation "서비스" "Apache MPM을 event로 변경 권장"
            deduct_score services 3
        else
            echo -e "  │   └── MPM: ${mpm} ${GREEN}✅${NC}"
        fi
    fi
}

check_nginx() {
    if ! systemctl is-active --quiet nginx 2>/dev/null; then
        return
    fi

    echo -e "  ├── Nginx: 실행 중 ${GREEN}✅${NC}"

    local nginx_conf="/etc/nginx/nginx.conf"
    if [[ -f "$nginx_conf" ]]; then
        local worker_processes=$(grep "worker_processes" "$nginx_conf" 2>/dev/null | awk '{print $2}' | tr -d ';')
        echo -e "  │   └── worker_processes: ${worker_processes:-auto}"
    fi
}

check_mysql() {
    if ! systemctl is-active --quiet mysqld 2>/dev/null && \
       ! systemctl is-active --quiet mariadb 2>/dev/null && \
       ! systemctl is-active --quiet mysql 2>/dev/null; then
        return
    fi

    echo -e "  ├── MySQL/MariaDB: 실행 중 ${GREEN}✅${NC}"

    # Check buffer pool size
    local buffer_pool=$(mysql -N -e "SHOW VARIABLES LIKE 'innodb_buffer_pool_size'" 2>/dev/null | awk '{print $2}')
    if [[ -n "$buffer_pool" ]]; then
        local buffer_mb=$((buffer_pool / 1024 / 1024))
        local total_mem=$(free -m | awk '/Mem:/ {print $2}')
        local recommended=$((total_mem * 70 / 100))

        if [[ $buffer_mb -lt $((recommended / 2)) ]]; then
            echo -e "  │   ├── 버퍼 풀: ${buffer_mb}MB ${YELLOW}⚠️ (${recommended}MB 권장)${NC}"
            add_recommendation "서비스" "MySQL 버퍼 풀 증가 권장 (${recommended}MB)"
            deduct_score services 5
        else
            echo -e "  │   ├── 버퍼 풀: ${buffer_mb}MB ${GREEN}✅${NC}"
        fi
    fi
}

check_php_fpm() {
    if ! systemctl is-active --quiet php-fpm 2>/dev/null && \
       ! systemctl list-units --type=service 2>/dev/null | grep -q "php.*fpm.*running"; then
        return
    fi

    echo -e "  └── PHP-FPM: 실행 중 ${GREEN}✅${NC}"

    # Find PHP-FPM config
    local php_conf=$(find /etc/php* -name "www.conf" 2>/dev/null | head -1)
    if [[ -f "$php_conf" ]]; then
        local max_children=$(grep "pm.max_children" "$php_conf" 2>/dev/null | grep -oE '[0-9]+' | head -1)
        if [[ -n "$max_children" && $max_children -lt 10 ]]; then
            echo -e "      └── pm.max_children: ${max_children} ${YELLOW}⚠️ (10+ 권장)${NC}"
            add_recommendation "서비스" "PHP-FPM pm.max_children 증가 권장"
            deduct_score services 3
        fi
    fi
}

run_services_checks() {
    echo -e "\n${BOLD}  🌐 서비스 진단${NC}"
    echo -e "  ────────────────────────────────────────────────────────────"

    check_apache
    check_nginx
    check_mysql
    check_php_fpm
}

# ============================================================
# Filesystem Checks
# ============================================================
check_log_files() {
    local log_size=$(du -sh /var/log 2>/dev/null | awk '{print $1}')
    echo -e "  ├── /var/log: ${log_size:-N/A} ${GREEN}✅${NC}"

    # Count old log files
    local old_logs=$(find /var/log -name "*.log" -mtime +${OLD_LOG_DAYS} 2>/dev/null | wc -l)
    local old_size=$(find /var/log -name "*.log" -mtime +${OLD_LOG_DAYS} -exec du -ch {} + 2>/dev/null | tail -1 | awk '{print $1}')

    if [[ $old_logs -gt 0 ]]; then
        echo -e "  ├── ${OLD_LOG_DAYS}일 이상 로그: ${old_logs}개 파일 (${old_size:-0})"
        echo -e "  │   └── ${DIM}💡 권장: auto-health --cleanup logs${NC}"
        add_recommendation "파일시스템" "오래된 로그 파일 정리 권장" true
        deduct_score filesystem 3
    fi
}

check_large_files() {
    local large_files=$(find / -xdev -type f -size +${LARGE_FILE_THRESHOLD} 2>/dev/null | wc -l)

    if [[ $large_files -gt 0 ]]; then
        echo -e "  └── 대용량 파일: ${large_files}개 발견 (>${LARGE_FILE_THRESHOLD})"
    else
        echo -e "  └── 대용량 파일: 없음 ${GREEN}✅${NC}"
    fi
}

check_inode_usage() {
    local inode_usage=$(df -i / 2>/dev/null | tail -1 | awk '{print $5}' | tr -d '%')

    if [[ $inode_usage -ge 80 ]]; then
        echo -e "  ├── inode 사용률: ${inode_usage}% ${YELLOW}⚠️${NC}"
        add_recommendation "파일시스템" "inode 사용률이 높음"
        deduct_score filesystem 5
    fi
}

run_filesystem_checks() {
    echo -e "\n${BOLD}  📁 파일시스템 진단${NC}"
    echo -e "  ────────────────────────────────────────────────────────────"

    check_log_files
    check_large_files
}

# ============================================================
# Auto-Fix Functions
# ============================================================
fix_ssh() {
    local sshd_config="/etc/ssh/sshd_config"
    [[ ! -f "$sshd_config" ]] && return

    echo -e "${BLUE}[FIX]${NC} SSH 설정 수정 중..."

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "${YELLOW}[DRY-RUN]${NC} 실제 수정하지 않음"
        return
    fi

    # Backup
    cp "$sshd_config" "${sshd_config}.bak.$(date +%Y%m%d)"

    # Disable root login
    if grep -q "^PermitRootLogin" "$sshd_config"; then
        sed -i 's/^PermitRootLogin.*/PermitRootLogin no/' "$sshd_config"
    else
        echo "PermitRootLogin no" >> "$sshd_config"
    fi

    echo -e "${GREEN}✓${NC} PermitRootLogin no 설정됨"

    # Restart SSH
    systemctl reload sshd 2>/dev/null || systemctl reload ssh 2>/dev/null

    echo -e "${GREEN}✓${NC} SSH 서비스 재시작됨"
}

fix_firewall() {
    echo -e "${BLUE}[FIX]${NC} 방화벽 설정 중..."

    if [[ "$DRY_RUN" == "true" ]]; then
        echo -e "${YELLOW}[DRY-RUN]${NC} 실제 수정하지 않음"
        return
    fi

    if check_command firewall-cmd; then
        systemctl enable --now firewalld
        firewall-cmd --permanent --add-service=ssh
        firewall-cmd --permanent --add-service=http
        firewall-cmd --permanent --add-service=https
        firewall-cmd --reload
        echo -e "${GREEN}✓${NC} firewalld 활성화됨"
    elif check_command ufw; then
        ufw allow ssh
        ufw allow http
        ufw allow https
        ufw --force enable
        echo -e "${GREEN}✓${NC} ufw 활성화됨"
    fi
}

run_fixes() {
    echo -e "\n${BOLD}🔧 자동 수정${NC}\n"

    case "$FIX_TARGET" in
        ssh)      fix_ssh ;;
        firewall) fix_firewall ;;
        "")
            fix_ssh
            fix_firewall
            ;;
        *)
            echo -e "${RED}알 수 없는 수정 대상: $FIX_TARGET${NC}"
            ;;
    esac

    echo -e "\n${GREEN}✓${NC} 자동 수정 완료"
}

# ============================================================
# Report Generation
# ============================================================
print_summary() {
    local total_score=$(( (SECURITY_SCORE + PERFORMANCE_SCORE + SERVICES_SCORE + FILESYSTEM_SCORE) / 4 ))

    # Ensure scores don't go below 0
    [[ $SECURITY_SCORE -lt 0 ]] && SECURITY_SCORE=0
    [[ $PERFORMANCE_SCORE -lt 0 ]] && PERFORMANCE_SCORE=0
    [[ $SERVICES_SCORE -lt 0 ]] && SERVICES_SCORE=0
    [[ $FILESYSTEM_SCORE -lt 0 ]] && FILESYSTEM_SCORE=0
    [[ $total_score -lt 0 ]] && total_score=0

    echo -e "\n${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📊 종합 점수: %-3d / 100                                      ║${NC}\n" "$total_score"
    echo -e "${BOLD}║                                                              ║${NC}"

    printf "║     보안: %3d  |  성능: %3d  |  서비스: %3d  |  파일: %3d    ║\n" \
        "$SECURITY_SCORE" "$PERFORMANCE_SCORE" "$SERVICES_SCORE" "$FILESYSTEM_SCORE"

    if [[ ${#RECOMMENDATIONS[@]} -gt 0 ]]; then
        echo -e "${BOLD}║                                                              ║${NC}"
        echo -e "${BOLD}║  📋 권장사항 요약 (${#RECOMMENDATIONS[@]}개)                                      ║${NC}"
        echo -e "${BOLD}║  ────────────────────────────────────────────────────────────║${NC}"

        local count=0
        for rec in "${RECOMMENDATIONS[@]}"; do
            ((count++))
            [[ $count -gt 5 ]] && break
            printf "║  %d. %-56s║\n" "$count" "${rec:0:56}"
        done
    fi

    if [[ ${#FIXABLE_ITEMS[@]} -gt 0 ]]; then
        echo -e "${BOLD}║                                                              ║${NC}"
        printf "${BOLD}║  🔧 자동 수정 가능: %d개                                      ║${NC}\n" "${#FIXABLE_ITEMS[@]}"
        echo -e "${BOLD}║  실행: auto-health --fix                                     ║${NC}"
    fi

    echo -e "${BOLD}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo
}

# ============================================================
# Main
# ============================================================
main() {
    load_config

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --quick)       QUICK_MODE=true ;;
            --full)        FULL_MODE=true ;;
            --security)    CHECK_PERFORMANCE=false; CHECK_SERVICES=false; CHECK_FILESYSTEM=false ;;
            --performance) CHECK_SECURITY=false; CHECK_SERVICES=false; CHECK_FILESYSTEM=false ;;
            --services)    CHECK_SECURITY=false; CHECK_PERFORMANCE=false; CHECK_FILESYSTEM=false ;;
            --filesystem)  CHECK_SECURITY=false; CHECK_PERFORMANCE=false; CHECK_SERVICES=false ;;
            --fix)
                FIX_MODE=true
                shift
                [[ -n "$1" && "$1" != -* ]] && FIX_TARGET="$1"
                continue
                ;;
            --dry-run)     DRY_RUN=true ;;
            --report)
                shift
                REPORT_FORMAT="${1:-terminal}"
                ;;
            --schedule)
                shift
                # Would implement scheduling
                echo "Scheduling: $1"
                exit 0
                ;;
            --cleanup)
                shift
                echo "Cleanup: $1"
                exit 0
                ;;
            -v|--verbose)  VERBOSE=true ;;
            -h|--help)     print_help; exit 0 ;;
            --version)     echo "AUTO-HEALTH v${VERSION}"; exit 0 ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
        shift
    done

    # Run fixes if requested
    if [[ "$FIX_MODE" == "true" ]]; then
        run_fixes
        exit 0
    fi

    # Run diagnostics
    print_banner

    echo -e "${BOLD}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║  🏥 AUTO-HEALTH v${VERSION} - 서버 건강 진단                        ║${NC}"
    echo -e "${BOLD}║  🤟 WIA AUTO-HEALTH · https://wiastandards.com/auto-health    ║${NC}"
    echo -e "${BOLD}║  弘益人間 · Benefit All Humanity                              ║${NC}"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${BOLD}║  📍 서버: %-49s║${NC}\n" "$HOSTNAME"
    printf "${BOLD}║  📅 진단: %-49s║${NC}\n" "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "${BOLD}╠══════════════════════════════════════════════════════════════╣${NC}"

    [[ "$CHECK_SECURITY" == "true" ]] && run_security_checks
    [[ "$CHECK_PERFORMANCE" == "true" ]] && run_performance_checks
    [[ "$CHECK_SERVICES" == "true" ]] && run_services_checks
    [[ "$CHECK_FILESYSTEM" == "true" ]] && run_filesystem_checks

    print_summary

    # Log results
    log "Health check completed - Score: $((SECURITY_SCORE + PERFORMANCE_SCORE + SERVICES_SCORE + FILESYSTEM_SCORE))/400"
}

main "$@"
