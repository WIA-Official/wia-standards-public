#!/bin/bash
#═══════════════════════════════════════════════════════════════
#
#  WIA AUTO-TOOLS 통합 설치 스크립트
#
#  World Certification Industry Association (WIA)
#  https://wia.family
#
#  弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
#
#  Usage: curl -sSL https://wia.family/tools | sudo bash
#
#═══════════════════════════════════════════════════════════════

set -e

VERSION="1.0.0"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GITHUB_RAW="https://raw.githubusercontent.com/WIA-Official/ISP/main"

# ============================================================
# Colors
# ============================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

# ============================================================
# Available Tools
# ============================================================
declare -A TOOLS
TOOLS[auto-secure]="SSL 인증서 자동화 (Let's Encrypt)"
TOOLS[auto-backup]="서버 백업 자동화 (웹, DB, 설정)"
TOOLS[auto-deploy]="Zero-Downtime 배포"
TOOLS[auto-monitor]="서버 모니터링"
TOOLS[auto-health]="서버 건강 진단"

# ============================================================
# Banner
# ============================================================
print_banner() {
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║  ██╗    ██╗██╗ █████╗                                        ║
║  ██║    ██║██║██╔══██╗                                       ║
║  ██║ █╗ ██║██║███████║                                       ║
║  ██║███╗██║██║██╔══██║                                       ║
║  ╚███╔███╔╝██║██║  ██║                                       ║
║   ╚══╝╚══╝ ╚═╝╚═╝  ╚═╝                                       ║
║                                                              ║
║   █████╗ ██╗   ██╗████████╗ ██████╗                          ║
║  ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗                         ║
║  ███████║██║   ██║   ██║   ██║   ██║                         ║
║  ██╔══██║██║   ██║   ██║   ██║   ██║                         ║
║  ██║  ██║╚██████╔╝   ██║   ╚██████╔╝                         ║
║  ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝                          ║
║                                                              ║
║  ████████╗ ██████╗  ██████╗ ██╗     ███████╗                 ║
║  ╚══██╔══╝██╔═══██╗██╔═══██╗██║     ██╔════╝                 ║
║     ██║   ██║   ██║██║   ██║██║     ███████╗                 ║
║     ██║   ██║   ██║██║   ██║██║     ╚════██║                 ║
║     ██║   ╚██████╔╝╚██████╔╝███████╗███████║                 ║
║     ╚═╝    ╚═════╝  ╚═════╝ ╚══════╝╚══════╝                 ║
║                                                              ║
║  WIA AUTO-TOOLS v1.0 · 통합 설치 스크립트                    ║
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
${BOLD}WIA AUTO-TOOLS v${VERSION}${NC} - 통합 설치 스크립트

${BOLD}USAGE:${NC}
    install.sh [OPTIONS] [TOOLS...]

${BOLD}OPTIONS:${NC}
    --all               모든 도구 설치 (기본값)
    --list              사용 가능한 도구 목록
    --uninstall TOOL    도구 제거
    -h, --help          도움말

${BOLD}AVAILABLE TOOLS:${NC}
    auto-secure         SSL 인증서 자동화 (Let's Encrypt)
    auto-backup         서버 백업 자동화 (웹, DB, 설정)
    auto-deploy         Zero-Downtime 배포
    auto-monitor        서버 모니터링
    auto-health         서버 건강 진단

${BOLD}EXAMPLES:${NC}
    # 모든 도구 설치
    ./install.sh --all

    # 특정 도구만 설치
    ./install.sh auto-secure auto-backup

    # 원클릭 설치 (curl)
    curl -sSL https://wia.family/tools | sudo bash

${BOLD}AFTER INSTALLATION:${NC}
    auto-secure --help
    auto-backup --help
    auto-deploy --help
    auto-monitor --help
    auto-health --help

${DIM}弘益人間 (홍익인간) · https://wia.family${NC}
EOF
}

# ============================================================
# Utility Functions
# ============================================================
log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[!]${NC} $1"; }
log_error()   { echo -e "${RED}[✗]${NC} $1"; }

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This installer must be run as root. Please use: sudo $0"
        exit 1
    fi
}

# ============================================================
# Tool Installation
# ============================================================
install_tool() {
    local tool="$1"
    local tool_dir="${SCRIPT_DIR}/${tool}"
    local install_dir="/opt/${tool}"

    echo ""
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    log_info "설치 중: ${BOLD}${tool}${NC}"
    echo -e "         ${DIM}${TOOLS[$tool]}${NC}"
    echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    # Check if local install script exists
    if [[ -f "${tool_dir}/install.sh" ]]; then
        log_info "로컬 설치 스크립트 사용"
        bash "${tool_dir}/install.sh"
    else
        # Download and install from GitHub
        log_info "GitHub에서 다운로드 중..."

        mkdir -p "$install_dir"

        curl -sSL "${GITHUB_RAW}/${tool}/${tool}.sh" -o "${install_dir}/${tool}.sh" 2>/dev/null || {
            # Try alternative naming
            local main_script="${tool#auto-}"
            curl -sSL "${GITHUB_RAW}/${tool}/auto-${main_script}.sh" -o "${install_dir}/${tool}.sh" 2>/dev/null || {
                log_warn "${tool} 다운로드 실패 - 건너뜀"
                return 1
            }
        }

        chmod +x "${install_dir}/${tool}.sh"
        ln -sf "${install_dir}/${tool}.sh" "/usr/local/bin/${tool}"

        log_success "${tool} 설치 완료"
    fi

    return 0
}

uninstall_tool() {
    local tool="$1"
    local install_dir="/opt/${tool}"

    log_info "제거 중: ${tool}"

    rm -rf "$install_dir"
    rm -f "/usr/local/bin/${tool}"

    log_success "${tool} 제거 완료"
}

list_tools() {
    echo ""
    echo -e "${BOLD}사용 가능한 도구:${NC}"
    echo ""

    for tool in "${!TOOLS[@]}"; do
        local installed=""
        [[ -f "/usr/local/bin/${tool}" ]] && installed="${GREEN}[설치됨]${NC}"

        printf "  ${CYAN}%-15s${NC} %s %s\n" "$tool" "${TOOLS[$tool]}" "$installed"
    done

    echo ""
}

# ============================================================
# Print Summary
# ============================================================
print_summary() {
    local installed=("$@")

    echo ""
    echo -e "${GREEN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                                                              ║"
    echo "║  ✅ WIA AUTO-TOOLS 설치 완료!                                ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  설치된 도구:                                                ║"

    for tool in "${installed[@]}"; do
        printf "║    ✓ %-54s ║\n" "$tool"
    done

    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  사용법:                                                     ║"
    echo "║    auto-secure --help     # SSL 자동화                       ║"
    echo "║    auto-backup --help     # 백업 자동화                      ║"
    echo "║    auto-deploy --help     # 배포 자동화                      ║"
    echo "║    auto-monitor --help    # 모니터링                         ║"
    echo "║    auto-health --help     # 건강 진단                        ║"
    echo "║                                                              ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  弘益人間 (홍익인간) · Benefit All Humanity                  ║"
    echo "║  WIA Standards · https://wia.family                          ║"
    echo "║                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# ============================================================
# Main
# ============================================================
main() {
    local install_all=false
    local tools_to_install=()
    local uninstall_mode=false
    local uninstall_target=""

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --all)
                install_all=true
                ;;
            --list)
                list_tools
                exit 0
                ;;
            --uninstall)
                uninstall_mode=true
                shift
                uninstall_target="$1"
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            auto-*)
                tools_to_install+=("$1")
                ;;
            *)
                log_warn "알 수 없는 옵션: $1"
                ;;
        esac
        shift
    done

    # Print banner
    print_banner

    # Check root
    check_root

    # Uninstall mode
    if [[ "$uninstall_mode" == "true" ]]; then
        if [[ -n "$uninstall_target" ]]; then
            uninstall_tool "$uninstall_target"
        else
            log_error "제거할 도구를 지정하세요"
        fi
        exit 0
    fi

    # Default to install all if no specific tools specified
    if [[ ${#tools_to_install[@]} -eq 0 ]] || [[ "$install_all" == "true" ]]; then
        tools_to_install=("auto-secure" "auto-backup" "auto-deploy" "auto-monitor" "auto-health")
    fi

    # Install tools
    local installed=()

    for tool in "${tools_to_install[@]}"; do
        if install_tool "$tool"; then
            installed+=("$tool")
        fi
    done

    # Print summary
    if [[ ${#installed[@]} -gt 0 ]]; then
        print_summary "${installed[@]}"
    else
        log_error "설치된 도구가 없습니다"
        exit 1
    fi
}

main "$@"
