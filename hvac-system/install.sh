#!/bin/bash

# WIA-CITY-010: HVAC System Standard - Installation Script
# 弘益人間 (홍익인간) - Benefit All Humanity
#
# This script installs the WIA-CITY-010 HVAC System Standard
# and its dependencies.

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

# Main installation
main() {
    print_header "WIA-CITY-010: 냉난방 시스템 표준 설치"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""

    # Check if running from correct directory
    if [ ! -f "spec/WIA-CITY-010-v1.0.md" ]; then
        print_error "설치 스크립트를 hvac-system 디렉토리에서 실행해주세요."
        exit 1
    fi

    # Check Node.js installation
    print_info "Node.js 설치 확인 중..."
    if ! command -v node &> /dev/null; then
        print_error "Node.js가 설치되어 있지 않습니다."
        print_info "https://nodejs.org 에서 Node.js를 설치해주세요."
        exit 1
    fi
    NODE_VERSION=$(node -v)
    print_success "Node.js 발견: $NODE_VERSION"

    # Check npm installation
    print_info "npm 설치 확인 중..."
    if ! command -v npm &> /dev/null; then
        print_error "npm이 설치되어 있지 않습니다."
        exit 1
    fi
    NPM_VERSION=$(npm -v)
    print_success "npm 발견: $NPM_VERSION"

    # Install TypeScript API dependencies
    print_info "TypeScript API 의존성 설치 중..."
    cd api/typescript
    if [ -f "package.json" ]; then
        npm install
        print_success "TypeScript API 의존성 설치 완료"
    else
        print_error "package.json을 찾을 수 없습니다."
        exit 1
    fi
    cd ../..

    # Make CLI executable
    print_info "CLI 도구 실행 권한 설정 중..."
    if [ -f "cli/hvac-system.sh" ]; then
        chmod +x cli/hvac-system.sh
        print_success "CLI 도구 실행 권한 설정 완료"
    else
        print_error "CLI 스크립트를 찾을 수 없습니다."
        exit 1
    fi

    # Create symlink (optional)
    print_info "전역 CLI 도구 설정 (선택사항)..."
    read -p "전역 CLI 도구를 설정하시겠습니까? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        if [ -w "/usr/local/bin" ]; then
            ln -sf "$SCRIPT_DIR/cli/hvac-system.sh" /usr/local/bin/wia-hvac
            print_success "전역 CLI 도구 설정 완료: wia-hvac"
        else
            print_error "권한이 없습니다. sudo를 사용해주세요."
            print_info "수동 설정: sudo ln -s $SCRIPT_DIR/cli/hvac-system.sh /usr/local/bin/wia-hvac"
        fi
    fi

    # Installation complete
    echo ""
    print_header "설치 완료!"
    echo ""
    print_success "WIA-CITY-010 냉난방 시스템 표준이 성공적으로 설치되었습니다."
    echo ""
    echo "시작하기:"
    echo "  1. 명세서 읽기:  cat spec/WIA-CITY-010-v1.0.md"
    echo "  2. CLI 도구:     ./cli/hvac-system.sh --help"
    echo "  3. TypeScript:   cd api/typescript && npm run build"
    echo ""
    echo "❄️ 효율적인 냉난방, 쾌적한 환경!"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""
}

# Run installation
main "$@"
