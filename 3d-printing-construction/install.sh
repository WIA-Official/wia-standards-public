#!/bin/bash

# WIA-CITY-008: 3D Printing Construction Standard - Installation Script
# 弘익人間 (홍익인간) - Benefit All Humanity
#
# This script installs the WIA-CITY-008 3D Printing Construction Standard
# and its dependencies.

set -e

# Colors for output
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
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

print_printer() {
    echo -e "${MAGENTA}🖨️${NC} $1"
}

# Main installation
main() {
    print_header "WIA-CITY-008: 3D 프린팅 건설 표준 설치"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""

    # Check if running from correct directory
    if [ ! -f "spec/WIA-CITY-008-v1.0.md" ]; then
        print_error "설치 스크립트를 3d-printing-construction 디렉토리에서 실행해주세요."
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
    if [ -f "cli/3d-printing-construction.sh" ]; then
        chmod +x cli/3d-printing-construction.sh
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
            ln -sf "$SCRIPT_DIR/cli/3d-printing-construction.sh" /usr/local/bin/wia-3dprint
            print_success "전역 CLI 도구 설정 완료: wia-3dprint"
        else
            print_error "권한이 없습니다. sudo를 사용해주세요."
            print_info "수동 설정: sudo ln -s $SCRIPT_DIR/cli/3d-printing-construction.sh /usr/local/bin/wia-3dprint"
        fi
    fi

    # Installation complete
    echo ""
    print_header "설치 완료!"
    echo ""
    print_success "WIA-CITY-008 3D 프린팅 건설 표준이 성공적으로 설치되었습니다."
    echo ""
    echo "시작하기:"
    echo "  1. 명세서 읽기:  cat spec/WIA-CITY-008-v1.0.md"
    echo "  2. CLI 도구:     ./cli/3d-printing-construction.sh --help"
    echo "  3. TypeScript:   cd api/typescript && npm run build"
    echo ""
    print_printer "3D 프린팅으로 더 빠르고 저렴하게! 지속가능한 건설의 미래를 함께 만들어갑니다."
    echo ""
    echo "프린터 종류:"
    echo "  🖨️ Gantry (갠트리): 고정 레일 시스템, 대형 건물 프린팅"
    echo "  🖨️ Robotic Arm (로봇 암): 유연한 동작, 복잡한 형상"
    echo "  🖨️ WASP: 이탈리아 WASP사의 델타 프린터"
    echo "  🖨️ Crane (크레인): 초대형 건물 프린팅"
    echo ""
    echo "재료 종류:"
    echo "  🏗️ Concrete (콘크리트): 전통 시멘트 기반"
    echo "  🏗️ Geopolymer (지오폴리머): 친환경 대체재, CO2 80% 감소"
    echo "  🏗️ Clay (점토): 천연 재료, 100% 재활용 가능"
    echo "  🏗️ Fiber-Reinforced (섬유 보강): 강도 향상"
    echo ""
    echo "주요 장점:"
    echo "  ⚡ 건설 시간 50-70% 단축"
    echo "  💰 건설 비용 30-50% 절감"
    echo "  🌍 CO2 배출 40-60% 감소"
    echo "  ♻️ 재료 낭비 60% 감소"
    echo "  🏘️ 저가 주택, 긴급 대피소 신속 건설"
    echo ""
    echo "弘益人間 (홍익인간) - Benefit All Humanity"
    echo ""
}

# Run installation
main "$@"
