#!/bin/bash

################################################################################
# WIA-CITY-015: Access Control System Standard - Installation Script
#
# 弘익人間 (홍익인간) - Benefit All Humanity
#
# This script installs dependencies and sets up the WIA-CITY-015
# Access Control System Standard SDK and CLI tools.
#
# 사용법:
#   ./install.sh              # 모두 설치
#   ./install.sh --sdk-only   # SDK만 설치
#   ./install.sh --cli-only   # CLI만 설치
#
# Version: 1.0.0
# License: MIT
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/api/typescript"
CLI_DIR="${SCRIPT_DIR}/cli"

# Parse command line arguments
INSTALL_SDK=true
INSTALL_CLI=true

while [[ $# -gt 0 ]]; do
  case $1 in
    --sdk-only)
      INSTALL_CLI=false
      shift
      ;;
    --cli-only)
      INSTALL_SDK=false
      shift
      ;;
    --help)
      echo "WIA-CITY-015 설치 스크립트"
      echo ""
      echo "사용법:"
      echo "  ./install.sh              모두 설치"
      echo "  ./install.sh --sdk-only   SDK만 설치"
      echo "  ./install.sh --cli-only   CLI만 설치"
      echo "  ./install.sh --help       도움말 표시"
      exit 0
      ;;
    *)
      echo -e "${RED}알 수 없는 옵션: $1${NC}"
      exit 1
      ;;
  esac
done

################################################################################
# Helper Functions
################################################################################

print_header() {
  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║                                                                ║"
  echo "║         🔐  WIA-CITY-015: 출입 통제 시스템 표준             ║"
  echo "║                                                                ║"
  echo "║              弘益人間 · Benefit All Humanity                   ║"
  echo "║                                                                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"
}

print_step() {
  echo -e "${GREEN}[✓]${NC} $1"
}

print_info() {
  echo -e "${BLUE}[ℹ]${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
  echo -e "${RED}[✗]${NC} $1"
}

check_command() {
  if command -v "$1" &> /dev/null; then
    return 0
  else
    return 1
  fi
}

################################################################################
# Pre-flight Checks
################################################################################

print_header

echo -e "${BLUE}사전 확인 중...${NC}"
echo ""

# Check for Node.js
if $INSTALL_SDK; then
  if check_command node; then
    NODE_VERSION=$(node --version)
    print_step "Node.js 발견: ${NODE_VERSION}"
  else
    print_error "Node.js를 찾을 수 없습니다. https://nodejs.org/ 에서 Node.js 18+ 을 설치하세요."
    exit 1
  fi

  # Check for npm
  if check_command npm; then
    NPM_VERSION=$(npm --version)
    print_step "npm 발견: v${NPM_VERSION}"
  else
    print_error "npm을 찾을 수 없습니다. npm을 설치하세요."
    exit 1
  fi
fi

# Check for bash
if $INSTALL_CLI; then
  if check_command bash; then
    BASH_VERSION=$(bash --version | head -n1)
    print_step "Bash 발견: ${BASH_VERSION}"
  else
    print_error "Bash를 찾을 수 없습니다. Bash 4+ 를 설치하세요."
    exit 1
  fi

  # Check for jq (optional but recommended)
  if check_command jq; then
    JQ_VERSION=$(jq --version)
    print_step "jq 발견: ${JQ_VERSION}"
  else
    print_warning "jq를 찾을 수 없습니다. JSON 파싱에 기본 방법을 사용합니다."
    print_info "더 나은 JSON 지원을 위해 jq를 설치하세요: https://stedolan.github.io/jq/"
  fi

  # Check for curl
  if check_command curl; then
    CURL_VERSION=$(curl --version | head -n1)
    print_step "curl 발견: ${CURL_VERSION}"
  else
    print_error "curl을 찾을 수 없습니다. curl을 설치하세요."
    exit 1
  fi
fi

echo ""

################################################################################
# Install SDK
################################################################################

if $INSTALL_SDK; then
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo -e "${BLUE}TypeScript SDK 설치 중...${NC}"
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo ""

  cd "$SDK_DIR"

  print_info "npm 의존성 설치 중..."
  npm install

  echo ""
  print_info "SDK 빌드 중..."
  npm run build

  echo ""
  print_step "SDK 설치 완료!"
  echo ""
  print_info "프로젝트에서 SDK를 사용하려면:"
  echo "  npm install ${SDK_DIR}"
  echo ""
  echo "  import { AccessControlSDK } from '@wia/city-access-control';"
  echo ""

  cd "$SCRIPT_DIR"
fi

################################################################################
# Install CLI
################################################################################

if $INSTALL_CLI; then
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo -e "${BLUE}CLI 도구 설치 중...${NC}"
  echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
  echo ""

  # Make CLI script executable
  chmod +x "${CLI_DIR}/access-control-system.sh"
  print_step "CLI 스크립트 실행 권한 부여 완료"

  # Create symbolic link (optional)
  if [ -w "/usr/local/bin" ]; then
    ln -sf "${CLI_DIR}/access-control-system.sh" /usr/local/bin/wia-access
    print_step "심볼릭 링크 생성: /usr/local/bin/wia-access"
    echo ""
    print_info "이제 CLI를 다음과 같이 사용할 수 있습니다:"
    echo "  wia-access dashboard BLD-001"
  else
    print_warning "/usr/local/bin 에 심볼릭 링크를 생성할 수 없습니다 (권한 부족)"
    print_info "CLI를 다음과 같이 사용할 수 있습니다:"
    echo "  ${CLI_DIR}/access-control-system.sh dashboard BLD-001"
  fi

  echo ""
  print_step "CLI 설치 완료!"
  echo ""
fi

################################################################################
# Configuration
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}설정${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_info "API 인증 정보를 설정하려면 다음을 실행하세요:"
echo "  ${CLI_DIR}/access-control-system.sh config"
echo ""
print_info "또는 환경 변수를 설정하세요:"
echo "  export WIA_API_KEY=your-api-key"
echo "  export WIA_API_ENDPOINT=https://api.wia.org/city-015/v1"
echo ""

################################################################################
# Documentation
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}문서${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_info "README: ${SCRIPT_DIR}/README.md"
print_info "명세서: ${SCRIPT_DIR}/spec/WIA-CITY-015-v1.0.md"
print_info "웹사이트: https://wia.org/standards/city-015"
echo ""

################################################################################
# Quick Start Examples
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}빠른 시작 예시${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if $INSTALL_CLI; then
  echo -e "${BLUE}CLI 예시:${NC}"
  echo ""
  echo "  # 실시간 대시보드 보기"
  echo "  ${CLI_DIR}/access-control-system.sh dashboard BLD-001"
  echo ""
  echo "  # 접근 권한 부여"
  echo "  ${CLI_DIR}/access-control-system.sh grant-access EMP-001 SERVER-ROOM"
  echo ""
  echo "  # 출입구 원격 개방"
  echo "  ${CLI_DIR}/access-control-system.sh unlock AP-MAIN-001 10"
  echo ""
  echo "  # 방문자 체크인"
  echo "  ${CLI_DIR}/access-control-system.sh check-in VIS-20251225-001"
  echo ""
  echo "  # 주차장 현황"
  echo "  ${CLI_DIR}/access-control-system.sh parking-status PL-001"
  echo ""
fi

if $INSTALL_SDK; then
  echo -e "${BLUE}TypeScript SDK 예시:${NC}"
  echo ""
  cat << 'EOF'
  import { AccessControlSDK } from '@wia/city-access-control';

  const sdk = new AccessControlSDK({
    apiKey: process.env.WIA_API_KEY,
    endpoint: 'https://api.wia.org/city-015/v1'
  });

  // 실시간 대시보드 조회
  const dashboard = await sdk.getRealtimeDashboard('BLD-001');
  console.log('현재 재실:', dashboard.data.occupancy.current, '명');

  // 접근 권한 부여
  await sdk.grantAccess({
    userId: 'EMP-001',
    accessZones: ['SERVER-ROOM'],
    validFrom: '2025-12-25T00:00:00+09:00'
  });

  // 방문자 등록
  const visitor = await sdk.registerVisitor({
    name: '홍길동',
    company: 'ABC 주식회사',
    visitDate: '2025-12-25',
    host: { employeeId: 'EMP-001', name: '김철수', ... }
  });
EOF
  echo ""
fi

################################################################################
# Completion
################################################################################

echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}설치 완료! ✓${NC}"
echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

print_step "WIA-CITY-015 출입 통제 시스템 표준을 사용할 준비가 되었습니다!"
echo ""
echo -e "${BLUE}弘益人間 (홍익인간) - Benefit All Humanity${NC}"
echo ""
echo -e "${BLUE}© 2025 SmileStory Inc. / WIA${NC}"
echo -e "${BLUE}License: MIT${NC}"
echo ""
