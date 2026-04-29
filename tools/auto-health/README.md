# AUTO-HEALTH v1.0

<p align="center">
  <b>🏥 Server Health Diagnostics</b><br>
  弘益人間 (홍익인간) · Benefit All Humanity
</p>

---

## 🎯 Overview

AUTO-HEALTH는 서버 건강 진단을 "원클릭"으로 수행하는 bash 스크립트입니다.
보안 취약점, 성능 문제, 권장사항을 자동 분석!

## ✨ Features

- **🔒 보안 진단** - SSH, 방화벽, fail2ban, SELinux, 보안 업데이트
- **💻 성능 진단** - CPU, 메모리, 스왑, 디스크, 부하
- **🌐 서비스 진단** - Apache, Nginx, MySQL, PHP-FPM 최적화
- **📁 파일시스템** - 로그 파일, 대용량 파일, inode
- **🔧 자동 수정** - SSH 설정, 방화벽 활성화
- **📊 점수 리포트** - 100점 만점 종합 점수

## 🚀 Quick Install

```bash
curl -sSL https://wia.family/health/install | sudo bash
```

## 📖 Usage

```bash
# 기본 진단
auto-health

# 빠른 진단
auto-health --quick

# 카테고리별 진단
auto-health --security
auto-health --performance
auto-health --services

# 자동 수정
auto-health --fix --dry-run    # 시뮬레이션
auto-health --fix              # 실제 수정
auto-health --fix ssh          # SSH만 수정
```

## 📊 Output Example

```
╔══════════════════════════════════════════════════════════════╗
║  🏥 AUTO-HEALTH v1.0 - 서버 건강 진단                        ║
╠══════════════════════════════════════════════════════════════╣
║  📍 서버: cqm.wiastandards.com                               ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  🔒 보안 진단                                    점수: 75/100║
║  ├── SSH 포트: 22 ⚠️ 기본 포트 사용 중                      ║
║  ├── SSH 루트 로그인: 허용 ⚠️                               ║
║  ├── 방화벽: firewalld 활성 ✅                              ║
║  └── SELinux: Enforcing ✅                                  ║
║                                                              ║
║  💻 성능 진단                                    점수: 92/100║
║  ├── CPU 사용률: 23% ✅                                     ║
║  ├── 메모리: 67% ✅                                         ║
║  └── 디스크: 54% ✅                                         ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  📊 종합 점수: 87.5 / 100                                    ║
║  🔧 자동 수정 가능: 3개                                      ║
╚══════════════════════════════════════════════════════════════╝
```

## 📜 License

MIT License

---

<p align="center">
  <b>弘益人間 (홍익인간)</b><br>
  © 2025 SmileStory Inc. / WIA
</p>
