# AUTO-MONITOR v1.0

<p align="center">
  <b>📊 One-Click Server Monitoring Solution</b><br>
  弘益人間 (홍익인간) · Benefit All Humanity
</p>

---

## 🎯 Overview

AUTO-MONITOR는 서버 모니터링을 "원클릭"으로 설정하는 bash 스크립트입니다.
$300/월 유료 서비스 대체! 무료로 강력한 모니터링!

## ✨ Features

- **💻 시스템 리소스** - CPU, 메모리, 스왑, 디스크, 부하
- **⚙️ 서비스 상태** - Apache, Nginx, MySQL, PHP-FPM, Redis
- **🌐 웹 모니터링** - HTTP 상태, 응답 시간
- **🔐 SSL 인증서** - 만료일 추적
- **📊 실시간 대시보드** - top처럼 실시간 모니터링
- **📢 알림** - Slack, Telegram, Discord, Email

## 🚀 Quick Install

```bash
# 원클릭 설치
curl -sSL https://wia.family/monitor/install | sudo bash

# 또는 git clone
git clone https://github.com/WIA-Official/ISP.git
cd ISP/auto-monitor && sudo ./install.sh
```

## 📖 Usage

```bash
# 기본 체크
auto-monitor

# 실시간 대시보드
auto-monitor --watch

# cron 등록 (5분마다)
auto-monitor --install

# 알림 설정
auto-monitor --setup-slack https://hooks.slack.com/xxx
auto-monitor --setup-telegram BOT_TOKEN CHAT_ID

# 임계값 설정
auto-monitor --set cpu_warning=80 cpu_critical=95

# URL/SSL 추가
auto-monitor --add-url https://example.com
auto-monitor --add-ssl example.com

# 프리셋 적용
auto-monitor --preset production   # 엄격한 임계값
auto-monitor --preset development  # 느슨한 임계값
```

## 📊 Output Example

```
╔══════════════════════════════════════════════════════════════╗
║  📊 AUTO-MONITOR v1.0                                        ║
║  弘益人間 (홍익인간) · Benefit All Humanity                  ║
╠══════════════════════════════════════════════════════════════╣
║  📍 서버: cqm.wiastandards.com                               ║
║  📅 체크: 2024-12-24 03:00:00 KST                            ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  💻 시스템 리소스                                            ║
║  ├── CPU:      23% [████░░░░░░░░░░░░░░░░] ✅                 ║
║  ├── 메모리:   67% [█████████████░░░░░░░] ✅                 ║
║  ├── 스왑:      0% [░░░░░░░░░░░░░░░░░░░░] ✅                 ║
║  ├── 디스크:   54% [██████████░░░░░░░░░░] ✅                 ║
║  └── 부하:    0.45 (4 cores) ✅                              ║
║                                                              ║
║  ⚙️ 서비스 상태                                              ║
║  ├── Apache:   Running ✅                                    ║
║  ├── MySQL:    Running ✅                                    ║
║  └── PHP-FPM:  Running ✅                                    ║
║                                                              ║
║  🌐 웹 모니터링                                              ║
║  ├── wiastandards.com    200 OK   (0.23s) ✅                ║
║  └── wiabook.com      200 OK   (0.45s) ✅                ║
║                                                              ║
║  🔐 SSL 인증서                                               ║
║  └── wiastandards.com    89일 남음 ✅                       ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  ✅ 모든 시스템 정상!                                        ║
╚══════════════════════════════════════════════════════════════╝
```

## ⚙️ Thresholds

| 메트릭 | Warning | Critical |
|--------|---------|----------|
| CPU | 70% | 90% |
| Memory | 80% | 95% |
| Swap | 50% | 80% |
| Disk | 80% | 90% |
| SSL | 30 days | 7 days |

## 📁 File Structure

```
/opt/auto-monitor/
├── auto-monitor.sh      # 메인 스크립트 (1,169줄)
├── auto-monitor.conf    # 설정 파일
├── data/                # 메트릭 데이터
└── logs/
    └── auto-monitor.log
```

## 📜 License

MIT License

---

<p align="center">
  <b>弘益人間 (홍익인간)</b><br>
  널리 인간을 이롭게 한다
</p>

<p align="center">
  © 2025 SmileStory Inc. / WIA<br>
  <a href="https://wia.family">wia.family</a>
</p>
