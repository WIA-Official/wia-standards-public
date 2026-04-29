# AUTO-BACKUP v1.0

<p align="center">
  <b>🔄 One-Click Server Backup Solution</b><br>
  弘益人間 (홍익인간) · Benefit All Humanity
</p>

---

## 🎯 Overview

AUTO-BACKUP은 서버 백업을 "원클릭"으로 자동화하는 bash 스크립트입니다.
AUTO-SECURE처럼 한 번 실행으로 모든 것이 자동으로!

## ✨ Features

- **🔍 환경 자동 감지** - 웹서버, DB, 설정 파일 자동 탐지
- **📦 다양한 백업 모드** - 전체/웹/DB/설정/증분 백업
- **☁️ 클라우드 업로드** - AWS S3, Google Drive 지원
- **⏰ 자동 스케줄** - cron 기반 자동 백업
- **🔄 복원 기능** - 날짜별 복원, 선택적 복원
- **📢 알림** - Email, Slack, Telegram, Discord
- **🔐 암호화** - OpenSSL AES-256 암호화 지원

## 🚀 Quick Install

```bash
# 원클릭 설치
curl -sSL https://wia.family/backup/install | sudo bash

# 또는 git clone
git clone https://github.com/WIA-Official/ISP.git
cd ISP/auto-backup && sudo ./install.sh
```

## 📖 Usage

```bash
# 기본 사용
auto-backup                      # 전체 백업
auto-backup --dry-run            # 시뮬레이션 (실제 백업 안 함)
auto-backup --verbose            # 상세 출력

# 선택적 백업
auto-backup --web                # 웹만
auto-backup --db                 # DB만
auto-backup --config             # 설정만

# 클라우드 업로드
auto-backup --upload s3          # S3 업로드
auto-backup --upload gdrive      # Google Drive 업로드

# 스케줄 설정
auto-backup --schedule daily     # 매일 새벽 3시 cron 등록
auto-backup --schedule weekly    # 매주 일요일
auto-backup --unschedule         # cron 제거

# 복원
auto-backup --list               # 백업 목록 보기
auto-backup --restore latest     # 최신 복원
auto-backup --restore 2024-12-24 # 특정 날짜 복원
auto-backup --restore latest --db-only  # DB만 복원

# 관리
auto-backup --status             # 상태 확인
auto-backup --cleanup            # 오래된 백업 정리
auto-backup --test-notify        # 알림 테스트
auto-backup --setup              # 설정 마법사
auto-backup --help               # 도움말
```

## 📊 Output Example

```
╔══════════════════════════════════════════════════════════════╗
║  🔄 AUTO-BACKUP v1.0                                         ║
║  弘益人間 (홍익인간) · Benefit All Humanity                  ║
╠══════════════════════════════════════════════════════════════╣
║  📍 서버: cqm.wiastandards.com                               ║
║  📅 시작: 2024-12-24 03:00:00 KST                            ║
╠══════════════════════════════════════════════════════════════╝
║                                                              ║
║  [1/5] 🔍 환경 감지                                          ║
║        ├── OS: Amazon Linux 2023                             ║
║        ├── 웹서버: Apache 2.4.58                             ║
║        ├── 웹사이트: 3개 감지                                ║
║        ├── 데이터베이스: MySQL 8.0                           ║
║        └── SSL 인증서: 5개 도메인                            ║
║                                                              ║
║  [2/5] 📂 웹 백업                                            ║
║        ├── 원본: 385MB                                       ║
║        ├── 압축: 142MB (63% 절약)                            ║
║        └── ✅ 완료 (45초)                                    ║
║                                                              ║
║  [3/5] 🗄️ DB 백업                                            ║
║        ├── 원본: 146MB                                       ║
║        ├── 압축: 28MB (81% 절약)                             ║
║        └── ✅ 완료 (23초)                                    ║
║                                                              ║
║  [4/5] ⚙️ 설정 백업                                          ║
║        └── ✅ 완료 (3초)                                     ║
║                                                              ║
║  [5/5] 📦 최종 패키징                                        ║
║        └── ✅ 완료                                           ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  ✅ 백업 완료!                                               ║
║  📁 위치: /var/backups/auto-backup/2024-12-24/               ║
║  📦 크기: 172.4MB                                            ║
║  ⏱️ 소요: 1분 23초                                           ║
╚══════════════════════════════════════════════════════════════╝
```

## ⚙️ Configuration

설정 파일: `/opt/auto-backup/auto-backup.conf`

```bash
# 보관 정책
KEEP_DAILY=7      # 일간 백업 7개
KEEP_WEEKLY=4     # 주간 백업 4개
KEEP_MONTHLY=3    # 월간 백업 3개

# 클라우드
S3_BUCKET="my-backup-bucket"
S3_REGION="ap-northeast-2"

# 알림
SLACK_ENABLED=true
SLACK_WEBHOOK="https://hooks.slack.com/..."
```

## 📁 File Structure

```
/opt/auto-backup/
├── auto-backup.sh       # 메인 스크립트 (1,481줄)
├── auto-backup.conf     # 설정 파일
└── logs/
    └── auto-backup.log  # 로그 파일

/var/backups/auto-backup/
├── 2024-12-23/
│   └── full-backup-20241223-030000.tar.gz
├── 2024-12-24/
│   └── full-backup-20241224-030000.tar.gz
└── ...
```

## 🔒 Security

- 설정 파일 권한: 600 (root만 읽기)
- DB 비밀번호: ~/.my.cnf 또는 환경변수 권장
- 백업 파일 암호화: OpenSSL AES-256-CBC

## 🖥️ Supported Systems

- Amazon Linux 2023
- CentOS 7/8/9
- RHEL 7/8/9
- Ubuntu 18/20/22/24
- Debian 10/11/12
- Fedora 37+

## 📜 License

MIT License - 자유롭게 사용, 수정, 배포 가능

---

<p align="center">
  <b>弘益人間 (홍익인간)</b><br>
  널리 인간을 이롭게 한다
</p>

<p align="center">
  © 2025 SmileStory Inc. / WIA<br>
  <a href="https://wia.family">wia.family</a>
</p>
