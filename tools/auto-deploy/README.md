# AUTO-DEPLOY v1.0

<p align="center">
  <b>🚀 Zero-Downtime Deployment Solution</b><br>
  弘益人間 (홍익인간) · Benefit All Humanity
</p>

---

## 🎯 Overview

AUTO-DEPLOY는 Zero-Downtime 배포를 "원클릭"으로 자동화하는 bash 스크립트입니다.
git push 한 번으로 서비스 중단 없이 배포 완료!

## ✨ Features

- **⚡ Zero-Downtime** - Symlink 방식으로 순간 전환
- **🔍 프로젝트 자동 감지** - Node.js, PHP, Python, Ruby, Java, Go, Rust
- **🔨 자동 빌드** - 프레임워크별 최적화된 빌드 명령어
- **↩️ 즉시 롤백** - 이전 버전으로 1초 내 복구
- **🔗 Webhook 지원** - GitHub/GitLab 자동 배포
- **📢 알림** - Slack, Discord 배포 알림

## 🚀 Quick Install

```bash
# 원클릭 설치
curl -sSL https://wia.family/deploy/install | sudo bash

# 또는 git clone
git clone https://github.com/WIA-Official/ISP.git
cd ISP/auto-deploy && sudo ./install.sh
```

## 📖 Usage

### 프로젝트 초기화

```bash
auto-deploy --init https://github.com/user/myapp.git --path /var/www/myapp
```

### 배포

```bash
auto-deploy myapp                    # 기본 배포
auto-deploy myapp --branch develop   # 특정 브랜치
auto-deploy myapp --tag v2.0.0       # 특정 태그
auto-deploy myapp --dry-run          # 시뮬레이션
```

### 롤백

```bash
auto-deploy myapp --rollback         # 이전 버전
auto-deploy myapp --rollback 3       # 3단계 전
auto-deploy myapp --list             # 배포 히스토리
```

### 관리

```bash
auto-deploy --projects               # 프로젝트 목록
auto-deploy myapp --status           # 프로젝트 상태
auto-deploy myapp --cleanup          # 오래된 릴리스 정리
```

## 🔧 How It Works

```
배포 전:
/var/www/myapp → releases/20241224-020000 (현재)

배포 후:
/var/www/myapp → releases/20241224-030000 (새 버전)
                 releases/20241224-020000 (롤백용)
                 releases/20241224-010000 (롤백용)

Symlink 전환 = 0.1초 ⚡ Zero Downtime!
```

## 📊 Output Example

```
╔══════════════════════════════════════════════════════════════╗
║  🚀 AUTO-DEPLOY v1.0                                         ║
║  弘益人間 (홍익인간) · Benefit All Humanity                  ║
╠══════════════════════════════════════════════════════════════╣
║  📦 프로젝트: wiastandards                                   ║
║  🌿 브랜치: main                                             ║
╠══════════════════════════════════════════════════════════════╝
║                                                              ║
║  [1/6] 📥 Git Clone                                          ║
║        └── ✅ 완료 (12초)                                    ║
║                                                              ║
║  [2/6] 🔍 프로젝트 감지                                      ║
║        ├── 타입: Next.js                                     ║
║        └── ✅ 감지 완료                                      ║
║                                                              ║
║  [3/6] 📦 의존성 설치                                        ║
║        └── ✅ 완료 (45초)                                    ║
║                                                              ║
║  [4/6] 🔨 빌드                                               ║
║        └── ✅ 완료 (38초)                                    ║
║                                                              ║
║  [5/6] 🔗 Symlink 전환                                       ║
║        └── ✅ 전환 완료 (0.1초) ⚡ Zero Downtime!            ║
║                                                              ║
║  [6/6] 🧹 정리                                               ║
║        └── ✅ 완료                                           ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  ✅ 배포 완료!                                               ║
║  ⏱️ 총 소요: 1분 35초                                        ║
║  ↩️ 롤백: auto-deploy wiastandards --rollback                ║
╚══════════════════════════════════════════════════════════════╝
```

## 🔍 Supported Project Types

| Type | Detection | Build Command |
|------|-----------|---------------|
| Next.js | `package.json` + "next" | `npm install && npm run build` |
| Nuxt.js | `package.json` + "nuxt" | `npm install && npm run build` |
| React | `package.json` + "react" | `npm install && npm run build` |
| Vue.js | `package.json` + "vue" | `npm install && npm run build` |
| Laravel | `composer.json` + "laravel" | `composer install && npm run build` |
| Django | `requirements.txt` + `manage.py` | `pip install -r requirements.txt` |
| Rails | `Gemfile` + "rails" | `bundle install && rails assets:precompile` |
| Go | `go.mod` | `go build -o app .` |
| Rust | `Cargo.toml` | `cargo build --release` |

## ⚙️ Configuration

### 프로젝트별 설정 (`.auto-deploy.conf`)

```bash
PROJECT_NAME="myapp"
GIT_URL="https://github.com/user/myapp.git"
PROJECT_PATH="/var/www/myapp"
GIT_BRANCH="main"

# 공유 리소스 (릴리스 간 유지)
SHARED_DIRS="storage logs uploads"
SHARED_FILES=".env"

# 배포 후 실행
POST_ACTIVATE="php artisan cache:clear"

# 서비스 재시작
RESTART_SERVICES="php-fpm nginx"
```

## 📁 File Structure

```
/opt/auto-deploy/
├── auto-deploy.sh       # 메인 스크립트 (1,407줄)
├── auto-deploy.conf     # 전역 설정
├── projects/            # 프로젝트 설정들
│   ├── myapp.conf
│   └── another.conf
└── logs/

/var/www/myapp           # symlink → releases/current
/var/www/myapp-releases/
├── 20241224-010000/     # 이전 버전
├── 20241224-020000/     # 이전 버전
└── 20241224-030000/     # 현재 버전
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
