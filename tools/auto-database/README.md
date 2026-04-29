# AUTO-DATABASE v1.0

<p align="center">
  <b>🗄️ Database Backup & Restore</b><br>
  弘益人間 (홍익인간) · Benefit All Humanity
</p>

---

## 🎯 Overview

AUTO-DATABASE는 데이터베이스 백업/복원을 "원클릭"으로 자동화하는 bash 스크립트입니다.
MySQL, PostgreSQL, MongoDB, Redis 모두 지원!

## ✨ Features

- **🔍 자동 감지** - MySQL, PostgreSQL, MongoDB, Redis 자동 감지
- **📦 원클릭 백업** - 전체 또는 개별 DB 백업
- **🔄 쉬운 복원** - 날짜별 백업에서 복원
- **🗜️ 압축** - gzip, bzip2, xz 지원
- **🔐 암호화** - OpenSSL AES-256 암호화
- **☁️ 클라우드** - AWS S3 업로드 지원
- **⏰ 스케줄** - cron 기반 자동 백업

## 🚀 Quick Install

```bash
curl -sSL https://wia.family/database/install | sudo bash
```

## 📖 Usage

```bash
# DB 감지
auto-database --detect

# 백업
auto-database backup                  # 전체
auto-database backup --db myapp       # 특정 DB
auto-database backup --mysql          # MySQL만
auto-database backup --encrypt        # 암호화

# 목록
auto-database list

# 복원
auto-database restore latest
auto-database restore 2024-12-24 --db myapp

# 유틸리티
auto-database status
auto-database size
auto-database optimize
auto-database clone myapp myapp_dev

# 스케줄
auto-database schedule daily 03:00
```

## 📊 Output Example

```
╔══════════════════════════════════════════════════════════════╗
║  🗄️ AUTO-DATABASE v1.0                                       ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  [1/3] 🔍 데이터베이스 감지                                  ║
║        ├── MySQL 8.0.35                                      ║
║        │   ├── wiabooks (89MB)                               ║
║        │   └── wiastandards (45MB)                           ║
║        └── Redis 7.0.12                                      ║
║                                                              ║
║  [2/3] 📦 백업 진행                                          ║
║        ├── wiabooks... 완료 (89MB → 18MB, 80% 압축)         ║
║        └── wiastandards... 완료 (45MB → 9MB, 80% 압축)      ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║  ✅ 백업 완료!                                               ║
║  📊 원본: 154MB → 압축: 34MB (78% 절약)                      ║
╚══════════════════════════════════════════════════════════════╝
```

## 🗃️ Supported Databases

| Database | Backup | Restore | Optimize |
|----------|--------|---------|----------|
| MySQL/MariaDB | ✅ | ✅ | ✅ |
| PostgreSQL | ✅ | ✅ | - |
| MongoDB | ✅ | ✅ | - |
| Redis | ✅ (RDB) | ✅ | - |

## 📜 License

MIT License

---

<p align="center">
  <b>弘益人間 (홍익인간)</b><br>
  © 2025 SmileStory Inc. / WIA
</p>
