# WIA-AUTO-SECURE 다운로드

> 원클릭 HTTPS - 다운받고 실행하면 끝

## 빠른 설치 (권장)

### Linux / Mac / WSL
```bash
curl -sSL https://wia.family/secure.sh | sudo bash
```

### 또는 wget 사용
```bash
wget -qO- https://wia.family/secure.sh | sudo bash
```

---

## 수동 다운로드

### 방법 1: Git Clone
```bash
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/auto-secure/linux
chmod +x wia-secure-install.sh
sudo ./wia-secure-install.sh
```

### 방법 2: 파일 직접 다운로드
```bash
# 스크립트만 다운로드
wget https://raw.githubusercontent.com/WIA-Official/wia-standards/main/auto-secure/linux/wia-secure-install.sh
chmod +x wia-secure-install.sh
sudo ./wia-secure-install.sh
```

### 방법 3: ZIP 다운로드
1. https://github.com/WIA-Official/wia-standards/archive/refs/heads/main.zip
2. 압축 해제
3. `auto-secure/linux/` 폴더로 이동
4. `./wia-secure-install.sh` 실행

---

## 파일 구조

```
auto-secure/
├── README.md                 # 전체 설명서
├── DOWNLOAD.md              # 이 파일
├── linux/
│   └── wia-secure-install.sh # Linux 설치 스크립트
├── web/
│   └── index.html           # 웹 UI (secure.wia.family)
└── windows/                 # (예정)
    └── wia-secure-setup.exe
```

---

## 체크섬 (무결성 검증)

파일이 변조되지 않았는지 확인:

```bash
# SHA256 체크섬 확인
sha256sum wia-secure-install.sh

# 예상 값 (v1.0)
# [체크섬은 릴리스 시 업데이트됨]
```

---

## 오프라인 설치

인터넷이 없는 환경에서 설치하려면:

1. 인터넷이 되는 PC에서 전체 패키지 다운로드
2. USB 등으로 서버에 복사
3. 스크립트 실행

```bash
# 오프라인 모드 (certbot이 이미 설치된 경우)
./wia-secure-install.sh --offline
```

---

## 시스템 요구사항

| 항목 | 최소 요구사항 |
|------|--------------|
| OS | Ubuntu 18.04+, Debian 10+, CentOS 7+, Amazon Linux 2+ |
| 권한 | root 또는 sudo |
| 네트워크 | 포트 80, 443 열려있어야 함 |
| 도메인 | DNS A 레코드가 서버 IP를 가리켜야 함 |

---

## 문제 해결

### "Permission denied"
```bash
sudo chmod +x wia-secure-install.sh
sudo ./wia-secure-install.sh
```

### "Command not found: curl"
```bash
# Ubuntu/Debian
sudo apt-get install curl

# CentOS/RHEL
sudo yum install curl
```

### "Domain not found"
DNS 설정을 확인하세요:
```bash
dig +short your-domain.com
# 서버 IP가 출력되어야 함
```

---

## 지원

- 문제 신고: https://github.com/WIA-Official/wia-standards/issues
- 이메일: support@wia.family
- 웹사이트: https://wia.family

---

**홍익인간 (弘益人間) - 널리 인간을 이롭게 하라**

MIT License - 무료로 사용, 수정, 배포 가능
