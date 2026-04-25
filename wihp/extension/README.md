# WIHP Chrome Extension

## IPA를 한글로! Universal Hangul Phonology

모든 언어의 발음을 한글로 변환하는 Chrome Extension입니다.

---

## 설치 방법

### 개발자 모드 설치 (현재)

1. Chrome에서 `chrome://extensions/` 접속
2. 우측 상단 **"개발자 모드"** 활성화
3. **"압축해제된 확장 프로그램을 로드합니다"** 클릭
4. `extension` 폴더 선택
5. 설치 완료!

### Chrome Web Store (예정)

추후 Chrome Web Store에서 "WIHP" 검색하여 설치 가능

---

## 사용 방법

### 1. 팝업 사용

1. 브라우저 우측 상단 WIHP 아이콘 클릭
2. 변환할 단어/IPA 입력
3. 자동으로 한글 변환!

### 2. 웹페이지에서 텍스트 선택

1. 아무 웹페이지에서 영어/외국어 텍스트 드래그 선택
2. 자동으로 WIHP 툴팁 표시
3. "Copy" 버튼으로 복사

### 3. 우클릭 메뉴

1. 텍스트 선택
2. 우클릭 → "Convert to WIHP Hangul"

---

## 지원 언어

| Tier | 언어 수 | 예시 |
|------|---------|------|
| Tier 1 | 20 | English, Japanese, Chinese, Spanish... |
| Tier 2 | 25 | Tamil, Persian, Swahili... |
| Tier 3 | 30 | Georgian, Hawaiian, Basque... |
| Tier 4 | 106 | Finnish, Cherokee, Tibetan... |
| Special | 7 | ASL, Latin, Sanskrit... |
| **Total** | **196** | |

---

## 변환 예시

| 입력 | 출력 |
|------|------|
| hello | 헬로 |
| konnichiwa | 곤니치와 |
| bonjour | 봉주르 |
| gracias | 그라시아스 |
| /həˈloʊ/ | 헐로우 |

---

## 기술 스택

- **Manifest V3** (최신 Chrome Extension API)
- **WIHP Engine** (IPA → Hangul 변환 엔진)
- **WIA Braille** 연동 (점자 출력)

---

## 파일 구조

```
extension/
├── manifest.json       # Extension 설정
├── popup.html          # 팝업 UI
├── popup.js            # 팝업 로직
├── wihp-engine.js      # 핵심 변환 엔진
├── background.js       # Service Worker
├── content.js          # 웹페이지 스크립트
├── content.css         # 스타일
└── icons/              # 아이콘
```

---

## 아이콘 생성

아이콘이 없는 경우, 아래 명령으로 생성:

```bash
# ImageMagick 사용
convert -size 16x16 xc:#1e40af -fill white -gravity center \
  -pointsize 10 -annotate 0 'W' icons/icon16.png

convert -size 48x48 xc:#1e40af -fill white -gravity center \
  -pointsize 24 -annotate 0 'W' icons/icon48.png

convert -size 128x128 xc:#1e40af -fill white -gravity center \
  -pointsize 64 -annotate 0 'W' icons/icon128.png
```

또는 `icons/` 폴더에 직접 PNG 파일 추가

---

## 개발

```bash
# 변경사항 적용
1. 코드 수정
2. chrome://extensions/ 에서 새로고침 버튼 클릭

# 디버깅
- 팝업: 팝업 우클릭 → "검사"
- Background: chrome://extensions/ → "서비스 워커" 클릭
- Content: 페이지에서 F12 → Console
```

---

## 라이선스

MIT License

---

## 철학

```
홍익인간 (弘益人間) - Benefit All Humanity

세종대왕 (1443): 한글 → 한국어를 누구나 읽게
WIHP (2025): 한글 → 모든 언어를 누구나 읽게

600년의 완성.
```

---

弘益人間 · Benefit All Humanity
WIA - World Certification Industry Association
© 2025 MIT License
