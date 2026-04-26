# WIA A11Y Language Support System

**Website**: https://a11y.wiabook.com

## Overview

WIA의 접근성 언어 지원 시스템입니다. 211개 언어를 지원하며, 각 언어별로 접근성 용어와 번역이 제공됩니다.

## Features

- **211 Languages**: 전 세계 211개 언어 지원
- **A11Y Checker**: 접근성 체커 (한국어, 영어)
- **Language Modal**: 언어 선택 모달 컴포넌트
- **WIA Badges**: WIA 인증 배지 (Certified, Gold, Silver)

## Directory Structure

```
a11y-wiabooks/
├── assets/              # React 빌드 파일
│   ├── charts-*.js     # Chart 컴포넌트
│   ├── index-*.css     # 스타일시트
│   ├── index-*.js      # 메인 JavaScript
│   └── react-vendor-*.js # React 라이브러리
├── badges/              # WIA 인증 배지
│   ├── wia-certified.* # 기본 인증
│   ├── wia-gold.*      # 골드 인증
│   └── wia-silver.*    # 실버 인증
├── components/          # UI 컴포넌트
│   └── language-modal-211/ # 언어 선택 모달
├── languages/           # 언어 파일 (476개)
│   ├── a11y-*.js      # A11Y 언어 파일
│   ├── *.js           # 표준 언어 파일
│   ├── loader.js      # 언어 로더
│   └── README.md      # 언어 문서
├── wialanguages/        # 언어 데이터
│   └── data/
│       └── languages-211-complete.json
├── index.html          # 메인 페이지
├── favicon.svg         # 파비콘
└── robots.txt          # SEO 설정
```

## Language Files

### A11Y Language Files (211개)
- Format: `a11y-{language-code}.js`
- Examples: `a11y-ko.js`, `a11y-en.js`, `a11y-ja.js`

### Standard Language Files
- Format: `{language-code}.js`
- Examples: `ko.js`, `en.js`, `ja.js`

### Special Files
- `a11y-checker-en.js` - English accessibility checker
- `a11y-checker-ko.js` - Korean accessibility checker
- `a11y-loader.js` - Dynamic language loader
- `loader.js` - Standard loader

## Supported Languages (211)

### Major Languages
- 한국어 (ko)
- English (en)
- 日本語 (ja)
- 中文 (zh)
- Español (es)
- Français (fr)
- Deutsch (de)
- Русский (ru)
- العربية (ar)
- हिन्दी (hi)
- ... and 201 more

## Integration with WIA Standards

이 언어 시스템은 WIA 표준들과 통합되어 사용됩니다:

- **BCI**: 뇌-컴퓨터 인터페이스 다국어 지원
- **AAC**: 보조 의사소통 다국어 지원
- **Voice-Sign**: 음성-수어 다국어 지원
- **Medical**: 의료 기기 다국어 지원
- **All Standards**: 모든 표준의 다국어 지원

## Usage

### 1. Language Modal Component

```html
<script src="/components/language-modal-211/wia-language-modal-211.js"></script>
```

### 2. Load Language File

```javascript
// Dynamic loading
import loader from '/languages/loader.js';
const translations = await loader.load('ko');

// Direct import
import koA11y from '/languages/a11y-ko.js';
```

### 3. A11Y Checker

```javascript
import checker from '/languages/a11y-checker-ko.js';
checker.validate(element);
```

## Statistics

- **Total Files**: 508
- **Language Files**: 476
- **Total Size**: 5.1 MB
- **Languages Supported**: 211

## Author

**Yeon Sam-Heum, Ph.D.**

## License

MIT License

---

**홍익인간 (弘益人間)** - Benefit All Humanity
