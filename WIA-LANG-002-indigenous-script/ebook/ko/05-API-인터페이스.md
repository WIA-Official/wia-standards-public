# 제5장: API 인터페이스

## 원주민 문자 서비스 API

WIA-LANG-002는 문자 레지스트리, 폰트 서비스, 키보드 배포, 렌더링 지원을 위한 통합 API를 제공합니다.

## 기본 구성

### 기본 URL

```
프로덕션: https://api.wia-lang.org/script/v1
스테이징: https://api-staging.wia-lang.org/script/v1
```

### 인증

```http
Authorization: Bearer {API_KEY}
Content-Type: application/json
```

## 문자 레지스트리 API

### 문자 목록 조회

```http
GET /scripts?type=alphabet&status=unicode&limit=20

Response 200 OK:
{
  "data": [
    {
      "code": "Olck",
      "name": "Ol Chiki",
      "type": "alphabet",
      "direction": "ltr",
      "unicodeRange": "U+1C50-U+1C7F",
      "languages": ["sat", "mun", "ho"],
      "resources": {
        "fonts": 5,
        "keyboards": 3
      }
    }
  ],
  "pagination": {
    "total": 50,
    "limit": 20,
    "offset": 0
  }
}
```

### 문자 상세 조회

```http
GET /scripts/Olck

Response 200 OK:
{
  "data": {
    "code": "Olck",
    "iso15924": "Olck",
    "name": {
      "en": "Ol Chiki",
      "sat": "ᱚᱞ ᱪᱤᱠᱤ",
      "ko": "올치키"
    },
    "type": "alphabet",
    "direction": "ltr",
    "unicode": {
      "range": "U+1C50-U+1C7F",
      "version": "5.1",
      "status": "standard"
    },
    "characters": {
      "total": 48,
      "letters": 30,
      "digits": 10,
      "other": 8
    },
    "history": {
      "created": 1925,
      "creator": "Raghunath Murmu"
    },
    "links": {
      "fonts": "/scripts/Olck/fonts",
      "keyboards": "/scripts/Olck/keyboards",
      "characters": "/scripts/Olck/characters"
    }
  }
}
```

### 문자 등록

```http
POST /scripts

{
  "code": "Wcho",
  "name": {
    "en": "Wancho",
    "wch": "𞋀𞋁𞋂𞋃"
  },
  "type": "alphabet",
  "direction": "ltr",
  "unicode": {
    "range": "U+1E2C0-U+1E2FF",
    "status": "standard"
  },
  "community": {
    "organization": "Wancho Council",
    "contact": "wancho@example.org"
  }
}

Response 201 Created
```

## 폰트 서비스 API

### 폰트 목록 조회

```http
GET /scripts/Olck/fonts

Response 200 OK:
{
  "data": [
    {
      "id": "noto-sans-ol-chiki",
      "name": "Noto Sans Ol Chiki",
      "version": "2.005",
      "license": "OFL-1.1",
      "formats": ["ttf", "woff2"],
      "weights": [400, 500, 600, 700],
      "download": {
        "ttf": "/fonts/noto-sans-ol-chiki/v2.005/NotoSansOlChiki.ttf",
        "woff2": "/fonts/noto-sans-ol-chiki/v2.005/NotoSansOlChiki.woff2"
      }
    }
  ]
}
```

### 폰트 다운로드

```http
GET /fonts/{font_id}/{version}/{filename}

Response 200 OK:
Content-Type: font/ttf
Content-Disposition: attachment; filename="NotoSansOlChiki.ttf"
[binary data]
```

### 웹폰트 CSS 제공

```http
GET /fonts/{font_id}/css?weights=400,700

Response 200 OK:
Content-Type: text/css

@font-face {
  font-family: 'Noto Sans Ol Chiki';
  font-style: normal;
  font-weight: 400;
  src: url('/fonts/noto-sans-ol-chiki/NotoSansOlChiki-Regular.woff2') format('woff2');
  unicode-range: U+1C50-U+1C7F;
}
```

### 폰트 등록

```http
POST /fonts

Content-Type: multipart/form-data

{
  "script": "Olck",
  "name": "Santali Font",
  "version": "1.0",
  "license": "OFL-1.1",
  "file": [binary font file]
}

Response 201 Created:
{
  "data": {
    "id": "santali-font",
    "status": "processing",
    "validation": {
      "pending": true
    }
  }
}
```

## 키보드 배포 API

### 키보드 목록

```http
GET /scripts/Olck/keyboards

Response 200 OK:
{
  "data": [
    {
      "id": "olck-standard",
      "name": "Ol Chiki Standard",
      "type": "layout",
      "platforms": ["windows", "macos", "linux", "web"],
      "version": "1.0",
      "downloads": {
        "windows": "/keyboards/olck-standard/olck-standard.klc",
        "macos": "/keyboards/olck-standard/olck-standard.bundle",
        "web": "/keyboards/olck-standard/olck-standard.json"
      }
    },
    {
      "id": "olck-phonetic",
      "name": "Ol Chiki Phonetic",
      "type": "ime",
      "platforms": ["web", "android"],
      "version": "1.2"
    }
  ]
}
```

### 웹 키보드 로드

```http
GET /keyboards/{keyboard_id}/web

Response 200 OK:
{
  "id": "olck-standard",
  "layers": {
    "default": [...],
    "shift": [...]
  },
  "css": "...",
  "js": "..."
}
```

### 키보드 등록

```http
POST /keyboards

{
  "script": "Olck",
  "name": "Ol Chiki Mobile",
  "type": "layout",
  "platform": "android",
  "definition": {
    "layers": {...}
  }
}

Response 201 Created
```

## 렌더링 서비스 API

### 텍스트 렌더링

```http
POST /render

{
  "text": "ᱚᱞ ᱪᱤᱠᱤ",
  "script": "Olck",
  "font": "noto-sans-ol-chiki",
  "size": 48,
  "format": "svg"
}

Response 200 OK:
Content-Type: image/svg+xml

<svg>...</svg>
```

### 텍스트 분석

```http
POST /analyze

{
  "text": "ᱚᱞ ᱪᱤᱠᱤ",
  "script": "Olck"
}

Response 200 OK:
{
  "data": {
    "characters": [
      {
        "char": "ᱚ",
        "codePoint": "U+1C5A",
        "name": "OL CHIKI LETTER LA",
        "category": "Lo"
      }
    ],
    "words": 2,
    "direction": "ltr",
    "scripts": ["Olck"],
    "languages": ["sat"]
  }
}
```

### 변환 서비스

```http
POST /convert

{
  "text": "ol chiki",
  "from": "romanization",
  "to": "Olck",
  "method": "phonetic"
}

Response 200 OK:
{
  "data": {
    "result": "ᱚᱞ ᱪᱤᱠᱤ",
    "confidence": 0.95
  }
}
```

## 검색 API

### 문자 검색

```http
GET /search?q=santali&type=script

Response 200 OK:
{
  "data": {
    "scripts": [
      {"code": "Olck", "name": "Ol Chiki", "match": "language"}
    ],
    "fonts": [
      {"id": "santali-font", "name": "Santali Font"}
    ],
    "keyboards": []
  }
}
```

### 유니코드 검색

```http
GET /unicode/search?q=1C50

Response 200 OK:
{
  "data": {
    "codePoint": "U+1C50",
    "name": "OL CHIKI DIGIT ZERO",
    "script": "Olck",
    "category": "Nd",
    "block": "Ol Chiki"
  }
}
```

## SDK 사용법

### JavaScript

```javascript
import { WIAScriptClient } from '@wia/script-sdk';

const client = new WIAScriptClient({
  apiKey: process.env.WIA_API_KEY
});

// 문자 조회
const script = await client.scripts.get('Olck');
console.log(script.name.en); // "Ol Chiki"

// 폰트 목록
const fonts = await client.fonts.list({ script: 'Olck' });

// 웹폰트 로드
await client.fonts.loadWebFont('noto-sans-ol-chiki');

// 키보드 활성화
const keyboard = await client.keyboards.load('olck-phonetic');
keyboard.activate();
```

### Python

```python
from wia_script import Client

client = Client(api_key=os.getenv('WIA_API_KEY'))

# 문자 조회
script = client.scripts.get('Olck')
print(script.name['en'])  # "Ol Chiki"

# 폰트 다운로드
client.fonts.download('noto-sans-ol-chiki', './fonts/')

# 텍스트 변환
result = client.convert(
    text='ol chiki',
    from_format='romanization',
    to_format='Olck'
)
print(result.text)  # ᱚᱞ ᱪᱤᱠᱤ
```

## 오류 처리

### 오류 응답 형식

```json
{
  "error": {
    "code": "SCRIPT_NOT_FOUND",
    "message": "Script with code 'Xxxx' not found",
    "details": {
      "suggestedCode": "Olck"
    }
  }
}
```

### 오류 코드

| 코드 | HTTP | 설명 |
|------|------|------|
| SCRIPT_NOT_FOUND | 404 | 문자 없음 |
| FONT_NOT_FOUND | 404 | 폰트 없음 |
| INVALID_FORMAT | 400 | 형식 오류 |
| VALIDATION_FAILED | 422 | 검증 실패 |

---

## 핵심 요약

**주요 내용:**

1. **문자 레지스트리 API**: 문자 조회, 등록, 메타데이터 관리

2. **폰트 서비스 API**: 폰트 다운로드, 웹폰트 CSS, 폰트 등록

3. **키보드 배포 API**: 다중 플랫폼 키보드, 웹 키보드 로드

4. **렌더링 서비스**: 텍스트 렌더링, 분석, 변환

5. **SDK**: JavaScript, Python 클라이언트 라이브러리

---

## 복습 문제

1. 문자 레지스트리 API의 주요 엔드포인트를 설명하시오.

2. 폰트 서비스에서 웹폰트 CSS를 제공하는 방식을 설명하시오.

3. 키보드 배포 API가 지원하는 플랫폼과 형식을 나열하시오.

4. 텍스트 변환 API의 사용 사례를 설명하시오.

5. SDK를 사용한 웹폰트 로드 과정을 설명하시오.

6. API 오류 처리의 표준 형식을 설명하시오.

---

**제5장 완료** | 예상 페이지: 14

[이전: 제4장 - 데이터 형식](./04-데이터-형식.md) | [다음: 제6장 - 프로토콜](./06-프로토콜.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
