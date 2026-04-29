# WIA 표준 생성 가이드

> **Version:** 2.0.0
> **Updated:** 2025-12-24
> **Purpose:** 새 표준 생성 시 참조하는 완전한 가이드

---

## 빠른 시작

```bash
# 1. 예정 표준 목록에서 표준 찾기
# docs/WIA_예정_표준.md 참조

# 2. 표준 폴더 생성
mkdir -p {폴더명}/{simulator,ebook/en,ebook/ko,spec,spec/ko}

# 3. 기본 파일 생성
# - index.html (랜딩페이지)
# - simulator/index.html (시뮬레이터)
# - ebook/en/index.html + chapter-01~08.html
# - ebook/ko/index.html + chapter-01~08.html
# - spec/PHASE-1~4.md
```

---

## 1. 표준 생성 워크플로우

### Step 1: 표준 선택
1. `docs/WIA_예정_표준.md` 열기
2. 생성할 표준 찾기 (표준ID, 폴더명, 이모지 확인)
3. 영역별 Primary 색상 확인

### Step 2: 디렉토리 생성
```
{표준명}/
├── index.html              # 랜딩페이지 (필수)
├── simulator/
│   └── index.html          # 시뮬레이터 (필수)
├── ebook/
│   ├── en/
│   │   ├── index.html
│   │   └── chapter-01~08.html
│   └── ko/
│       ├── index.html
│       └── chapter-01~08.html
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API.md
│   ├── PHASE-3-PROTOCOL.md
│   ├── PHASE-4-INTEGRATION.md
│   └── ko/
│       ├── PHASE-1-DATA-FORMAT.md
│       └── ...
└── README.md
```

### Step 3: 파일 생성
1. 기존 표준 복사 (템플릿으로 사용)
2. 메타데이터 수정 (제목, 설명, 이모지)
3. Primary 색상 변경

### Step 4: 검증
- [ ] 다크 테마 (#0f172a) 적용
- [ ] 모든 링크 target="_blank"
- [ ] EN/KO 언어 토글 작동
- [ ] 반응형 레이아웃 확인
- [ ] 弘益人間 푸터 포함

---

## 2. 영역별 Primary 색상

| 영역 | 코드 | Primary | Hex |
|------|------|---------|-----|
| 접근성/보조기술 | AAC | Blue | `#3B82F6` |
| AI/지능시스템 | AI | Green | `#10B981` |
| 의료/헬스케어 | MED | Teal | `#14B8A6` |
| 바이오/생명공학 | BIO | Teal | `#14B8A6` |
| 냉동보존/생명연장 | CRYO | Cyan | `#06B6D4` |
| 에너지/환경 | ENE | Red | `#EF4444` |
| 반도체/전자 | SEMI | Indigo | `#6366F1` |
| 로봇/자동화 | ROB | Orange | `#F97316` |
| 우주/항공 | SPACE | Indigo | `#6366F1` |
| 통신/네트워크 | COMM | Blue | `#3B82F6` |
| 보안/암호화 | SEC | Pink | `#EC4899` |
| 교통/모빌리티 | AUTO | Orange | `#F97316` |
| 농업/식량 | AGRI | Lime | `#84CC16` |
| 건축/도시 | CITY | Sky | `#0EA5E9` |
| 금융/경제 | FIN | Green | `#22C55E` |
| 교육/문화 | EDU | Purple | `#A855F7` |
| 소재/화학 | MAT | Indigo | `#6366F1` |
| 국방/안보 | DEF | Slate | `#64748B` |
| 사회/인프라 | SOC | Purple | `#8B5CF6` |
| 디지털 레거시 | LEG | Slate | `#64748B` |
| 반려동물/동물복지 | PET | Amber | `#F59E0B` |
| 미래기술/양자/물리 | QUA | Indigo | `#6366F1` |
| 타임머신/시간여행 | TIME | Violet | `#8B5CF6` |
| 통일/평화 | UNI | Blue | `#3B82F6` |
| 기타 산업/소비재 | IND | Amber | `#F59E0B` |
| 컴퓨팅/소프트웨어 | COMP | Blue | `#3B82F6` |
| 데이터/빅데이터 | DATA | Emerald | `#10B981` |
| 인간증강/바이오닉스 | AUG | Cyan | `#06B6D4` |
| 범용 통합 표준 | CORE | Gray | `#6B7280` |

---

## 3. 필수 CSS 변수

```css
:root {
    --primary: #3B82F6;      /* 영역별 변경 */
    --primary-dark: #2563EB;
    --secondary: #10b981;
    --warning: #f59e0b;
    --danger: #ef4444;
    --bg: #0f172a;           /* 절대 변경 금지! */
    --bg-card: #1e293b;
    --text: #f8fafc;
    --text-muted: #94a3b8;
    --border: #334155;
}
```

---

## 4. 랜딩페이지 필수 구조

### 4.1 Top Navigation
```html
<div class="top-nav">
    <div class="nav-left">
        <a href="https://wiastandards.com" class="nav-home" target="_blank">🤟 WIA Standards</a>
        <div class="breadcrumb">
            <a href="https://wiastandards.com" target="_blank">Home</a>
            <span class="sep">›</span>
            <a href="https://wiastandards.com/#standards" target="_blank">Standards</a>
            <span class="sep">›</span>
            <span class="current">{표준명}</span>
        </div>
    </div>
    <div class="nav-right">
        <nav class="nav-links">
            <a href="https://cert.wiastandards.com" target="_blank">🏆 Certification</a>
            <a href="https://wiabook.com/reader/simulators/" target="_blank">🎮 Simulators</a>
            <a href="https://wiabook.com" target="_blank">📚 Ebook</a>
            <a href="https://github.com/WIA-Official/wia-standards" target="_blank">GitHub</a>
        </nav>
        <div class="lang-switch">
            <button class="lang-btn active" onclick="setLang('en')">EN</button>
            <button class="lang-btn" onclick="setLang('ko')">KO</button>
        </div>
    </div>
</div>
```

### 4.2 Hero Section
```html
<section class="hero">
    <div class="hero-emoji">{이모지}</div>
    <h1 data-en="WIA {표준명} Standard" data-ko="WIA {표준명} 표준">...</h1>
    <p class="subtitle" data-en="{영문 부제}" data-ko="{한글 부제}">...</p>
</section>
```

### 4.3 Action Buttons (2개만!)
```html
<div class="actions">
    <a href="simulator/" class="btn btn-primary" target="_blank"
       data-en="🎮 Try Simulator" data-ko="🎮 시뮬레이터 체험">🎮 Try Simulator</a>
    <a href="https://wiabook.com/{표준명}/" class="btn btn-secondary" target="_blank"
       data-en="📚 Read Ebook" data-ko="📚 전자책 읽기">📚 Read Ebook</a>
</div>
```

### 4.4 Phases Grid (2x2)
```html
<div class="phases">
    <a href="spec/PHASE-1-DATA-FORMAT.md" class="phase done" target="_blank">
        <div class="phase-num">1</div>
        <div class="phase-title" data-en="Data Format" data-ko="데이터 형식">Data Format</div>
        <div class="phase-status">✅ Complete</div>
    </a>
    <!-- Phase 2, 3, 4 동일 구조 -->
</div>
```

### 4.5 Footer (필수)
```html
<footer>
    <p class="philosophy" data-en="弘益人間 · Benefit All Humanity"
       data-ko="弘益人間 · 널리 인간을 이롭게 하라">弘益人間 · Benefit All Humanity</p>
    <p>WIA - World Certification Industry Association</p>
    <p>© 2025 MIT License</p>
</footer>
```

---

## 5. 시뮬레이터 필수 구조

### 5.1 5탭 구조 (필수)
```html
<div class="tabs">
    <div class="tab active" onclick="showPanel(0)"
         data-en="📊 Data Format" data-ko="📊 데이터 형식">📊 Data Format</div>
    <div class="tab" onclick="showPanel(1)"
         data-en="🔢 Algorithms" data-ko="🔢 알고리즘">🔢 Algorithms</div>
    <div class="tab" onclick="showPanel(2)"
         data-en="📡 Protocol" data-ko="📡 프로토콜">📡 Protocol</div>
    <div class="tab" onclick="showPanel(3)"
         data-en="🔗 Integration" data-ko="🔗 통합">🔗 Integration</div>
    <div class="tab" onclick="showPanel(4)"
         data-en="📱 QR & VC" data-ko="📱 QR & VC">📱 QR & VC</div>
</div>
```

### 5.2 Panel 구조
```html
<div class="panel active" id="panel-0">
    <!-- 데이터 형식 시뮬레이터 -->
</div>
<div class="panel" id="panel-1">
    <!-- 알고리즘 시뮬레이터 -->
</div>
<!-- Panel 2, 3, 4 -->
```

### 5.3 버튼 가운데 정렬
```html
<div class="btn-center">
    <button class="btn btn-primary" onclick="action()">
        <span data-en="Generate" data-ko="생성">Generate</span>
    </button>
</div>
```

---

## 6. EN/KO 토글 JavaScript

```javascript
let currentLang = localStorage.getItem('wiaLang') || 'en';

function setLang(lang) {
    currentLang = lang;
    localStorage.setItem('wiaLang', lang);
    document.querySelectorAll('[data-en]').forEach(el => {
        el.textContent = el.getAttribute('data-' + lang);
    });
    document.querySelectorAll('.lang-btn').forEach(btn => {
        btn.classList.toggle('active', btn.textContent.toLowerCase() === lang);
    });
}

document.addEventListener('DOMContentLoaded', function() {
    setLang(currentLang);
});
```

---

## 7. 반응형 CSS

```css
@media (max-width: 768px) {
    .top-nav {
        flex-direction: column;
        gap: 15px;
    }
    .nav-right {
        flex-direction: column;
        gap: 15px;
    }
    .hero h1 {
        font-size: 2rem;
    }
    .breadcrumb {
        display: none;
    }
    .tabs {
        flex-direction: column;
        align-items: center;
    }
    .tab {
        width: 100%;
        text-align: center;
    }
    .phases {
        grid-template-columns: 1fr;
    }
}
```

---

## 8. Spec 문서 구조

### PHASE-1: Data Format
```markdown
# WIA {표준명} - Phase 1: Data Format

## 1. Overview
## 2. Data Schema
## 3. Field Definitions
## 4. Validation Rules
## 5. Examples
```

### PHASE-2: API Interface
```markdown
# WIA {표준명} - Phase 2: API Interface

## 1. Endpoints
## 2. Request/Response Format
## 3. Authentication
## 4. Error Codes
## 5. Rate Limiting
```

### PHASE-3: Protocol
```markdown
# WIA {표준명} - Phase 3: Protocol

## 1. Communication Flow
## 2. Message Format
## 3. Security
## 4. Error Handling
## 5. Versioning
```

### PHASE-4: Integration
```markdown
# WIA {표준명} - Phase 4: WIA Integration

## 1. Ecosystem Connection
## 2. Cross-Standard Interoperability
## 3. Registry Integration
## 4. Certification Requirements
```

---

## 9. Ebook 챕터 구조 (8챕터)

> **🚨 상세 가이드: `docs/WIA_Ebook_Master_Prompt.md` 반드시 참조!**

### 품질 기준 (필수!)

| 항목 | 최소 기준 | 권장 기준 |
|------|----------|----------|
| **챕터당 크기** | 15KB | 20-25KB |
| **섹션 수 (h2)** | 6개 | 8-10개 |
| **전체 Ebook** | 120KB | 160-200KB |

### 8챕터 구조

| 챕터 | EN 제목 | KO 제목 | 필수 섹션 |
|------|---------|---------|-----------|
| 01 | Introduction | 소개 | 정의, 시장, 역사, 활용, 弘益人間 |
| 02 | Current Challenges | 현재 과제 | 문제점, 상호운용 이슈 |
| 03 | Standard Overview | 표준 개요 | 아키텍처, 4-Phase |
| 04 | Phase 1: Data Format | 데이터 형식 | JSON 스키마, 타입 |
| 05 | Phase 2: API Interface | API 인터페이스 | REST API, 인증 |
| 06 | Phase 3: Protocol | 프로토콜 | 스트리밍, 보안 |
| 07 | Phase 4: Integration | 통합 | 시스템 통합 |
| 08 | Implementation | 구현 및 인증 | WIA 인증 |

### 각 챕터 필수 요소

```
□ 8-10개 h2 섹션
□ Chapter Summary (Key Takeaways 5개)
□ Review Questions (6개)
□ Looking Ahead (다음 장 예고)
□ 테이블 2-5개
□ 코드 블록 (기술 챕터)
```

### 참조 템플릿 (품질 표준!)

| 순위 | 표준 | 크기 | 비고 |
|:----:|------|------|------|
| 🥇 | `emotion-ai/ebook/en/` | 832KB | 최고 품질 |
| 🥈 | `pet-health-passport/ebook/en/` | 912KB | 대안 |
| 🥉 | `battery-passport/ebook/en/` | 612KB | 대안 |

## 10. 체크리스트

### 생성 전
- [ ] 표준ID 확인 (docs/WIA_예정_표준.md)
- [ ] 폴더명 확인
- [ ] 이모지 확인
- [ ] Primary 색상 확인

### 랜딩페이지
- [ ] 다크 테마 (#0f172a)
- [ ] 카테고리별 --primary 색상
- [ ] top-nav + breadcrumb
- [ ] nav-links (Cert, Sims, Ebook, GitHub)
- [ ] EN/KO 토글 + localStorage
- [ ] 모든 텍스트 data-en/data-ko
- [ ] 모든 링크 target="_blank"
- [ ] 액션 버튼 2개 (Simulator, Ebook)
- [ ] Phases 2x2 그리드
- [ ] 弘益人間 푸터
- [ ] 반응형 @media

### 시뮬레이터
- [ ] 다크 테마 (#0f172a)
- [ ] 5탭 구조
- [ ] 탭 가운데 정렬
- [ ] 버튼 가운데 정렬
- [ ] EN/KO 토글 + localStorage
- [ ] 弘益人間 푸터
- [ ] 반응형 @media

### Spec 문서
- [ ] PHASE-1 완성
- [ ] PHASE-2 완성
- [ ] PHASE-3 완성
- [ ] PHASE-4 완성
- [ ] ko 폴더 한글 버전

### Ebook
- [ ] EN 8챕터 완성
- [ ] KO 8챕터 완성

---

## 11. 참조 템플릿

| 용도 | 참조 경로 |
|------|-----------|
| 랜딩페이지 | `aac/index.html` |
| 시뮬레이터 | `refugee-credential/simulator/index.html` |
| Spec | `aac/spec/PHASE-1-SIGNAL-FORMAT.md` |
| Ebook | `pet-health-passport/ebook/en/` |

---

## 12. 하지 말 것!

```
❌ 밝은 배경 (white, #ffffff, gradient)
❌ target="_blank" 누락
❌ data-en/data-ko 누락
❌ localStorage 미사용
❌ 弘益人間 푸터 누락
❌ 반응형 @media 누락
❌ 탭 3개 또는 4개 (5개 필수!)
❌ 좌측 정렬 버튼 (가운데 정렬!)
❌ 완성 목록에 있는 표준 중복 생성
```

---

## 13. 자주 묻는 질문

### Q: 새 표준은 어디서 찾나요?
A: `docs/WIA_예정_표준.md` 파일에서 538개 예정 표준을 확인하세요.

### Q: 이미 완성된 표준은?
A: `docs/WIA_완성_표준.md` 파일에서 72개 완성 표준을 확인하세요.

### Q: Primary 색상은 어떻게 정하나요?
A: 영역 코드 (AAC, AI, MED 등)에 따라 색상이 정해져 있습니다. 위 표 참조.

### Q: 템플릿은 어디 있나요?
A: 기존 표준 폴더를 복사해서 사용하세요. `aac/`, `refugee-credential/` 추천.

---

## 14. 문서 관리

| 문서 | 용도 | 업데이트 시점 |
|------|------|---------------|
| `WIA_완성_표준.md` | 완성 표준 목록 | 새 표준 완성 시 |
| `WIA_예정_표준.md` | 예정 표준 목록 | 새 표준 완성 시 해당 항목 제거 |
| `WIA_생성_가이드.md` | 생성 가이드 | 규칙 변경 시 |

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
