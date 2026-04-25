# 🇰🇷 WIHP 페이지 EN/KO 완벽 번역 요청

## 📋 프로젝트 개요

**목적:** WIHP (World International Hangul Phonology) 웹사이트의 EN/KO 이중 언어 지원 완성

**GitHub 레포:** https://github.com/WIA-Official/wia-standards
**경로:** wihp/index.html, wihp/viewer.html

---

## 🎯 작업 목표

두 HTML 파일에 있는 **모든 사용자 대면 텍스트**에 `data-en`과 `data-ko` 속성 추가

### 현재 상태
- EN/KO 토글 버튼 있음 ✅
- setLang() 함수 작동 ✅
- **문제:** 일부 요소에만 data-en/data-ko 있음 ❌

### 목표 상태
- **모든** 텍스트에 data-en, data-ko 속성 완비
- EN 선택 시 영어, KO 선택 시 한국어 완벽 표시

---

## 📂 번역 대상 파일

### 1. wihp/index.html
번역 필요한 요소:
- 모든 링크 텍스트
- 모든 제목/부제목
- 모든 설명문
- 배지 텍스트
- 통계 라벨
- 푸터 텍스트

### 2. wihp/viewer.html
번역 필요한 요소:
- 페이지 제목/설명
- 모든 Tier 섹션 제목
- 언어 이름 (언어명은 그대로, 한글 번역만 추가)
- 카테고리 제목
- 버튼 텍스트
- 에러 메시지
- 푸터 텍스트

---

## 🔧 기술 사양

### 번역 방식
```html
<!-- 변경 전 -->
<span class="current">Spec Viewer</span>

<!-- 변경 후 -->
<span class="current" data-en="Spec Viewer" data-ko="스펙 뷰어">Spec Viewer</span>
```

### JavaScript (이미 구현됨)
```javascript
function setLang(lang) {
    localStorage.setItem('wiaLang', lang);
    document.querySelectorAll('.lang-btn').forEach(btn => btn.classList.remove('active'));
    document.querySelector(`.lang-btn:${lang === 'en' ? 'first-child' : 'last-child'}`).classList.add('active');
    document.querySelectorAll('[data-en]').forEach(el => {
        el.textContent = el.getAttribute('data-' + lang);
    });
}
```

---

## 📝 번역 가이드라인

### 1. 용어 일관성
| EN | KO |
|---|---|
| Certification | 인증 |
| Ebook | 전자책 |
| Simulator | 시뮬레이터 |
| Standards | 표준 |
| Languages | 언어 |
| Tier | 티어 |
| Core | 코어 |
| Mapping | 매핑 |
| Major | 주요 언어 |
| Regional | 지역 언어 |
| Extended | 확장 언어 |
| Specialized | 특수 언어 |
| Special Categories | 특수 카테고리 |
| Sign Languages | 수어 |
| Historical | 역사적 언어 |
| Braille | 점자 |

### 2. 번역하지 않는 것
- WIA, WIHP, IPA (약어)
- GitHub, MIT License
- 언어 이름 영어 부분 (Arabic, Hebrew 등)
- 弘益人間 (한자 그대로)
- Benefit All Humanity (영어 그대로)

### 3. 특수 케이스
```html
<!-- 철학 문구: 번역 안 함 -->
<p class="philosophy">弘益人間 · Benefit All Humanity</p>

<!-- 언어 링크: 영어명은 그대로, 한글명만 표시 -->
<span class="lang-name">Hebrew</span> 히브리어
<!-- → 둘 다 항상 표시, 번역 불필요 -->
```

---

## ✅ 체크리스트

### index.html
- [ ] 헤더 nav-links 모든 링크
- [ ] Breadcrumb 텍스트
- [ ] 히어로 subtitle
- [ ] 히어로 desc
- [ ] 버튼 텍스트
- [ ] 통계 라벨 (Languages, Tiers, etc.)
- [ ] 푸터 링크

### viewer.html
- [ ] 헤더 nav-links 모든 링크
- [ ] Breadcrumb 텍스트
- [ ] 페이지 제목
- [ ] 페이지 설명
- [ ] 통계 라벨
- [ ] 모든 Tier 섹션 제목
- [ ] Subsection 제목
- [ ] "+ 93 more languages..." 텍스트
- [ ] Error 메시지
- [ ] Back 버튼
- [ ] 푸터 링크

---

## 🚀 실행 지침

1. **GitHub에서 파일 가져오기**
   - wihp/index.html
   - wihp/viewer.html

2. **번역 속성 추가**
   - 모든 사용자 대면 텍스트에 data-en, data-ko 추가
   - 기본값은 영어 (data-en 값과 동일)

3. **테스트**
   - EN 버튼 클릭 → 영어 표시 확인
   - KO 버튼 클릭 → 한국어 표시 확인
   - localStorage 저장 확인

4. **커밋 & 푸시**
   - 커밋 메시지: "feat(wihp): Complete EN/KO translation for all UI elements"

---

## 💡 참고

### 브라우저 테스트 방법
```javascript
// 콘솔에서 실행
document.querySelectorAll('[data-en]').length
// → 번역된 요소 개수 확인

// 누락된 텍스트 찾기
document.querySelectorAll('*:not([data-en])').forEach(el => {
    if (el.textContent.trim() && el.children.length === 0) {
        console.log(el.tagName, el.textContent.trim().substring(0, 50));
    }
});
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

---
*WIA Standards | WIHP Translation Task*
