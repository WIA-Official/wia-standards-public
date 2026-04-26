# 🌍 WIA Language Modal 211 Component - 사용 가이드

## 📋 개요
WIA Family 프로젝트에서 사용하는 **211개 언어 지원 모달 컴포넌트**입니다.
모든 WIA 프로젝트에서 통일된 언어 선택 UI를 제공합니다.

## 🚀 빠른 시작

### 1. 기본 사용법
```html
<!-- HTML에 버튼 추가 -->
<button class="language-btn" onclick="openLanguageModal()">
    <i class="fas fa-globe"></i>
    <span id="currentLangDisplay">EN</span>
    <i class="fas fa-chevron-down"></i>
</button>

<!-- 스크립트 로드 (</body> 직전) -->
<script src="/components/language-modal-211/wia-language-modal-211.js"></script>
```

### 2. 기본 CSS 스타일
```css
.language-btn {
    background: var(--accent);
    color: white;
    border: 2px solid var(--accent);
    padding: 12px 18px;
    border-radius: 25px;
    cursor: pointer;
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 600;
    transition: all 0.3s ease;
}

.language-btn:hover {
    background: transparent;
    color: var(--accent);
}
```

## 💻 완전한 예제

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WIA Project - 언어 모달 테스트</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
    <style>
        :root {
            --accent: #667eea;
        }
        
        .language-btn {
            background: var(--accent);
            color: white;
            border: 2px solid var(--accent);
            padding: 12px 18px;
            border-radius: 25px;
            cursor: pointer;
            display: flex;
            align-items: center;
            gap: 8px;
            font-weight: 600;
            transition: all 0.3s ease;
        }
        
        .language-btn:hover {
            background: transparent;
            color: var(--accent);
        }
    </style>
</head>
<body>
    <div style="padding: 50px; text-align: center;">
        <h1>🌍 WIA Language Modal Test</h1>
        
        <button class="language-btn" onclick="openLanguageModal()">
            <i class="fas fa-globe"></i>
            <span id="currentLangDisplay">EN</span>
            <i class="fas fa-chevron-down"></i>
        </button>
    </div>

    <!-- WIA Language Modal Component -->
    <script src="/components/language-modal-211/wia-language-modal-211.js"></script>
    
    <script>
        // 언어 변경 이벤트 리스너
        window.addEventListener('wia-language-changed', function(event) {
            const newLang = event.detail.language;
            console.log('언어 변경됨:', newLang);
            
            // 현재 언어 표시 업데이트
            const display = document.getElementById('currentLangDisplay');
            if (display) {
                display.textContent = newLang.split('-')[0].toUpperCase().substring(0, 2);
            }
            
            // 로컬스토리지에 저장
            localStorage.setItem('wia_language', newLang);
        });
        
        // 페이지 로드시 저장된 언어 표시
        document.addEventListener('DOMContentLoaded', function() {
            const savedLang = localStorage.getItem('wia_language') || 'en';
            const display = document.getElementById('currentLangDisplay');
            if (display) {
                display.textContent = savedLang.split('-')[0].toUpperCase().substring(0, 2);
            }
        });
    </script>
</body>
</html>
```

## 🎯 주요 기능

### ✅ 자동 기능
- **211개 언어** 자동 로드
- **5열 그리드** 레이아웃
- **검색 기능** 내장
- **키보드 네비게이션** (ESC키로 닫기)
- **반응형 디자인**

### ✅ 이벤트 시스템
```javascript
// 언어 변경 감지
window.addEventListener('wia-language-changed', function(event) {
    const selectedLang = event.detail.language; // 'ko', 'en', 'ja' 등
    const languageData = event.detail.data;     // 언어 상세 정보
    
    console.log('선택된 언어:', selectedLang);
    
    // 여기서 필요한 작업 수행
    updatePageLanguage(selectedLang);
});
```

## 📁 프로젝트별 설정

### WIA Book
```javascript
localStorage.setItem('wiabooks_language', newLang);
const savedLang = localStorage.getItem('wiabooks_language') || 'en';
```

### WIA Academy  
```javascript
localStorage.setItem('wiaacademy_language', newLang);
const savedLang = localStorage.getItem('wiaacademy_language') || 'en';
```

### 기타 WIA 프로젝트
```javascript
localStorage.setItem('wia_language', newLang);
const savedLang = localStorage.getItem('wia_language') || 'en';
```

## 🛠️ 커스터마이징

### 모달 스타일 수정
```css
/* 컴포넌트 로드 후 CSS 오버라이드 */
#wiaLanguageModal211 {
    background: rgba(0, 0, 0, 0.9) !important;
}

#wiaLanguageModal211 .modal-content {
    background: linear-gradient(135deg, #1e293b, #334155) !important;
    border-radius: 20px !important;
}
```

### 버튼 스타일 커스터마이징
```css
.language-btn {
    /* 원하는 스타일로 수정 */
    background: linear-gradient(135deg, #667eea, #764ba2);
    border-radius: 50px;
    box-shadow: 0 4px 15px rgba(102, 126, 234, 0.3);
}
```

## ❌ 주의사항

### 하지 말아야 할 것들:
1. **구버전 언어모달과 함께 사용 금지**
   ```javascript
   // ❌ 이런 함수들 같이 쓰면 안됨
   function loadLanguages() { ... }
   function initializeLanguageModal() { ... }
   ```

2. **중복 스크립트 로드 금지**
   ```html
   <!-- ❌ 이런식으로 중복 로드 하면 안됨 -->
   <script src="/components/language-modal-211/wia-language-modal-211.js"></script>
   <script src="old-language-system.js"></script>
   ```

3. **ID 충돌 방지**
   ```html
   <!-- ❌ 이미 컴포넌트에서 사용중인 ID -->
   <div id="wiaLanguageModal211">내 모달</div>
   ```

## 🔧 문제 해결

### 모달이 안열려요!
```javascript
// 브라우저 콘솔에서 확인
console.log('WIALang211 로드됨?', typeof window.WIALang211);

// 수동 초기화
if (window.WIALang211) {
    window.WIALang211.init();
}
```

### 언어 변경이 감지 안돼요!
```javascript
// 이벤트 리스너가 제대로 등록됐는지 확인
document.addEventListener('DOMContentLoaded', function() {
    window.addEventListener('wia-language-changed', function(event) {
        console.log('언어 변경 이벤트 발생:', event.detail);
    });
});
```

## 📞 개발자 연락처
- **CEO**: 연삼흠 박사 (Yeon Sam-heum, Ph.D.)
- **Email**: support@wia.family
- **Component 작성**: PM Claude → 기술 동생 Claude

## 🎉 마지막으로
이 컴포넌트는 **WIA Family 생태계**의 핵심 컴포넌트입니다!
모든 WIA 프로젝트에서 **통일된 사용자 경험**을 제공하기 위해 만들어졌어요.

**Happy Coding! 🚀**

