# WIA Language Modal 211 Component

## 🌍 완벽한 211개 언어 선택 모달

### 📦 설치 방법

#### 방법 1: 직접 include (추천)
```html
<script src="/components/language-modal-211/wia-language-modal-211.js"></script>
```

#### 방법 2: 복사해서 사용
```bash
cp /var/www/wiabooks/components/language-modal-211/wia-language-modal-211.js .
```

### 🚀 사용법

#### 기본 사용 (자동)
```html
<!-- 그냥 버튼에 language-btn 클래스만 추가 -->
<button class="language-btn">
    <i class="fas fa-globe"></i>
    <span id="currentLangDisplay">EN</span>
</button>

<!-- 컴포넌트 include -->
<script src="/components/language-modal-211/wia-language-modal-211.js"></script>
<!-- 끝! 자동으로 작동합니다 -->
```

#### 수동 사용
```javascript
// 모달 열기
WIALanguageModal.open();

// 모달 닫기
WIALanguageModal.close();

// 언어 변경
WIALanguageModal.selectLanguage('ko');
```

#### 고급 사용
```javascript
// 초기화 (자동으로 되지만 수동도 가능)
await WIALanguageModal.init();

// 커스텀 버튼에 연결
WIALanguageModal.attachToButtons('.my-custom-button');

// 언어 변경 콜백
WIALanguageModal.onLanguageChange = function(langCode) {
    console.log('Language changed to:', langCode);
    // 번역 시스템 호출 등
};
```

### ✨ 기능

- ✅ **211개 언어 지원**
- ✅ **대륙별 필터링** (Asian, European, African, Americas, Oceanic)
- ✅ **실시간 검색**
- ✅ **언어 저장** (localStorage)
- ✅ **반응형 디자인**
- ✅ **부드러운 애니메이션**
- ✅ **ESC 키로 닫기**
- ✅ **자동 버튼 연결**

### 📊 언어 데이터

자동으로 다음 경로에서 언어 데이터를 로드합니다:
1. `/wialanguages/data/languages-211-complete.json`
2. `/assets/data/languages-211.json`
3. `https://wiabook.com/wialanguages/data/languages-211-complete.json`

로드 실패시 기본 20개 언어 + 더미로 211개까지 채웁니다.

### 🎨 커스터마이징

#### 스타일 변경
```css
/* 모달 헤더 색상 변경 */
.wia-modal-header {
    background: linear-gradient(135deg, #your-color1, #your-color2);
}

/* 선택된 언어 색상 */
.wia-lang-item.selected {
    background: #your-color;
}
```

#### 언어 데이터 커스텀
```javascript
// 커스텀 언어 데이터 설정
WIALanguageModal.languages = {
    'en': { native: 'English' },
    'ko': { native: '한국어' },
    // ... 211개
};
WIALanguageModal.renderLanguages('all');
```

### 🔧 API

#### Properties
- `languages` - 언어 데이터 객체
- `currentLang` - 현재 선택된 언어
- `currentFilter` - 현재 필터 (all, popular, asian 등)
- `onLanguageChange` - 언어 변경 콜백 함수

#### Methods
- `init()` - 컴포넌트 초기화
- `open()` - 모달 열기
- `close()` - 모달 닫기
- `selectLanguage(code)` - 언어 선택
- `filterLanguages(filter)` - 필터 적용
- `searchLanguages(query)` - 언어 검색
- `attachToButtons(selector)` - 버튼에 이벤트 연결

### 📝 예제

#### 기본 예제
```html
<!DOCTYPE html>
<html>
<head>
    <title>WIA Book</title>
</head>
<body>
    <button class="language-btn">Change Language</button>
    
    <script src="/components/language-modal-211/wia-language-modal-211.js"></script>
</body>
</html>
```

#### React에서 사용
```jsx
import { useEffect } from 'react';

function LanguageSelector() {
    useEffect(() => {
        // 컴포넌트 로드
        const script = document.createElement('script');
        script.src = '/components/language-modal-211/wia-language-modal-211.js';
        document.body.appendChild(script);
        
        return () => {
            document.body.removeChild(script);
        };
    }, []);
    
    return (
        <button className="language-btn">
            Change Language
        </button>
    );
}
```

### 🐛 문제 해결

#### 모달이 안 열려요
```javascript
// 콘솔에서 확인
console.log(WIALanguageModal);  // 객체가 있어야 함
WIALanguageModal.open();  // 수동으로 열기
```

#### 언어가 211개가 안 보여요
```javascript
// 언어 개수 확인
console.log(Object.keys(WIALanguageModal.languages).length);

// 수동으로 언어 로드
await WIALanguageModal.loadLanguages();
```

#### 버튼 클릭이 안 돼요
```javascript
// 수동으로 이벤트 연결
WIALanguageModal.attachToButtons('.language-btn');
```

### 📄 라이센스
WIA Book Internal Use

### 👨‍💻 제작
PM Claude for WIA Book

### 📞 문의
다른 클로드에게 물어보세요!

---

**"다른 클로드님들, 이거 그냥 include만 하면 끝입니다!"**
