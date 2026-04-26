# WIA A11Y - Claude 번역 프롬프트 (형님용)

## 사용 방법
1. 아래 언어별 프롬프트 복사
2. Claude에게 붙여넣기
3. 결과를 SSH로 파일 저장
4. 255개 언어 완성!

---

## 일본어 (ja)

**프롬프트:**
```
다음 영어 키-값을 일본어로 번역해서 JavaScript 파일로 출력해주세요.

파일명: a11y-ja.js
변수명: window.translations_a11y_ja

**번역 금지 (고유명사):**
- WIA A11Y
- SmileStory Inc.
- World Certification Industry Association
- © 2025 SmileStory Inc. & WIA. All rights reserved.
- South Korea (2009.04.08)
- Estonia (2018.04.19)

**번역 필요:**
나머지 모든 텍스트를 자연스러운 일본어로

**출력 형식:**
// JA
window.translations_a11y_ja = {
  "skip_to_content": "メインコンテンツへスキップ",
  ... (71개 전체)
};

**영어 원문 (복사해서 사용):**
"skip_to_content": "Skip to content",
"logo_title": "WIA A11Y",
"logo_subtitle": "Accessibility Checker",
"nav_label": "Main menu",
"language_button": "Language",
"language_button_aria": "Select language (211 languages supported)",
"about": "About",
"accessibility_banner": "Perfect Accessibility: 211 Languages | Braille | TTS | Sign Language | High Contrast",
"badge_free": "✅ Free Service",
"hero_title": "E-Publication Accessibility Check",
"hero_subtitle": "Verify Korean DDA & EU EAA 2025 Compliance",
"feature_languages": "211 Languages",
"feature_braille": "Braille",
"feature_tts": "TTS",
"feature_sts": "Sign Language",
"feature_sign": "Sign Language",
"tab_url": "🌐 Website URL",
"tab_file": "📄 Upload File",
"url_label": "Enter website URL to check",
"url_help": "Enter the full URL of the website you want to check.",
"file_label": "Upload File",
"file_click": "Click to select file",
"file_formats": "PDF, EPUB, HTML, DOCX, TXT, MD",
"file_selected": "✅ Selected file:",
"button_analyze": "🔍 Start Accessibility Check",
"button_analyzing": "Analyzing...",
"features_title": "✨ Features",
"feature1_title": "Real Analysis",
"feature1_desc": "WCAG 2.1 AA actual analysis with axe-core engine",
"feature2_title": "WIA Badge Issuance",
"feature2_desc": "Automatic Gold/Silver/Certified badge for scores 70+",
"feature3_title": "Perfect Accessibility",
"feature3_desc": "211 languages, Braille, TTS, Sign Language, full keyboard support",
"cta_title": "Need accessibility improvement?",
"cta_desc": "Get professional accessibility consulting from WIA Book.",
"cta_button": "Go to WIA Book →",
"footer_desc": "Free e-publication accessibility checking service.",
"footer_wcag": "Verify compliance with WCAG 2.1 AA, Korean DDA, and EU EAA 2025",
"footer_services": "Services",
"footer_operator": "Operators",
"footer_smilestory": "SmileStory Inc.",
"footer_smilestory_location": "South Korea (2009.04.08)",
"footer_wia": "World Certification Industry Association",
"footer_wia_location": "Estonia (2018.04.19)",
"footer_copyright": "© 2025 SmileStory Inc. & WIA. All rights reserved.",
"footer_slogan": "Realizing the social value of technology",
"footer_accessibility": "This site complies with WCAG 2.1 AA standards.",
"a11y_toolbar_open": "Open accessibility toolbar",
"a11y_toolbar_close": "Close accessibility toolbar",
"a11y_toolbar_title": "♿ Accessibility Tools",
"a11y_high_contrast": "High Contrast Mode",
"a11y_font_size": "Font Size",
"a11y_tts": "Text-to-Speech (TTS)",
"a11y_sts": "Sign Language (STS)",
"a11y_braille": "Braille Display",
"a11y_welcome_tts": "Welcome to the accessibility checker.",
"result_title": "Accessibility Check Result",
"result_score": "Accessibility Score",
"result_issues": "Issues Found",
"result_compliance": "Legal Compliance",
"result_pass": "Compliant",
"result_fail": "Non-compliant",
"badge_title": "WIA Certification Badge",
"badge_congrats": "🎉 Congratulations!",
"badge_download": "Download Badge",
"badge_verify": "Verify Badge",
"error_invalid_url": "Please enter a valid URL",
"error_analysis_failed": "Analysis failed. Please try again."

**번역 후 SSH 명령어:**
sudo bash -c 'cat > /var/www/a11y.wiabook.com/languages/a11y-ja.js << "ENDF"
[Claude 번역 결과 붙여넣기]
ENDF'
```

---

## 중국어 간체 (zh-CN)

**동일한 프롬프트, 언어만 변경:**
- 파일명: a11y-zh-CN.js
- 변수명: window.translations_a11y_zh_CN
- 번역 대상 언어: 중국어 간체

---

## 스페인어 (es)

**동일한 프롬프트, 언어만 변경:**
- 파일명: a11y-es.js  
- 변수명: window.translations_a11y_es
- 번역 대상 언어: 스페인어

---

## 🤖 자동 처리 (형님이 직접 사용)

**프롬프트 템플릿:**
```
[언어명]으로 번역: a11y-[code].js, window.translations_a11y_[code]
고유명사 유지, 71개 키 전체 번역, JavaScript 형식 출력
[71개 영어 키-값]
```

**형님께서 반복:**
1. 언어 선택 (ja, zh-CN, es, fr, de...)
2. 프롬프트 복사해서 Claude에게
3. 결과를 SSH로 파일 저장
4. 다음 언어...

255개 완성!
