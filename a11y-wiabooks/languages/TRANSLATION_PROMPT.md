# WIA A11Y - 언어별 번역 프롬프트

## 📊 현재 상태
- **필요한 키**: 71개
- **완료**: 2개 언어 (en, ko)
- **작업 필요**: 255개 언어

## 🎯 각 언어별 프롬프트

### 사용 방법
1. 아래 언어 코드 확인
2. 해당 프롬프트 복사
3. Claude에게 전달
4. 결과를 `/var/www/a11y.wiabook.com/languages/a11y-[언어코드].js` 파일로 저장

---

## 🇯🇵 일본어 (ja)

```
다음 71개 키를 일본어로 번역해서 JavaScript 파일 형식으로 출력해주세요.

**번역 규칙:**
1. 고유명사는 그대로 유지: WIA A11Y, SmileStory Inc., World Certification Industry Association, WCAG 2.1 AA, Korean DDA, EU EAA 2025
2. 이모지는 그대로 유지: ✅, 🌐, 📄, 🔍, ✨, 🎉, ♿
3. 날짜/저작권은 그대로 유지: © 2025, (2009.04.08), (2018.04.19)
4. 출력 형식: window.translations_a11y_ja = { ... };

**영어 원문 71개 키:**
[여기에 71개 키-값 JSON 붙여넣기]

**출력 형식:**
// JA
window.translations_a11y_ja = {
  "skip_to_content": "[일본어 번역]",
  "logo_title": "WIA A11Y",
  ...
};
```

---

## 🇨🇳 중국어 간체 (zh-CN)

```
다음 71개 키를 중국어 간체로 번역해서 JavaScript 파일 형식으로 출력해주세요.

**번역 규칙:**
1. 고유명사는 그대로 유지: WIA A11Y, SmileStory Inc., World Certification Industry Association
2. WCAG 2.1 AA, Korean DDA, EU EAA 2025 → 의미 전달 위해 번역 가능
3. 이모지 유지, 날짜/저작권 유지
4. 출력 형식: window.translations_a11y_zh_CN = { ... };

**영어 원문 71개 키:**
[여기에 71개 키-값 JSON 붙여넣기]

**출력 형식:**
// ZH-CN
window.translations_a11y_zh_CN = {
  "skip_to_content": "[中文简体翻译]",
  "logo_title": "WIA A11Y",
  ...
};
```

---

## 🇪🇸 스페인어 (es)

```
다음 71개 키를 스페인어로 번역해서 JavaScript 파일 형식으로 출력해주세요.

**번역 규칙:**
1. 고유명사 유지: WIA A11Y, SmileStory Inc., World Certification Industry Association
2. WCAG, DDA, EAA 등 약어는 의미 전달 위해 번역 가능
3. 출력 형식: window.translations_a11y_es = { ... };

**영어 원문 71개 키:**
[여기에 71개 키-값 JSON 붙여넣기]

**출력 형식:**
// ES
window.translations_a11y_es = {
  "skip_to_content": "[Traducción al español]",
  "logo_title": "WIA A11Y",
  ...
};
```

---

## 🤖 자동 번역 스크립트 (Python)

```python
import anthropic
import os

client = anthropic.Anthropic(api_key=os.environ.get("ANTHROPIC_API_KEY"))

# 71개 키 JSON
base_translations = """ [여기에 71개 키 붙여넣기] """

# 번역할 언어 목록
languages = {
    "ja": "일본어",
    "zh-CN": "중국어 간체",
    "es": "스페인어",
    "fr": "프랑스어",
    "de": "독일어",
    # ... 255개 언어
}

for lang_code, lang_name in languages.items():
    prompt = f"""다음 71개 키를 {lang_name}로 번역해서 JavaScript 형식으로 출력해주세요.
    
고유명사 유지: WIA A11Y, SmileStory Inc., World Certification Industry Association
출력 형식: window.translations_a11y_{lang_code.replace("-", "_")} = {{ ... }};

{base_translations}
"""
    
    message = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=4000,
        messages=[{"role": "user", "content": prompt}]
    )
    
    # 파일 저장
    with open(f"/var/www/a11y.wiabook.com/languages/a11y-{lang_code}.js", "w", encoding="utf-8") as f:
        f.write(message.content[0].text)
    
    print(f"✅ {lang_code} 완료")
```

---

## 📝 수동 번역 프롬프트 (형님용)

```
다음 71개 키를 [언어명]으로 번역해서 JavaScript 파일로 만들어주세요.

파일명: a11y-[언어코드].js
변수명: window.translations_a11y_[언어코드_언더스코어]

고유명사는 절대 번역 금지:
- WIA A11Y
- SmileStory Inc.
- World Certification Industry Association  
- © 2025 SmileStory Inc. & WIA. All rights reserved.
- South Korea (2009.04.08)
- Estonia (2018.04.19)

기술 용어는 의미 전달 위해 번역 가능:
- WCAG 2.1 AA
- Korean DDA (한국 장애인차별금지법)
- EU EAA 2025

**영어 원문:**
[71개 키-값 붙여넣기]

**출력:**
/var/www/a11y.wiabook.com/languages/a11y-[언어코드].js 파일로 저장
```
