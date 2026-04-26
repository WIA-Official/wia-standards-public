# WIA A11Y Checker - 211 Languages Support

## 📚 Language Files

This directory contains 211 language translation files for the WIA Accessibility Checker.

### File Naming Convention

- All files follow the pattern: `a11y-{language_code}.js`
- Example: `a11y-ko.js`, `a11y-en.js`, `a11y-ja.js`

### File Structure

Each language file contains:

```javascript
window.translations_a11y_{language_code} = {
  "skip_to_content": "...",
  "logo_subtitle": "...",
  // ... all translation keys
};
```

### Usage

1. Load the language loader:
```html
<script src="/languages/a11y-loader.js"></script>
```

2. Load a specific language:
```javascript
await A11yTranslationLoader.loadLanguage('ko');
```

3. Translate the page:
```javascript
await A11yTranslationLoader.translatePage('ko');
```

4. Get a translation:
```javascript
const text = A11yTranslationLoader.t('skip_to_content');
```

### HTML Attributes

Use these data attributes for automatic translation:

- `data-a11y="key"` - Translates textContent
- `data-a11y-html="key"` - Translates innerHTML
- `data-a11y-placeholder="key"` - Translates placeholder
- `data-a11y-aria="key"` - Translates aria-label

Example:
```html
<button data-a11y="button_analyze">Start Check</button>
<input data-a11y-placeholder="url_label" />
<div data-a11y-aria="a11y_toolbar_title"></div>
```

### Supported Languages (211)

Major Languages:
- Korean (ko)
- English (en, en-US, en-GB, en-AU, en-CA, en-NZ)
- Japanese (ja)
- Chinese Simplified (zh-CN)
- Chinese Traditional (zh-HK, zh-TW)
- Spanish (es, es-AR, es-CL, es-CO, es-MX, es-PE)
- French (fr, fr-CA)
- German (de)
- Russian (ru)
- Arabic (ar)
- Portuguese (pt, pt-BR)
- Italian (it)
- And 190+ more languages...

### RTL (Right-to-Left) Support

The following languages automatically enable RTL mode:
- Arabic (ar)
- Hebrew (he)
- Persian (fa)
- Urdu (ur)

### Translation Keys

All translation files include these keys:

#### Header
- skip_to_content
- logo_subtitle
- nav_label
- language_button
- language_button_aria
- about
- accessibility_banner

#### Hero Section
- badge_free
- hero_title
- hero_subtitle
- feature_languages
- feature_braille
- feature_tts
- feature_sts

#### Checker
- tab_url
- tab_file
- url_label
- url_help
- file_label
- file_click
- file_formats
- file_selected
- button_analyze
- button_analyzing

#### Features
- features_title
- feature1_title
- feature1_desc
- feature2_title
- feature2_desc
- feature3_title
- feature3_desc

#### CTA
- cta_title
- cta_desc
- cta_button

#### Footer
- footer_desc
- footer_services
- footer_operator
- footer_slogan

#### Accessibility Toolbar
- a11y_toolbar_open
- a11y_toolbar_close
- a11y_toolbar_title
- a11y_high_contrast
- a11y_font_size
- a11y_tts
- a11y_sts
- a11y_braille
- a11y_welcome_tts

#### Result
- result_title
- result_score
- result_issues
- result_compliance
- result_pass
- result_fail

#### Badge
- badge_title
- badge_congrats
- badge_download
- badge_verify

#### Errors
- error_invalid_url
- error_analysis_failed

### Adding New Languages

1. Create a new file: `a11y-{code}.js`
2. Copy the template from `a11y-en.js`
3. Translate all keys
4. Test with `A11yTranslationLoader.loadLanguage('code')`

### Notes

- All language files are loaded dynamically on demand
- English (en) is used as fallback for missing translations
- Language preference is saved in localStorage
- Browser language is auto-detected on first visit

---

© 2025 SmileStory Inc. & WIA  
"기술의 사회적 가치 실현 - Realizing the social value of technology"
