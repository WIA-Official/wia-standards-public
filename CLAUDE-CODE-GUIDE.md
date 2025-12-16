# WIA Standards - Claude Code Generation Guide

## Overview

This guide defines how Claude Code should generate WIA Standards packages.
Each standard includes technical specifications, simulator, and ebook-ready HTML.

**Price:** $99 per standard ebook
**Languages:** English (en), Korean (ko)
**Format:** HTML (WordPress compatible for PDF/EPUB export)

---

## File Structure

```
{standard-name}/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-ALGORITHMS.md      # Includes Rust implementation
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── simulator/
│   ├── index.html                 # Interactive simulator
│   └── languages/
│       ├── en.js
│       ├── ko.js
│       ├── loader.js
│       └── modal.js
├── ebook/
│   ├── en/
│   │   ├── 00-cover.html
│   │   ├── 01-introduction.html
│   │   ├── 02-phase1-data-format.html
│   │   ├── 03-phase2-algorithms.html
│   │   ├── 04-phase3-protocol.html
│   │   ├── 05-phase4-integration.html
│   │   ├── 06-simulator-guide.html
│   │   ├── 07-case-studies.html
│   │   ├── 08-faq.html
│   │   └── 09-glossary.html
│   └── ko/
│       ├── 00-cover.html
│       ├── 01-introduction.html
│       ├── ... (same structure)
│       └── 09-glossary.html
└── README.md
```

---

## Critical Rules

### 1. NO EMOJI - Use Text Labels

PDF/EPUB generators cannot render emojis properly. Use text labels instead.

```html
<!-- WRONG - Emoji will break -->
<h2>🐾 Pet Health Passport</h2>
<h2>🏥 Healthcare Standard</h2>
<h2>📊 Data Format</h2>

<!-- CORRECT - Text labels -->
<h2>[PET] Pet Health Passport</h2>
<h2>[HEALTH] Healthcare Standard</h2>
<h2>[DATA] Data Format</h2>
```

**Label Reference:**
| Category | Label |
|----------|-------|
| Pet/Animal | [PET] |
| Healthcare | [HEALTH] |
| Education | [EDU] |
| Finance | [FIN] |
| Manufacturing | [MFG] |
| Data/Format | [DATA] |
| Algorithm | [ALGO] |
| Protocol | [PROTO] |
| Integration | [INT] |
| Warning | [!] |
| Important | [*] |
| Note | [i] |
| Check/Done | [v] |
| Cross/Fail | [x] |

---

### 2. Ebook HTML Template

Each ebook chapter must follow this structure:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{Chapter Title} - WIA {Standard Name}</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Noto Sans', 'Nanum Gothic', sans-serif;
            line-height: 1.8;
            color: #333;
            max-width: 800px;
            margin: 0 auto;
            padding: 40px 20px;
        }
        h1 { font-size: 28pt; margin-bottom: 20px; color: #1a1a2e; }
        h2 { font-size: 18pt; margin: 30px 0 15px; color: #16213e; border-bottom: 2px solid #4a90e2; padding-bottom: 5px; }
        h3 { font-size: 14pt; margin: 20px 0 10px; color: #1f4068; }
        p { margin-bottom: 15px; text-align: justify; }
        code { background: #f4f4f4; padding: 2px 6px; border-radius: 3px; font-family: 'Courier New', monospace; }
        pre { background: #1a1a2e; color: #e0e0e0; padding: 20px; border-radius: 8px; overflow-x: auto; margin: 20px 0; }
        pre code { background: none; color: inherit; }
        table { width: 100%; border-collapse: collapse; margin: 20px 0; }
        th, td { border: 1px solid #ddd; padding: 12px; text-align: left; }
        th { background: #4a90e2; color: white; }
        tr:nth-child(even) { background: #f9f9f9; }
        .note { background: #e7f3ff; border-left: 4px solid #4a90e2; padding: 15px; margin: 20px 0; }
        .warning { background: #fff3e0; border-left: 4px solid #ff9800; padding: 15px; margin: 20px 0; }
        .label { font-weight: bold; color: #4a90e2; }
    </style>
</head>
<body>
    <header>
        <p style="color: #666; font-size: 10pt;">WIA Standards | {Standard Name} | Chapter {N}</p>
    </header>
    
    <main>
        <h1>{Chapter Title}</h1>
        
        <!-- Content here -->
        
    </main>
    
    <footer style="margin-top: 50px; padding-top: 20px; border-top: 1px solid #ddd; color: #666; font-size: 9pt;">
        <p>Copyright 2025 WIA Standards (wiastandards.com) | SmileStory Inc.</p>
        <p>Published by WIA Books (wiabooks.store) | Price: $99</p>
    </footer>
</body>
</html>
```

---

### 3. Simulator Template

Use WIA Books header/footer. Languages: English, Korean only.

```html
<!DOCTYPE html>
<html lang="en" data-tool-name="{standard-id}" data-tool-category="{category}">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{Standard Name} Simulator | WIA Standards</title>
    <link rel="icon" type="image/png" href="https://wiastandards.com/images/favi.png">
    <style>
        /* Base styles */
        * { margin: 0; padding: 0; box-sizing: border-box; }
        :root {
            --primary: #4a90e2;
            --secondary: #1a1a2e;
            --accent: #00d4ff;
            --bg: #0a0e27;
            --surface: #151937;
            --text: #e0e0e0;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
        }
        
        /* Header */
        .header {
            background: var(--surface);
            padding: 15px 30px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid rgba(255,255,255,0.1);
        }
        .logo {
            display: flex;
            align-items: center;
            gap: 10px;
            color: white;
            text-decoration: none;
            font-weight: bold;
        }
        .logo img { height: 32px; }
        .nav-links { display: flex; gap: 20px; align-items: center; }
        .nav-link { color: var(--text); text-decoration: none; }
        .nav-link:hover { color: var(--accent); }
        .lang-btn {
            background: var(--primary);
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 6px;
            cursor: pointer;
        }
        
        /* Main */
        .main { max-width: 1200px; margin: 0 auto; padding: 40px 20px; }
        .tool-header { text-align: center; margin-bottom: 40px; }
        .tool-title { font-size: 32px; color: white; margin-bottom: 10px; }
        .tool-tagline { color: var(--accent); font-size: 18px; }
        
        /* Simulator Panel */
        .sim-panel {
            background: var(--surface);
            border-radius: 12px;
            padding: 30px;
            margin-bottom: 30px;
        }
        .sim-title { font-size: 20px; margin-bottom: 20px; color: white; }
        
        /* CTA Section */
        .cta-section {
            background: linear-gradient(135deg, var(--primary), #6c5ce7);
            border-radius: 12px;
            padding: 40px;
            text-align: center;
            margin: 40px 0;
        }
        .cta-title { font-size: 24px; color: white; margin-bottom: 15px; }
        .cta-desc { color: rgba(255,255,255,0.9); margin-bottom: 20px; }
        .cta-buttons { display: flex; gap: 20px; justify-content: center; flex-wrap: wrap; }
        .cta-btn {
            background: white;
            color: var(--primary);
            padding: 15px 30px;
            border-radius: 8px;
            text-decoration: none;
            font-weight: bold;
            transition: transform 0.3s;
        }
        .cta-btn:hover { transform: translateY(-3px); }
        .cta-btn.secondary { background: transparent; border: 2px solid white; color: white; }
        
        /* Footer */
        .footer {
            background: var(--surface);
            padding: 40px 30px;
            margin-top: 60px;
        }
        .footer-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 30px; max-width: 1200px; margin: 0 auto; }
        .footer-section h4 { color: white; margin-bottom: 15px; }
        .footer-links { list-style: none; }
        .footer-links a { color: var(--text); text-decoration: none; display: block; padding: 5px 0; }
        .footer-links a:hover { color: var(--accent); }
        .footer-bottom { text-align: center; padding-top: 30px; margin-top: 30px; border-top: 1px solid rgba(255,255,255,0.1); color: #666; font-size: 12px; }
        
        @media (max-width: 768px) {
            .footer-grid { grid-template-columns: 1fr 1fr; }
            .nav-links { display: none; }
        }
    </style>
</head>
<body>
    <!-- Header -->
    <header class="header">
        <a href="https://wiabooks.store" class="logo">
            <img src="https://wiabooks.store/wia-book-icon.png" alt="WIA">
            <span>WIA Books</span>
        </a>
        <nav class="nav-links">
            <a href="https://wiastandards.com" class="nav-link">WIA Standards</a>
            <a href="https://wia.family" class="nav-link">WIA Family</a>
            <a href="https://thekoreantoday.global" class="nav-link">KONAN</a>
            <button class="lang-btn" onclick="toggleLanguage()">
                <span id="currentLang">EN</span>
            </button>
        </nav>
    </header>
    
    <!-- Main Content -->
    <main class="main">
        <div class="tool-header">
            <h1 class="tool-title" data-i18n="toolTitle">{Standard Name} Simulator</h1>
            <p class="tool-tagline" data-i18n="toolTagline">{Standard Tagline}</p>
        </div>
        
        <!-- Simulator Panel -->
        <div class="sim-panel">
            <h2 class="sim-title" data-i18n="simTitle">Interactive Simulation</h2>
            <!-- Simulator content here -->
        </div>
        
        <!-- CTA Section -->
        <div class="cta-section">
            <h3 class="cta-title" data-i18n="ctaTitle">Get the Complete Standard Guide</h3>
            <p class="cta-desc" data-i18n="ctaDesc">100+ pages technical documentation with code examples</p>
            <div class="cta-buttons">
                <a href="https://wiabooks.store/standards/{standard-id}/" class="cta-btn" data-i18n="ctaEbook">
                    Get Ebook - $99
                </a>
                <a href="https://cert.wiastandards.com" class="cta-btn secondary" data-i18n="ctaCert">
                    Get Certified
                </a>
            </div>
        </div>
    </main>
    
    <!-- Footer -->
    <footer class="footer">
        <div class="footer-grid">
            <div class="footer-section">
                <h4>About WIA Books</h4>
                <p style="color: var(--text); font-size: 14px;">
                    World's most advanced AR/VR/WIAverse reading platform with 200+ tools.
                </p>
            </div>
            <div class="footer-section">
                <h4>WIA Standards</h4>
                <ul class="footer-links">
                    <li><a href="https://wiastandards.com">Standards Portal</a></li>
                    <li><a href="https://cert.wiastandards.com">Certification</a></li>
                    <li><a href="https://sim.wiastandards.com">Device Simulator</a></li>
                </ul>
            </div>
            <div class="footer-section">
                <h4>WIA Family</h4>
                <ul class="footer-links">
                    <li><a href="https://wiabooks.store">WIA Books</a></li>
                    <li><a href="https://wiatrip.com">WIA Trip</a></li>
                    <li><a href="https://thekoreantoday.com">The Korean Today</a></li>
                </ul>
            </div>
            <div class="footer-section">
                <h4>Resources</h4>
                <ul class="footer-links">
                    <li><a href="https://wiabooks.store/docs/api-documentation.html">API Docs</a></li>
                    <li><a href="https://github.com/anthropics/claude-code">GitHub</a></li>
                </ul>
            </div>
        </div>
        <div class="footer-bottom">
            <p>Publisher: WIA Books | SmileStory Inc. | CEO: Yeon Sam-Heum</p>
            <p>Copyright 2025 WIA Standards - All Rights Reserved</p>
        </div>
    </footer>
    
    <!-- Language Scripts -->
    <script src="./languages/en.js"></script>
    <script src="./languages/ko.js"></script>
    <script src="./languages/loader.js"></script>
</body>
</html>
```

---

### 4. Language Files (2 Languages Only)

**languages/en.js:**
```javascript
const translations_en = {
    "toolTitle": "{Standard Name} Simulator",
    "toolTagline": "{Standard Description}",
    "simTitle": "Interactive Simulation",
    "ctaTitle": "Get the Complete Standard Guide",
    "ctaDesc": "100+ pages technical documentation with code examples",
    "ctaEbook": "Get Ebook - $99",
    "ctaCert": "Get Certified"
};
if (typeof window !== 'undefined') window.translations_en = translations_en;
```

**languages/ko.js:**
```javascript
const translations_ko = {
    "toolTitle": "{표준명} 시뮬레이터",
    "toolTagline": "{표준 설명}",
    "simTitle": "인터랙티브 시뮬레이션",
    "ctaTitle": "완벽한 표준 가이드를 받아보세요",
    "ctaDesc": "100페이지 이상의 기술 문서와 코드 예제",
    "ctaEbook": "전자책 구매 - $99",
    "ctaCert": "인증 받기"
};
if (typeof window !== 'undefined') window.translations_ko = translations_ko;
```

**languages/loader.js:**
```javascript
const SimTranslation = {
    current: 'en',
    t(key) {
        const lang = this.current === 'ko' ? translations_ko : translations_en;
        return lang[key] || translations_en[key] || key;
    },
    apply() {
        document.querySelectorAll('[data-i18n]').forEach(el => {
            el.textContent = this.t(el.dataset.i18n);
        });
    },
    toggle() {
        this.current = this.current === 'en' ? 'ko' : 'en';
        document.getElementById('currentLang').textContent = this.current.toUpperCase();
        localStorage.setItem('wia_lang', this.current);
        this.apply();
    }
};

function toggleLanguage() { SimTranslation.toggle(); }

document.addEventListener('DOMContentLoaded', () => {
    SimTranslation.current = localStorage.getItem('wia_lang') || 'en';
    document.getElementById('currentLang').textContent = SimTranslation.current.toUpperCase();
    SimTranslation.apply();
});
```

---

### 5. Ebook Chapter Guidelines

| Chapter | Content | Pages |
|---------|---------|-------|
| 00-cover | Title, Author, Version, Copyright | 2 |
| 01-introduction | Overview, Philosophy (Hongik Ingan), Scope | 8 |
| 02-phase1 | Data Format specification | 15 |
| 03-phase2 | Algorithms + Rust code | 25 |
| 04-phase3 | Protocol specification | 15 |
| 05-phase4 | Integration guide | 15 |
| 06-simulator | How to use the simulator | 10 |
| 07-case-studies | Real-world implementation examples | 10 |
| 08-faq | Frequently asked questions | 5 |
| 09-glossary | Technical terms | 5 |
| **Total** | | **~110 pages** |

---

### 6. Phase 2 Structure (Algorithms + Rust)

Phase 2 must include both algorithm specification AND Rust implementation:

```markdown
# Phase 2: {Standard Name} Algorithms

## 1. Overview
[Algorithm overview]

## 2. Core Algorithms

### 2.1 {Algorithm 1 Name}
[Description]

#### Specification
[Formal specification]

#### Rust Implementation
\`\`\`rust
// {Algorithm 1} implementation
pub struct Algorithm1 {
    // fields
}

impl Algorithm1 {
    pub fn new() -> Self { ... }
    pub fn execute(&self, input: Input) -> Output { ... }
}
\`\`\`

### 2.2 {Algorithm 2 Name}
[Continue pattern...]
```

---

### 7. Philosophy Reminder

Every standard MUST include this philosophy section:

```html
<div class="note">
    <p class="label">[PHILOSOPHY]</p>
    <p><strong>Hongik Ingan (弘益人間)</strong> - Benefit All Humanity</p>
    <p>This standard is designed to serve all people, including those with disabilities,
    elderly, refugees, and underserved communities worldwide.</p>
</div>
```

---

## Summary Checklist

Before submitting a standard package, verify:

- [ ] NO EMOJI anywhere (use text labels)
- [ ] Ebook chapters are HTML (not MD)
- [ ] Languages: English + Korean only
- [ ] Simulator has WIA Books header/footer
- [ ] CTA links to wiabooks.store ($99) and cert.wiastandards.com
- [ ] Phase 2 includes Rust implementation
- [ ] Hongik Ingan philosophy included
- [ ] ~100+ pages total content

---

## Example Request to Claude Code

```
Create WIA Standard: Smart Wheelchair Accessibility

Include:
- Phase 1-4 specifications
- Simulator (HTML with en/ko languages)
- Ebook chapters (HTML format, en/ko)
- No emoji - use text labels
- CTA: Ebook $99, Certification link
- Follow CLAUDE-CODE-GUIDE.md structure
```

---

**Document Version:** 1.0.0
**Last Updated:** 2025-12-17
**Author:** WIA Standards Team
