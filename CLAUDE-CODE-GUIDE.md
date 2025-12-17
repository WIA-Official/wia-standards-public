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

---

## 8. Visual Simulator Template (REQUIRED)

**IMPORTANT:** All simulators MUST use this visual template. Simple forms are NOT acceptable.
The simulator should look professional and impressive like wiabooks.store/reader/simulators/

### Complete Visual Simulator HTML Template

```html
<!DOCTYPE html>
<html lang="en" data-tool-name="{standard-id}" data-tool-category="{category}">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{Standard Name} Simulator | WIA Standards</title>
    <meta name="description" content="Interactive simulator for WIA {Standard Name} standard. Test and explore the specification.">
    <link rel="icon" type="image/png" href="https://wiastandards.com/images/favi.png">
    
    <style>
        /* ========== CSS RESET & VARIABLES ========== */
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        :root {
            /* Theme Colors - Customize per standard */
            --primary: #1E88E5;
            --secondary: #5E35B1;
            --accent: #00ACC1;
            --gradient: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            --glow-gradient: linear-gradient(135deg, #00BCD4 0%, #5E35B1 50%, #E91E63 100%);
            
            /* Neutral */
            --text: #E0E0E0;
            --text-secondary: #A0A0A0;
            --bg: #0A0E27;
            --surface: #151937;
            --surface-hover: #1e2548;
            --border: rgba(255,255,255,0.1);
            
            /* Status */
            --success: #10B981;
            --warning: #F59E0B;
            --error: #EF4444;
            
            /* Effects */
            --shadow: 0 8px 32px rgba(0,0,0,0.3);
            --shadow-glow: 0 0 30px rgba(30, 136, 229, 0.3);
            --radius: 12px;
            --radius-lg: 20px;
            
            /* Animation */
            --transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
            line-height: 1.6;
        }
        
        /* ========== RAINBOW HEADER ========== */
        .header {
            background: linear-gradient(270deg, #ff6b6b, #4ecdc4, #45b7d1, #96ceb4, #ffeaa7, #dfe6e9, #ff6b6b);
            background-size: 400% 400%;
            animation: rainbowFlow 15s ease-in-out infinite;
            padding: 15px 30px;
            position: sticky;
            top: 0;
            z-index: 1000;
        }
        
        @keyframes rainbowFlow {
            0%, 100% { background-position: 0% 50%; }
            50% { background-position: 100% 50%; }
        }
        
        .header-container {
            max-width: 1200px;
            margin: 0 auto;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .logo {
            display: flex;
            align-items: center;
            gap: 10px;
            color: white;
            text-decoration: none;
            font-weight: bold;
            font-size: 20px;
            text-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        
        .logo img { height: 32px; border-radius: 50%; }
        
        .nav-links {
            display: flex;
            gap: 20px;
            align-items: center;
        }
        
        .nav-link {
            color: white;
            text-decoration: none;
            text-shadow: 0 1px 2px rgba(0,0,0,0.2);
            transition: var(--transition);
        }
        
        .nav-link:hover { transform: translateY(-2px); }
        
        .lang-switch {
            display: flex;
            gap: 5px;
        }
        
        .lang-btn {
            padding: 8px 16px;
            border: 2px solid white;
            background: transparent;
            color: white;
            border-radius: 20px;
            cursor: pointer;
            font-weight: 600;
            transition: var(--transition);
        }
        
        .lang-btn.active, .lang-btn:hover {
            background: white;
            color: var(--primary);
        }
        
        /* ========== MAIN CONTENT ========== */
        .main {
            max-width: 1200px;
            margin: 0 auto;
            padding: 40px 20px;
        }
        
        /* ========== HERO SECTION ========== */
        .hero {
            text-align: center;
            margin-bottom: 50px;
        }
        
        .hero-icon {
            font-size: 64px;
            margin-bottom: 20px;
            display: inline-block;
            animation: float 3s ease-in-out infinite;
        }
        
        @keyframes float {
            0%, 100% { transform: translateY(0); }
            50% { transform: translateY(-10px); }
        }
        
        .hero-title {
            font-size: 48px;
            font-weight: 800;
            background: var(--glow-gradient);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 15px;
        }
        
        .hero-subtitle {
            font-size: 20px;
            color: var(--accent);
            margin-bottom: 10px;
        }
        
        .hero-desc {
            font-size: 18px;
            color: var(--text-secondary);
            max-width: 700px;
            margin: 0 auto;
        }
        
        /* ========== STATS BANNER ========== */
        .stats-banner {
            background: var(--gradient);
            border-radius: var(--radius-lg);
            padding: 30px;
            margin-bottom: 40px;
            position: relative;
            overflow: hidden;
            box-shadow: var(--shadow-glow);
        }
        
        .stats-banner::before {
            content: '';
            position: absolute;
            top: -50%;
            right: -50%;
            width: 200%;
            height: 200%;
            background: radial-gradient(circle, rgba(255,255,255,0.1) 0%, transparent 70%);
            animation: pulse 4s ease-in-out infinite;
        }
        
        @keyframes pulse {
            0%, 100% { transform: scale(0.8); opacity: 0.5; }
            50% { transform: scale(1.2); opacity: 1; }
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 20px;
            position: relative;
            z-index: 1;
        }
        
        .stat-item {
            text-align: center;
            padding: 20px;
            background: rgba(255,255,255,0.1);
            border-radius: var(--radius);
            backdrop-filter: blur(10px);
        }
        
        .stat-value {
            font-size: 32px;
            font-weight: 800;
            color: white;
        }
        
        .stat-label {
            font-size: 14px;
            color: rgba(255,255,255,0.9);
            margin-top: 5px;
        }
        
        /* ========== INTERACTIVE TABS ========== */
        .tabs-container {
            margin-bottom: 40px;
        }
        
        .tabs-header {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            justify-content: center;
            margin-bottom: 30px;
        }
        
        .tab-btn {
            padding: 12px 24px;
            background: var(--surface);
            border: 2px solid var(--border);
            color: var(--text);
            border-radius: 30px;
            cursor: pointer;
            font-weight: 600;
            transition: var(--transition);
        }
        
        .tab-btn:hover {
            border-color: var(--primary);
            transform: translateY(-2px);
        }
        
        .tab-btn.active {
            background: var(--gradient);
            border-color: transparent;
            color: white;
            box-shadow: var(--shadow-glow);
        }
        
        .tab-content {
            display: none;
            animation: fadeIn 0.5s ease;
        }
        
        .tab-content.active { display: block; }
        
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(20px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        /* ========== SIMULATOR PANEL ========== */
        .sim-panel {
            background: var(--surface);
            border-radius: var(--radius-lg);
            padding: 30px;
            margin-bottom: 30px;
            border: 1px solid var(--border);
            transition: var(--transition);
        }
        
        .sim-panel:hover {
            border-color: var(--primary);
            box-shadow: var(--shadow-glow);
        }
        
        .panel-title {
            font-size: 24px;
            font-weight: 700;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .panel-title .icon {
            width: 40px;
            height: 40px;
            background: var(--gradient);
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 20px;
        }
        
        /* ========== FORM ELEMENTS ========== */
        .form-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 20px;
        }
        
        .form-group {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }
        
        .form-group.full-width {
            grid-column: 1 / -1;
        }
        
        .form-label {
            font-size: 14px;
            font-weight: 600;
            color: var(--text-secondary);
        }
        
        .form-input, .form-select {
            padding: 14px 18px;
            background: var(--bg);
            border: 2px solid var(--border);
            border-radius: var(--radius);
            color: var(--text);
            font-size: 16px;
            transition: var(--transition);
        }
        
        .form-input:focus, .form-select:focus {
            outline: none;
            border-color: var(--primary);
            box-shadow: 0 0 0 4px rgba(30, 136, 229, 0.2);
        }
        
        .form-input::placeholder {
            color: var(--text-secondary);
        }
        
        /* ========== BUTTONS ========== */
        .btn {
            padding: 14px 28px;
            border: none;
            border-radius: var(--radius);
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: var(--transition);
            display: inline-flex;
            align-items: center;
            gap: 10px;
        }
        
        .btn-primary {
            background: var(--gradient);
            color: white;
            box-shadow: 0 4px 15px rgba(30, 136, 229, 0.4);
        }
        
        .btn-primary:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 25px rgba(30, 136, 229, 0.5);
        }
        
        .btn-secondary {
            background: var(--surface);
            color: var(--text);
            border: 2px solid var(--border);
        }
        
        .btn-secondary:hover {
            border-color: var(--primary);
            background: var(--surface-hover);
        }
        
        .btn-success {
            background: linear-gradient(135deg, var(--success), #059669);
            color: white;
        }
        
        /* ========== RESULT DISPLAY ========== */
        .result-panel {
            background: linear-gradient(135deg, rgba(16, 185, 129, 0.1), rgba(5, 150, 105, 0.1));
            border: 2px solid var(--success);
            border-radius: var(--radius-lg);
            padding: 30px;
            margin-top: 30px;
            animation: slideUp 0.5s ease;
        }
        
        @keyframes slideUp {
            from { opacity: 0; transform: translateY(30px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        .result-header {
            display: flex;
            align-items: center;
            gap: 15px;
            margin-bottom: 20px;
        }
        
        .result-icon {
            width: 50px;
            height: 50px;
            background: var(--success);
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            color: white;
        }
        
        .result-title {
            font-size: 24px;
            font-weight: 700;
            color: var(--success);
        }
        
        .result-data {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
        }
        
        .result-item {
            background: rgba(255,255,255,0.05);
            padding: 15px;
            border-radius: var(--radius);
        }
        
        .result-item-label {
            font-size: 12px;
            color: var(--text-secondary);
            margin-bottom: 5px;
        }
        
        .result-item-value {
            font-size: 18px;
            font-weight: 600;
        }
        
        /* ========== VISUALIZATION CANVAS ========== */
        .viz-container {
            background: var(--bg);
            border-radius: var(--radius);
            padding: 20px;
            min-height: 300px;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            overflow: hidden;
        }
        
        .viz-canvas {
            width: 100%;
            height: 300px;
            border-radius: var(--radius);
        }
        
        /* ========== CARDS GRID ========== */
        .cards-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 20px;
            margin: 30px 0;
        }
        
        .card {
            background: var(--surface);
            border-radius: var(--radius);
            padding: 25px;
            border: 1px solid var(--border);
            transition: var(--transition);
            cursor: pointer;
        }
        
        .card:hover {
            transform: translateY(-5px);
            border-color: var(--primary);
            box-shadow: var(--shadow-glow);
        }
        
        .card-icon {
            width: 50px;
            height: 50px;
            background: var(--gradient);
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            margin-bottom: 15px;
        }
        
        .card-title {
            font-size: 18px;
            font-weight: 700;
            margin-bottom: 10px;
        }
        
        .card-desc {
            font-size: 14px;
            color: var(--text-secondary);
        }
        
        /* ========== CTA SECTION ========== */
        .cta-section {
            background: var(--gradient);
            border-radius: var(--radius-lg);
            padding: 50px;
            text-align: center;
            margin: 50px 0;
            position: relative;
            overflow: hidden;
        }
        
        .cta-section::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: url("data:image/svg+xml,%3Csvg width='60' height='60' viewBox='0 0 60 60' xmlns='http://www.w3.org/2000/svg'%3E%3Cg fill='none' fill-rule='evenodd'%3E%3Cg fill='%23ffffff' fill-opacity='0.05'%3E%3Cpath d='M36 34v-4h-2v4h-4v2h4v4h2v-4h4v-2h-4zm0-30V0h-2v4h-4v2h4v4h2V6h4V4h-4zM6 34v-4H4v4H0v2h4v4h2v-4h4v-2H6zM6 4V0H4v4H0v2h4v4h2V6h4V4H6z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E");
        }
        
        .cta-content {
            position: relative;
            z-index: 1;
        }
        
        .cta-title {
            font-size: 32px;
            font-weight: 800;
            color: white;
            margin-bottom: 15px;
        }
        
        .cta-desc {
            font-size: 18px;
            color: rgba(255,255,255,0.9);
            margin-bottom: 30px;
            max-width: 600px;
            margin-left: auto;
            margin-right: auto;
        }
        
        .cta-price {
            font-size: 48px;
            font-weight: 800;
            color: white;
            margin-bottom: 30px;
        }
        
        .cta-buttons {
            display: flex;
            gap: 20px;
            justify-content: center;
            flex-wrap: wrap;
        }
        
        .cta-btn {
            padding: 16px 32px;
            border-radius: 30px;
            font-size: 18px;
            font-weight: 700;
            text-decoration: none;
            transition: var(--transition);
        }
        
        .cta-btn-primary {
            background: white;
            color: var(--primary);
        }
        
        .cta-btn-primary:hover {
            transform: translateY(-3px);
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
        }
        
        .cta-btn-secondary {
            background: transparent;
            color: white;
            border: 2px solid white;
        }
        
        .cta-btn-secondary:hover {
            background: white;
            color: var(--primary);
        }
        
        /* ========== FOOTER ========== */
        .footer {
            background: var(--surface);
            padding: 50px 30px 30px;
            margin-top: 60px;
        }
        
        .footer-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 40px;
            max-width: 1200px;
            margin: 0 auto 40px;
        }
        
        .footer-section h4 {
            color: white;
            font-size: 16px;
            margin-bottom: 20px;
        }
        
        .footer-links {
            list-style: none;
        }
        
        .footer-links li {
            margin-bottom: 10px;
        }
        
        .footer-links a {
            color: var(--text-secondary);
            text-decoration: none;
            transition: var(--transition);
        }
        
        .footer-links a:hover {
            color: var(--accent);
        }
        
        .footer-bottom {
            text-align: center;
            padding-top: 30px;
            border-top: 1px solid var(--border);
            color: var(--text-secondary);
            font-size: 14px;
        }
        
        /* ========== RESPONSIVE ========== */
        @media (max-width: 768px) {
            .hero-title { font-size: 32px; }
            .stats-grid { grid-template-columns: repeat(2, 1fr); }
            .form-grid { grid-template-columns: 1fr; }
            .cards-grid { grid-template-columns: 1fr; }
            .footer-grid { grid-template-columns: repeat(2, 1fr); }
            .nav-links { display: none; }
        }
    </style>
</head>
<body>
    <!-- HEADER -->
    <header class="header">
        <div class="header-container">
            <a href="https://wiabooks.store" class="logo">
                <img src="https://wiabooks.store/wia-book-icon.png" alt="WIA">
                <span>WIA Books</span>
            </a>
            <nav class="nav-links">
                <a href="https://wiastandards.com" class="nav-link">WIA Standards</a>
                <a href="https://wia.family" class="nav-link">WIA Family</a>
                <a href="https://thekoreantoday.global" class="nav-link">KONAN</a>
                <div class="lang-switch">
                    <button class="lang-btn active" onclick="setLang('en')">EN</button>
                    <button class="lang-btn" onclick="setLang('ko')">KO</button>
                </div>
            </nav>
        </div>
    </header>
    
    <!-- MAIN -->
    <main class="main">
        <!-- Hero Section -->
        <section class="hero">
            <div class="hero-icon">[ICON]</div>
            <h1 class="hero-title" data-i18n="title">{Standard Name} Simulator</h1>
            <p class="hero-subtitle" data-i18n="subtitle">Interactive Demo of WIA-{STANDARD-ID}</p>
            <p class="hero-desc" data-i18n="desc">{Description of what this simulator does}</p>
        </section>
        
        <!-- Stats Banner -->
        <section class="stats-banner">
            <div class="stats-grid">
                <div class="stat-item">
                    <div class="stat-value" data-i18n="stat1Value">{Value 1}</div>
                    <div class="stat-label" data-i18n="stat1Label">{Label 1}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" data-i18n="stat2Value">{Value 2}</div>
                    <div class="stat-label" data-i18n="stat2Label">{Label 2}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" data-i18n="stat3Value">{Value 3}</div>
                    <div class="stat-label" data-i18n="stat3Label">{Label 3}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" data-i18n="stat4Value">{Value 4}</div>
                    <div class="stat-label" data-i18n="stat4Label">{Label 4}</div>
                </div>
            </div>
        </section>
        
        <!-- Interactive Tabs -->
        <section class="tabs-container">
            <div class="tabs-header">
                <button class="tab-btn active" data-tab="tab1" data-i18n="tab1">Tab 1</button>
                <button class="tab-btn" data-tab="tab2" data-i18n="tab2">Tab 2</button>
                <button class="tab-btn" data-tab="tab3" data-i18n="tab3">Tab 3</button>
            </div>
            
            <!-- Tab 1 Content -->
            <div id="tab1" class="tab-content active">
                <div class="sim-panel">
                    <h2 class="panel-title">
                        <span class="icon">[+]</span>
                        <span data-i18n="panel1Title">Panel Title</span>
                    </h2>
                    
                    <div class="form-grid">
                        <div class="form-group">
                            <label class="form-label" data-i18n="field1Label">Field 1</label>
                            <input type="text" class="form-input" placeholder="Enter value..." data-i18n-placeholder="field1Placeholder">
                        </div>
                        <div class="form-group">
                            <label class="form-label" data-i18n="field2Label">Field 2</label>
                            <select class="form-select">
                                <option value="">Select...</option>
                            </select>
                        </div>
                    </div>
                    
                    <div style="margin-top: 20px;">
                        <button class="btn btn-primary" onclick="runSimulation()">
                            <span>[RUN]</span>
                            <span data-i18n="runBtn">Run Simulation</span>
                        </button>
                    </div>
                </div>
                
                <!-- Result Panel (hidden by default) -->
                <div id="result-panel" class="result-panel" style="display: none;">
                    <div class="result-header">
                        <div class="result-icon">[OK]</div>
                        <h3 class="result-title" data-i18n="resultTitle">Simulation Complete</h3>
                    </div>
                    <div class="result-data" id="result-data">
                        <!-- Results will be inserted here -->
                    </div>
                </div>
            </div>
        </section>
        
        <!-- Feature Cards -->
        <section class="cards-grid">
            <div class="card">
                <div class="card-icon">[1]</div>
                <h3 class="card-title" data-i18n="feature1Title">Feature 1</h3>
                <p class="card-desc" data-i18n="feature1Desc">Description of feature 1</p>
            </div>
            <div class="card">
                <div class="card-icon">[2]</div>
                <h3 class="card-title" data-i18n="feature2Title">Feature 2</h3>
                <p class="card-desc" data-i18n="feature2Desc">Description of feature 2</p>
            </div>
            <div class="card">
                <div class="card-icon">[3]</div>
                <h3 class="card-title" data-i18n="feature3Title">Feature 3</h3>
                <p class="card-desc" data-i18n="feature3Desc">Description of feature 3</p>
            </div>
        </section>
        
        <!-- CTA Section -->
        <section class="cta-section">
            <div class="cta-content">
                <h2 class="cta-title" data-i18n="ctaTitle">Master the WIA {Standard} Standard</h2>
                <p class="cta-desc" data-i18n="ctaDesc">Get the official ebook with complete specifications, implementation guides, and certification preparation materials.</p>
                <div class="cta-price">$99 USD</div>
                <div class="cta-buttons">
                    <a href="https://wiabooks.store/standards/{standard-id}/" class="cta-btn cta-btn-primary" data-i18n="ctaBook">[BOOK] Get the Ebook</a>
                    <a href="https://cert.wiastandards.com" class="cta-btn cta-btn-secondary" data-i18n="ctaCert">[CERT] Get Certified</a>
                </div>
            </div>
        </section>
    </main>
    
    <!-- FOOTER -->
    <footer class="footer">
        <div class="footer-grid">
            <div class="footer-section">
                <h4>About WIA Books</h4>
                <p style="color: var(--text-secondary); font-size: 14px; line-height: 1.6;">
                    World's most advanced AR/VR/WIAverse reading platform with 200+ interactive tools.
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
                    <li><a href="https://github.com/WIA-Official">GitHub</a></li>
                </ul>
            </div>
        </div>
        <div class="footer-bottom">
            <p>Publisher: WIA Books | SmileStory Inc. | CEO: Yeon Sam-Heum</p>
            <p style="margin-top: 10px;">Copyright 2025 WIA Standards - All Rights Reserved</p>
            <p style="margin-top: 10px; color: var(--accent);">Hongik Ingan - Benefit All Humanity</p>
        </div>
    </footer>
    
    <!-- LANGUAGE SCRIPT -->
    <script>
        // Translations
        const translations = {
            en: {
                title: "{Standard Name} Simulator",
                subtitle: "Interactive Demo of WIA-{STANDARD-ID}",
                desc: "{Description}",
                // ... add all translations
            },
            ko: {
                title: "{표준명} 시뮬레이터",
                subtitle: "WIA-{STANDARD-ID} 인터랙티브 데모",
                desc: "{설명}",
                // ... add all translations
            }
        };
        
        let currentLang = localStorage.getItem('wia_lang') || 'en';
        
        function setLang(lang) {
            currentLang = lang;
            localStorage.setItem('wia_lang', lang);
            
            // Update buttons
            document.querySelectorAll('.lang-btn').forEach(btn => {
                btn.classList.toggle('active', btn.textContent === lang.toUpperCase());
            });
            
            // Update translations
            document.querySelectorAll('[data-i18n]').forEach(el => {
                const key = el.getAttribute('data-i18n');
                if (translations[lang][key]) {
                    el.textContent = translations[lang][key];
                }
            });
        }
        
        // Tab switching
        document.querySelectorAll('.tab-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const tabId = btn.getAttribute('data-tab');
                
                document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
                document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
                
                btn.classList.add('active');
                document.getElementById(tabId).classList.add('active');
            });
        });
        
        // Simulation function (customize per standard)
        function runSimulation() {
            // Show result panel with animation
            const resultPanel = document.getElementById('result-panel');
            resultPanel.style.display = 'block';
            resultPanel.scrollIntoView({ behavior: 'smooth' });
            
            // Add result data
            document.getElementById('result-data').innerHTML = `
                <div class="result-item">
                    <div class="result-item-label">Result 1</div>
                    <div class="result-item-value">Value 1</div>
                </div>
                <div class="result-item">
                    <div class="result-item-label">Result 2</div>
                    <div class="result-item-value">Value 2</div>
                </div>
            `;
        }
        
        // Initialize
        document.addEventListener('DOMContentLoaded', () => {
            setLang(currentLang);
        });
    </script>
</body>
</html>
```

---

## 9. Simulator Design Requirements

### REQUIRED Visual Elements

1. **Rainbow Header** - Animated gradient header with WIA Books branding
2. **Hero Section** - Large title with gradient text, floating icon animation
3. **Stats Banner** - Animated background pulse, 4 metric cards
4. **Tab Navigation** - Pill-style buttons with hover effects
5. **Interactive Panels** - Hover glow effect, smooth transitions
6. **Result Display** - Slide-up animation, success styling
7. **Feature Cards** - 3-column grid with hover lift effect
8. **CTA Section** - Gradient background, pattern overlay, prominent pricing
9. **Footer** - 4-column grid, WIA ecosystem links

### FORBIDDEN

- Simple HTML forms without styling
- Missing animations
- No language switching
- Missing CTA section
- Plain white backgrounds
- Missing WIA Books header/footer

### Color Themes by Category

| Category | Primary | Secondary | Accent |
|----------|---------|-----------|--------|
| PET/Animal | #10B981 | #059669 | #34D399 |
| Healthcare | #EF4444 | #DC2626 | #F87171 |
| AI/LLM | #8B5CF6 | #7C3AED | #A78BFA |
| Finance | #F59E0B | #D97706 | #FBBF24 |
| Education | #3B82F6 | #2563EB | #60A5FA |
| Security | #6366F1 | #4F46E5 | #818CF8 |
| Robot | #EC4899 | #DB2777 | #F472B6 |
| Environment | #22C55E | #16A34A | #4ADE80 |

---

## 10. Updated Summary Checklist

Before submitting a standard package, verify:

- [ ] NO EMOJI anywhere (use text labels like [PET], [OK], [+])
- [ ] Ebook chapters are HTML (not MD)
- [ ] Languages: English + Korean only
- [ ] **Simulator uses VISUAL TEMPLATE (Section 8)**
- [ ] Rainbow header with WIA Books branding
- [ ] Stats banner with animated background
- [ ] Tab navigation with smooth transitions
- [ ] Result panels with slide-up animation
- [ ] Feature cards with hover effects
- [ ] CTA links to wiabooks.store ($99) and cert.wiastandards.com
- [ ] Footer with WIA ecosystem links
- [ ] Phase 2 includes Rust implementation
- [ ] Hongik Ingan philosophy included
- [ ] ~100+ pages total content

---

**Document Version:** 2.0.0
**Last Updated:** 2025-12-17
**Major Update:** Added Visual Simulator Template (Section 8-10)
