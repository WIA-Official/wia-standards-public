# WIA Standards AI Assistant

## 📋 Project Overview

**Created:** December 18, 2025  
**Developer:** Claude (Anthropic) & SmileStory Inc. CEO  
**Version:** 2.0.0

AI Assistant for WIA Family website. Supports 177+ standards search and guidance.

---

## 🎯 Core Features

### 1. Standard Search (JSON-based)
- Auto-scans 41+ standard folders
- Real-time search
- Clickable links to standard pages
- **Auto-updates via cron** (hourly)

### 2. AI Conversation (3 Engines)
| AI | Feature | API Key |
|----|---------|---------|
| Gemini | Free tier | Required |
| Claude | High quality | Required |
| ChatGPT | General purpose | Required |

### 3. Category Navigation
- Accessibility, Robotics, Medical, Space, Quantum, etc.

### 4. UI Features
- **Drag & Drop**: Move modal by header
- **Resize**: Adjust width by edges
- **Auto-collapse**: Sections collapse after 2.5s
- **Responsive**: Mobile-optimized

### 5. Context Awareness
- Detects current standard page
- Provides relevant explanations

---

## 📁 File Structure

```
/ai-assistant/
├── README.md              # This document
├── wia-ai-modal.js        # Main JavaScript
├── wia-ai-modal.css       # Styles (dark theme)
├── standards.json         # Standard list (auto-generated)
├── generate-standards.sh  # JSON generator script
└── apply-to-all.sh        # Apply to all pages script
```

---

## 🚀 Auto-Update (Cron)

Standards JSON auto-updates every hour:
```
0 * * * * /var/www/wiastandards/ai-assistant/generate-standards.sh
```

No manual intervention needed!

---

## ⚙️ Add to HTML

```html
<!-- Before </body> -->
<link rel="stylesheet" href="/ai-assistant/wia-ai-modal.css">
<script src="/ai-assistant/wia-ai-modal.js"></script>
```

---

## 🎨 Design

- **Theme:** Dark (matches WIA site)
- **Colors:** Cyan (#00d4ff), Purple (#7b2cbf)
- **Background:** #0a0a0f, #1a1a2e

---

## 📝 Customization

### Quick Buttons (wia-ai-modal.js)
```javascript
<button class="wia-tool-btn" onclick="wiaAI.quickAsk('your question')">Button</button>
```

### Add Response Patterns (getBasicResponse function)
```javascript
if (msg.includes('keyword')) {
    return 'Your response';
}
```

---

## 🌐 Applied Sites

- **Main:** wiastandards.com / wia.family
- **41 Standard Pages:** /aac/, /bci/, /robot/, etc.

---

## 📜 Update History

### 2025-12-18 (v2.0.0)
- Full-featured modal (matching Korean Today AI)
- 3 AI engines support
- Drag & resize functionality
- Auto-collapse sections
- Context-aware responses
- Cron auto-update
- Applied to all 41 standard pages

---

## 📜 License

Copyright © 2025 SmileStory Inc. (WIA Family)

홍익인간 (弘益人間) - Benefit all humanity through technology.
