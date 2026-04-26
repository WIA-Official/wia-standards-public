# 🌟 WIA PubScript WASM Demo

**다중 감각 출판 엔진 - 브라우저 버전**

## Philosophy

이것은 "장애인을 위한 기술"이 아니라 **"모든 인간을 위한 선택지"** 입니다!

**ALL FIVE REPRESENTATIONS ARE EQUAL. NO DEFAULT EXISTS.**

## Features

### Demo 1: Text to Braille Converter 🤲
- 한국어/English 점자 변환
- 실시간 브라우저 변환
- Korean Braille: "안녕하세요" → "⠅⠣⠉⠉⠱⠎⠅⠟⠣⠎⠝⠅⠍"

### Demo 2: Markdown to ALL Formats 🌟
마크다운을 다음 형식으로 동시 변환:
- 🤲 **Braille (BRF)**: 촉각 표현
- 👁️ **HTML with ARIA**: 시각적 표현 + 접근성
- 🔊 **SSML**: 청각 표현 (TTS)
- ⚙️ **IR (JSON)**: 중간 표현

## Running the Demo

### Option 1: Python HTTP Server

```bash
# Navigate to pubscript directory
cd /home/user/pdf-studio/pubscript

# Start HTTP server
python3 -m http.server 8000

# Open browser to:
# http://localhost:8000/examples/wasm-demo.html
```

### Option 2: Node.js HTTP Server

```bash
# Install http-server globally
npm install -g http-server

# Navigate to pubscript directory
cd /home/user/pdf-studio/pubscript

# Start server
http-server -p 8000

# Open browser to:
# http://localhost:8000/examples/wasm-demo.html
```

### Option 3: VS Code Live Server

1. Install "Live Server" extension in VS Code
2. Right-click on `wasm-demo.html`
3. Select "Open with Live Server"

## Using in Your Own Project

### Installation

```bash
# Copy the pkg directory to your project
cp -r /home/user/pdf-studio/pubscript/pkg ./wia_pubscript
```

### JavaScript Example

```javascript
import init, { textToBraille, markdownToAll } from './wia_pubscript/wia_pubscript.js';

// Initialize WASM
await init();

// Convert text to braille
const braille = textToBraille("안녕하세요");
console.log(braille); // "⠅⠣⠉⠉⠱⠎⠅⠟⠣⠎⠝⠅⠍"

// Convert Markdown to all formats
const result = JSON.parse(markdownToAll("# Hello\n\nWorld!"));
console.log(result.braille); // Braille output
console.log(result.html);    // HTML output
console.log(result.ssml);    // SSML output
```

### Available Functions

```typescript
// Convert text directly to braille
textToBraille(text: string): string

// Convert Markdown to specific format
markdownToBraille(markdown: string): string
markdownToHtml(markdown: string): string
markdownToSsml(markdown: string): string
markdownToIr(markdown: string): string

// Convert Markdown to ALL formats at once
markdownToAll(markdown: string): string // Returns JSON with {braille, html, ssml}
```

## Architecture

```
Markdown Input
      ↓
   [Parser]
      ↓
   [IR v3.0] ← 5 Equal Representations
      ↓
 [Renderers]
   ↙  ↓  ↘
Braille HTML SSML
```

## Five Equal Representations

1. **👁️ Visual**: Text, images, layout, colors, typography
2. **🔊 Auditory**: Speech synthesis, music, sound effects, spatial audio
3. **🤲 Tactile**: Braille, haptic feedback, textures
4. **📍 Spatial**: 3D positioning, AR/VR, navigation
5. **✋ Gestural**: Swipe, tap, voice commands, eye tracking

## Technical Details

- **Language**: Rust (compiled to WebAssembly)
- **WASM Size**: ~400KB (unoptimized)
- **Target**: web (ES modules)
- **Korean Braille**: Full syllable decomposition (초성/중성/종성)
- **HTML Output**: Semantic HTML5 + full ARIA support
- **SSML Output**: Compatible with Google TTS, Amazon Polly, Azure TTS

## License

MIT - WIA Team

---

**핵심 철학**: 점자가 '추가 기능'이 아니라 텍스트와 동등한 일급 시민!
