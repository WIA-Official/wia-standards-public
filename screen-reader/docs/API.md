# WIA Screen Reader API Documentation

> 弘益人間 (홍익인간) - Benefit All Humanity

## Overview

The WIA Screen Reader API provides programmatic access to screen reader functionality including WIHP pronunciation, braille conversion, and TTS optimization.

## REST API

### Base URL

```
https://api.wiastandards.com/screen-reader/v1
```

### Authentication

```http
Authorization: Bearer YOUR_API_KEY
```

## Endpoints

### POST /convert

Convert text to screen reader format.

**Request:**
```json
{
  "text": "Hello World",
  "source_lang": "en",
  "output_format": ["wihp", "braille", "tts"]
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "wihp": "헬로우 월드",
    "braille": {
      "grade1": "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙",
      "grade2": "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙",
      "wia": "ㅎㅓㄹㄹㅗㅇ ㅇㅝㄹㄷㅡ",
      "cells": 11
    },
    "tts_ssml": "<speak>Hello World</speak>"
  },
  "processing_time_ms": 12
}
```

### POST /speak

Generate speech audio.

**Request:**
```json
{
  "text": "Hello World",
  "voice": "en-US-Neural",
  "rate": 1.0,
  "pitch": 1.0,
  "format": "mp3"
}
```

**Response:**
```json
{
  "success": true,
  "audio_url": "https://cdn.wia.codes/audio/abc123.mp3",
  "duration_ms": 1200
}
```

### POST /braille

Convert text to braille.

**Request:**
```json
{
  "text": "Hello World",
  "grade": 1,
  "language": "en"
}
```

**Response:**
```json
{
  "success": true,
  "unicode": "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙",
  "dots": [
    {"char": "h", "dots": [1,2,5], "unicode": "⠓"},
    {"char": "e", "dots": [1,5], "unicode": "⠑"}
  ],
  "cells": 11
}
```

## SDK Reference

### TypeScript

```typescript
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader({
  defaultLanguage: 'en',
  defaultBrailleGrade: 1
});

// Process text
const result = await reader.process("Hello");

// Speak
reader.speak("Hello", { rate: 1.2 });

// Get specific outputs
const wihp = reader.getWIHP("Hello");
const braille = reader.getBraille("Hello");
```

### Python

```python
from wia_screen_reader import WIAScreenReader

reader = WIAScreenReader(language="en")

# Process text
result = reader.process("Hello")

# Speak
reader.speak("Hello", use_wihp=True)

# Get specific outputs
wihp = reader.get_wihp("Hello")
braille = reader.get_braille("Hello")
```

### Rust

```rust
use wia_screen_reader::WIAScreenReader;

let reader = WIAScreenReader::new();
let result = reader.process("Hello");

println!("{}", result.pronunciation.wihp);
println!("{}", result.braille.grade1);
```

## Error Codes

| Code | Description |
|------|-------------|
| INVALID_INPUT | Invalid input text |
| INVALID_LANGUAGE | Unsupported language code |
| RATE_LIMIT_EXCEEDED | API rate limit exceeded |
| AUTHENTICATION_FAILED | Invalid API key |

## Rate Limits

| Tier | Requests/minute | Requests/day |
|------|-----------------|--------------|
| Free | 60 | 10,000 |
| Pro | 600 | 100,000 |
| Enterprise | Unlimited | Unlimited |

---

© 2025 WIA Standards
