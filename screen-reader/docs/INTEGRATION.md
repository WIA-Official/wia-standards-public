# WIA Screen Reader Integration Guide

> 弘益人間 (홍익인간) - Benefit All Humanity

## Overview

This guide covers integrating WIA Screen Reader into your applications and websites.

## Web Integration

### Using the TypeScript SDK

```bash
npm install @wia/screen-reader
```

```typescript
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader();

// Enhance all elements with aria-label
document.querySelectorAll('[aria-label]').forEach(async (el) => {
  const result = await reader.processElement(el);
  el.setAttribute('data-wia-wihp', result.pronunciation.wihp);
  el.setAttribute('data-wia-braille', result.braille.grade1);
});
```

### Adding Accessibility Attributes

```html
<button
  aria-label="Submit form"
  data-wia-wihp="서브밋 폼"
  data-wia-braille="⠎⠥⠃⠍⠊⠞ ⠋⠕⠗⠍">
  Submit
</button>
```

### Event Handling

```typescript
// Listen for focus events
document.addEventListener('focusin', async (e) => {
  const element = e.target as Element;
  const text = element.textContent;

  if (text) {
    const result = await reader.process(text);
    // Update ARIA attributes or speak
  }
});
```

## React Integration

```tsx
import { useEffect, useState } from 'react';
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader();

function AccessibleButton({ label }: { label: string }) {
  const [wihp, setWihp] = useState('');

  useEffect(() => {
    reader.process(label).then(result => {
      setWihp(result.pronunciation.wihp);
    });
  }, [label]);

  return (
    <button
      aria-label={label}
      data-wia-wihp={wihp}
    >
      {label}
    </button>
  );
}
```

## Vue Integration

```vue
<script setup lang="ts">
import { ref, onMounted } from 'vue';
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader();
const wihp = ref('');

const props = defineProps<{ text: string }>();

onMounted(async () => {
  const result = await reader.process(props.text);
  wihp.value = result.pronunciation.wihp;
});
</script>

<template>
  <span :data-wia-wihp="wihp">{{ text }}</span>
</template>
```

## Server-Side Integration

### Node.js

```typescript
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader();

app.post('/api/convert', async (req, res) => {
  const { text, language } = req.body;
  const result = await reader.process(text, language);
  res.json(result);
});
```

### Python (Flask)

```python
from flask import Flask, request, jsonify
from wia_screen_reader import WIAScreenReader

app = Flask(__name__)
reader = WIAScreenReader()

@app.route('/api/convert', methods=['POST'])
def convert():
    data = request.json
    result = reader.process(data['text'])
    return jsonify(result.to_dict())
```

## Best Practices

1. **Progressive Enhancement**: Always provide fallbacks
2. **Caching**: Cache conversion results for performance
3. **Lazy Loading**: Don't block page load with conversions
4. **Error Handling**: Gracefully handle conversion failures

---

© 2025 WIA Standards
