# WIA Accessibility Guidelines

> 弘益人間 (홍익인간) - Benefit All Humanity

## Overview

This document provides guidelines for implementing WIA Screen Reader Standard in compliance with WCAG 2.2 AAA.

## WCAG 2.2 Compliance

### Perceivable

#### 1.1 Text Alternatives

- Provide WIHP pronunciation for all non-text content
- Include braille equivalents for images with `alt` text
- Use `aria-label` with WIA data attributes

```html
<img
  src="logo.png"
  alt="Company Logo"
  data-wia-wihp="컴퍼니 로고"
  data-wia-braille="⠉⠕⠍⠏⠁⠝⠽ ⠇⠕⠛⠕">
```

#### 1.4 Distinguishable

- Ensure contrast ratios meet AAA standards (7:1)
- Provide audio descriptions with WIHP pronunciation

### Operable

#### 2.1 Keyboard Accessible

All WIA features must be accessible via keyboard:

| Shortcut | Action |
|----------|--------|
| Alt+W | Convert to WIHP |
| Alt+B | Convert to Braille |
| Alt+S | Speak selection |

#### 2.4 Navigable

- Skip links must include WIHP pronunciation
- Page titles must be accessible in multiple formats

### Understandable

#### 3.1 Readable

- Identify language of content
- Provide pronunciation for unusual words
- Use WIHP for consistent pronunciation

### Robust

#### 4.1 Compatible

- Use valid HTML/ARIA
- Support all major screen readers
- Test with WIA certification tools

## Implementation Checklist

### Required

- [ ] All text content has WIHP pronunciation
- [ ] Braille conversion available for all content
- [ ] Keyboard navigation fully functional
- [ ] Screen reader compatibility verified
- [ ] Language identification implemented

### Recommended

- [ ] TTS optimization enabled
- [ ] Custom pronunciation dictionary
- [ ] Real-time conversion for dynamic content
- [ ] Multiple braille grade support

## Testing

### Automated Testing

```typescript
import { WIAAccessibilityTester } from '@wia/screen-reader';

const tester = new WIAAccessibilityTester();
const results = await tester.testPage(document);

console.log(results.score);  // 0-100
console.log(results.issues); // List of issues
```

### Manual Testing

1. Navigate with screen reader
2. Verify WIHP pronunciation accuracy
3. Check braille display output
4. Test keyboard-only navigation
5. Validate language switching

## Resources

- [WCAG 2.2 Quick Reference](https://www.w3.org/WAI/WCAG22/quickref/)
- [WIA Screen Reader Spec](../spec/wia-screen-reader-v1.0.json)
- [NVDA Testing Guide](./NVDA-PLUGIN-GUIDE.md)

---

© 2025 WIA Standards
