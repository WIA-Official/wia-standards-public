# WIA Accessible UI/UX Standard
## Phase 4: Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee
**Standard ID:** WIA-AAC-014

---

## Table of Contents

1. [Introduction](#introduction)
2. [Framework Integration](#framework-integration)
3. [Design System Integration](#design-system-integration)
4. [WIA Ecosystem Integration](#wia-ecosystem-integration)
5. [Development Tool Integration](#development-tool-integration)
6. [Testing Framework Integration](#testing-framework-integration)
7. [CI/CD Integration](#cicd-integration)
8. [Content Management Systems](#content-management-systems)
9. [Browser Extension Integration](#browser-extension-integration)
10. [Migration Guide](#migration-guide)

---

## 1. Introduction

This specification defines how the WIA Accessible UI/UX Standard integrates with existing frameworks, tools, and the broader WIA ecosystem. It provides implementation patterns, best practices, and migration strategies.

### 1.1 Integration Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

Accessibility should integrate seamlessly into existing workflows, not require complete rewrites. This standard provides drop-in compatibility with popular frameworks and tools.

### 1.2 Integration Levels

- **Level 1**: Basic WCAG compliance checking
- **Level 2**: Automated accessibility fixes
- **Level 3**: Runtime accessibility monitoring
- **Level 4**: Full accessibility automation with AI assistance

---

## 2. Framework Integration

### 2.1 React Integration

#### 2.1.1 React Hooks

```typescript
import { useAccessibility } from '@wia/accessible-ui/react';

interface UseAccessibilityOptions {
  /** Component type */
  type: ComponentType;

  /** WCAG level */
  wcagLevel?: "A" | "AA" | "AAA";

  /** Accessibility metadata */
  metadata?: Partial<AccessibilityMetadata>;

  /** Auto-fix issues */
  autoFix?: boolean;
}

function useAccessibility(options: UseAccessibilityOptions) {
  const [aria, setAria] = useState<ARIAAttributes>({});
  const [keyboard, setKeyboard] = useState<KeyboardProps>({});
  const elementRef = useRef<HTMLElement>(null);

  useEffect(() => {
    if (!elementRef.current) return;

    const manager = WiaAccessibility.getInstance();
    const handle = manager.registerComponent(
      elementRef.current,
      {
        id: generateId(),
        type: options.type,
        wcag: {
          version: "2.1",
          level: options.wcagLevel || "AA"
        },
        ...options.metadata
      }
    );

    // Validate on mount
    const validation = WiaAccessibility.validate(
      elementRef.current,
      options.wcagLevel
    );

    if (!validation.valid && options.autoFix) {
      applyFixes(elementRef.current, validation.issues);
    }

    return () => manager.unregisterComponent(handle);
  }, [options]);

  return {
    ref: elementRef,
    aria,
    keyboard
  };
}
```

#### 2.1.2 React Component Example

```tsx
import { useAccessibility } from '@wia/accessible-ui/react';

interface ButtonProps {
  label: string;
  onClick: () => void;
  variant?: 'primary' | 'secondary';
  disabled?: boolean;
}

export function AccessibleButton({
  label,
  onClick,
  variant = 'primary',
  disabled = false
}: ButtonProps) {
  const { ref, aria, keyboard } = useAccessibility({
    type: 'button',
    wcagLevel: 'AA',
    metadata: {
      aria: {
        role: 'button',
        label,
        state: { disabled }
      },
      keyboard: {
        accessible: true,
        tabIndex: disabled ? -1 : 0
      }
    },
    autoFix: true
  });

  return (
    <button
      ref={ref}
      className={`btn btn-${variant}`}
      onClick={onClick}
      disabled={disabled}
      {...aria}
      {...keyboard}
    >
      {label}
    </button>
  );
}
```

#### 2.1.3 React Context Provider

```tsx
import { WiaAccessibilityProvider } from '@wia/accessible-ui/react';

function App() {
  return (
    <WiaAccessibilityProvider
      config={{
        wcagLevel: 'AA',
        autoFix: true,
        announcements: true
      }}
    >
      <YourApp />
    </WiaAccessibilityProvider>
  );
}
```

### 2.2 Vue Integration

#### 2.2.1 Vue Composable

```typescript
import { ref, onMounted, onUnmounted } from 'vue';

export function useAccessibility(options: UseAccessibilityOptions) {
  const elementRef = ref<HTMLElement | null>(null);
  const aria = ref({});
  const keyboard = ref({});
  let handle: ComponentHandle | null = null;

  onMounted(() => {
    if (!elementRef.value) return;

    const manager = WiaAccessibility.getInstance();
    handle = manager.registerComponent(
      elementRef.value,
      options.metadata
    );

    // Update reactive properties
    aria.value = computeARIA(options.metadata);
    keyboard.value = computeKeyboard(options.metadata);
  });

  onUnmounted(() => {
    if (handle) {
      WiaAccessibility.getInstance().unregisterComponent(handle);
    }
  });

  return {
    elementRef,
    aria,
    keyboard
  };
}
```

#### 2.2.2 Vue Directive

```typescript
import { Directive } from 'vue';

export const accessibleDirective: Directive = {
  mounted(el: HTMLElement, binding) {
    const { type, label, wcagLevel } = binding.value;

    const manager = WiaAccessibility.getInstance();
    const handle = manager.registerComponent(el, {
      id: generateId(),
      type,
      wcag: { version: "2.1", level: wcagLevel || "AA" },
      aria: { role: type, label }
    });

    // Store handle for cleanup
    (el as any).__wiaHandle = handle;
  },

  unmounted(el: HTMLElement) {
    const handle = (el as any).__wiaHandle;
    if (handle) {
      WiaAccessibility.getInstance().unregisterComponent(handle);
    }
  }
};
```

#### 2.2.3 Vue Component Example

```vue
<template>
  <button
    ref="elementRef"
    v-bind="aria"
    v-bind="keyboard"
    @click="handleClick"
  >
    {{ label }}
  </button>
</template>

<script setup lang="ts">
import { useAccessibility } from '@wia/accessible-ui/vue';

const props = defineProps<{
  label: string;
  onClick: () => void;
}>();

const { elementRef, aria, keyboard } = useAccessibility({
  type: 'button',
  metadata: {
    aria: { role: 'button', label: props.label }
  }
});

function handleClick() {
  props.onClick();
}
</script>
```

### 2.3 Angular Integration

#### 2.3.1 Angular Directive

```typescript
import { Directive, ElementRef, Input, OnInit, OnDestroy } from '@angular/core';

@Directive({
  selector: '[wiaAccessible]'
})
export class AccessibleDirective implements OnInit, OnDestroy {
  @Input() wiaType!: ComponentType;
  @Input() wiaLabel?: string;
  @Input() wiaLevel: "A" | "AA" | "AAA" = "AA";

  private handle?: ComponentHandle;

  constructor(private el: ElementRef<HTMLElement>) {}

  ngOnInit() {
    const manager = WiaAccessibility.getInstance();
    this.handle = manager.registerComponent(
      this.el.nativeElement,
      {
        id: generateId(),
        type: this.wiaType,
        wcag: { version: "2.1", level: this.wiaLevel },
        aria: {
          role: this.wiaType,
          label: this.wiaLabel
        }
      }
    );
  }

  ngOnDestroy() {
    if (this.handle) {
      WiaAccessibility.getInstance().unregisterComponent(this.handle);
    }
  }
}
```

#### 2.3.2 Angular Component Example

```typescript
import { Component } from '@angular/core';

@Component({
  selector: 'accessible-button',
  template: `
    <button
      wiaAccessible
      [wiaType]="'button'"
      [wiaLabel]="label"
      [wiaLevel]="'AA'"
      (click)="onClick.emit()"
    >
      {{ label }}
    </button>
  `
})
export class AccessibleButtonComponent {
  @Input() label!: string;
  @Output() onClick = new EventEmitter<void>();
}
```

### 2.4 Svelte Integration

#### 2.4.1 Svelte Action

```typescript
import { ActionReturn } from 'svelte/action';

export function accessible(
  node: HTMLElement,
  options: AccessibilityOptions
): ActionReturn {
  const manager = WiaAccessibility.getInstance();
  const handle = manager.registerComponent(node, {
    id: generateId(),
    type: options.type,
    wcag: { version: "2.1", level: options.wcagLevel || "AA" },
    aria: options.aria,
    keyboard: options.keyboard
  });

  return {
    update(newOptions: AccessibilityOptions) {
      manager.updateComponent(handle, newOptions);
    },
    destroy() {
      manager.unregisterComponent(handle);
    }
  };
}
```

#### 2.4.2 Svelte Component Example

```svelte
<script lang="ts">
  import { accessible } from '@wia/accessible-ui/svelte';

  export let label: string;
  export let onClick: () => void;
</script>

<button
  use:accessible={{
    type: 'button',
    aria: { role: 'button', label }
  }}
  on:click={onClick}
>
  {label}
</button>
```

---

## 3. Design System Integration

### 3.1 Design Tokens

```typescript
interface AccessibilityDesignTokens {
  /** Color contrast requirements */
  contrast: {
    minimum: {
      normal: number; // 4.5:1
      large: number;  // 3:1
    };
    enhanced: {
      normal: number; // 7:1
      large: number;  // 4.5:1
    };
  };

  /** Typography scale */
  typography: {
    minFontSize: number; // 16px
    baseLineHeight: number; // 1.5
    headingScale: number[]; // [2.5, 2, 1.75, 1.5, 1.25, 1.125]
  };

  /** Spacing scale */
  spacing: {
    touchTarget: number; // 44px minimum
    focusOutline: number; // 2px minimum
    textSpacing: number; // 0.12em minimum
  };

  /** Focus indicators */
  focus: {
    outlineWidth: string; // '2px'
    outlineStyle: string; // 'solid'
    outlineColor: string; // '#3B82F6'
    outlineOffset: string; // '2px'
  };
}
```

### 3.2 Component Library Integration

```typescript
// Example: Integrating with Material-UI
import { ThemeOptions } from '@mui/material/styles';
import { getAccessibilityTheme } from '@wia/accessible-ui/mui';

const theme: ThemeOptions = {
  ...getAccessibilityTheme({
    wcagLevel: 'AA',
    includeMotion: false // Respect prefers-reduced-motion
  }),
  palette: {
    primary: {
      main: '#3B82F6'
    }
  }
};
```

---

## 4. WIA Ecosystem Integration

### 4.1 Integration with WIA-AAC (Augmentative Communication)

```typescript
import { WiaAccessibility } from '@wia/accessible-ui';
import { WiaAAC } from '@wia/aac';

// Enable AAC device integration
const a11y = WiaAccessibility.init({
  wcagLevel: 'AAA',
  extensions: {
    aac: {
      enabled: true,
      devices: ['eyetracker', 'switch', 'headmouse']
    }
  }
});

// Listen for AAC input
WiaAAC.on('input', (event) => {
  if (event.type === 'eyetracker') {
    // Handle eye tracker input
    const target = event.target as HTMLElement;
    target.click();
  }
});
```

### 4.2 Integration with WIA-BRAILLE-DISPLAY

```typescript
import { WiaAccessibility } from '@wia/accessible-ui';
import { WiaBrailleDisplay } from '@wia/braille-display';

// Enable Braille display support
const display = await WiaBrailleDisplay.discover();
if (display.length > 0) {
  const braille = await display[0].connect();

  // Sync UI text to Braille display
  const a11y = WiaAccessibility.getInstance();
  a11y.on('focus', async (element) => {
    const text = a11y.aria.getLabel(element);
    await braille.writeText(text);
  });
}
```

### 4.3 Integration with WIA-INTENT

```typescript
import { WiaAccessibility } from '@wia/accessible-ui';
import { WiaIntent } from '@wia/intent';

// Interpret user intents accessibly
WiaIntent.on('intent', async (intent) => {
  if (intent.action === 'navigate') {
    const target = document.querySelector(intent.target);
    if (target instanceof HTMLElement) {
      // Accessible navigation
      const a11y = WiaAccessibility.getInstance();
      await a11y.focus.focus({
        target,
        scroll: true,
        scrollBehavior: 'smooth',
        options: { focusVisible: true }
      });

      // Announce navigation
      a11y.announce.announceNavigation(
        target.getAttribute('aria-label') || ''
      );
    }
  }
});
```

---

## 5. Development Tool Integration

### 5.1 ESLint Plugin

```javascript
// eslint-plugin-wia-a11y
module.exports = {
  rules: {
    'button-has-label': {
      create(context) {
        return {
          JSXOpeningElement(node) {
            if (node.name.name === 'button') {
              const hasLabel =
                hasAriaLabel(node) ||
                hasAriaLabelledBy(node) ||
                hasTextContent(node);

              if (!hasLabel) {
                context.report({
                  node,
                  message: 'Buttons must have an accessible label'
                });
              }
            }
          }
        };
      }
    },

    'contrast-ratio': {
      create(context) {
        return {
          JSXAttribute(node) {
            if (node.name.name === 'style') {
              const style = parseStyle(node.value);
              if (style.color && style.backgroundColor) {
                const ratio = calculateContrast(
                  style.color,
                  style.backgroundColor
                );

                if (ratio < 4.5) {
                  context.report({
                    node,
                    message: `Contrast ratio ${ratio.toFixed(2)}:1 is below 4.5:1 minimum`
                  });
                }
              }
            }
          }
        };
      }
    }
  }
};
```

### 5.2 Webpack Plugin

```javascript
const { WiaAccessibilityWebpackPlugin } = require('@wia/accessible-ui/webpack');

module.exports = {
  plugins: [
    new WiaAccessibilityWebpackPlugin({
      wcagLevel: 'AA',
      failOnErrors: true,
      generateReport: true,
      outputPath: './a11y-report.html'
    })
  ]
};
```

### 5.3 Vite Plugin

```typescript
import { wiaAccessibility } from '@wia/accessible-ui/vite';

export default {
  plugins: [
    wiaAccessibility({
      wcagLevel: 'AA',
      devOverlay: true, // Show accessibility issues in overlay
      hotReload: true   // Hot reload on a11y fixes
    })
  ]
};
```

---

## 6. Testing Framework Integration

### 6.1 Jest Integration

```typescript
import '@wia/accessible-ui/jest';

describe('AccessibleButton', () => {
  it('should meet WCAG AA standards', () => {
    const button = render(<AccessibleButton label="Submit" />);

    expect(button).toBeAccessible('AA');
    expect(button).toHaveAccessibleName('Submit');
    expect(button).toHaveRole('button');
    expect(button).toBeKeyboardAccessible();
  });

  it('should have sufficient color contrast', () => {
    const button = render(<AccessibleButton label="Submit" />);

    expect(button).toHaveContrastRatio(
      4.5,
      'foreground',
      'background'
    );
  });
});
```

### 6.2 Playwright Integration

```typescript
import { test, expect } from '@playwright/test';
import { injectAccessibility } from '@wia/accessible-ui/playwright';

test.beforeEach(async ({ page }) => {
  await injectAccessibility(page);
});

test('homepage is accessible', async ({ page }) => {
  await page.goto('https://example.com');

  // Run accessibility audit
  const violations = await page.checkAccessibility();

  expect(violations).toHaveLength(0);
});

test('button has correct ARIA', async ({ page }) => {
  await page.goto('https://example.com');

  const button = page.locator('button:has-text("Submit")');

  await expect(button).toHaveAttribute('aria-label', 'Submit form');
  await expect(button).toHaveRole('button');
});
```

### 6.3 Cypress Integration

```typescript
import '@wia/accessible-ui/cypress';

describe('Accessibility', () => {
  beforeEach(() => {
    cy.visit('/');
    cy.injectAccessibility();
  });

  it('should be accessible', () => {
    cy.checkAccessibility();
  });

  it('should be keyboard navigable', () => {
    cy.get('button').first().focus();
    cy.focused().should('have.attr', 'aria-label');

    cy.realPress('Tab');
    cy.focused().should('be.visible');
  });

  it('should announce to screen readers', () => {
    cy.get('button').click();

    cy.checkAnnouncement('Form submitted successfully');
  });
});
```

---

## 7. CI/CD Integration

### 7.1 GitHub Actions

```yaml
name: Accessibility Audit

on: [push, pull_request]

jobs:
  a11y:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'

      - name: Install dependencies
        run: npm ci

      - name: Run accessibility audit
        run: npm run a11y:audit

      - name: Upload audit report
        uses: actions/upload-artifact@v3
        with:
          name: a11y-report
          path: ./a11y-report.html

      - name: Comment on PR
        if: github.event_name == 'pull_request'
        uses: wia-official/a11y-comment-action@v1
        with:
          report-path: ./a11y-report.json
          fail-on-violations: true
```

### 7.2 GitLab CI

```yaml
accessibility:
  stage: test
  image: node:18
  script:
    - npm ci
    - npm run a11y:audit
  artifacts:
    paths:
      - a11y-report.html
    reports:
      accessibility: a11y-report.json
  only:
    - merge_requests
```

---

## 8. Content Management Systems

### 8.1 WordPress Integration

```php
<?php
/**
 * Plugin Name: WIA Accessibility
 * Description: Adds WIA accessibility standards to WordPress
 */

function wia_enqueue_accessibility() {
  wp_enqueue_script(
    'wia-accessibility',
    'https://cdn.wiastandards.com/accessible-ui/v1.0.0/wia.min.js',
    array(),
    '1.0.0',
    true
  );

  wp_add_inline_script('wia-accessibility', '
    WiaAccessibility.init({
      wcagLevel: "AA",
      autoFix: true
    });
  ');
}
add_action('wp_enqueue_scripts', 'wia_enqueue_accessibility');

// Add accessibility checker to editor
function wia_add_editor_checker($settings) {
  $settings['wia_accessibility'] = true;
  return $settings;
}
add_filter('tiny_mce_before_init', 'wia_add_editor_checker');
?>
```

### 8.2 Drupal Integration

```php
<?php
/**
 * Implements hook_page_attachments().
 */
function wia_accessibility_page_attachments(array &$attachments) {
  $attachments['#attached']['library'][] = 'wia_accessibility/core';
  $attachments['#attached']['drupalSettings']['wiaAccessibility'] = [
    'wcagLevel' => 'AA',
    'autoFix' => TRUE,
  ];
}
?>
```

---

## 9. Browser Extension Integration

### 9.1 Chrome Extension

```javascript
// manifest.json
{
  "manifest_version": 3,
  "name": "WIA Accessibility Checker",
  "version": "1.0.0",
  "permissions": ["activeTab", "scripting"],
  "action": {
    "default_popup": "popup.html"
  },
  "content_scripts": [{
    "matches": ["<all_urls>"],
    "js": ["wia-accessibility.js"]
  }]
}

// content script
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
  if (request.action === 'audit') {
    WiaAccessibility.audit({ level: 'AA' }).then(report => {
      sendResponse({ report });
    });
    return true;
  }
});
```

---

## 10. Migration Guide

### 10.1 From WCAG Manual Checks to WIA

```typescript
// Before: Manual WCAG checks
function checkButton(button: HTMLElement) {
  // Manual checks
  const hasLabel = button.getAttribute('aria-label') !== null;
  const hasRole = button.getAttribute('role') === 'button';
  // ... many more manual checks
}

// After: WIA Accessibility
import { WiaAccessibility } from '@wia/accessible-ui';

function checkButton(button: HTMLElement) {
  const result = WiaAccessibility.validate(button, 'AA');
  return result.valid;
}
```

### 10.2 From axe-core to WIA

```typescript
// Before: axe-core
import axe from 'axe-core';

const results = await axe.run();
console.log(results.violations);

// After: WIA (includes axe-core + more)
import { WiaAccessibility } from '@wia/accessible-ui';

const results = await WiaAccessibility.audit({
  level: 'AA',
  includeBestPractices: true
});
console.log(results.issues);
```

### 10.3 Migration Checklist

- [ ] Install `@wia/accessible-ui` package
- [ ] Initialize WIA in application entry point
- [ ] Replace manual ARIA attribute management with WIA API
- [ ] Update test suite to use WIA matchers
- [ ] Configure CI/CD to run WIA audits
- [ ] Train team on WIA best practices
- [ ] Update design system with accessibility tokens
- [ ] Migrate existing accessibility tests
- [ ] Set up monitoring and alerting
- [ ] Document accessibility standards for team

---

## Appendix A: Integration Examples

See complete integration examples at:
https://wiastandards.com/accessible-ui/integration/

---

## Appendix B: WIA Ecosystem

### Compatible WIA Standards

- **WIA-AAC**: Augmentative & Alternative Communication
- **WIA-BRAILLE-DISPLAY**: Refreshable Braille Interface
- **WIA-INTENT**: Universal Intent Protocol
- **WIA-OMNI-API**: Universal API Gateway
- **WIA-SOCIAL**: Social Media Integration

### Integration Matrix

| Framework | Support Level | Package |
|-----------|---------------|---------|
| React | ✅ Full | `@wia/accessible-ui/react` |
| Vue | ✅ Full | `@wia/accessible-ui/vue` |
| Angular | ✅ Full | `@wia/accessible-ui/angular` |
| Svelte | ✅ Full | `@wia/accessible-ui/svelte` |
| Vanilla JS | ✅ Full | `@wia/accessible-ui` |

---

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
