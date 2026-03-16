# WIA Accessible UI/UX Standard
## Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee
**Standard ID:** WIA-AAC-014

---

## Table of Contents

1. [Introduction](#introduction)
2. [Core API Classes](#core-api-classes)
3. [Accessibility Manager](#accessibility-manager)
4. [Component Wrappers](#component-wrappers)
5. [WCAG Validation API](#wcag-validation-api)
6. [ARIA Helper API](#aria-helper-api)
7. [Keyboard Navigation API](#keyboard-navigation-api)
8. [Screen Reader API](#screen-reader-api)
9. [Color Contrast API](#color-contrast-api)
10. [Testing and Audit API](#testing-and-audit-api)

---

## 1. Introduction

This specification defines the Application Programming Interface (API) for implementing the WIA Accessible UI/UX Standard. It provides a comprehensive set of classes, methods, and utilities for building accessible user interfaces.

### 1.1 Design Goals

- **Easy Integration**: Drop-in compatibility with existing frameworks
- **Type Safety**: Full TypeScript support with comprehensive types
- **Developer Experience**: Intuitive API with helpful error messages
- **Performance**: Minimal runtime overhead
- **Framework Agnostic**: Works with React, Vue, Angular, Svelte, vanilla JS

### 1.2 Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

Accessibility should be the default, not an afterthought. This API makes it easy to build inclusive experiences.

---

## 2. Core API Classes

### 2.1 WiaAccessibility Class

The main entry point for the WIA Accessibility API.

```typescript
class WiaAccessibility {
  /**
   * Initialize the accessibility manager
   * @param options Configuration options
   */
  static init(options?: WiaAccessibilityOptions): WiaAccessibilityManager;

  /**
   * Get the current accessibility manager instance
   */
  static getInstance(): WiaAccessibilityManager;

  /**
   * Validate WCAG compliance for an element
   */
  static validate(
    element: HTMLElement,
    level?: "A" | "AA" | "AAA"
  ): ValidationResult;

  /**
   * Run accessibility audit on entire page
   */
  static audit(options?: AuditOptions): Promise<AuditReport>;

  /**
   * Check if assistive technology is active
   */
  static isAssistiveTechActive(): boolean;

  /**
   * Get user preferences (reduced motion, high contrast, etc.)
   */
  static getUserPreferences(): UserPreferences;
}

interface WiaAccessibilityOptions {
  /** WCAG level to enforce (default: "AA") */
  wcagLevel?: "A" | "AA" | "AAA";

  /** Auto-fix issues when possible */
  autoFix?: boolean;

  /** Log warnings to console */
  logWarnings?: boolean;

  /** Enable live region announcements */
  announcements?: boolean;

  /** Custom validation rules */
  customRules?: ValidationRule[];
}
```

### 2.2 Usage Example

```typescript
import { WiaAccessibility } from '@wia/accessible-ui';

// Initialize
const a11y = WiaAccessibility.init({
  wcagLevel: "AA",
  autoFix: true,
  logWarnings: true
});

// Validate an element
const result = WiaAccessibility.validate(buttonElement);
if (!result.valid) {
  console.error("Accessibility issues:", result.issues);
}

// Run audit
const audit = await WiaAccessibility.audit();
console.log(`Accessibility score: ${audit.score}/100`);
```

---

## 3. Accessibility Manager

### 3.1 WiaAccessibilityManager Class

```typescript
class WiaAccessibilityManager {
  /**
   * Register a component for accessibility monitoring
   */
  registerComponent(
    element: HTMLElement,
    metadata: AccessibilityMetadata
  ): ComponentHandle;

  /**
   * Unregister a component
   */
  unregisterComponent(handle: ComponentHandle): void;

  /**
   * Update component metadata
   */
  updateComponent(
    handle: ComponentHandle,
    metadata: Partial<AccessibilityMetadata>
  ): void;

  /**
   * Get component metadata
   */
  getComponent(handle: ComponentHandle): AccessibilityMetadata | null;

  /**
   * Focus management
   */
  focus: FocusManager;

  /**
   * Live region announcements
   */
  announce: AnnouncementManager;

  /**
   * Keyboard navigation
   */
  keyboard: KeyboardManager;

  /**
   * ARIA helper
   */
  aria: AriaHelper;

  /**
   * Color and visual utilities
   */
  visual: VisualHelper;

  /**
   * WCAG validator
   */
  wcag: WcagValidator;
}
```

### 3.2 Component Registration Example

```typescript
// Register a button
const handle = a11y.registerComponent(buttonElement, {
  id: "submit-btn",
  type: "button",
  wcag: { version: "2.1", level: "AA" },
  aria: {
    role: "button",
    label: "Submit form"
  },
  keyboard: {
    accessible: true,
    tabIndex: 0
  }
});

// Update metadata
a11y.updateComponent(handle, {
  aria: {
    state: { disabled: true }
  }
});

// Cleanup
a11y.unregisterComponent(handle);
```

---

## 4. Component Wrappers

### 4.1 Accessible Button

```typescript
class AccessibleButton {
  constructor(
    element: HTMLElement,
    options: AccessibleButtonOptions
  );

  /**
   * Set button label
   */
  setLabel(label: string): void;

  /**
   * Enable/disable button
   */
  setDisabled(disabled: boolean): void;

  /**
   * Set pressed state (for toggle buttons)
   */
  setPressed(pressed: boolean | "mixed"): void;

  /**
   * Add keyboard shortcut
   */
  addShortcut(keys: string, handler: () => void): void;

  /**
   * Validate WCAG compliance
   */
  validate(): ValidationResult;

  /**
   * Destroy and cleanup
   */
  destroy(): void;
}

interface AccessibleButtonOptions {
  label: string;
  description?: string;
  variant?: "primary" | "secondary" | "danger";
  icon?: string;
  iconPosition?: "left" | "right";
  shortcuts?: string[];
}
```

### 4.2 Accessible Form

```typescript
class AccessibleForm {
  constructor(
    element: HTMLFormElement,
    options: AccessibleFormOptions
  );

  /**
   * Add form field
   */
  addField(field: FormField): void;

  /**
   * Validate form
   */
  validate(): FormValidationResult;

  /**
   * Submit with accessibility checks
   */
  submit(): Promise<void>;

  /**
   * Show error messages accessibly
   */
  showErrors(errors: FieldError[]): void;

  /**
   * Focus first invalid field
   */
  focusFirstError(): void;
}

interface FormField {
  name: string;
  label: string;
  type: "text" | "email" | "password" | "select" | "checkbox" | "radio";
  required?: boolean;
  hint?: string;
  validation?: ValidationRule[];
}
```

### 4.3 Accessible Dialog

```typescript
class AccessibleDialog {
  constructor(
    element: HTMLElement,
    options: AccessibleDialogOptions
  );

  /**
   * Open dialog
   */
  open(): void;

  /**
   * Close dialog
   */
  close(): void;

  /**
   * Set dialog content
   */
  setContent(content: string | HTMLElement): void;

  /**
   * Trap focus within dialog
   */
  private trapFocus(): void;

  /**
   * Restore focus to trigger element
   */
  private restoreFocus(): void;
}

interface AccessibleDialogOptions {
  title: string;
  description?: string;
  modal?: boolean;
  closeOnEscape?: boolean;
  closeOnOutsideClick?: boolean;
  initialFocus?: HTMLElement;
}
```

---

## 5. WCAG Validation API

### 5.1 WcagValidator Class

```typescript
class WcagValidator {
  /**
   * Validate element against WCAG criteria
   */
  validateElement(
    element: HTMLElement,
    level?: "A" | "AA" | "AAA"
  ): ValidationResult;

  /**
   * Check specific WCAG criterion
   */
  checkCriterion(
    element: HTMLElement,
    criterion: string // e.g., "1.4.3"
  ): CriterionResult;

  /**
   * Validate perceivable principle
   */
  validatePerceivable(element: HTMLElement): ValidationResult;

  /**
   * Validate operable principle
   */
  validateOperable(element: HTMLElement): ValidationResult;

  /**
   * Validate understandable principle
   */
  validateUnderstandable(element: HTMLElement): ValidationResult;

  /**
   * Validate robust principle
   */
  validateRobust(element: HTMLElement): ValidationResult;

  /**
   * Get all failing criteria
   */
  getFailingCriteria(element: HTMLElement): string[];
}

interface ValidationResult {
  valid: boolean;
  score: number; // 0-100
  level: "A" | "AA" | "AAA";
  issues: AccessibilityIssue[];
  passed: string[]; // Criteria that passed
  failed: string[]; // Criteria that failed
}

interface AccessibilityIssue {
  criterion: string; // WCAG criterion (e.g., "1.4.3")
  level: "A" | "AA" | "AAA";
  severity: "critical" | "serious" | "moderate" | "minor";
  message: string;
  element: HTMLElement;
  selector: string;
  fix?: string; // Suggested fix
}
```

### 5.2 Validation Example

```typescript
const validator = a11y.wcag;

// Validate entire element
const result = validator.validateElement(element, "AA");

if (!result.valid) {
  result.issues.forEach(issue => {
    console.error(
      `[${issue.severity}] ${issue.criterion}: ${issue.message}`
    );
    if (issue.fix) {
      console.log(`Suggested fix: ${issue.fix}`);
    }
  });
}

// Check specific criterion (contrast)
const contrastResult = validator.checkCriterion(element, "1.4.3");
```

---

## 6. ARIA Helper API

### 6.1 AriaHelper Class

```typescript
class AriaHelper {
  /**
   * Set ARIA role
   */
  setRole(element: HTMLElement, role: string): void;

  /**
   * Set ARIA label
   */
  setLabel(element: HTMLElement, label: string): void;

  /**
   * Set ARIA description
   */
  setDescription(element: HTMLElement, description: string): void;

  /**
   * Set ARIA state
   */
  setState(
    element: HTMLElement,
    state: Record<string, boolean | string>
  ): void;

  /**
   * Set ARIA expanded
   */
  setExpanded(element: HTMLElement, expanded: boolean): void;

  /**
   * Set ARIA selected
   */
  setSelected(element: HTMLElement, selected: boolean): void;

  /**
   * Set ARIA disabled
   */
  setDisabled(element: HTMLElement, disabled: boolean): void;

  /**
   * Set ARIA live region
   */
  setLive(
    element: HTMLElement,
    live: "off" | "polite" | "assertive"
  ): void;

  /**
   * Set ARIA relationships
   */
  setRelationship(
    element: HTMLElement,
    relationship: string, // e.g., "controls", "describedby"
    targetId: string
  ): void;

  /**
   * Build complete ARIA attributes
   */
  buildARIA(
    element: HTMLElement,
    attributes: ARIAAttributes
  ): void;

  /**
   * Validate ARIA usage
   */
  validate(element: HTMLElement): ARIAValidationResult;
}
```

### 6.2 ARIA Example

```typescript
const aria = a11y.aria;

// Simple ARIA setup
aria.setRole(element, "button");
aria.setLabel(element, "Submit form");
aria.setState(element, { pressed: false });

// Complete ARIA setup
aria.buildARIA(element, {
  role: "dialog",
  label: "Confirm action",
  description: "Are you sure?",
  state: {
    hidden: false
  },
  relationships: {
    labelledBy: "dialog-title",
    describedBy: "dialog-content"
  },
  properties: {
    modal: true
  }
});

// Validate ARIA
const validation = aria.validate(element);
if (!validation.valid) {
  console.warn("ARIA issues:", validation.errors);
}
```

---

## 7. Keyboard Navigation API

### 7.1 KeyboardManager Class

```typescript
class KeyboardManager {
  /**
   * Make element keyboard accessible
   */
  makeAccessible(
    element: HTMLElement,
    options?: KeyboardOptions
  ): void;

  /**
   * Add keyboard shortcut
   */
  addShortcut(
    keys: string,
    handler: (event: KeyboardEvent) => void,
    options?: ShortcutOptions
  ): ShortcutHandle;

  /**
   * Remove keyboard shortcut
   */
  removeShortcut(handle: ShortcutHandle): void;

  /**
   * Set navigation pattern
   */
  setNavigationPattern(
    container: HTMLElement,
    pattern: NavigationPattern,
    options?: NavigationOptions
  ): void;

  /**
   * Trap focus within element
   */
  trapFocus(element: HTMLElement): FocusTrap;

  /**
   * Release focus trap
   */
  releaseFocusTrap(trap: FocusTrap): void;

  /**
   * Get focusable elements within container
   */
  getFocusableElements(container: HTMLElement): HTMLElement[];

  /**
   * Focus first element
   */
  focusFirst(container: HTMLElement): void;

  /**
   * Focus last element
   */
  focusLast(container: HTMLElement): void;
}

interface KeyboardOptions {
  tabIndex?: number;
  focusVisible?: boolean;
  focusOutline?: string;
}

interface ShortcutOptions {
  description?: string;
  preventDefault?: boolean;
  stopPropagation?: boolean;
  platform?: "windows" | "mac" | "linux";
}

interface NavigationOptions {
  wrap?: boolean; // Wrap around at ends
  orientation?: "horizontal" | "vertical" | "both";
}
```

### 7.2 Keyboard Example

```typescript
const keyboard = a11y.keyboard;

// Make element keyboard accessible
keyboard.makeAccessible(button, {
  tabIndex: 0,
  focusVisible: true,
  focusOutline: "2px solid #3B82F6"
});

// Add keyboard shortcut
const shortcut = keyboard.addShortcut("Ctrl+S", (e) => {
  saveForm();
}, {
  description: "Save form",
  preventDefault: true
});

// Arrow key navigation for menu
keyboard.setNavigationPattern(menuElement, "arrows", {
  wrap: true,
  orientation: "vertical"
});

// Focus trap for modal
const trap = keyboard.trapFocus(modalElement);
// Later...
keyboard.releaseFocusTrap(trap);
```

---

## 8. Screen Reader API

### 8.1 AnnouncementManager Class

```typescript
class AnnouncementManager {
  /**
   * Announce message to screen readers
   */
  announce(
    message: string,
    priority?: "polite" | "assertive"
  ): void;

  /**
   * Create persistent live region
   */
  createLiveRegion(
    id: string,
    options?: LiveRegionOptions
  ): LiveRegion;

  /**
   * Update live region
   */
  updateLiveRegion(id: string, content: string): void;

  /**
   * Remove live region
   */
  removeLiveRegion(id: string): void;

  /**
   * Announce navigation change
   */
  announceNavigation(pageName: string): void;

  /**
   * Announce loading state
   */
  announceLoading(loading: boolean): void;

  /**
   * Announce error
   */
  announceError(message: string): void;

  /**
   * Announce success
   */
  announceSuccess(message: string): void;
}

interface LiveRegionOptions {
  priority?: "polite" | "assertive";
  atomic?: boolean;
  relevant?: string;
  role?: "status" | "alert" | "log";
}

class LiveRegion {
  update(content: string): void;
  destroy(): void;
}
```

### 8.2 Screen Reader Example

```typescript
const announce = a11y.announce;

// Simple announcement
announce.announce("Form submitted successfully", "polite");

// Create persistent live region
const statusRegion = announce.createLiveRegion("status", {
  priority: "polite",
  role: "status"
});

// Update live region
announce.updateLiveRegion("status", "Loading...");
announce.updateLiveRegion("status", "3 new items");

// Convenience methods
announce.announceNavigation("Dashboard");
announce.announceLoading(true);
announce.announceError("Invalid email address");
announce.announceSuccess("Settings saved");
```

---

## 9. Color Contrast API

### 9.1 VisualHelper Class

```typescript
class VisualHelper {
  /**
   * Calculate color contrast ratio
   */
  getContrastRatio(
    foreground: string,
    background: string
  ): number;

  /**
   * Check if contrast meets WCAG requirements
   */
  meetsContrast(
    foreground: string,
    background: string,
    level: "AA" | "AAA",
    size: "normal" | "large"
  ): boolean;

  /**
   * Find accessible foreground color
   */
  findAccessibleForeground(
    background: string,
    level: "AA" | "AAA"
  ): string;

  /**
   * Validate element color contrast
   */
  validateContrast(element: HTMLElement): ContrastValidation;

  /**
   * Get computed colors for element
   */
  getColors(element: HTMLElement): {
    foreground: string;
    background: string;
  };

  /**
   * Convert color formats
   */
  convertColor(
    color: string,
    format: "hex" | "rgb" | "hsl"
  ): string;

  /**
   * Get luminance value
   */
  getLuminance(color: string): number;

  /**
   * Check if color is dark
   */
  isDark(color: string): boolean;

  /**
   * Generate accessible color palette
   */
  generatePalette(
    baseColor: string,
    level: "AA" | "AAA"
  ): ColorPalette;
}

interface ContrastValidation {
  ratio: number;
  foreground: string;
  background: string;
  passes: {
    AA: boolean;
    AAA: boolean;
  };
  textSize: "normal" | "large";
}

interface ColorPalette {
  primary: string;
  onPrimary: string; // Accessible text color
  secondary: string;
  onSecondary: string;
  background: string;
  onBackground: string;
  surface: string;
  onSurface: string;
}
```

### 9.2 Color Contrast Example

```typescript
const visual = a11y.visual;

// Calculate contrast ratio
const ratio = visual.getContrastRatio("#FFFFFF", "#3B82F6");
console.log(`Contrast ratio: ${ratio}:1`);

// Check if meets requirements
const meetsAA = visual.meetsContrast(
  "#FFFFFF",
  "#3B82F6",
  "AA",
  "normal"
);

// Validate element
const validation = visual.validateContrast(buttonElement);
if (!validation.passes.AA) {
  console.warn("Insufficient contrast!");
}

// Find accessible color
const accessibleFg = visual.findAccessibleForeground("#3B82F6", "AA");

// Generate palette
const palette = visual.generatePalette("#3B82F6", "AA");
```

---

## 10. Testing and Audit API

### 10.1 Audit Functions

```typescript
interface AuditOptions {
  /** WCAG level to audit against */
  level?: "A" | "AA" | "AAA";

  /** Include best practices (beyond WCAG) */
  includeBestPractices?: boolean;

  /** Specific rules to run */
  rules?: string[];

  /** Elements to exclude */
  exclude?: string[];

  /** Output format */
  format?: "json" | "html" | "markdown";
}

interface AuditReport {
  /** Overall accessibility score (0-100) */
  score: number;

  /** WCAG level tested */
  level: "A" | "AA" | "AAA";

  /** Timestamp */
  timestamp: string;

  /** Summary statistics */
  summary: {
    totalIssues: number;
    critical: number;
    serious: number;
    moderate: number;
    minor: number;
  };

  /** Issues found */
  issues: AccessibilityIssue[];

  /** Passed criteria */
  passed: string[];

  /** Failed criteria */
  failed: string[];

  /** WCAG principle breakdown */
  principles: {
    perceivable: number; // 0-100
    operable: number;
    understandable: number;
    robust: number;
  };

  /** Recommendations */
  recommendations: string[];
}
```

### 10.2 Testing Example

```typescript
// Run comprehensive audit
const audit = await WiaAccessibility.audit({
  level: "AA",
  includeBestPractices: true
});

console.log(`Accessibility Score: ${audit.score}/100`);
console.log(`Issues found: ${audit.summary.totalIssues}`);

// Filter critical issues
const critical = audit.issues.filter(
  issue => issue.severity === "critical"
);

// Generate report
const htmlReport = await WiaAccessibility.audit({
  level: "AA",
  format: "html"
});
document.body.innerHTML = htmlReport;
```

---

## 11. Framework Integration

### 11.1 React Integration

```tsx
import { useAccessibility } from '@wia/accessible-ui/react';

function AccessibleButton({ label, onClick }) {
  const { ref, aria, keyboard } = useAccessibility({
    type: 'button',
    label,
    wcagLevel: 'AA'
  });

  return (
    <button
      ref={ref}
      {...aria}
      {...keyboard}
      onClick={onClick}>
      {label}
    </button>
  );
}
```

### 11.2 Vue Integration

```vue
<template>
  <button v-accessible="{ type: 'button', label: 'Submit' }">
    Submit
  </button>
</template>

<script>
import { accessibleDirective } from '@wia/accessible-ui/vue';

export default {
  directives: {
    accessible: accessibleDirective
  }
};
</script>
```

---

## Appendix A: Complete API Reference

See full API documentation at:
https://wiastandards.com/accessible-ui/api/

---

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
