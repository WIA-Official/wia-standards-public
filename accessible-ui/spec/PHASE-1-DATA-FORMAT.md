# WIA Accessible UI/UX Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee
**Standard ID:** WIA-AAC-014

---

## Table of Contents

1. [Introduction](#introduction)
2. [Accessibility Data Model](#accessibility-data-model)
3. [JSON Schema Definitions](#json-schema-definitions)
4. [WCAG Compliance Metadata](#wcag-compliance-metadata)
5. [ARIA Attribute Mapping](#aria-attribute-mapping)
6. [Component Accessibility Profiles](#component-accessibility-profiles)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)
9. [Internationalization](#internationalization)
10. [Error Handling](#error-handling)

---

## 1. Introduction

This specification defines the standardized data format for representing accessibility metadata in the WIA Accessible UI/UX ecosystem. The format enables automated accessibility testing, runtime validation, and assistive technology integration.

### 1.1 Design Principles

1. **Universal Accessibility**: Support all WCAG 2.1 Level AAA criteria
2. **Semantic Clarity**: Clear, self-documenting data structures
3. **Machine-Readable**: Enable automated compliance checking
4. **Extensible**: Support custom accessibility requirements
5. **Developer-Friendly**: Intuitive API for implementation

### 1.2 Core Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

This standard embodies the principle that digital experiences should be accessible to everyone, regardless of ability, language, or device.

### 1.3 Scope

This specification covers:
- Component accessibility metadata
- WCAG compliance tracking
- ARIA attribute standardization
- Keyboard navigation patterns
- Color contrast requirements
- Screen reader optimization
- Cognitive accessibility features

---

## 2. Accessibility Data Model

### 2.1 Core Data Types

#### 2.1.1 AccessibilityMetadata

```typescript
interface AccessibilityMetadata {
  /** Unique identifier for this component */
  id: string;

  /** Component type (button, link, input, etc.) */
  type: ComponentType;

  /** WCAG compliance level */
  wcag: WCAGCompliance;

  /** ARIA attributes */
  aria: ARIAAttributes;

  /** Keyboard navigation support */
  keyboard: KeyboardSupport;

  /** Color and visual accessibility */
  visual: VisualAccessibility;

  /** Cognitive accessibility features */
  cognitive: CognitiveAccessibility;

  /** Screen reader optimization */
  screenReader: ScreenReaderSupport;

  /** Custom extensions */
  extensions?: Record<string, any>;
}
```

#### 2.1.2 WCAGCompliance

```typescript
interface WCAGCompliance {
  /** WCAG version (2.0, 2.1, 2.2) */
  version: string;

  /** Compliance level (A, AA, AAA) */
  level: "A" | "AA" | "AAA";

  /** Perceivable principle compliance */
  perceivable: {
    textAlternatives: boolean;
    timeBased: boolean;
    adaptable: boolean;
    distinguishable: boolean;
  };

  /** Operable principle compliance */
  operable: {
    keyboardAccessible: boolean;
    enoughTime: boolean;
    seizuresPhysical: boolean;
    navigable: boolean;
    inputModalities: boolean;
  };

  /** Understandable principle compliance */
  understandable: {
    readable: boolean;
    predictable: boolean;
    inputAssistance: boolean;
  };

  /** Robust principle compliance */
  robust: {
    compatible: boolean;
  };

  /** Automated test results */
  tests?: AccessibilityTestResults[];
}
```

#### 2.1.3 ARIAAttributes

```typescript
interface ARIAAttributes {
  /** ARIA role */
  role?: string;

  /** Accessible name/label */
  label?: string;

  /** Extended description */
  description?: string;

  /** Current state (expanded, selected, etc.) */
  state?: {
    expanded?: boolean;
    selected?: boolean;
    checked?: boolean | "mixed";
    disabled?: boolean;
    hidden?: boolean;
    pressed?: boolean | "mixed";
  };

  /** Relationship attributes */
  relationships?: {
    controls?: string;
    describedBy?: string;
    labelledBy?: string;
    owns?: string;
    flowTo?: string;
  };

  /** Live region properties */
  live?: {
    atomic?: boolean;
    busy?: boolean;
    live?: "off" | "polite" | "assertive";
    relevant?: string;
  };

  /** Additional ARIA properties */
  properties?: Record<string, any>;
}
```

#### 2.1.4 KeyboardSupport

```typescript
interface KeyboardSupport {
  /** Is keyboard accessible? */
  accessible: boolean;

  /** Tab index (-1, 0, positive) */
  tabIndex?: number;

  /** Keyboard shortcuts */
  shortcuts?: KeyboardShortcut[];

  /** Navigation pattern (tab, arrows, etc.) */
  navigationPattern?: NavigationPattern;

  /** Focus management */
  focus?: {
    visible: boolean;
    outline: string;
    trapFocus?: boolean;
    restoreFocus?: boolean;
  };
}

interface KeyboardShortcut {
  /** Key combination (e.g., "Ctrl+S") */
  keys: string;

  /** Description of action */
  description: string;

  /** Platform-specific (windows, mac, linux) */
  platform?: string;
}

type NavigationPattern =
  | "tab"           // Tab/Shift+Tab navigation
  | "arrows"        // Arrow key navigation
  | "arrows-2d"     // 2D grid navigation
  | "custom";       // Custom navigation
```

#### 2.1.5 VisualAccessibility

```typescript
interface VisualAccessibility {
  /** Color contrast information */
  contrast: {
    ratio: number;
    foreground: string;
    background: string;
    passes: {
      AA: boolean;
      AAA: boolean;
    };
  };

  /** Font and text sizing */
  typography: {
    minFontSize: number;
    maxFontSize: number;
    lineHeight: number;
    letterSpacing?: number;
    scalable: boolean;
  };

  /** Visual indicators */
  indicators: {
    focusIndicator: boolean;
    errorIndicator: boolean;
    requiredIndicator: boolean;
  };

  /** Animation and motion */
  motion: {
    respectsReducedMotion: boolean;
    duration?: number;
    pausable?: boolean;
  };
}
```

#### 2.1.6 CognitiveAccessibility

```typescript
interface CognitiveAccessibility {
  /** Reading level (Flesch-Kincaid) */
  readingLevel?: number;

  /** Clear error messages */
  errorHandling: {
    clear: boolean;
    suggestCorrection: boolean;
    preventErrors: boolean;
  };

  /** Consistent patterns */
  consistency: {
    navigation: boolean;
    identification: boolean;
    terminology: boolean;
  };

  /** Help and documentation */
  help?: {
    available: boolean;
    contextual: boolean;
    examples: boolean;
  };

  /** Time limits */
  timing?: {
    hasTimeLimit: boolean;
    adjustable: boolean;
    warningProvided: boolean;
  };
}
```

#### 2.1.7 ScreenReaderSupport

```typescript
interface ScreenReaderSupport {
  /** Optimized for screen readers */
  optimized: boolean;

  /** Announcement text */
  announcement?: string;

  /** Reading order */
  readingOrder?: number;

  /** Skip links available */
  skipLinks?: string[];

  /** Landmark regions */
  landmarks?: {
    role: string;
    label?: string;
  }[];

  /** Alternative content */
  alternatives?: {
    type: "text" | "audio" | "video";
    content: string;
  }[];
}
```

---

## 3. JSON Schema Definitions

### 3.1 Complete Accessibility Metadata Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/accessible-ui/v1.0.0/metadata.json",
  "title": "WIA Accessible UI/UX Metadata",
  "type": "object",
  "required": ["id", "type", "wcag", "aria", "keyboard"],
  "properties": {
    "id": {
      "type": "string",
      "description": "Unique identifier for this component"
    },
    "type": {
      "type": "string",
      "enum": ["button", "link", "input", "select", "checkbox", "radio", "dialog", "menu", "navigation", "form", "image", "video", "custom"]
    },
    "wcag": {
      "$ref": "#/definitions/wcagCompliance"
    },
    "aria": {
      "$ref": "#/definitions/ariaAttributes"
    },
    "keyboard": {
      "$ref": "#/definitions/keyboardSupport"
    },
    "visual": {
      "$ref": "#/definitions/visualAccessibility"
    },
    "cognitive": {
      "$ref": "#/definitions/cognitiveAccessibility"
    },
    "screenReader": {
      "$ref": "#/definitions/screenReaderSupport"
    },
    "extensions": {
      "type": "object"
    }
  },
  "definitions": {
    "wcagCompliance": {
      "type": "object",
      "required": ["version", "level"],
      "properties": {
        "version": {
          "type": "string",
          "enum": ["2.0", "2.1", "2.2"]
        },
        "level": {
          "type": "string",
          "enum": ["A", "AA", "AAA"]
        },
        "perceivable": {
          "type": "object",
          "properties": {
            "textAlternatives": {"type": "boolean"},
            "timeBased": {"type": "boolean"},
            "adaptable": {"type": "boolean"},
            "distinguishable": {"type": "boolean"}
          }
        }
      }
    },
    "ariaAttributes": {
      "type": "object",
      "properties": {
        "role": {"type": "string"},
        "label": {"type": "string"},
        "description": {"type": "string"},
        "state": {"type": "object"},
        "relationships": {"type": "object"},
        "live": {"type": "object"}
      }
    },
    "keyboardSupport": {
      "type": "object",
      "required": ["accessible"],
      "properties": {
        "accessible": {"type": "boolean"},
        "tabIndex": {"type": "number"},
        "shortcuts": {"type": "array"},
        "navigationPattern": {
          "type": "string",
          "enum": ["tab", "arrows", "arrows-2d", "custom"]
        }
      }
    }
  }
}
```

---

## 4. WCAG Compliance Metadata

### 4.1 WCAG 2.1 Success Criteria Mapping

| Criterion | Level | Category | Data Field |
|-----------|-------|----------|------------|
| 1.1.1 Non-text Content | A | Perceivable | `wcag.perceivable.textAlternatives` |
| 1.2.1 Audio-only and Video-only | A | Perceivable | `wcag.perceivable.timeBased` |
| 1.3.1 Info and Relationships | A | Perceivable | `wcag.perceivable.adaptable` |
| 1.4.1 Use of Color | A | Perceivable | `wcag.perceivable.distinguishable` |
| 1.4.3 Contrast (Minimum) | AA | Perceivable | `visual.contrast` |
| 1.4.6 Contrast (Enhanced) | AAA | Perceivable | `visual.contrast` |
| 2.1.1 Keyboard | A | Operable | `keyboard.accessible` |
| 2.1.2 No Keyboard Trap | A | Operable | `keyboard.focus.trapFocus` |
| 2.4.1 Bypass Blocks | A | Operable | `screenReader.skipLinks` |
| 2.4.3 Focus Order | A | Operable | `keyboard.tabIndex` |
| 2.4.7 Focus Visible | AA | Operable | `keyboard.focus.visible` |
| 3.1.1 Language of Page | A | Understandable | `extensions.language` |
| 3.2.1 On Focus | A | Understandable | `wcag.understandable.predictable` |
| 3.3.1 Error Identification | A | Understandable | `cognitive.errorHandling.clear` |
| 3.3.2 Labels or Instructions | A | Understandable | `aria.label` |
| 4.1.1 Parsing | A | Robust | `wcag.robust.compatible` |
| 4.1.2 Name, Role, Value | A | Robust | `aria.role`, `aria.label` |

### 4.2 Automated Compliance Checking

```typescript
interface AccessibilityTestResults {
  /** Test ID */
  testId: string;

  /** WCAG criterion (e.g., "1.4.3") */
  criterion: string;

  /** Test result */
  result: "pass" | "fail" | "warning" | "not-applicable";

  /** Score (0-100) */
  score?: number;

  /** Issues found */
  issues?: AccessibilityIssue[];

  /** Timestamp */
  timestamp: string;
}

interface AccessibilityIssue {
  /** Issue severity */
  severity: "critical" | "serious" | "moderate" | "minor";

  /** Issue description */
  description: string;

  /** Element selector */
  selector?: string;

  /** Suggested fix */
  fix?: string;

  /** WCAG reference */
  wcagRef?: string;
}
```

---

## 5. ARIA Attribute Mapping

### 5.1 Common Component ARIA Patterns

#### Button
```json
{
  "aria": {
    "role": "button",
    "label": "Submit form",
    "state": {
      "disabled": false,
      "pressed": false
    }
  }
}
```

#### Modal Dialog
```json
{
  "aria": {
    "role": "dialog",
    "label": "Confirmation dialog",
    "description": "Are you sure you want to proceed?",
    "state": {
      "hidden": false
    },
    "relationships": {
      "labelledBy": "dialog-title",
      "describedBy": "dialog-content"
    },
    "properties": {
      "modal": true
    }
  }
}
```

#### Navigation Menu
```json
{
  "aria": {
    "role": "navigation",
    "label": "Main navigation",
    "relationships": {
      "owns": "menu-items"
    }
  }
}
```

#### Tab Panel
```json
{
  "aria": {
    "role": "tablist",
    "label": "Content sections",
    "relationships": {
      "owns": "tab1 tab2 tab3"
    }
  }
}
```

---

## 6. Component Accessibility Profiles

### 6.1 Button Profile

```json
{
  "id": "submit-button",
  "type": "button",
  "wcag": {
    "version": "2.1",
    "level": "AAA",
    "perceivable": {
      "textAlternatives": true,
      "distinguishable": true
    },
    "operable": {
      "keyboardAccessible": true,
      "navigable": true
    },
    "understandable": {
      "readable": true,
      "predictable": true
    },
    "robust": {
      "compatible": true
    }
  },
  "aria": {
    "role": "button",
    "label": "Submit registration form",
    "description": "Clicking this button will submit your registration information",
    "state": {
      "disabled": false
    }
  },
  "keyboard": {
    "accessible": true,
    "tabIndex": 0,
    "shortcuts": [
      {
        "keys": "Enter",
        "description": "Activate button"
      },
      {
        "keys": "Space",
        "description": "Activate button"
      }
    ],
    "navigationPattern": "tab",
    "focus": {
      "visible": true,
      "outline": "2px solid #3B82F6"
    }
  },
  "visual": {
    "contrast": {
      "ratio": 7.2,
      "foreground": "#FFFFFF",
      "background": "#3B82F6",
      "passes": {
        "AA": true,
        "AAA": true
      }
    },
    "typography": {
      "minFontSize": 16,
      "maxFontSize": 24,
      "lineHeight": 1.5,
      "scalable": true
    },
    "indicators": {
      "focusIndicator": true,
      "errorIndicator": false,
      "requiredIndicator": false
    },
    "motion": {
      "respectsReducedMotion": true
    }
  },
  "cognitive": {
    "errorHandling": {
      "clear": true,
      "suggestCorrection": true,
      "preventErrors": true
    },
    "consistency": {
      "navigation": true,
      "identification": true,
      "terminology": true
    }
  },
  "screenReader": {
    "optimized": true,
    "announcement": "Submit registration form button",
    "readingOrder": 10
  }
}
```

---

## 7. Validation Rules

### 7.1 Required Fields

All accessibility metadata must include:
- `id`: Unique identifier
- `type`: Component type
- `wcag.version`: WCAG version
- `wcag.level`: Compliance level
- `aria.role` or `aria.label`: Accessible name
- `keyboard.accessible`: Keyboard support indicator

### 7.2 Color Contrast Validation

```typescript
function validateContrast(
  foreground: string,
  background: string,
  level: "AA" | "AAA",
  size: "normal" | "large"
): boolean {
  const ratio = calculateContrastRatio(foreground, background);

  if (level === "AA") {
    return size === "large" ? ratio >= 3 : ratio >= 4.5;
  } else {
    return size === "large" ? ratio >= 4.5 : ratio >= 7;
  }
}
```

### 7.3 ARIA Validation

```typescript
function validateARIA(aria: ARIAAttributes): ValidationResult {
  const errors: string[] = [];

  // Role must be valid
  if (aria.role && !VALID_ARIA_ROLES.includes(aria.role)) {
    errors.push(`Invalid ARIA role: ${aria.role}`);
  }

  // Label or labelledBy required for most interactive elements
  if (!aria.label && !aria.relationships?.labelledBy) {
    errors.push("Missing accessible name (label or labelledBy)");
  }

  // State properties must be boolean or specific values
  if (aria.state?.checked &&
      typeof aria.state.checked !== "boolean" &&
      aria.state.checked !== "mixed") {
    errors.push("Invalid checked state value");
  }

  return {
    valid: errors.length === 0,
    errors
  };
}
```

---

## 8. Example Payloads

### 8.1 Form Input

```json
{
  "id": "email-input",
  "type": "input",
  "wcag": {
    "version": "2.1",
    "level": "AA"
  },
  "aria": {
    "role": "textbox",
    "label": "Email address",
    "description": "Enter your email address for login",
    "state": {
      "required": true,
      "invalid": false
    },
    "relationships": {
      "labelledBy": "email-label",
      "describedBy": "email-hint"
    }
  },
  "keyboard": {
    "accessible": true,
    "tabIndex": 0,
    "navigationPattern": "tab"
  },
  "visual": {
    "contrast": {
      "ratio": 4.8,
      "foreground": "#1a1a1a",
      "background": "#ffffff",
      "passes": {
        "AA": true,
        "AAA": false
      }
    }
  },
  "cognitive": {
    "errorHandling": {
      "clear": true,
      "suggestCorrection": true,
      "preventErrors": true
    }
  },
  "screenReader": {
    "optimized": true,
    "announcement": "Email address, required field"
  }
}
```

### 8.2 Navigation Menu

```json
{
  "id": "main-nav",
  "type": "navigation",
  "wcag": {
    "version": "2.1",
    "level": "AAA"
  },
  "aria": {
    "role": "navigation",
    "label": "Main navigation",
    "relationships": {
      "owns": "nav-home nav-about nav-contact"
    }
  },
  "keyboard": {
    "accessible": true,
    "tabIndex": 0,
    "shortcuts": [
      {
        "keys": "Alt+N",
        "description": "Jump to navigation"
      }
    ],
    "navigationPattern": "arrows",
    "focus": {
      "visible": true,
      "outline": "2px solid #3B82F6",
      "trapFocus": false
    }
  },
  "screenReader": {
    "optimized": true,
    "skipLinks": ["#main-content"],
    "landmarks": [
      {
        "role": "navigation",
        "label": "Main navigation"
      }
    ]
  }
}
```

---

## 9. Internationalization

### 9.1 Language Support

```typescript
interface I18nAccessibility {
  /** Primary language (BCP 47) */
  lang: string;

  /** Text direction */
  dir: "ltr" | "rtl" | "auto";

  /** Translations for accessibility text */
  translations?: {
    [locale: string]: {
      label?: string;
      description?: string;
      announcement?: string;
    };
  };
}
```

### 9.2 Example with Translations

```json
{
  "id": "submit-btn",
  "type": "button",
  "aria": {
    "label": "Submit",
    "description": "Submit the form"
  },
  "extensions": {
    "i18n": {
      "lang": "en",
      "dir": "ltr",
      "translations": {
        "ko": {
          "label": "제출",
          "description": "양식 제출"
        },
        "es": {
          "label": "Enviar",
          "description": "Enviar el formulario"
        }
      }
    }
  }
}
```

---

## 10. Error Handling

### 10.1 Validation Errors

```typescript
interface ValidationError {
  /** Error code */
  code: string;

  /** Human-readable message */
  message: string;

  /** Field that failed validation */
  field: string;

  /** Current value */
  value: any;

  /** Expected value or pattern */
  expected?: any;
}
```

### 10.2 Common Validation Errors

- `MISSING_REQUIRED_FIELD`: Required field not provided
- `INVALID_WCAG_LEVEL`: Invalid WCAG level
- `INVALID_ARIA_ROLE`: Unknown or invalid ARIA role
- `INSUFFICIENT_CONTRAST`: Color contrast below minimum
- `MISSING_KEYBOARD_SUPPORT`: Interactive element not keyboard accessible
- `INVALID_TAB_INDEX`: Tab index value invalid

---

## Appendix A: References

- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [ARIA Authoring Practices](https://www.w3.org/WAI/ARIA/apg/)
- [WebAIM Contrast Checker](https://webaim.org/resources/contrastchecker/)
- [JSON Schema](https://json-schema.org/)

---

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
