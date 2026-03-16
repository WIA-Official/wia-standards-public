# WIA Braille Display Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee

---

## Table of Contents

1. [Introduction](#introduction)
2. [Braille Cell Data Model](#braille-cell-data-model)
3. [JSON Schema Definitions](#json-schema-definitions)
4. [Unicode Braille Mapping](#unicode-braille-mapping)
5. [6-Dot and 8-Dot Formats](#6-dot-and-8-dot-formats)
6. [Multi-Language Support](#multi-language-support)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)
9. [Binary Encoding](#binary-encoding)
10. [Compression and Optimization](#compression-and-optimization)

---

## 1. Introduction

This specification defines the standardized data format for representing Braille cell information in the WIA Braille Display ecosystem. The format is designed to be:

- **Universal**: Support all Braille cell types (6-dot, 8-dot)
- **Extensible**: Allow for future enhancements
- **Efficient**: Optimize for transmission over various protocols
- **Language-agnostic**: Support all writing systems
- **Machine-readable**: Enable automated processing and validation

### 1.1 Design Principles

1. **Interoperability**: Compatible with existing Braille standards (Unicode Braille Patterns, BRLTTY)
2. **Accessibility**: Human-readable JSON with binary optimization options
3. **Versioning**: Support multiple format versions simultaneously
4. **Error Resilience**: Graceful degradation and comprehensive error reporting

### 1.2 Scope

This specification covers:
- Braille cell representation (dots 1-8)
- Cell positioning and layout
- Metadata and attributes
- Character encoding and Unicode mapping
- Validation and error handling

---

## 2. Braille Cell Data Model

### 2.1 Fundamental Cell Structure

A Braille cell consists of up to 8 tactile dots arranged in two columns:

```
Column 1:  Column 2:
   1         4
   2         5
   3         6
   7         8
```

### 2.2 Core Data Types

#### 2.2.1 BrailleCell (Basic)

```typescript
interface BrailleCell {
  /** Dot pattern as 8-bit value (0-255) */
  dots: number;

  /** Cell format: 6 or 8 dots */
  format: 6 | 8;

  /** Unicode Braille Pattern character (U+2800 to U+28FF) */
  unicode?: string;

  /** Original text character this represents */
  char?: string;

  /** Cell attributes (cursor, selection, etc.) */
  attributes?: CellAttributes;
}
```

#### 2.2.2 CellAttributes

```typescript
interface CellAttributes {
  /** Is this cell under the cursor? */
  cursor?: boolean;

  /** Is this cell selected? */
  selected?: boolean;

  /** Blinking state */
  blink?: boolean;

  /** Cell emphasis level (0-3) */
  emphasis?: number;

  /** Custom metadata */
  metadata?: Record<string, any>;
}
```

#### 2.2.3 BrailleLine

```typescript
interface BrailleLine {
  /** Array of Braille cells */
  cells: BrailleCell[];

  /** Line number (0-indexed) */
  lineNumber: number;

  /** Total cells in this line */
  length: number;

  /** Line-level attributes */
  attributes?: LineAttributes;
}
```

#### 2.2.4 LineAttributes

```typescript
interface LineAttributes {
  /** Scroll position offset */
  scrollOffset?: number;

  /** Line wrapping enabled */
  wrap?: boolean;

  /** Line type (normal, status, input) */
  type?: 'normal' | 'status' | 'input' | 'menu';

  /** Language/locale for this line */
  locale?: string;
}
```

### 2.3 Display Configuration

```typescript
interface DisplayConfiguration {
  /** Display dimensions */
  dimensions: {
    /** Number of cells horizontally */
    width: number;

    /** Number of lines vertically */
    height: number;
  };

  /** Cell format (6 or 8 dots) */
  cellFormat: 6 | 8;

  /** Total number of cells */
  totalCells: number;

  /** Cursor routing keys available */
  cursorRouting?: boolean;

  /** Number of status cells */
  statusCells?: number;
}
```

---

## 3. JSON Schema Definitions

### 3.1 Complete Display State Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/braille-display/v1.0/display-state.json",
  "title": "WIA Braille Display State",
  "description": "Complete state representation of a Braille display",
  "type": "object",
  "required": ["version", "timestamp", "display"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version (semver)"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "display": {
      "$ref": "#/definitions/Display"
    }
  },
  "definitions": {
    "Display": {
      "type": "object",
      "required": ["config", "lines"],
      "properties": {
        "config": {
          "$ref": "#/definitions/DisplayConfiguration"
        },
        "lines": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/BrailleLine"
          },
          "minItems": 1
        },
        "cursor": {
          "$ref": "#/definitions/CursorPosition"
        },
        "status": {
          "$ref": "#/definitions/StatusLine"
        }
      }
    },
    "DisplayConfiguration": {
      "type": "object",
      "required": ["width", "height", "cellFormat"],
      "properties": {
        "width": {
          "type": "integer",
          "minimum": 1,
          "maximum": 160
        },
        "height": {
          "type": "integer",
          "minimum": 1,
          "maximum": 4
        },
        "cellFormat": {
          "type": "integer",
          "enum": [6, 8]
        },
        "totalCells": {
          "type": "integer",
          "minimum": 1
        },
        "cursorRouting": {
          "type": "boolean"
        },
        "statusCells": {
          "type": "integer",
          "minimum": 0,
          "maximum": 20
        }
      }
    },
    "BrailleLine": {
      "type": "object",
      "required": ["lineNumber", "cells"],
      "properties": {
        "lineNumber": {
          "type": "integer",
          "minimum": 0
        },
        "cells": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/BrailleCell"
          }
        },
        "length": {
          "type": "integer"
        },
        "attributes": {
          "$ref": "#/definitions/LineAttributes"
        }
      }
    },
    "BrailleCell": {
      "type": "object",
      "required": ["dots", "format"],
      "properties": {
        "dots": {
          "type": "integer",
          "minimum": 0,
          "maximum": 255
        },
        "format": {
          "type": "integer",
          "enum": [6, 8]
        },
        "unicode": {
          "type": "string",
          "pattern": "^[\u2800-\u28FF]$"
        },
        "char": {
          "type": "string",
          "maxLength": 4
        },
        "attributes": {
          "$ref": "#/definitions/CellAttributes"
        }
      }
    },
    "CellAttributes": {
      "type": "object",
      "properties": {
        "cursor": {
          "type": "boolean"
        },
        "selected": {
          "type": "boolean"
        },
        "blink": {
          "type": "boolean"
        },
        "emphasis": {
          "type": "integer",
          "minimum": 0,
          "maximum": 3
        },
        "metadata": {
          "type": "object"
        }
      }
    },
    "LineAttributes": {
      "type": "object",
      "properties": {
        "scrollOffset": {
          "type": "integer"
        },
        "wrap": {
          "type": "boolean"
        },
        "type": {
          "type": "string",
          "enum": ["normal", "status", "input", "menu"]
        },
        "locale": {
          "type": "string",
          "pattern": "^[a-z]{2}(-[A-Z]{2})?$"
        }
      }
    },
    "CursorPosition": {
      "type": "object",
      "required": ["line", "cell"],
      "properties": {
        "line": {
          "type": "integer",
          "minimum": 0
        },
        "cell": {
          "type": "integer",
          "minimum": 0
        },
        "visible": {
          "type": "boolean"
        },
        "shape": {
          "type": "string",
          "enum": ["block", "underline", "vertical"]
        }
      }
    },
    "StatusLine": {
      "type": "object",
      "properties": {
        "cells": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/BrailleCell"
          }
        },
        "position": {
          "type": "string",
          "enum": ["top", "bottom", "left", "right"]
        }
      }
    }
  }
}
```

### 3.2 Cell Update Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/braille-display/v1.0/cell-update.json",
  "title": "Braille Cell Update",
  "description": "Partial update for specific cells",
  "type": "object",
  "required": ["updates"],
  "properties": {
    "updates": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["position", "cell"],
        "properties": {
          "position": {
            "type": "object",
            "required": ["line", "cell"],
            "properties": {
              "line": {
                "type": "integer",
                "minimum": 0
              },
              "cell": {
                "type": "integer",
                "minimum": 0
              }
            }
          },
          "cell": {
            "$ref": "display-state.json#/definitions/BrailleCell"
          }
        }
      }
    },
    "partial": {
      "type": "boolean",
      "default": true
    }
  }
}
```

---

## 4. Unicode Braille Mapping

### 4.1 Unicode Braille Patterns Block

The Unicode Braille Patterns block (U+2800 to U+28FF) provides 256 characters representing all possible 8-dot Braille patterns.

#### 4.1.1 Encoding Formula

```
Unicode Code Point = 0x2800 + Dot Pattern Value
```

Where the dot pattern value is calculated as:

```
Value = (dot1 × 1) + (dot2 × 2) + (dot3 × 4) + (dot4 × 8) +
        (dot5 × 16) + (dot6 × 32) + (dot7 × 64) + (dot8 × 128)
```

Each dot is either 0 (not raised) or 1 (raised).

#### 4.1.2 Example Mappings

| Character | Unicode | Hex    | Decimal | Binary    | Dots   | Description |
|-----------|---------|--------|---------|-----------|--------|-------------|
| ⠀        | U+2800  | 0x2800 | 10240   | 00000000  | (none) | Empty cell  |
| ⠁        | U+2801  | 0x2801 | 10241   | 00000001  | 1      | Dot 1 only  |
| ⠃        | U+2803  | 0x2803 | 10243   | 00000011  | 1,2    | Dots 1-2    |
| ⠇        | U+2807  | 0x2807 | 10247   | 00000111  | 1,2,3  | Dots 1-2-3  |
| ⠏        | U+280F  | 0x280F | 10255   | 00001111  | 1,2,3,4| Dots 1-2-3-4|
| ⠿        | U+283F  | 0x283F | 10303   | 00111111  | 1-6    | 6-dot full  |
| ⣿        | U+28FF  | 0x28FF | 10495   | 11111111  | 1-8    | 8-dot full  |

### 4.2 Conversion Functions

#### 4.2.1 Dots to Unicode

```typescript
function dotsToUnicode(dots: number): string {
  if (dots < 0 || dots > 255) {
    throw new Error(`Invalid dot pattern: ${dots}`);
  }
  return String.fromCharCode(0x2800 + dots);
}

// Examples
dotsToUnicode(0);    // ⠀ (empty)
dotsToUnicode(1);    // ⠁ (dot 1)
dotsToUnicode(255);  // ⣿ (all dots)
```

#### 4.2.2 Unicode to Dots

```typescript
function unicodeToDots(char: string): number {
  const codePoint = char.charCodeAt(0);
  if (codePoint < 0x2800 || codePoint > 0x28FF) {
    throw new Error(`Not a Braille character: ${char}`);
  }
  return codePoint - 0x2800;
}

// Examples
unicodeToDots('⠀');  // 0
unicodeToDots('⠁');  // 1
unicodeToDots('⣿');  // 255
```

#### 4.2.3 Individual Dots to Pattern

```typescript
function dotsToBinary(dots: number[]): number {
  let value = 0;
  for (const dot of dots) {
    if (dot < 1 || dot > 8) {
      throw new Error(`Invalid dot number: ${dot}`);
    }
    value |= (1 << (dot - 1));
  }
  return value;
}

// Examples
dotsToBinary([1]);        // 1   (0b00000001)
dotsToBinary([1, 2]);     // 3   (0b00000011)
dotsToBinary([1, 2, 3, 4, 5, 6]); // 63 (0b00111111)
```

#### 4.2.4 Pattern to Individual Dots

```typescript
function binaryToDots(value: number): number[] {
  const dots: number[] = [];
  for (let i = 0; i < 8; i++) {
    if (value & (1 << i)) {
      dots.push(i + 1);
    }
  }
  return dots;
}

// Examples
binaryToDots(1);   // [1]
binaryToDots(3);   // [1, 2]
binaryToDots(63);  // [1, 2, 3, 4, 5, 6]
binaryToDots(255); // [1, 2, 3, 4, 5, 6, 7, 8]
```

### 4.3 Common Braille Characters

#### 4.3.1 Letters (Grade 1 English Braille)

| Letter | Unicode | Dots    | Hex    |
|--------|---------|---------|--------|
| A      | ⠁      | 1       | 0x2801 |
| B      | ⠃      | 1,2     | 0x2803 |
| C      | ⠉      | 1,4     | 0x2809 |
| D      | ⠙      | 1,4,5   | 0x2819 |
| E      | ⠑      | 1,5     | 0x2811 |
| F      | ⠋      | 1,2,4   | 0x280B |
| G      | ⠛      | 1,2,4,5 | 0x281B |
| H      | ⠓      | 1,2,5   | 0x2813 |
| I      | ⠊      | 2,4     | 0x280A |
| J      | ⠚      | 2,4,5   | 0x281A |
| K      | ⠅      | 1,3     | 0x2805 |
| L      | ⠇      | 1,2,3   | 0x2807 |
| M      | ⠍      | 1,3,4   | 0x280D |
| N      | ⠝      | 1,3,4,5 | 0x281D |
| O      | ⠕      | 1,3,5   | 0x2815 |
| P      | ⠏      | 1,2,3,4 | 0x280F |
| Q      | ⠟      | 1,2,3,4,5 | 0x281F |
| R      | ⠗      | 1,2,3,5 | 0x2817 |
| S      | ⠎      | 2,3,4   | 0x280E |
| T      | ⠞      | 2,3,4,5 | 0x281E |
| U      | ⠥      | 1,3,6   | 0x2825 |
| V      | ⠧      | 1,2,3,6 | 0x2827 |
| W      | ⠺      | 2,4,5,6 | 0x283A |
| X      | ⠭      | 1,3,4,6 | 0x282D |
| Y      | ⠽      | 1,3,4,5,6 | 0x283D |
| Z      | ⠵      | 1,3,5,6 | 0x2835 |

#### 4.3.2 Numbers (with number sign)

```
Number Sign: ⠼ (dots 3,4,5,6)

⠼⠁ = 1
⠼⠃ = 2
⠼⠉ = 3
⠼⠙ = 4
⠼⠑ = 5
⠼⠋ = 6
⠼⠛ = 7
⠼⠓ = 8
⠼⠊ = 9
⠼⠚ = 0
```

#### 4.3.3 Punctuation

| Symbol | Unicode | Dots      | Description        |
|--------|---------|-----------|-------------------|
| ,      | ⠂      | 2         | Comma             |
| ;      | ⠆      | 2,3       | Semicolon         |
| :      | ⠒      | 2,5       | Colon             |
| .      | ⠲      | 2,5,6     | Period            |
| !      | ⠖      | 2,3,5     | Exclamation       |
| ?      | ⠦      | 2,3,6     | Question mark     |
| "      | ⠦⠴    | 2,3,6 + 3,5,6 | Quotes        |
| '      | ⠄      | 3         | Apostrophe        |
| -      | ⠤      | 3,6       | Hyphen            |
| (      | ⠐⠣    | 5 + 1,2,6 | Left parenthesis  |
| )      | ⠐⠜    | 5 + 3,4,5 | Right parenthesis |

---

## 5. 6-Dot and 8-Dot Formats

### 5.1 6-Dot Braille

Traditional Braille uses 6 dots, providing 2^6 = 64 possible combinations.

```
Dot Layout:
   1  4
   2  5
   3  6
```

**Range:** 0-63 (0x00-0x3F)
**Unicode:** U+2800 to U+283F
**Use Cases:**
- Literary Braille
- Most refreshable Braille displays
- Standard education materials

#### 5.1.1 6-Dot Cell Structure

```typescript
interface BrailleCell6Dot {
  dots: number;  // 0-63
  format: 6;
  unicode: string; // U+2800 to U+283F
}
```

### 5.2 8-Dot Braille

Computer Braille uses 8 dots for full ASCII/Unicode representation.

```
Dot Layout:
   1  4
   2  5
   3  6
   7  8
```

**Range:** 0-255 (0x00-0xFF)
**Unicode:** U+2800 to U+28FF
**Use Cases:**
- Computer screen readers
- Programming and technical content
- Extended character sets

#### 5.2.1 8-Dot Cell Structure

```typescript
interface BrailleCell8Dot {
  dots: number;  // 0-255
  format: 8;
  unicode: string; // U+2800 to U+28FF
}
```

### 5.3 Format Conversion

#### 5.3.1 6-Dot to 8-Dot

```typescript
function convert6to8(cell6: BrailleCell6Dot): BrailleCell8Dot {
  return {
    dots: cell6.dots, // Same value, expanded range
    format: 8,
    unicode: String.fromCharCode(0x2800 + cell6.dots),
  };
}
```

#### 5.3.2 8-Dot to 6-Dot (Lossy)

```typescript
function convert8to6(cell8: BrailleCell8Dot): BrailleCell6Dot {
  // Mask off dots 7 and 8
  const dots6 = cell8.dots & 0x3F;

  return {
    dots: dots6,
    format: 6,
    unicode: String.fromCharCode(0x2800 + dots6),
  };
}
```

### 5.4 Format Detection

```typescript
function detectFormat(dots: number): 6 | 8 {
  return (dots > 63) ? 8 : 6;
}

function needsEightDot(text: string): boolean {
  // Check if text contains characters requiring 8-dot
  for (const char of text) {
    const code = char.charCodeAt(0);

    // Extended ASCII, Unicode, control characters
    if (code > 127 || code < 32) {
      return true;
    }
  }
  return false;
}
```

---

## 6. Multi-Language Support

### 6.1 Supported Braille Codes

| Language/Region | Braille Code | Dots | Standard |
|----------------|-------------|------|----------|
| English (US)    | Unified English Braille (UEB) | 6/8 | BANA |
| English (UK)    | Unified English Braille (UEB) | 6/8 | UKAAF |
| Spanish         | Spanish Braille | 6 | ONCE |
| French          | French Braille | 6 | AVH |
| German          | German Braille | 6 | SBS |
| Italian         | Italian Braille | 6 | UIC |
| Portuguese      | Portuguese Braille | 6 | CAP |
| Russian         | Russian Braille | 6 | GOST |
| Arabic          | Arabic Braille | 6 | ABAF |
| Chinese         | Mainland Chinese Braille | 6 | CBRS |
| Japanese        | Japanese Braille | 6 | JBA |
| Korean          | Korean Braille | 6 | KBS |
| Hindi           | Bharati Braille | 6 | NBA |

### 6.2 Language-Specific Metadata

```typescript
interface LanguageMetadata {
  /** BCP 47 language tag */
  locale: string;

  /** Braille code/standard name */
  brailleCode: string;

  /** Reading direction */
  direction: 'ltr' | 'rtl';

  /** Grade (1=uncontracted, 2=contracted) */
  grade?: 1 | 2;

  /** Translation table identifier */
  translationTable?: string;
}
```

### 6.3 Translation Table References

```typescript
const TRANSLATION_TABLES = {
  'en-US': {
    grade1: 'en-us-g1.ctb',
    grade2: 'en-us-g2.ctb',
    computer: 'en-us-comp8.ctb',
  },
  'es': {
    grade1: 'es.ctb',
    grade2: 'es-g2.ctb',
  },
  'fr-FR': {
    grade1: 'fr-bfu-g1.ctb',
    grade2: 'fr-bfu-g2.ctb',
    computer: 'fr-bfu-comp8.ctb',
  },
  'de': {
    grade1: 'de-g1.ctb',
    grade2: 'de-g2.ctb',
    computer: 'de-comp8.ctb',
  },
  'zh-CN': {
    grade1: 'zh-cn.ctb',
    computer: 'zh-cn-comp8.ctb',
  },
  'ja': {
    grade1: 'ja.ctb',
    computer: 'ja-kantenji.ctb',
  },
  'ko': {
    grade1: 'ko.ctb',
    grade2: 'ko-g2.ctb',
  },
  'ar': {
    grade1: 'ar.ctb',
    grade2: 'ar-g2.ctb',
  },
  'ru': {
    grade1: 'ru.ctb',
    computer: 'ru-comp8.ctb',
  },
};
```

### 6.4 Multi-Language Example Payload

```json
{
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "display": {
    "config": {
      "width": 40,
      "height": 1,
      "cellFormat": 8
    },
    "lines": [
      {
        "lineNumber": 0,
        "cells": [
          {
            "dots": 33,
            "format": 6,
            "unicode": "⠡",
            "char": "H",
            "attributes": {
              "metadata": {
                "locale": "en-US",
                "brailleCode": "UEB",
                "grade": 1
              }
            }
          }
        ],
        "attributes": {
          "locale": "en-US",
          "type": "normal"
        }
      }
    ]
  }
}
```

---

## 7. Validation Rules

### 7.1 Cell-Level Validation

```typescript
class BrailleCellValidator {
  validate(cell: BrailleCell): ValidationResult {
    const errors: string[] = [];

    // Rule 1: Dots value range
    if (cell.format === 6 && cell.dots > 63) {
      errors.push(`6-dot cell cannot have dots value > 63 (got ${cell.dots})`);
    }
    if (cell.format === 8 && cell.dots > 255) {
      errors.push(`8-dot cell cannot have dots value > 255 (got ${cell.dots})`);
    }
    if (cell.dots < 0) {
      errors.push(`Dots value cannot be negative (got ${cell.dots})`);
    }

    // Rule 2: Unicode consistency
    if (cell.unicode) {
      const expectedUnicode = String.fromCharCode(0x2800 + cell.dots);
      if (cell.unicode !== expectedUnicode) {
        errors.push(
          `Unicode mismatch: expected ${expectedUnicode}, got ${cell.unicode}`
        );
      }
    }

    // Rule 3: Format consistency
    if (cell.format !== 6 && cell.format !== 8) {
      errors.push(`Invalid format: must be 6 or 8 (got ${cell.format})`);
    }

    // Rule 4: Attributes validation
    if (cell.attributes?.emphasis !== undefined) {
      if (cell.attributes.emphasis < 0 || cell.attributes.emphasis > 3) {
        errors.push(`Emphasis must be 0-3 (got ${cell.attributes.emphasis})`);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }
}
```

### 7.2 Line-Level Validation

```typescript
class BrailleLineValidator {
  validate(line: BrailleLine, maxWidth: number): ValidationResult {
    const errors: string[] = [];

    // Rule 1: Line length
    if (line.cells.length > maxWidth) {
      errors.push(`Line exceeds max width: ${line.cells.length} > ${maxWidth}`);
    }

    // Rule 2: Line number
    if (line.lineNumber < 0) {
      errors.push(`Line number cannot be negative`);
    }

    // Rule 3: Length consistency
    if (line.length && line.length !== line.cells.length) {
      errors.push(
        `Length mismatch: declared ${line.length}, actual ${line.cells.length}`
      );
    }

    // Rule 4: Cell validation
    const cellValidator = new BrailleCellValidator();
    line.cells.forEach((cell, index) => {
      const result = cellValidator.validate(cell);
      if (!result.valid) {
        errors.push(`Cell ${index}: ${result.errors.join(', ')}`);
      }
    });

    // Rule 5: Locale validation
    if (line.attributes?.locale) {
      if (!/^[a-z]{2}(-[A-Z]{2})?$/.test(line.attributes.locale)) {
        errors.push(`Invalid locale format: ${line.attributes.locale}`);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }
}
```

### 7.3 Display-Level Validation

```typescript
class DisplayValidator {
  validate(state: DisplayState): ValidationResult {
    const errors: string[] = [];

    // Rule 1: Version format
    if (!/^\d+\.\d+\.\d+$/.test(state.version)) {
      errors.push(`Invalid version format: ${state.version}`);
    }

    // Rule 2: Configuration validation
    const config = state.display.config;
    if (config.width < 1 || config.width > 160) {
      errors.push(`Width must be 1-160 (got ${config.width})`);
    }
    if (config.height < 1 || config.height > 4) {
      errors.push(`Height must be 1-4 (got ${config.height})`);
    }

    // Rule 3: Line count matches height
    if (state.display.lines.length !== config.height) {
      errors.push(
        `Line count ${state.display.lines.length} doesn't match height ${config.height}`
      );
    }

    // Rule 4: Line validation
    const lineValidator = new BrailleLineValidator();
    state.display.lines.forEach((line) => {
      const result = lineValidator.validate(line, config.width);
      if (!result.valid) {
        errors.push(`Line ${line.lineNumber}: ${result.errors.join(', ')}`);
      }
    });

    // Rule 5: Cursor position validation
    if (state.display.cursor) {
      const cursor = state.display.cursor;
      if (cursor.line >= config.height || cursor.line < 0) {
        errors.push(`Cursor line ${cursor.line} out of range`);
      }
      if (cursor.cell >= config.width || cursor.cell < 0) {
        errors.push(`Cursor cell ${cursor.cell} out of range`);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
    };
  }
}

interface ValidationResult {
  valid: boolean;
  errors: string[];
}
```

---

## 8. Example Payloads

### 8.1 Simple Text Display

```json
{
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:00:00Z",
  "display": {
    "config": {
      "width": 40,
      "height": 1,
      "cellFormat": 6,
      "totalCells": 40,
      "cursorRouting": true
    },
    "lines": [
      {
        "lineNumber": 0,
        "cells": [
          {"dots": 33, "format": 6, "unicode": "⠡", "char": "H"},
          {"dots": 23, "format": 6, "unicode": "⠗", "char": "e"},
          {"dots": 15, "format": 6, "unicode": "⠇", "char": "l"},
          {"dots": 15, "format": 6, "unicode": "⠇", "char": "l"},
          {"dots": 135, "format": 6, "unicode": "⠕", "char": "o"}
        ],
        "length": 5,
        "attributes": {
          "type": "normal",
          "locale": "en-US"
        }
      }
    ],
    "cursor": {
      "line": 0,
      "cell": 0,
      "visible": true,
      "shape": "block"
    }
  }
}
```

### 8.2 Multi-Line Display with Status

```json
{
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:15:00Z",
  "display": {
    "config": {
      "width": 40,
      "height": 2,
      "cellFormat": 8,
      "totalCells": 80,
      "statusCells": 5
    },
    "lines": [
      {
        "lineNumber": 0,
        "cells": [
          {"dots": 33, "format": 8, "unicode": "⠡", "char": "H",
           "attributes": {"cursor": true}},
          {"dots": 23, "format": 8, "unicode": "⠗", "char": "e"},
          {"dots": 15, "format": 8, "unicode": "⠇", "char": "l"},
          {"dots": 15, "format": 8, "unicode": "⠇", "char": "l"},
          {"dots": 135, "format": 8, "unicode": "⠕", "char": "o"}
        ],
        "length": 5,
        "attributes": {"type": "normal"}
      },
      {
        "lineNumber": 1,
        "cells": [
          {"dots": 25, "format": 8, "unicode": "⠚", "char": "W"},
          {"dots": 135, "format": 8, "unicode": "⠕", "char": "o"},
          {"dots": 23, "format": 8, "unicode": "⠗", "char": "r"},
          {"dots": 15, "format": 8, "unicode": "⠇", "char": "l"},
          {"dots": 134, "format": 8, "unicode": "⠙", "char": "d"}
        ],
        "length": 5,
        "attributes": {"type": "normal"}
      }
    ],
    "status": {
      "cells": [
        {"dots": 123456, "format": 8, "unicode": "⣿", "char": "█"},
        {"dots": 0, "format": 8, "unicode": "⠀", "char": " "},
        {"dots": 1, "format": 8, "unicode": "⠁", "char": "1"},
        {"dots": 12, "format": 8, "unicode": "⠃", "char": "2"},
        {"dots": 0, "format": 8, "unicode": "⠀", "char": " "}
      ],
      "position": "right"
    }
  }
}
```

### 8.3 Partial Update

```json
{
  "updates": [
    {
      "position": {"line": 0, "cell": 5},
      "cell": {
        "dots": 2,
        "format": 6,
        "unicode": "⠂",
        "char": ",",
        "attributes": {"blink": true}
      }
    },
    {
      "position": {"line": 0, "cell": 6},
      "cell": {
        "dots": 0,
        "format": 6,
        "unicode": "⠀",
        "char": " "
      }
    }
  ],
  "partial": true
}
```

### 8.4 Complex Document with Selection

```json
{
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "display": {
    "config": {
      "width": 80,
      "height": 4,
      "cellFormat": 8,
      "totalCells": 320,
      "cursorRouting": true,
      "statusCells": 8
    },
    "lines": [
      {
        "lineNumber": 0,
        "cells": [
          {"dots": 60, "format": 8, "unicode": "⠼", "char": "#"},
          {"dots": 1, "format": 8, "unicode": "⠁", "char": "1"}
        ],
        "attributes": {
          "type": "normal",
          "locale": "en-US"
        }
      },
      {
        "lineNumber": 1,
        "cells": [
          {"dots": 33, "format": 8, "unicode": "⠡", "char": "T",
           "attributes": {"selected": true, "emphasis": 2}},
          {"dots": 23, "format": 8, "unicode": "⠗", "char": "e",
           "attributes": {"selected": true, "emphasis": 2}},
          {"dots": 1356, "format": 8, "unicode": "⠵", "char": "x",
           "attributes": {"selected": true, "emphasis": 2}},
          {"dots": 2345, "format": 8, "unicode": "⠞", "char": "t",
           "attributes": {"selected": true, "emphasis": 2}}
        ],
        "attributes": {"type": "normal"}
      },
      {
        "lineNumber": 2,
        "cells": [],
        "attributes": {"type": "normal"}
      },
      {
        "lineNumber": 3,
        "cells": [
          {"dots": 46, "format": 8, "unicode": "⠨", "char": ">"}
        ],
        "attributes": {"type": "input"}
      }
    ],
    "cursor": {
      "line": 3,
      "cell": 1,
      "visible": true,
      "shape": "underline"
    },
    "status": {
      "cells": [
        {"dots": 15, "format": 8, "unicode": "⠇", "char": "L"},
        {"dots": 1, "format": 8, "unicode": "⠁", "char": "1"},
        {"dots": 0, "format": 8, "unicode": "⠀", "char": " "},
        {"dots": 14, "format": 8, "unicode": "⠉", "char": "C"},
        {"dots": 1, "format": 8, "unicode": "⠁", "char": "1"}
      ],
      "position": "right"
    }
  }
}
```

---

## 9. Binary Encoding

For performance-critical applications, a compact binary format is provided.

### 9.1 Binary Format Specification

```
Header (16 bytes):
  - Magic: "WIABRL" (6 bytes)
  - Version: uint16 (2 bytes)
  - Width: uint8 (1 byte)
  - Height: uint8 (1 byte)
  - Cell Format: uint8 (1 byte, value: 6 or 8)
  - Flags: uint8 (1 byte)
  - Reserved: uint16 (2 bytes)
  - Timestamp: uint32 (4 bytes, Unix timestamp)

Cell Data (variable):
  For each line:
    For each cell:
      - Dots: uint8 (1 byte)
      - Attributes: uint8 (1 byte, bit flags)

Attribute Flags:
  Bit 0: Cursor
  Bit 1: Selected
  Bit 2: Blink
  Bit 3-4: Emphasis (0-3)
  Bit 5-7: Reserved
```

### 9.2 Binary Encoding Functions

```typescript
class BinaryEncoder {
  encode(state: DisplayState): Uint8Array {
    const config = state.display.config;
    const totalCells = config.width * config.height;
    const bufferSize = 16 + (totalCells * 2); // Header + cells

    const buffer = new Uint8Array(bufferSize);
    const view = new DataView(buffer.buffer);

    // Header
    buffer.set([0x57, 0x49, 0x41, 0x42, 0x52, 0x4C], 0); // "WIABRL"
    view.setUint16(6, 0x0100, false); // Version 1.0
    view.setUint8(8, config.width);
    view.setUint8(9, config.height);
    view.setUint8(10, config.cellFormat);
    view.setUint8(11, 0); // Flags
    view.setUint16(12, 0, false); // Reserved
    view.setUint32(14, Math.floor(Date.now() / 1000), false);

    // Cell data
    let offset = 16;
    for (const line of state.display.lines) {
      for (const cell of line.cells) {
        view.setUint8(offset++, cell.dots);
        view.setUint8(offset++, this.encodeAttributes(cell.attributes));
      }
    }

    return buffer;
  }

  private encodeAttributes(attrs?: CellAttributes): number {
    if (!attrs) return 0;

    let flags = 0;
    if (attrs.cursor) flags |= 0x01;
    if (attrs.selected) flags |= 0x02;
    if (attrs.blink) flags |= 0x04;
    if (attrs.emphasis) flags |= (attrs.emphasis & 0x03) << 3;

    return flags;
  }

  decode(buffer: Uint8Array): DisplayState {
    const view = new DataView(buffer.buffer);

    // Validate magic
    const magic = String.fromCharCode(...buffer.slice(0, 6));
    if (magic !== 'WIABRL') {
      throw new Error('Invalid magic number');
    }

    // Read header
    const version = view.getUint16(6, false);
    const width = view.getUint8(8);
    const height = view.getUint8(9);
    const cellFormat = view.getUint8(10) as 6 | 8;
    const timestamp = new Date(view.getUint32(14, false) * 1000);

    // Read cells
    const lines: BrailleLine[] = [];
    let offset = 16;

    for (let lineNum = 0; lineNum < height; lineNum++) {
      const cells: BrailleCell[] = [];

      for (let cellNum = 0; cellNum < width; cellNum++) {
        const dots = view.getUint8(offset++);
        const attrFlags = view.getUint8(offset++);

        cells.push({
          dots,
          format: cellFormat,
          unicode: String.fromCharCode(0x2800 + dots),
          attributes: this.decodeAttributes(attrFlags),
        });
      }

      lines.push({
        lineNumber: lineNum,
        cells,
        length: cells.length,
      });
    }

    return {
      version: `${version >> 8}.${version & 0xFF}.0`,
      timestamp: timestamp.toISOString(),
      display: {
        config: {
          width,
          height,
          cellFormat,
          totalCells: width * height,
        },
        lines,
      },
    };
  }

  private decodeAttributes(flags: number): CellAttributes {
    const attrs: CellAttributes = {};

    if (flags & 0x01) attrs.cursor = true;
    if (flags & 0x02) attrs.selected = true;
    if (flags & 0x04) attrs.blink = true;

    const emphasis = (flags >> 3) & 0x03;
    if (emphasis > 0) attrs.emphasis = emphasis;

    return attrs;
  }
}
```

---

## 10. Compression and Optimization

### 10.1 Run-Length Encoding (RLE)

For repeated cells (common in status bars and padding):

```typescript
interface RLESegment {
  cell: BrailleCell;
  count: number;
}

function compressRLE(cells: BrailleCell[]): RLESegment[] {
  const segments: RLESegment[] = [];
  let current: RLESegment | null = null;

  for (const cell of cells) {
    if (!current || !cellsEqual(current.cell, cell)) {
      if (current) segments.push(current);
      current = { cell, count: 1 };
    } else {
      current.count++;
    }
  }

  if (current) segments.push(current);
  return segments;
}

function decompressRLE(segments: RLESegment[]): BrailleCell[] {
  const cells: BrailleCell[] = [];

  for (const segment of segments) {
    for (let i = 0; i < segment.count; i++) {
      cells.push({ ...segment.cell });
    }
  }

  return cells;
}

function cellsEqual(a: BrailleCell, b: BrailleCell): boolean {
  return a.dots === b.dots &&
         a.format === b.format &&
         JSON.stringify(a.attributes) === JSON.stringify(b.attributes);
}
```

### 10.2 Delta Encoding

Send only changed cells:

```typescript
interface CellDelta {
  position: { line: number; cell: number };
  oldCell?: BrailleCell;
  newCell: BrailleCell;
}

function computeDelta(
  oldState: DisplayState,
  newState: DisplayState
): CellDelta[] {
  const deltas: CellDelta[] = [];

  for (let lineNum = 0; lineNum < newState.display.lines.length; lineNum++) {
    const oldLine = oldState.display.lines[lineNum];
    const newLine = newState.display.lines[lineNum];

    for (let cellNum = 0; cellNum < newLine.cells.length; cellNum++) {
      const oldCell = oldLine?.cells[cellNum];
      const newCell = newLine.cells[cellNum];

      if (!oldCell || !cellsEqual(oldCell, newCell)) {
        deltas.push({
          position: { line: lineNum, cell: cellNum },
          oldCell,
          newCell,
        });
      }
    }
  }

  return deltas;
}

function applyDelta(state: DisplayState, deltas: CellDelta[]): DisplayState {
  const newState = JSON.parse(JSON.stringify(state)); // Deep clone

  for (const delta of deltas) {
    const { line, cell } = delta.position;
    newState.display.lines[line].cells[cell] = delta.newCell;
  }

  return newState;
}
```

### 10.3 Compression Statistics

```typescript
interface CompressionStats {
  original: number;
  compressed: number;
  ratio: number;
  method: string;
}

function analyzeCompression(cells: BrailleCell[]): CompressionStats {
  const originalSize = cells.length * 2; // 2 bytes per cell (dots + attrs)
  const rleSegments = compressRLE(cells);
  const compressedSize = rleSegments.length * 3; // 3 bytes per segment

  return {
    original: originalSize,
    compressed: compressedSize,
    ratio: compressedSize / originalSize,
    method: 'RLE',
  };
}
```

---

## Appendix A: Complete TypeScript Type Definitions

```typescript
// Core types
export type CellFormat = 6 | 8;
export type LineType = 'normal' | 'status' | 'input' | 'menu';
export type CursorShape = 'block' | 'underline' | 'vertical';
export type StatusPosition = 'top' | 'bottom' | 'left' | 'right';

export interface BrailleCell {
  dots: number;
  format: CellFormat;
  unicode?: string;
  char?: string;
  attributes?: CellAttributes;
}

export interface CellAttributes {
  cursor?: boolean;
  selected?: boolean;
  blink?: boolean;
  emphasis?: number;
  metadata?: Record<string, any>;
}

export interface BrailleLine {
  lineNumber: number;
  cells: BrailleCell[];
  length: number;
  attributes?: LineAttributes;
}

export interface LineAttributes {
  scrollOffset?: number;
  wrap?: boolean;
  type?: LineType;
  locale?: string;
}

export interface DisplayConfiguration {
  dimensions: {
    width: number;
    height: number;
  };
  cellFormat: CellFormat;
  totalCells: number;
  cursorRouting?: boolean;
  statusCells?: number;
}

export interface CursorPosition {
  line: number;
  cell: number;
  visible: boolean;
  shape?: CursorShape;
}

export interface StatusLine {
  cells: BrailleCell[];
  position?: StatusPosition;
}

export interface Display {
  config: DisplayConfiguration;
  lines: BrailleLine[];
  cursor?: CursorPosition;
  status?: StatusLine;
}

export interface DisplayState {
  version: string;
  timestamp: string;
  display: Display;
}

export interface LanguageMetadata {
  locale: string;
  brailleCode: string;
  direction: 'ltr' | 'rtl';
  grade?: 1 | 2;
  translationTable?: string;
}

export interface ValidationResult {
  valid: boolean;
  errors: string[];
}
```

---

**Document End**

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

For questions or contributions, please visit:
https://github.com/WIA-Official/wia-standards
