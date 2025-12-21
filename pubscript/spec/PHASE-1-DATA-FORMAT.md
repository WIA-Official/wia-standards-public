# WIA PubScript Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Primary Color**: #8B5CF6 (Violet)

---

## Overview

### 1.1 Purpose

WIA PubScript Data Format defines a comprehensive markup language and structured format for creating accessible, multi-format publications. This standard enables content creators to author once and publish everywhere—from web to print, from EPUB to Braille, from PDF to audio—while maintaining full accessibility compliance and semantic richness.

### 1.2 Scope

- **In Scope**:
  - Document structure and semantic markup
  - Accessibility metadata (WCAG 2.1 AA)
  - Multi-format output specifications
  - Metadata standards (Dublin Core, ONIX)
  - Version control schema
  - Collaborative editing data structures
  - Translation workflow formats

- **Out of Scope** (Phase 1):
  - Real-time collaboration protocol (Phase 3)
  - Publishing platform APIs (Phase 2)
  - Third-party integration specifics (Phase 4)

### 1.3 Design Principles

1. **Accessibility First**: WCAG 2.1 AA compliance built into core structure
2. **Semantic Richness**: Meaningful markup beyond presentation
3. **Format Agnostic**: Single source, multiple outputs
4. **Extensibility**: Support for custom elements and metadata
5. **Human Readable**: JSON-based with clear structure
6. **Version Control Friendly**: Diff-friendly format design

---

## Document Structure

### 2.1 Top-Level Architecture

```
pubscript-document/
├── document.json         # Main document metadata
├── metadata.json         # Publishing metadata (Dublin Core, ONIX)
├── content/
│   ├── chapters/
│   │   ├── chapter-01.json
│   │   ├── chapter-02.json
│   │   └── ...
│   ├── sections/
│   │   └── ...
│   └── assets/
│       ├── images/
│       ├── tables/
│       └── multimedia/
├── accessibility/
│   ├── alt-text.json
│   ├── aria-labels.json
│   └── reading-order.json
├── translations/
│   ├── en-US/
│   ├── ko-KR/
│   └── ...
└── versions/
    └── version-manifest.json
```

### 2.2 File Naming Convention

```
{document-id}_{content-type}_{version}.json

Examples:
- doc-001_chapter_v1.0.0.json
- doc-001_metadata_v1.0.0.json
- doc-001_translations_ko-KR_v1.0.0.json
```

---

## Document Metadata

### 3.1 document.json

```json
{
  "$schema": "https://wia.live/schemas/pubscript/document.schema.json",
  "wia_version": "1.0.0",
  "format_version": "1.0.0",
  "document_id": "doc-20250115-001",

  "document_info": {
    "title": "Accessible Publishing in the Digital Age",
    "subtitle": "A Comprehensive Guide",
    "language": "en-US",
    "created": "2025-01-15T10:00:00.000Z",
    "modified": "2025-01-20T14:30:00.000Z",
    "version": "1.2.0",
    "status": "draft"
  },

  "authors": [
    {
      "id": "author-001",
      "name": "Dr. Sarah Chen",
      "role": "primary_author",
      "affiliation": "University of Washington",
      "orcid": "0000-0001-2345-6789",
      "bio": "Expert in accessible publishing technologies"
    },
    {
      "id": "author-002",
      "name": "James Wilson",
      "role": "contributing_author",
      "affiliation": "Accessibility Institute"
    }
  ],

  "structure": {
    "type": "book",
    "chapters": 12,
    "sections": 48,
    "word_count": 85000,
    "reading_time_minutes": 340,
    "outline": [
      {
        "id": "chapter-01",
        "title": "Introduction to Accessible Publishing",
        "sections": ["section-01-01", "section-01-02", "section-01-03"],
        "file": "content/chapters/chapter-01.json"
      },
      {
        "id": "chapter-02",
        "title": "Document Structure and Semantics",
        "sections": ["section-02-01", "section-02-02"],
        "file": "content/chapters/chapter-02.json"
      }
    ]
  },

  "output_formats": {
    "enabled": ["html", "pdf", "epub3", "braille", "audio"],
    "primary": "html",
    "print_ready": true,
    "digital_optimized": true
  },

  "accessibility": {
    "wcag_level": "AA",
    "wcag_version": "2.1",
    "features": [
      "alt_text",
      "aria_labels",
      "semantic_markup",
      "keyboard_navigation",
      "screen_reader_optimized",
      "high_contrast",
      "text_resize_support"
    ],
    "reading_order_defined": true,
    "language_tags_complete": true
  },

  "rights": {
    "copyright": "© 2025 Sarah Chen",
    "license": "CC BY-SA 4.0",
    "permissions": "open_access",
    "doi": "10.1234/pubscript.2025.001"
  }
}
```

---

## Publishing Metadata

### 4.1 metadata.json (Dublin Core + ONIX)

```json
{
  "$schema": "https://wia.live/schemas/pubscript/metadata.schema.json",

  "dublin_core": {
    "dc:title": "Accessible Publishing in the Digital Age",
    "dc:creator": ["Chen, Sarah", "Wilson, James"],
    "dc:subject": [
      "Accessible Publishing",
      "Digital Accessibility",
      "WCAG Compliance",
      "Multi-format Publishing"
    ],
    "dc:description": "A comprehensive guide to creating accessible publications that work across all formats and assistive technologies.",
    "dc:publisher": "WIA Press",
    "dc:contributor": ["Editor: Maria Rodriguez"],
    "dc:date": "2025-01-15",
    "dc:type": "Text.Book",
    "dc:format": "application/pubscript+json",
    "dc:identifier": "ISBN:978-1-234567-89-0",
    "dc:source": "https://pubscript.wia.org/docs/doc-001",
    "dc:language": "en-US",
    "dc:relation": "Part of WIA Accessibility Series",
    "dc:coverage": "Worldwide",
    "dc:rights": "CC BY-SA 4.0"
  },

  "onix": {
    "NotificationType": "03",
    "ProductIdentifier": [
      {
        "ProductIDType": "15",
        "IDValue": "978-1-234567-89-0"
      },
      {
        "ProductIDType": "06",
        "IDValue": "10.1234/pubscript.2025.001"
      }
    ],
    "DescriptiveDetail": {
      "ProductComposition": "00",
      "ProductForm": "ED",
      "Measure": [
        {
          "MeasureType": "08",
          "Measurement": "85000",
          "MeasureUnitCode": "02"
        }
      ],
      "Subject": [
        {
          "SubjectSchemeIdentifier": "10",
          "SubjectCode": "COM060000"
        }
      ],
      "Audience": [
        {
          "AudienceCodeType": "01",
          "AudienceCodeValue": "06"
        }
      ]
    },
    "PublishingDetail": {
      "Publisher": {
        "PublishingRole": "01",
        "PublisherName": "WIA Press"
      },
      "PublishingDate": [
        {
          "PublishingDateRole": "01",
          "Date": "20250115"
        }
      ]
    }
  }
}
```

---

## Content Structure

### 5.1 Chapter Format

```json
{
  "$schema": "https://wia.live/schemas/pubscript/chapter.schema.json",
  "chapter_id": "chapter-01",
  "chapter_number": 1,
  "title": "Introduction to Accessible Publishing",
  "language": "en-US",

  "sections": [
    {
      "section_id": "section-01-01",
      "type": "heading",
      "level": 2,
      "content": "What is Accessible Publishing?",
      "aria_label": "Section: What is Accessible Publishing?",
      "anchor": "what-is-accessible-publishing"
    },
    {
      "section_id": "section-01-02",
      "type": "paragraph",
      "content": "Accessible publishing ensures that digital and print content can be consumed by all readers, regardless of ability or assistive technology used.",
      "semantic_role": "introduction"
    },
    {
      "section_id": "section-01-03",
      "type": "list",
      "list_style": "unordered",
      "items": [
        {
          "id": "item-01",
          "content": "Screen reader compatibility",
          "aria_label": "First benefit: Screen reader compatibility"
        },
        {
          "id": "item-02",
          "content": "Keyboard navigation support",
          "aria_label": "Second benefit: Keyboard navigation support"
        },
        {
          "id": "item-03",
          "content": "Multiple format outputs",
          "aria_label": "Third benefit: Multiple format outputs"
        }
      ]
    },
    {
      "section_id": "section-01-04",
      "type": "figure",
      "figure_id": "fig-01-01",
      "caption": "The accessible publishing workflow",
      "alt_text": "Flowchart showing content creation flowing through PubScript conversion to multiple output formats including HTML, PDF, EPUB, Braille, and Audio",
      "long_description_id": "longdesc-01-01",
      "image": {
        "src": "content/assets/images/workflow-diagram.svg",
        "format": "svg",
        "width": 800,
        "height": 600
      },
      "accessibility": {
        "aria_describedby": "longdesc-01-01",
        "role": "img",
        "tactile_graphic_available": true
      }
    },
    {
      "section_id": "section-01-05",
      "type": "table",
      "table_id": "table-01-01",
      "caption": "Comparison of output formats",
      "summary": "Table comparing features of HTML, PDF, EPUB, Braille, and Audio formats",
      "headers": {
        "columns": ["Format", "Accessibility", "Interactivity", "Print Ready"],
        "rows": false
      },
      "data": [
        ["HTML", "High", "High", "No"],
        ["PDF", "Medium", "Low", "Yes"],
        ["EPUB3", "High", "Medium", "No"],
        ["Braille", "High", "Low", "Yes"],
        ["Audio", "High", "Low", "No"]
      ],
      "accessibility": {
        "scope_defined": true,
        "headers_associated": true,
        "navigation_enabled": true
      }
    }
  ],

  "footnotes": [
    {
      "id": "footnote-01",
      "number": 1,
      "content": "According to WHO, over 1 billion people worldwide have some form of disability.",
      "backlink": "section-01-02"
    }
  ],

  "reading_order": [
    "section-01-01",
    "section-01-02",
    "section-01-03",
    "section-01-04",
    "section-01-05"
  ]
}
```

---

## Semantic Elements

### 6.1 Element Types

| Element Type | Code | Semantic Purpose | ARIA Role |
|-------------|------|------------------|-----------|
| Heading | `heading` | Document structure | `heading` |
| Paragraph | `paragraph` | Body text | `paragraph` |
| List | `list` | Enumerated items | `list` |
| Figure | `figure` | Visual content | `figure` |
| Table | `table` | Tabular data | `table` |
| Code Block | `code` | Programming code | `code` |
| Quote | `quote` | Quotations | `blockquote` |
| Note | `note` | Aside information | `note` |
| Warning | `warning` | Important notice | `alert` |
| Definition | `definition` | Term definition | `definition` |

### 6.2 Heading Hierarchy

```json
{
  "headings": [
    {
      "level": 1,
      "content": "Document Title",
      "role": "title",
      "aria_level": 1
    },
    {
      "level": 2,
      "content": "Chapter Title",
      "role": "chapter",
      "aria_level": 2
    },
    {
      "level": 3,
      "content": "Section Title",
      "role": "section",
      "aria_level": 3
    },
    {
      "level": 4,
      "content": "Subsection Title",
      "role": "subsection",
      "aria_level": 4
    }
  ],
  "validation": {
    "no_skipped_levels": true,
    "unique_h1": true,
    "logical_hierarchy": true
  }
}
```

---

## Accessibility Features

### 7.1 alt-text.json

```json
{
  "$schema": "https://wia.live/schemas/pubscript/alt-text.schema.json",

  "images": [
    {
      "image_id": "fig-01-01",
      "alt_text_short": "Accessible publishing workflow diagram",
      "alt_text_long": "Flowchart showing the accessible publishing process. Content starts at the top, flows into PubScript converter in the center, which then branches into five output formats: HTML for web, PDF for print, EPUB3 for e-readers, Braille for tactile reading, and Audio for listening. Each format is represented by an icon and connects to end users at the bottom.",
      "description_file": "content/assets/descriptions/fig-01-01.txt",
      "tactile_graphic": {
        "available": true,
        "file": "content/assets/tactile/fig-01-01.brf",
        "description": "Raised line diagram on swell paper showing workflow connections"
      }
    }
  ],

  "decorative_images": [
    {
      "image_id": "decorative-divider-01",
      "alt_text": "",
      "aria_hidden": true,
      "role": "presentation"
    }
  ],

  "complex_images": [
    {
      "image_id": "chart-01-01",
      "alt_text_short": "Bar chart showing accessibility compliance rates",
      "long_description": "content/assets/descriptions/chart-01-01-longdesc.html",
      "data_table_alternative": "content/assets/tables/chart-01-01-data.json",
      "sonification": {
        "available": true,
        "file": "content/assets/audio/chart-01-01-sonification.mp3"
      }
    }
  ]
}
```

### 7.2 aria-labels.json

```json
{
  "$schema": "https://wia.live/schemas/pubscript/aria.schema.json",

  "landmarks": [
    {
      "id": "main-content",
      "role": "main",
      "label": "Main content"
    },
    {
      "id": "navigation",
      "role": "navigation",
      "label": "Table of contents"
    },
    {
      "id": "search",
      "role": "search",
      "label": "Search this document"
    }
  ],

  "interactive_elements": [
    {
      "id": "footnote-link-01",
      "role": "doc-noteref",
      "label": "Footnote 1",
      "describedby": "footnote-01"
    },
    {
      "id": "expand-section-01",
      "role": "button",
      "label": "Expand section: Additional resources",
      "aria_expanded": false,
      "aria_controls": "additional-resources-content"
    }
  ],

  "custom_roles": [
    {
      "id": "author-bio",
      "role": "doc-credit",
      "label": "About the author"
    }
  ]
}
```

### 7.3 reading-order.json

```json
{
  "$schema": "https://wia.live/schemas/pubscript/reading-order.schema.json",

  "document_reading_order": [
    {
      "sequence": 1,
      "element_id": "document-title",
      "element_type": "heading",
      "level": 1
    },
    {
      "sequence": 2,
      "element_id": "author-info",
      "element_type": "metadata"
    },
    {
      "sequence": 3,
      "element_id": "table-of-contents",
      "element_type": "navigation"
    },
    {
      "sequence": 4,
      "element_id": "chapter-01",
      "element_type": "chapter",
      "subsections": [
        "section-01-01",
        "section-01-02",
        "section-01-03"
      ]
    }
  ],

  "alternative_reading_orders": [
    {
      "id": "simplified-order",
      "description": "Simplified reading order without sidebars",
      "exclude_types": ["aside", "sidebar"]
    },
    {
      "id": "reference-only",
      "description": "Reference materials only",
      "include_types": ["table", "figure", "bibliography"]
    }
  ]
}
```

---

## Multi-Format Output Specifications

### 8.1 HTML Output Configuration

```json
{
  "html_output": {
    "version": "HTML5",
    "doctype": "<!DOCTYPE html>",
    "semantic_elements": true,
    "aria_roles": true,
    "lang_attributes": true,
    "structure": {
      "header": true,
      "main": true,
      "nav": true,
      "aside": true,
      "footer": true
    },
    "accessibility": {
      "skip_links": true,
      "landmark_roles": true,
      "heading_hierarchy": true,
      "form_labels": true,
      "alt_text": true
    },
    "responsive": true,
    "css_framework": "pubscript-accessible.css",
    "javascript": "pubscript-enhancements.js"
  }
}
```

### 8.2 PDF Output Configuration

```json
{
  "pdf_output": {
    "version": "PDF/UA-1",
    "standard": "ISO 14289-1",
    "tagged": true,
    "structure_tree": true,
    "accessibility": {
      "alternative_text": true,
      "logical_reading_order": true,
      "bookmarks": true,
      "metadata": true,
      "language_specification": true
    },
    "print": {
      "page_size": "A4",
      "margins": "2.5cm",
      "bleed": "3mm",
      "color_space": "CMYK",
      "resolution": 300
    },
    "fonts": {
      "embed": true,
      "subset": true,
      "fallbacks": ["Arial", "Helvetica", "sans-serif"]
    }
  }
}
```

### 8.3 EPUB3 Output Configuration

```json
{
  "epub3_output": {
    "version": "EPUB 3.3",
    "media_type": "application/epub+zip",
    "accessibility": {
      "wcag_conformance": "WCAG2.1-AA",
      "schema_org": true,
      "aria": true,
      "epub_accessibility_1_1": true
    },
    "features": {
      "reflowable": true,
      "fixed_layout_available": false,
      "media_overlays": true,
      "mathml": true,
      "svg": true,
      "javascript": false
    },
    "metadata": {
      "dc_elements": true,
      "accessibility_metadata": true,
      "schema_org_metadata": true
    },
    "navigation": {
      "toc_ncx": true,
      "toc_xhtml": true,
      "landmarks": true,
      "page_list": true
    }
  }
}
```

### 8.4 Braille Output Configuration

```json
{
  "braille_output": {
    "grade": 2,
    "translation_table": "en-us-g2.ctb",
    "formats": ["BRF", "PEF", "BRL"],
    "page_format": {
      "cells_per_line": 40,
      "lines_per_page": 25
    },
    "features": {
      "contracted_braille": true,
      "transcribers_notes": true,
      "page_numbers": true,
      "running_headers": false
    },
    "special_symbols": {
      "math": "Nemeth",
      "music": "Music Braille",
      "chemistry": "BANA Chemistry"
    }
  }
}
```

### 8.5 Audio Output Configuration

```json
{
  "audio_output": {
    "format": "DAISY 3.0",
    "alternative_formats": ["audiobook", "podcast"],
    "tts_engine": "neural_tts",
    "voice": {
      "language": "en-US",
      "gender": "neutral",
      "rate": "medium",
      "style": "narrative"
    },
    "features": {
      "navigation": true,
      "bookmarks": true,
      "search": true,
      "adjustable_speed": true,
      "synchronized_text": true
    },
    "quality": {
      "sample_rate": 44100,
      "bit_depth": 16,
      "channels": "stereo"
    }
  }
}
```

---

## Version Control

### 9.1 version-manifest.json

```json
{
  "$schema": "https://wia.live/schemas/pubscript/version.schema.json",

  "document_id": "doc-20250115-001",
  "current_version": "1.2.0",

  "versions": [
    {
      "version": "1.0.0",
      "date": "2025-01-15T10:00:00Z",
      "author": "author-001",
      "status": "published",
      "changes": "Initial release",
      "git_commit": "a1b2c3d4e5f6",
      "files_changed": ["document.json", "chapter-01.json"]
    },
    {
      "version": "1.1.0",
      "date": "2025-01-18T14:00:00Z",
      "author": "author-002",
      "status": "draft",
      "changes": "Added chapter 2, updated accessibility metadata",
      "git_commit": "f6e5d4c3b2a1",
      "files_changed": ["document.json", "chapter-02.json", "accessibility/alt-text.json"]
    },
    {
      "version": "1.2.0",
      "date": "2025-01-20T14:30:00Z",
      "author": "author-001",
      "status": "draft",
      "changes": "Editorial revisions to chapter 1",
      "git_commit": "1a2b3c4d5e6f",
      "files_changed": ["chapter-01.json"]
    }
  ],

  "branching": {
    "main_branch": "main",
    "current_branch": "feature/accessibility-improvements",
    "merge_requests": [
      {
        "id": "mr-001",
        "from": "feature/new-chapter",
        "to": "main",
        "status": "pending",
        "reviewer": "author-002"
      }
    ]
  }
}
```

---

## Translation Workflow

### 10.1 Translation Structure

```json
{
  "$schema": "https://wia.live/schemas/pubscript/translation.schema.json",

  "source_language": "en-US",
  "target_languages": ["ko-KR", "ja-JP", "es-ES", "fr-FR"],

  "translations": [
    {
      "language": "ko-KR",
      "status": "complete",
      "translator": "translator-ko-001",
      "reviewer": "reviewer-ko-001",
      "completion_date": "2025-01-25",
      "files": {
        "document": "translations/ko-KR/document.json",
        "chapters": "translations/ko-KR/chapters/",
        "metadata": "translations/ko-KR/metadata.json"
      }
    }
  ],

  "translation_memory": {
    "enabled": true,
    "tm_file": "translations/tm-database.tmx",
    "glossary": "translations/glossary.json"
  },

  "localization": {
    "date_format": "locale-specific",
    "number_format": "locale-specific",
    "currency": "locale-specific",
    "measurements": "locale-specific"
  }
}
```

---

## Validation Schema

### 11.1 Document Validation Rules

```yaml
required_fields:
  document.json:
    - wia_version
    - document_id
    - document_info.title
    - document_info.language
    - authors
    - structure

  metadata.json:
    - dublin_core.dc:title
    - dublin_core.dc:creator
    - dublin_core.dc:date

  chapter.json:
    - chapter_id
    - title
    - sections
    - reading_order

accessibility_requirements:
  wcag_level: "AA"
  alt_text_coverage: 100%
  heading_hierarchy: "valid"
  aria_labels: "complete"
  reading_order: "defined"

format_constraints:
  title_max_length: 200
  alt_text_max_length: 125
  long_description_required_for: ["chart", "diagram", "complex_image"]
```

---

## Examples

### 11.1 Minimal Document

```json
{
  "wia_version": "1.0.0",
  "document_id": "doc-minimal-001",
  "document_info": {
    "title": "Getting Started with PubScript",
    "language": "en-US"
  },
  "authors": [
    {
      "name": "John Doe",
      "role": "primary_author"
    }
  ],
  "structure": {
    "type": "article",
    "outline": [
      {
        "id": "chapter-01",
        "title": "Introduction",
        "file": "content/chapters/chapter-01.json"
      }
    ]
  },
  "accessibility": {
    "wcag_level": "AA",
    "wcag_version": "2.1"
  }
}
```

---

## TypeScript Interfaces

```typescript
interface PubScriptDocument {
  $schema: string;
  wia_version: string;
  format_version: string;
  document_id: string;
  document_info: DocumentInfo;
  authors: Author[];
  structure: DocumentStructure;
  output_formats: OutputFormats;
  accessibility: AccessibilityInfo;
  rights: RightsInfo;
}

interface DocumentInfo {
  title: string;
  subtitle?: string;
  language: string;
  created: string;
  modified: string;
  version: string;
  status: 'draft' | 'review' | 'published' | 'archived';
}

interface Author {
  id: string;
  name: string;
  role: 'primary_author' | 'contributing_author' | 'editor' | 'translator';
  affiliation?: string;
  orcid?: string;
  bio?: string;
}

interface Section {
  section_id: string;
  type: 'heading' | 'paragraph' | 'list' | 'figure' | 'table' | 'code' | 'quote';
  content?: string;
  level?: number;
  aria_label?: string;
  accessibility?: SectionAccessibility;
}
```

---

## Python Data Classes

```python
from dataclasses import dataclass
from typing import List, Optional, Dict
from datetime import datetime

@dataclass
class DocumentInfo:
    title: str
    language: str
    created: datetime
    modified: datetime
    version: str
    status: str
    subtitle: Optional[str] = None

@dataclass
class Author:
    id: str
    name: str
    role: str
    affiliation: Optional[str] = None
    orcid: Optional[str] = None
    bio: Optional[str] = None

@dataclass
class PubScriptDocument:
    wia_version: str
    document_id: str
    document_info: DocumentInfo
    authors: List[Author]
    structure: Dict
    output_formats: Dict
    accessibility: Dict
    rights: Dict

    def validate(self) -> bool:
        """Validate document against schema"""
        # Validation logic
        return True

    def export_format(self, format: str) -> bytes:
        """Export to specified format"""
        # Export logic
        pass
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-15
**Author**: WIA PubScript Working Group

---

弘益人間 - *Benefit All Humanity*
