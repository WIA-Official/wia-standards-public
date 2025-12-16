# WIA Education Standard - Phase 1 Research

**Educational Technology Accessibility Standards Research**

---

## 1. Overview

This document presents research findings on educational technology accessibility standards, existing specifications, regulatory requirements, and best practices for inclusive e-learning.

---

## 2. Regulatory Landscape

### 2.1 United States

#### ADA Title II (2024 Update)
- **New Rule**: DOJ issued final regulations in April 2024 mandating WCAG 2.1 Level AA compliance
- **Deadline**: April 2026 for institutions with populations over 50,000; April 2027 for smaller institutions
- **Scope**: All web content, online course materials, mobile apps, LMS platforms

#### Section 508 (Rehabilitation Act)
- Requires federal agencies and federally funded programs to ensure accessible electronic and information technology
- Applies to all educational institutions receiving federal funding

#### Colorado HB 21-1110
- State precedent requiring WCAG 2.1 Level AA compliance by July 1, 2025
- Applies to all state and local government digital services including educational platforms

### 2.2 International

#### European Accessibility Act (EAA)
- Effective June 2025
- Sets strict digital accessibility requirements for products and services sold in the EU
- Includes educational technology and e-learning platforms

#### Web Content Accessibility Guidelines (WCAG)
- **Current Version**: WCAG 2.2 (released October 2023)
- **Legal Standard**: WCAG 2.1 Level AA is the baseline for most regulations
- **ISO Standard**: Same as ISO/IEC 40500:2012

---

## 3. Existing Standards and Specifications

### 3.1 W3C/WAI Standards

#### WCAG (Web Content Accessibility Guidelines)
Four core principles (POUR):
1. **Perceivable**: Distinguishable UI, text alternatives, adaptable content
2. **Operable**: Keyboard accessible, navigable, no seizure-inducing content
3. **Understandable**: Readable, predictable, input assistance
4. **Robust**: Compatible with assistive technologies

#### ATAG (Authoring Tool Accessibility Guidelines)
- International standard for LMS and education authoring tools
- Part A: Accessible user interface for content creators
- Part B: Support for creating accessible content

#### UAAG (User Agent Accessibility Guidelines)
- For browsers and media players used in educational contexts

### 3.2 1EdTech (formerly IMS Global) Standards

#### AccessForAll (AfA) v3.0
- **Purpose**: Match learner needs with accessible resources
- **Components**:
  - Personal Needs & Preferences (PNP): What the learner needs
  - Digital Resource Description (DRD): What the resource provides
- **ISO Standard**: ISO/IEC 24751
- **Categories**:
  - Display: How resources are presented
  - Control: How resources are operated
  - Content: Alternative/supplementary resources

#### LTI (Learning Tools Interoperability) 1.3
- Seamless integration of external tools into LMS
- Single sign-on and grade passback
- Accessibility features can be passed between systems

#### QTI (Question and Test Interoperability) 3.0
- Standard format for assessments
- Supports accessibility metadata
- Portable between different platforms

#### Common Cartridge
- Content packaging standard
- Includes accessibility metadata support

#### Caliper Analytics
- Learning analytics standard
- Can track accessibility feature usage

### 3.3 Other E-Learning Standards

#### SCORM (Sharable Content Object Reference Model)
- Developed by ADL (Advanced Distributed Learning)
- Content packaging and runtime tracking
- Being superseded by xAPI/cmi5

#### xAPI (Experience API)
- Modern learning data standard
- Can track diverse learning experiences
- More flexible than SCORM

### 3.4 Metadata Standards

#### Dublin Core
- 15 core metadata elements
- Foundation for educational resource description
- Extended by IEEE LOM

#### IEEE LOM (Learning Object Metadata)
- Extends Dublin Core for education
- Includes accessibility metadata fields

#### Schema.org Accessibility Properties
- Modern web-based approach
- Used for SEO and resource discovery

---

## 4. Universal Design for Learning (UDL)

### 4.1 Overview
Developed by CAST, UDL is a framework for designing inclusive learning experiences based on how the human brain learns.

### 4.2 Three Core Principles

#### 1. Engagement (The "Why" of Learning)
- **Recruiting Interest**: Choice, autonomy, relevance
- **Sustaining Effort**: Goals, collaboration, feedback
- **Self-Regulation**: Motivation, coping skills, self-assessment

#### 2. Representation (The "What" of Learning)
- **Perception**: Multiple formats (audio, visual, tactile)
- **Language & Symbols**: Clarify vocabulary, math notation, syntax
- **Comprehension**: Activate background knowledge, highlight patterns

#### 3. Action & Expression (The "How" of Learning)
- **Physical Action**: Multiple methods for navigation and response
- **Expression & Communication**: Multiple media for communication
- **Executive Function**: Goal-setting, planning, progress monitoring

### 4.3 UDL Guidelines 3.0 (July 2024)
- Updated for learner-centered language
- Asset-based approach focusing on learner identity
- Emphasis on interdependence and collective learning

---

## 5. Key Disability Types and Accommodations

### 5.1 Visual Disabilities
| Need | Accommodation |
|------|---------------|
| Blindness | Screen reader support, audio descriptions |
| Low Vision | Magnification, high contrast, adjustable fonts |
| Color Blindness | Color-independent design, patterns/labels |

### 5.2 Auditory Disabilities
| Need | Accommodation |
|------|---------------|
| Deafness | Captions, transcripts, sign language videos |
| Hard of Hearing | Adjustable volume, visual alerts |

### 5.3 Motor/Physical Disabilities
| Need | Accommodation |
|------|---------------|
| Limited Dexterity | Keyboard navigation, large click targets |
| Mobility Impairment | Voice control, switch access |
| Tremors | Stabilization features, extended timeouts |

### 5.4 Cognitive Disabilities
| Need | Accommodation |
|------|---------------|
| Learning Disabilities | Text-to-speech, simplified language, chunked content |
| ADHD | Focus mode, reduced distractions, flexible deadlines |
| Dyslexia | Dyslexia-friendly fonts, audio alternatives |
| Autism | Predictable structure, sensory controls |

### 5.5 Multiple Disabilities
- Combination of accommodations
- Customizable profiles
- Flexible presentation options

---

## 6. LMS Accessibility Requirements

### 6.1 Core Features
- **Keyboard Navigation**: All functions accessible without mouse
- **Screen Reader Support**: ARIA labels, logical reading order
- **Captions & Transcripts**: All multimedia content
- **Alternative Text**: All images and non-text content
- **Color Contrast**: Minimum 4.5:1 ratio (AAA: 7:1)
- **Resizable Text**: Up to 200% without loss of functionality

### 6.2 Content Authoring
- Accessibility checking tools
- Templates with built-in accessibility
- Guidance for content creators

### 6.3 Assessment Features
- Extended time options
- Alternative question formats
- Assistive technology compatibility

---

## 7. Key Statistics

- **70%** of online students with disabilities do not disclose or request accommodations
- Design for accessibility from the beginning is crucial
- Retrofit accessibility is expensive and often incomplete

---

## 8. Major Stakeholders

### 8.1 LMS Providers
- Canvas (Instructure)
- Blackboard
- Moodle
- D2L Brightspace
- Google Classroom
- Microsoft Teams for Education

### 8.2 Assessment Platforms
- Pearson
- McGraw-Hill
- Respondus
- Proctorio

### 8.3 Standards Organizations
- W3C/WAI
- 1EdTech (IMS Global)
- CAST (UDL)
- ADL (xAPI)

---

## 9. WIA EDU Standard Focus Areas

Based on research, WIA EDU should address:

### 9.1 Learner Profile
- Personal accessibility preferences
- Disability types and accommodations needed
- Learning style preferences (UDL-aligned)
- Device and assistive technology settings

### 9.2 Content Accessibility Metadata
- Accessibility features present
- Accessibility barriers
- Alternative formats available
- Conformance level (WCAG)

### 9.3 Course/Module Structure
- Learning objectives with UDL mapping
- Content delivery options
- Assessment accommodations
- Progress tracking

### 9.4 Assessment Accessibility
- Question accessibility metadata
- Accommodation settings
- Alternative formats
- Timing modifications

### 9.5 Integration with WIA Ecosystem
- AAC device integration
- Eye tracking support
- BCI compatibility
- Screen reader coordination

---

## 10. References

### Regulatory
- [ADA Title II Accessibility Rule](https://edtechmagazine.com/higher/article/2025/06/guide-ada-title-ii-accessibility-rule-perfcon)
- [Federal Digital Accessibility Requirements](https://onlinelearningconsortium.org/olc-insights/2025/09/federal-digital-a11y-requirements/)

### Standards
- [W3C WAI - ATAG for Education](https://www.w3.org/WAI/standards-guidelines/atag/education/)
- [1EdTech Accessibility Standards](https://www.1edtech.org/standards/accessibility)
- [1EdTech AccessForAll Overview](https://www.imsglobal.org/accessibility/accmdv1p0/imsaccmd_oviewv1p0.html)
- [CAST UDL Guidelines](https://udlguidelines.cast.org/)

### Best Practices
- [Making Online Learning Accessible - DO-IT](https://www.washington.edu/doit/tutorial-making-online-learning-accessible-students-disabilities)
- [LMS Accessibility Checklist](https://synergy-learning.com/blog/lms-accessibility-checklist/)

---

弘益人間 - Education for Everyone
