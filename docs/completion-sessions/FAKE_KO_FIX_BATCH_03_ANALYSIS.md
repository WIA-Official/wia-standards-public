# Batch 3 Korean Translation Analysis

**Date:** 2026-01-12
**Branch:** claude/translate-completion-docs-NymBi
**Status:** Investigation Phase

---

## Problem Identification

Found **duplicate/legacy ebook files** in the three public-* standards from Batch 3:

### File Sets Discovered

#### Set 1: New/Correct Files (Batch 3)
- **Naming:** `chapter-01.html` through `chapter-08.html` (with dashes)
- **Branding:** WIA-SOCIAL (#EC4899 pink)
- **Status:** ✅ Complete and verified (from Batch 3 Final Report)
- **Quality:** A+ (100%)

#### Set 2: Old/Legacy Files (Pre-Batch 3)
- **Naming:** `chapter1.html` through `chapter8.html` (without dashes)
- **Branding:** WIA-PUBLIC_DOCUMENT (#6366f1 indigo)
- **Status:** ❌ Outdated content, wrong branding
- **Content:** Different chapter titles and structure

---

## Files Requiring Action

### Total Count
- **24 old-style chapter files** across 3 standards (8 per standard)
- **3 index files** (may also need checking)
- **Total:** 27 potentially problematic files

### Breakdown by Standard

#### 1. public-document
**Old files (8):**
- `standards/public-document/ebook/ko/chapter1.html` → "디지털 공문서 소개"
- `standards/public-document/ebook/ko/chapter2.html` → "문서 인증 및 전자서명"
- `standards/public-document/ebook/ko/chapter3.html`
- `standards/public-document/ebook/ko/chapter4.html`
- `standards/public-document/ebook/ko/chapter5.html`
- `standards/public-document/ebook/ko/chapter6.html`
- `standards/public-document/ebook/ko/chapter7.html`
- `standards/public-document/ebook/ko/chapter8.html`

**New files (8):** ✅ Already correct
- `standards/public-document/ebook/ko/chapter-01.html` → "공공문서 시스템 소개"
- `standards/public-document/ebook/ko/chapter-02.html` → "문서 데이터 형식 및 표준"
- etc.

#### 2. public-safety
**Old files (8):**
- `standards/public-safety/ebook/ko/chapter1.html` through `chapter8.html`

**New files (8):** ✅ Already correct
- `standards/public-safety/ebook/ko/chapter-01.html` through `chapter-08.html`

#### 3. public-transportation
**Old files (8):**
- `standards/public-transportation/ebook/ko/chapter1.html` through `chapter8.html`

**New files (8):** ✅ Already correct
- `standards/public-transportation/ebook/ko/chapter-01.html` through `chapter-08.html`

---

## Content Comparison Example

### Chapter 2 - public-document

| Version | File | English Title | Korean Title | Branding |
|---------|------|---------------|--------------|----------|
| **Old** | `chapter2.html` | "Document Authentication & Digital Signatures" | "문서 인증 및 전자서명" | WIA-PUBLIC_DOCUMENT |
| **New** | `chapter-02.html` | "Document Data Formats and Standards" | "문서 데이터 형식 및 표준" | WIA-SOCIAL |

The old and new versions have **completely different content**, not just styling differences.

---

## Recommended Action

### Option A: Delete Old Files (RECOMMENDED)
Since Batch 3 created complete, verified new files with correct:
- Content structure (8 chapters per standard)
- WIA-SOCIAL branding
- Quality verification (A+ rating)
- Proper naming convention (chapter-01 through chapter-08)

**Action:** Delete all 24 old-style `chapter[1-8].html` files as they are:
- Outdated content
- Wrong branding
- Potentially confusing for users
- Superseded by Batch 3 files

### Option B: Retranslate Old Files
If the old files serve a different purpose or represent a different ebook version:
- Read corresponding English originals
- Retranslate Korean content accurately
- Update branding to match current standards

---

## Question for Clarification

**User mentioned "13개" (13 files)** - Need to identify which specific 13 files:

Possibilities:
1. First 13 alphabetically from the 24 old chapter files?
2. All of public-document (8) + first 5 of public-safety?
3. A specific subset with identified translation issues?
4. Index files (3) + 10 specific chapters?

**Awaiting clarification on which 13 files to focus on.**

---

## Next Steps

1. ⏳ **Clarify scope:** Which 13 files specifically?
2. ⏳ **Choose action:** Delete old files OR retranslate?
3. ⏳ **Execute:** Perform chosen action
4. ⏳ **Verify:** Run quality checks
5. ⏳ **Commit:** Git commit and push changes

---

**弘益人間 · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
