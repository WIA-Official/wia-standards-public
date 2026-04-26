# WIA ISBN Manager Plugin Data

이 폴더는 wiabook.com ISBN Manager 플러그인의 데이터입니다.

## 파일 설명

- `class-wia-isbn-data.php`: 75개 완성 표준의 메타데이터
  - 제목 (bundle/ko/en)
  - 부제목 (bundle/ko/en)
  - cover_prompt (DALL-E 3용)

## 용도

Claude Code가 이 파일을 참고하여:
1. 기존 93개 완성 표준의 패턴 학습
2. 580개 예정 표준의 메타데이터 생성
3. 동일한 품질/형식 유지

## 표준 형식

```php
'표준명' => [
    'emoji' => '🎨',
    'name_en' => 'English Name',
    'name_ko' => '한글 이름',
    'bundle_title' => 'WIA {Name} Standard Guide Set (KO/EN)',
    'bundle_subtitle' => 'Feature1, Feature2, Feature3, Feature4, Feature5 & Feature6',
    'ko_title' => 'WIA {이름} 표준화 가이드',
    'ko_subtitle' => '기능1, 기능2, 기능3, 기능4, 기능5 및 기능6',
    'en_title' => 'WIA {Name} Standard Guide',
    'en_subtitle' => '{bundle_subtitle과 동일}',
    'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, {이미지 설명 200-300자}, dark navy background (#0f172a), no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
],
```

---

**홍익인간 (弘益人間)** - Benefit All Humanity 🌍

---

## Conformance tiers

WIA conformance for **plugins** is evaluated across three tiers, mirrored in every PHASE document under `spec/`:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days of detection. The conformity assessment process for Tier 3 is aligned with ISO/IEC 17065:2012 and depends on the documentary evidence retention policy described in the per-PHASE Annex A under `spec/`.

## Layout

```
standards/plugins/
├── README.md            # this document
├── index.html           # human-readable landing page
├── simulator/           # interactive browser-based simulator
├── spec/                # PHASE-1..N normative specifications
├── api/                 # reference TypeScript SDK skeleton
├── cli/                 # POSIX shell client
├── press/               # press kit (article + DALL·E prompts)
└── ebook/{en,ko}/      # eight-chapter ebook editions
```

The PHASE documents under `spec/` are the normative source. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

## Open governance

Comments, proposals, and conformance reports are accepted via the GitHub issues tracker on the WIA-Official organization. Major version bumps follow the WIA Standards governance process documented at <https://wiastandards.com/governance>.

---

**弘益人間 · Benefit All Humanity** — © 2026 WIA. Licensed under MIT.
