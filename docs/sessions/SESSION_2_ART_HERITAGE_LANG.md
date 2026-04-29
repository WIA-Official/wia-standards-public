# Session 2: ART + HERITAGE + LANG (32개 표준)

**테마:** 예술, 문화유산, 언어
**Primary Colors:**
- ART: #F59E0B (amber)
- HERITAGE: #D97706 (orange)
- LANG: #10B981 (emerald)

---

## 작업 목록 (32개)

### WIA-ART (12개)
```
WIA-ART-001-digital-art
WIA-ART-002-ai-generated-art
WIA-ART-003-music-production
WIA-ART-004-film-technology
WIA-ART-005-performing-arts-digital
WIA-ART-006-art-authentication
WIA-ART-007-creative-ai
WIA-ART-008-digital-fashion
WIA-ART-009-sound-audio
WIA-ART-010-interactive-art
WIA-ART-011-virtual-exhibition
WIA-ART-012-art-preservation
```

### WIA-HERITAGE (10개)
```
WIA-HERITAGE-001-cultural-artifact
WIA-HERITAGE-002-archaeological-data
WIA-HERITAGE-003-historical-document
WIA-HERITAGE-004-intangible-heritage
WIA-HERITAGE-005-traditional-knowledge
WIA-HERITAGE-006-heritage-3d-scan
WIA-HERITAGE-007-museum-digital
WIA-HERITAGE-008-folklore-archive
WIA-HERITAGE-009-cultural-repatriation
WIA-HERITAGE-010-world-heritage
```

### WIA-LANG (10개)
```
WIA-LANG-001-endangered-language
WIA-LANG-002-indigenous-script
WIA-LANG-003-oral-tradition
WIA-LANG-004-dialect-preservation
WIA-LANG-005-language-ai
WIA-LANG-006-multilingual-interface
WIA-LANG-007-sign-language-digital
WIA-LANG-008-language-learning-ai
WIA-LANG-009-translation-quality
WIA-LANG-010-linguistic-data
```

---

## 작업 명령어

```bash
# 브랜치 생성
git checkout -b claude/ebook-session-2-[SESSION_ID]

# 작업할 표준 목록 확인
ls -d standards/WIA-ART-* standards/WIA-HERITAGE-* standards/WIA-LANG-*

# Spec 파일 읽기
cat standards/WIA-ART-001-digital-art/spec/*.md

# 품질 확인
find standards/WIA-ART-*/ebook standards/WIA-HERITAGE-*/ebook standards/WIA-LANG-*/ebook -name "*.html" -exec wc -c {} \; | awk '{if($1 < 15000) print "FAIL:", $2}'

# 커밋 및 푸시
git add -A
git commit -m "feat: Expand ebook content for ART/HERITAGE/LANG standards"
git push -u origin claude/ebook-session-2-[SESSION_ID]
```

---

## 콘텐츠 가이드

### ART 표준
- 디지털 아트 인증, AI 생성 아트
- 음악 제작, 영화 기술
- 가상 전시회, 예술 보존

### HERITAGE 표준
- 문화유산 디지털화, 고고학 데이터
- 무형 문화재, 전통 지식
- 3D 스캔, 박물관 디지털화

### LANG 표준
- 멸종 위기 언어, 토착 문자
- 구전 전통, 방언 보존
- AI 번역, 수화 디지털화

---

**참조:** docs/EBOOK_DISTRIBUTION_GUIDE.md, docs/EBOOK_TEMPLATE.html
