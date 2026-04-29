# Session 4: LEGAL + IND + ROB (23개 표준)

**테마:** 법률, 산업, 로봇
**Primary Colors:**
- LEGAL: #64748B (slate)
- IND: #EF4444 (red)
- ROB: #8B5CF6 (purple)

---

## 작업 목록 (23개)

### WIA-LEGAL (10개)
```
WIA-LEGAL-001-digital-court
WIA-LEGAL-002-online-dispute
WIA-LEGAL-003-smart-legal-contract
WIA-LEGAL-004-digital-evidence
WIA-LEGAL-005-legal-ai
WIA-LEGAL-006-ip-digital
WIA-LEGAL-007-digital-forensics
WIA-LEGAL-008-e-notary
WIA-LEGAL-009-legal-data-exchange
WIA-LEGAL-010-international-law-digital
```

### WIA-IND (11개)
```
WIA-IND-001-fashion-tech
WIA-IND-002-smart-textile
WIA-IND-003-wearable-fashion
WIA-IND-004-beauty-tech
WIA-IND-008-smart-kitchen
WIA-IND-009-food-delivery
WIA-IND-010-personalized-nutrition
WIA-IND-011-sports-analytics
WIA-IND-012-fitness-wearable
WIA-IND-013-esports
WIA-IND-014-virtual-fitness
```

### WIA-ROB (2개)
```
WIA-ROB-009
WIA-ROB-019
```

---

## 작업 명령어

```bash
# 브랜치 생성
git checkout -b claude/ebook-session-4-[SESSION_ID]

# 작업할 표준 목록 확인
ls -d standards/WIA-LEGAL-* standards/WIA-IND-* standards/WIA-ROB-*

# Spec 파일 읽기
cat standards/WIA-LEGAL-001-digital-court/spec/*.md

# 품질 확인
find standards/WIA-LEGAL-*/ebook standards/WIA-IND-*/ebook standards/WIA-ROB-*/ebook -name "*.html" -exec wc -c {} \; | awk '{if($1 < 15000) print "FAIL:", $2}'

# 커밋 및 푸시
git add -A
git commit -m "feat: Expand ebook content for LEGAL/IND/ROB standards"
git push -u origin claude/ebook-session-4-[SESSION_ID]
```

---

## 콘텐츠 가이드

### LEGAL 표준
- 디지털 법원, 온라인 분쟁 해결
- 스마트 법적 계약, 디지털 증거
- 법률 AI, 디지털 포렌식

### IND 표준
- 패션 테크, 스마트 텍스타일
- 스마트 주방, 음식 배달
- 스포츠 분석, e스포츠

### ROB 표준
- 로봇 표준
- spec 파일 확인 후 관련 콘텐츠 작성

---

**참조:** docs/EBOOK_DISTRIBUTION_GUIDE.md, docs/EBOOK_TEMPLATE.html
