# Session 3: MENTAL + CHILD + SENIOR (37개 표준)

**테마:** 정신건강, 아동, 노인
**Primary Colors:**
- MENTAL: #EC4899 (pink)
- CHILD: #F97316 (orange)
- SENIOR: #6366F1 (indigo)

---

## 작업 목록 (37개)

### WIA-MENTAL (15개)
```
WIA-MENTAL-001-digital-therapy
WIA-MENTAL-002-mental-health-ai
WIA-MENTAL-003-depression-detection
WIA-MENTAL-004-anxiety-management
WIA-MENTAL-005-suicide-prevention
WIA-MENTAL-006-ptsd-support
WIA-MENTAL-007-addiction-treatment
WIA-MENTAL-008-sleep-disorder
WIA-MENTAL-009-neurofeedback
WIA-MENTAL-010-mindfulness-app
WIA-MENTAL-011-grief-support
WIA-MENTAL-012-workplace-wellbeing
WIA-MENTAL-013-youth-mental-health
WIA-MENTAL-014-therapy-chatbot
WIA-MENTAL-015-mental-data-privacy
```

### WIA-CHILD (12개)
```
WIA-CHILD-001-online-safety
WIA-CHILD-002-age-verification
WIA-CHILD-003-content-rating
WIA-CHILD-004-parental-control
WIA-CHILD-005-cyberbullying-prevention
WIA-CHILD-006-child-data-privacy
WIA-CHILD-007-digital-addiction-youth
WIA-CHILD-008-educational-content-cert
WIA-CHILD-009-child-ai-interaction
WIA-CHILD-010-predator-detection
WIA-CHILD-011-screen-time-management
WIA-CHILD-012-child-digital-rights
```

### WIA-SENIOR (10개)
```
WIA-SENIOR-001-elder-care-tech
WIA-SENIOR-002-dementia-care
WIA-SENIOR-003-fall-detection
WIA-SENIOR-004-aging-in-place
WIA-SENIOR-005-loneliness-prevention
WIA-SENIOR-006-age-friendly-ui
WIA-SENIOR-007-senior-wearable
WIA-SENIOR-008-memory-assistance
WIA-SENIOR-009-senior-mobility
WIA-SENIOR-010-intergenerational
```

---

## 작업 명령어

```bash
# 브랜치 생성
git checkout -b claude/ebook-session-3-[SESSION_ID]

# 작업할 표준 목록 확인
ls -d standards/WIA-MENTAL-* standards/WIA-CHILD-* standards/WIA-SENIOR-*

# Spec 파일 읽기
cat standards/WIA-MENTAL-001-digital-therapy/spec/*.md

# 품질 확인
find standards/WIA-MENTAL-*/ebook standards/WIA-CHILD-*/ebook standards/WIA-SENIOR-*/ebook -name "*.html" -exec wc -c {} \; | awk '{if($1 < 15000) print "FAIL:", $2}'

# 커밋 및 푸시
git add -A
git commit -m "feat: Expand ebook content for MENTAL/CHILD/SENIOR standards"
git push -u origin claude/ebook-session-3-[SESSION_ID]
```

---

## 콘텐츠 가이드

### MENTAL 표준
- 디지털 치료, AI 정신건강
- 우울증/불안 감지, 자살 예방
- 뉴로피드백, 마음챙김 앱

### CHILD 표준
- 온라인 안전, 나이 인증
- 콘텐츠 등급, 부모 통제
- 사이버 불링 예방, 아동 데이터 프라이버시

### SENIOR 표준
- 노인 돌봄 기술, 치매 케어
- 낙상 감지, 재가 노화
- 고령 친화적 UI, 시니어 웨어러블

---

**참조:** docs/EBOOK_DISTRIBUTION_GUIDE.md, docs/EBOOK_TEMPLATE.html
