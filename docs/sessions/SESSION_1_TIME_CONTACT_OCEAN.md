# Session 1: TIME + CONTACT + OCEAN (55개 표준)

**테마:** 시간, 우주, 해양 탐사
**Primary Colors:**
- TIME: #06B6D4 (cyan)
- CONTACT: #8B5CF6 (purple)
- OCEAN: #0EA5E9 (sky blue)

---

## 작업 목록 (55개)

### WIA-TIME (35개)
```
WIA-TIME-001
WIA-TIME-002
WIA-TIME-003
WIA-TIME-004
WIA-TIME-005
WIA-TIME-006
WIA-TIME-007
WIA-TIME-008
WIA-TIME-009
WIA-TIME-010
WIA-TIME-011
WIA-TIME-012
WIA-TIME-013
WIA-TIME-014
WIA-TIME-015
WIA-TIME-016
WIA-TIME-017
WIA-TIME-018
WIA-TIME-019
WIA-TIME-020
WIA-TIME-021
WIA-TIME-022
WIA-TIME-023
WIA-TIME-024
WIA-TIME-025
WIA-TIME-026
WIA-TIME-027
WIA-TIME-028
WIA-TIME-029
WIA-TIME-030
WIA-TIME-031
WIA-TIME-032
WIA-TIME-033
WIA-TIME-034
WIA-TIME-035
```

### WIA-CONTACT (10개)
```
WIA-CONTACT-001-first-contact-protocol
WIA-CONTACT-002-seti-data-standard
WIA-CONTACT-003-interstellar-message
WIA-CONTACT-004-alien-language-decoding
WIA-CONTACT-005-planetary-defense
WIA-CONTACT-006-biosignature-detection
WIA-CONTACT-007-non-human-intelligence
WIA-CONTACT-008-extraterrestrial-law
WIA-CONTACT-009-cosmic-communication
WIA-CONTACT-010-galactic-registry
```

### WIA-OCEAN (10개)
```
WIA-OCEAN-001-deep-sea-exploration
WIA-OCEAN-002-underwater-drone
WIA-OCEAN-003-underwater-communication
WIA-OCEAN-004-marine-biology-data
WIA-OCEAN-005-ocean-floor-mapping
WIA-OCEAN-006-submarine-tech
WIA-OCEAN-007-marine-sensor
WIA-OCEAN-008-port-automation
WIA-OCEAN-009-ship-autonomous
WIA-OCEAN-010-ocean-resource
```

---

## 작업 명령어

```bash
# 브랜치 생성
git checkout -b claude/ebook-session-1-[SESSION_ID]

# 작업할 표준 목록 확인
ls -d standards/WIA-TIME-* standards/WIA-CONTACT-* standards/WIA-OCEAN-*

# Spec 파일 읽기 (예: TIME-001)
cat standards/WIA-TIME-001/spec/*.md

# 각 ebook chapter 생성 (15KB 이상)
# docs/EBOOK_TEMPLATE.html 참조

# 품질 확인
find standards/WIA-TIME-*/ebook standards/WIA-CONTACT-*/ebook standards/WIA-OCEAN-*/ebook -name "*.html" -exec wc -c {} \; | awk '{if($1 < 15000) print "FAIL:", $2}'

# 커밋 및 푸시
git add -A
git commit -m "feat: Expand ebook content for TIME/CONTACT/OCEAN standards"
git push -u origin claude/ebook-session-1-[SESSION_ID]
```

---

## 콘텐츠 가이드

### TIME 표준
- 시간 여행 물리학, 시간선 관리, 인과율 보호
- spec/PHASE-1~4.md 참조
- 상대성 이론, 시공간 좌표계, 에너지 계산

### CONTACT 표준
- 외계 문명 접촉, SETI, 행성 방어
- 첫 접촉 프로토콜, 외계 언어 해독
- 국제 우주법, 은하계 등록

### OCEAN 표준
- 심해 탐사, 수중 통신, 해양 자원
- 자율 선박, 항만 자동화
- 해양 센서, 해저 매핑

---

**참조:** docs/EBOOK_DISTRIBUTION_GUIDE.md, docs/EBOOK_TEMPLATE.html
