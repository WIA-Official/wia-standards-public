# Session 1E: OCEAN (10개 표준)

**Primary Color:** #0EA5E9 (sky blue)
**테마:** 해양 탐사 기술

---

## 프롬프트 (복사해서 사용)

```
wia-standards 레포에서 ebook 작업해줘.

## 사전 작업
git fetch origin main && git merge origin/main

## 할당
WIA-OCEAN-001 ~ WIA-OCEAN-010 (10개 표준)
각 표준당 ebook/en, ebook/ko 폴더에 chapter-01.html ~ chapter-08.html + index.html

## 필독 (반드시 먼저 읽기)
1. docs/EBOOK_DISTRIBUTION_GUIDE.md - 품질 기준
2. docs/EBOOK_TEMPLATE.html - HTML 템플릿
3. standards/fire-safety-system/ebook/en/chapter-01.html - 완성 샘플 (30KB)

## 각 표준의 spec 파일 반드시 참조
cat standards/WIA-OCEAN-001-deep-sea-exploration/spec/*.md
각 chapter 내용은 해당 표준의 PHASE-1~4.md와 매칭되어야 함

## 핵심 품질 규칙
- 각 chapter 15KB 이상 (실제 콘텐츠로, 패딩 금지)
- UTF-8 인코딩 (弘益人間, 한국어 정상 표시)
- 카테고리 색상: --primary: #0EA5E9 (sky blue)
- 테이블 2개 이상, Summary, Review Questions 6개 이상
- en/ko 둘 다 작업 (ko는 한국어로 번역)

## Chapter 매핑
- chapter-01: 소개, 개요 (PHASE-1 기반)
- chapter-02: 현재 과제, 문제점
- chapter-03: 데이터 형식 심화 (PHASE-1)
- chapter-04: API 인터페이스 (PHASE-2)
- chapter-05: 프로토콜 (PHASE-3)
- chapter-06: 보안, 인증 (PHASE-3)
- chapter-07: 통합, 배포 (PHASE-4)
- chapter-08: 미래 전망, 결론

## 완료 후
git add -A
git commit -m "feat: Complete OCEAN ebook content (en/ko)"
git push -u origin claude/ebook-ocean-[SESSION_ID]
PR 생성 → main 머지
```

---

## 작업 목록

| 표준 | 주제 | 파일 수 |
|------|------|---------|
| WIA-OCEAN-001-deep-sea-exploration | 심해 탐사 | 18 |
| WIA-OCEAN-002-underwater-drone | 수중 드론 | 18 |
| WIA-OCEAN-003-underwater-communication | 수중 통신 | 18 |
| WIA-OCEAN-004-marine-biology-data | 해양 생물 데이터 | 18 |
| WIA-OCEAN-005-ocean-floor-mapping | 해저 매핑 | 18 |
| WIA-OCEAN-006-submarine-tech | 잠수함 기술 | 18 |
| WIA-OCEAN-007-marine-sensor | 해양 센서 | 18 |
| WIA-OCEAN-008-port-automation | 항만 자동화 | 18 |
| WIA-OCEAN-009-ship-autonomous | 자율 선박 | 18 |
| WIA-OCEAN-010-ocean-resource | 해양 자원 | 18 |
| **총계** | | **180 파일** |
