# Session 1C: TIME-021 ~ TIME-035 (15개 표준)

**Primary Color:** #06B6D4 (cyan)
**테마:** 시간 여행 응용 및 안전

---

## 프롬프트 (복사해서 사용)

```
wia-standards 레포에서 ebook 작업해줘.

## 사전 작업
git fetch origin main && git merge origin/main

## 할당
WIA-TIME-021 ~ WIA-TIME-035 (15개 표준)
각 표준당 ebook/en, ebook/ko 폴더에 chapter-01.html ~ chapter-08.html + index.html

## 필독 (반드시 먼저 읽기)
1. docs/EBOOK_DISTRIBUTION_GUIDE.md - 품질 기준
2. docs/EBOOK_TEMPLATE.html - HTML 템플릿
3. standards/fire-safety-system/ebook/en/chapter-01.html - 완성 샘플 (30KB)

## 각 표준의 spec 파일 반드시 참조
cat standards/WIA-TIME-021/spec/*.md
각 chapter 내용은 해당 표준의 PHASE-1~4.md와 매칭되어야 함

## 핵심 품질 규칙
- 각 chapter 15KB 이상 (실제 콘텐츠로, 패딩 금지)
- UTF-8 인코딩 (弘益人間, 한국어 정상 표시)
- 카테고리 색상: --primary: #06B6D4
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
git commit -m "feat: Complete TIME-021~035 ebook content (en/ko)"
git push -u origin claude/ebook-time-021-035-[SESSION_ID]
PR 생성 → main 머지
```

---

## 작업 목록

| 표준 | 주제 | 파일 수 |
|------|------|---------|
| WIA-TIME-021 | Return Protocol | 18 |
| WIA-TIME-022 | Temporal Rescue | 18 |
| WIA-TIME-023 | Time Traveler Registry | 18 |
| WIA-TIME-024 | Time Measurement | 18 |
| WIA-TIME-025 | Temporal Certification | 18 |
| WIA-TIME-026 | Time Travel Ethics | 18 |
| WIA-TIME-027 | Historical Preservation | 18 |
| WIA-TIME-028 | Temporal Commerce | 18 |
| WIA-TIME-029 | Time Tourism | 18 |
| WIA-TIME-030 | Temporal Law | 18 |
| WIA-TIME-031 | Time Communication | 18 |
| WIA-TIME-032 | Temporal Medicine | 18 |
| WIA-TIME-033 | Time Archaeology | 18 |
| WIA-TIME-034 | Future Prediction | 18 |
| WIA-TIME-035 | Information Security | 18 |
| **총계** | | **270 파일** |

---

## 검증 명령어

```bash
# 파일 크기 확인
for i in $(seq 21 35); do
  find standards/WIA-TIME-0$i/ebook -name "*.html" -exec wc -c {} \;
done | awk '{if($1 < 15000) print "FAIL:", $2}'
```
