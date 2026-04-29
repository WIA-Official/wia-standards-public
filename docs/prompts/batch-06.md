# 배치 06 프롬프트

복사해서 새 Claude Code 세션에 붙여넣으세요.

---

## 임무: WIA Ebook 배치 06 생성

당신은 WIA 표준 ebook 생성 전문가입니다.
아래 10개 표준에 대해 EN/KO ebook을 생성합니다.

### 대상 표준
1. WIA-CIVIC_PARTICIPATION
2. WIA-CLIMATE
3. WIA-CLIMATE_CHANGE_MITIGATION
4. WIA-CLIMATE_REFUGEE
5. WIA-COGNITIVE_AAC
6. WIA-CONSTRUCTION_ROBOT_CITY
7. WIA-CORAL_REEF_RESTORATION
8. WIA-CREDIT_SCORING
9. WIA-CROP_MONITORING
10. WIA-CROSS_BORDER_PAYMENT

### 표준별 주제 설명
| 표준명 | 영문 | 한글 |
|--------|------|------|
| CIVIC_PARTICIPATION | Digital Civic Participation | 디지털 시민 참여 |
| CLIMATE | Climate Data Standards | 기후 데이터 표준 |
| CLIMATE_CHANGE_MITIGATION | Climate Change Mitigation | 기후변화 완화 |
| CLIMATE_REFUGEE | Climate Refugee Support | 기후 난민 지원 |
| COGNITIVE_AAC | Cognitive AAC Systems | 인지 AAC 시스템 |
| CONSTRUCTION_ROBOT_CITY | Urban Construction Robot | 도시 건설 로봇 |
| CORAL_REEF_RESTORATION | Coral Reef Restoration | 산호초 복원 |
| CREDIT_SCORING | AI Credit Scoring | AI 신용 평가 |
| CROP_MONITORING | Smart Crop Monitoring | 스마트 작물 모니터링 |
| CROSS_BORDER_PAYMENT | Cross-Border Payment | 국경간 결제 |

---

## 필수 규칙 (WordPress 호환)

### ❌ 절대 금지
- `<nav>` 태그 금지
- `<footer>` 태그 금지
- `href="index.html"` 링크 금지
- `href="chapter-*.html"` 링크 금지

### ✅ 필수 포함
- `<h1>` 태그: 제목 (1개)
- `<h2>` 태그: 섹션 제목 (여러 개)
- `<pre>` 태그: 코드 블록 (최소 1개)
- `.highlight` 클래스: 강조 박스
- 파일 크기: 8,000~17,000 bytes

---

## 파일 구조

```
standards/{표준명}/ebook/
├── en/ (index.html + chapter-01~08.html)
└── ko/ (index.html + chapter-01~08.html)
```

---

## 완료 후

1. PROGRESS-TRACKER.md 업데이트 (batch-06 → ✅)
2. `git commit -m "feat(ebook): Complete batch-06"`
3. `git push`

---

**弘益人間 · 널리 인간을 이롭게 하라**
