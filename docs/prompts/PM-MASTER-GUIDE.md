# WIA Ebook 프로젝트 PM 마스터 가이드

> **목적:** 여러 Claude Code 세션이 바톤 터치하며 연속 작업
> **버전:** 1.0
> **최종 업데이트:** 2026-01-24

---

## 🎯 프로젝트 개요

| 항목 | 값 |
|------|-----|
| **총 표준 (기존)** | 223개 |
| **신규 표준 (KAITRUST)** | 17개 |
| **총 표준** | **240개** |
| **표준당 파일** | 18개 (EN 9 + KO 9) |
| **총 생성 파일** | **4,320개** |
| **배치 수** | 25개 (기존 23 + 신규 2) |

---

## 📁 Git 저장소

```
https://github.com/WIA-Official/wia-standards
```

### 핵심 경로
```
/docs/prompts/
├── PM-MASTER-GUIDE.md      ← 이 파일 (PM 가이드)
├── PROGRESS-TRACKER.md     ← 진행 현황 (실시간 업데이트)
├── README.md               ← 배치 인덱스
├── batch-01.md ~ batch-25.md  ← 배치별 프롬프트
└── KAITRUST-NEW-STANDARDS.md  ← 17개 신규 표준 목록
```

---

## 🔄 바톤 터치 프로세스

### 새 세션 시작 시 (필수!)

```bash
# 1. 최신 코드 받기
cd /var/www/wia-standards
git pull origin main

# 2. 진행 현황 확인
cat docs/prompts/PROGRESS-TRACKER.md

# 3. 다음 할 일 확인
# → PROGRESS-TRACKER.md에서 ⏳ 상태인 배치 찾기
```

### 작업 완료 시 (필수!)

```bash
# 1. 커밋
git add .
git commit -m "feat(ebook): Complete batch-XX (표준명들)"

# 2. 진행 현황 업데이트
# → PROGRESS-TRACKER.md에서 해당 배치 ✅ 로 변경

# 3. 푸시
git push origin main
```

---

## 📊 배치 구성

### 기존 표준 (batch-01 ~ batch-23)
| 배치 | 표준 수 | 범위 |
|------|---------|------|
| 01-22 | 10개씩 | WIA-AAC ~ WIA-SEAWATER |
| 23 | 3개 | WIA-SEAWEED ~ WIA-SLEEP |
| **소계** | **223개** | |

### 신규 표준 (batch-24 ~ batch-25) - KAITRUST용
| 배치 | 표준 수 | 범위 |
|------|---------|------|
| 24 | 10개 | WIA-POLICY-001 ~ WIA-COMP-DASH-001 |
| 25 | 7개 | WIA-DOC-GEN-001 ~ WIA-CASE-001 |
| **소계** | **17개** | |

---

## 🚀 병렬 실행 전략

```
Round 1: batch-01~05 (5세션 동시)
Round 2: batch-06~10 (5세션 동시)
Round 3: batch-11~15 (5세션 동시)
Round 4: batch-16~20 (5세션 동시)
Round 5: batch-21~25 (5세션 동시)
```

---

## ✅ 품질 체크리스트

각 표준 완료 시:
- [ ] EN 폴더에 9개 파일
- [ ] KO 폴더에 9개 파일
- [ ] 각 파일 8KB 이상
- [ ] `<nav>`, `<footer>` 태그 없음
- [ ] `href="index.html"`, `href="chapter-"` 링크 없음
- [ ] `<h1>`, `<h2>`, `<pre>`, `.highlight` 있음
- [ ] PROGRESS-TRACKER.md 업데이트
- [ ] 커밋 & 푸시 완료

---

## 📋 세션 시작 프롬프트 (복사용)

```
나는 WIA Ebook 프로젝트의 작업자입니다.

1. 먼저 Git pull 하세요
2. docs/prompts/PROGRESS-TRACKER.md 확인
3. 다음 ⏳ 배치의 프롬프트(batch-XX.md) 실행
4. 완료 후 PROGRESS-TRACKER.md 업데이트
5. 커밋 & 푸시

시작하세요.
```

---

## ⚠️ 핵심 규칙

1. **항상 Git pull 먼저** - 다른 세션 작업 반영
2. **PROGRESS-TRACKER.md 업데이트 필수** - 중복 작업 방지
3. **커밋 메시지 형식** - `feat(ebook): Complete batch-XX (표준명들)`
4. **충돌 시** - main 브랜치 기준으로 해결

---

## 🔗 관련 문서

- `WIA-STANDARDS-PATH-MASTER.md` - 경로 가이드
- `KAITRUST-WIA-MAPPING.md` - KAITRUST 매핑
- `WIA_Ebook_Master_Prompt.md` - Ebook 품질 기준

---

**弘益人間 · 널리 인간을 이롭게 하라**
