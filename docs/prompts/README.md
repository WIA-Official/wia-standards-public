# WIA Ebook 배치 프롬프트 인덱스

> 240개 표준 × 18개 파일 = **4,320개 파일** 생성 필요
> 최종 업데이트: 2026-01-24

---

## 🎯 핵심 문서

| 문서 | 용도 |
|------|------|
| [PM-MASTER-GUIDE.md](PM-MASTER-GUIDE.md) | PM 마스터 가이드 |
| [PROGRESS-TRACKER.md](PROGRESS-TRACKER.md) | 진행 현황 추적 |
| [KAITRUST-NEW-STANDARDS.md](KAITRUST-NEW-STANDARDS.md) | 17개 신규 표준 |

---

## 📊 배치 현황

| 배치 | 표준 수 | 프롬프트 | 상태 |
|------|---------|----------|------|
| [batch-01](batch-01.md) | 10 | ✅ | ⏳ |
| [batch-02](batch-02.md) | 10 | ✅ | ⏳ |
| [batch-03](batch-03.md) | 10 | ✅ | ⏳ |
| [batch-04](batch-04.md) | 10 | ✅ | ⏳ |
| [batch-05](batch-05.md) | 10 | ✅ | ⏳ |
| [batch-06](batch-06.md) | 10 | ✅ | ⏳ |
| [batch-07](batch-07.md) | 10 | ✅ | ⏳ |
| [batch-08](batch-08.md) | 10 | ✅ | ⏳ |
| [batch-09](batch-09.md) | 10 | ✅ | ⏳ |
| [batch-10](batch-10.md) | 10 | ✅ | ⏳ |
| [batch-11](batch-11.md) | 10 | ✅ | ⏳ |
| [batch-12](batch-12.md) | 10 | ✅ | ⏳ |
| [batch-13](batch-13.md) | 10 | ✅ | ⏳ |
| [batch-14](batch-14.md) | 10 | ✅ | ⏳ |
| [batch-15](batch-15.md) | 10 | ✅ | ⏳ |
| [batch-16](batch-16.md) | 10 | ✅ | ⏳ |
| [batch-17](batch-17.md) | 10 | ✅ | ⏳ |
| [batch-18](batch-18.md) | 10 | ✅ | ⏳ |
| [batch-19](batch-19.md) | 10 | ✅ | ⏳ |
| [batch-20](batch-20.md) | 10 | ✅ | ⏳ |
| [batch-21](batch-21.md) | 10 | ✅ | ⏳ |
| [batch-22](batch-22.md) | 10 | ✅ | ⏳ |
| [batch-23](batch-23.md) | 3 | ✅ | ⏳ |
| [batch-24](batch-24.md) | 10 | ✅ | ⏳ (KAITRUST 신규) |
| [batch-25](batch-25.md) | 7 | ✅ | ⏳ (KAITRUST 신규) |

**총계: 240개 표준, 25개 배치, 4,320개 파일**

---

## 🚀 병렬 실행 계획

```
Round 1: batch-01~05 (5세션 동시)
Round 2: batch-06~10 (5세션 동시)
Round 3: batch-11~15 (5세션 동시)
Round 4: batch-16~20 (5세션 동시)
Round 5: batch-21~25 (5세션 동시)
```

---

## 📋 새 세션 시작 프롬프트 (복사용)

```
나는 WIA Ebook 프로젝트 작업자입니다.

1. git pull origin main
2. cat docs/prompts/PROGRESS-TRACKER.md
3. 다음 ⏳ 배치 프롬프트 실행
4. 완료 후 PROGRESS-TRACKER.md 업데이트
5. git commit & push

시작하세요.
```

---

## ✅ 품질 체크리스트

- [ ] EN 폴더 9개 파일
- [ ] KO 폴더 9개 파일
- [ ] 8KB 이상
- [ ] `<nav>`, `<footer>` 없음
- [ ] `href="index.html"`, `href="chapter-"` 없음
- [ ] `<h1>`, `<h2>`, `<pre>`, `.highlight` 있음

---

**弘益人間 · 널리 인간을 이롭게 하라**
