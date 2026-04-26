# WIA Standards 기사 프로젝트 지식 v2.0

> **목적:** WIA 표준 보도자료 작성을 위한 종합 프로젝트 지식
> **최종 업데이트:** 2025-12-25
> **버전:** 2.0

---

## 📁 파일 구조

```
prompts/
├── 📄 README.md                    ← 이 파일 (프로젝트 안내)
├── 📄 Master-Prompt-v4.1.md        ← 기사 생성 마스터 프롬프트
├── 📄 TEMPLATE-KO-v2.1.html        ← 한글 보도자료 템플릿
├── 📄 TEMPLATE-EN-v2.1.html        ← 영문 보도자료 템플릿
├── 📄 CEO-QUOTES-LIBRARY.md        ← CEO 코멘트 패턴 라이브러리
├── 📄 DALLE-PROMPTS-LIBRARY.md     ← 달리 이미지 프롬프트 패턴
├── 📄 CATEGORY-STATS.md            ← 카테고리별 통계 데이터
└── 📄 TERMINOLOGY-KO-EN.md         ← 용어 일관성 가이드
```

---

## 🎯 파일별 용도

| 파일 | 용도 | 언제 참조 |
|------|------|----------|
| **Master-Prompt-v4.1.md** | 기사 생성 전체 가이드 | 기사 작성 시작 시 |
| **TEMPLATE-KO-v2.1.html** | 한글 HTML 구조 | 한글 기사 작성 시 |
| **TEMPLATE-EN-v2.1.html** | 영문 HTML 구조 | 영문 기사 작성 시 |
| **CEO-QUOTES-LIBRARY.md** | CEO 코멘트 작성 | 코멘트 3개 작성 시 |
| **DALLE-PROMPTS-LIBRARY.md** | 이미지 생성 | 달리 프롬프트 작성 시 |
| **CATEGORY-STATS.md** | 문제점 통계 | 문제점 섹션 작성 시 |
| **TERMINOLOGY-KO-EN.md** | 용어 통일 | 번역/용어 확인 시 |

---

## 🚀 기사 생성 프로세스

```
1️⃣ Claude Code가 표준별 press/ 폴더에 자동 생성:
   └── {표준명}/press/article-ko.html
   └── {표준명}/press/article-en.html
   └── {표준명}/press/dalle-prompts.md

2️⃣ 사용자가 GitHub에서 다운로드

3️⃣ 달리 이미지 생성 후 기사에 삽입

4️⃣ 코리안투데이에 포스팅
```

---

## ✅ 기사 필수 요소

```
□ 헤드라인 (세계 최초 / 대한민국 주도)
□ 부제목 (핵심 가치 한 줄)
□ Executive Summary (MIT 라이선스 무료 공개 강조)
□ 문제점 섹션 (통계 포함)
□ CEO 코멘트 1 (문제 인식 + 권리)
□ 솔루션 섹션 (Phase 1-4)
□ 비교 테이블 (기존 vs WIA)
□ CEO 코멘트 2 (기술적 자신감)
□ 홍익인간 (弘益人間) 섹션 (오픈소스 선언)
□ CEO 코멘트 3 (철학적 의미)
□ 관련 서비스 (전자책/시뮬/인증/컨설팅)
□ WIA 소개 (Rust 기술 스택)
□ 대표 소개 (1인 표준화 아키텍트)
□ 연락처
```

---

## ⚠️ 핵심 규칙

| 규칙 | 설명 |
|------|------|
| "대한민국" 사용 | ❌ "한국" 금지 |
| 날짜 표기 없음 | 발행 시 자동 생성 |
| CEO 코멘트 3개 | 위치별 역할 다름 |
| 홍익인간 (弘益人間) 필수 | 오픈소스 선언 섹션 |
| 이미지 플레이스홀더 | [IMAGE_PLACEHOLDER_1] 형식 |
| 인라인 스타일 | 모든 CSS 인라인으로 |

---

## 📊 카테고리별 테마 색상

| 카테고리 | HEX | Light HEX |
|---------|-----|-----------|
| 🤟 접근성 | #00d4ff | #e0f7fa |
| 🔐 보안 | #ef4444 | #ffebee |
| 🐾 반려동물 | #F59E0B | #fff8e1 |
| 🧊 크라이오닉스 | #06B6D4 | #E0F7FA |
| 🌍 환경 | #10B981 | #e8f5e9 |
| 🤖 AI/로봇 | #8B5CF6 | #f3e8ff |
| 🏥 의료 | #3B82F6 | #e3f2fd |
| 💀 디지털사후 | #8B5CF6 | #f3e8ff |

---

**홍익인간 (弘益人間) - Benefit All Humanity**

*WIA Standards Article Project Knowledge v2.0*
