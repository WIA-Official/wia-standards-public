# WIA Standards 보도자료 마스터 프롬프트 v4.1

> **용도:** 표준명만 주면 기사 자동 생성
> **출력:** 달리 프롬프트 3개 → 한글 HTML → 영문 HTML

---

## 🎯 사용법

```
표준명: emotion-ai
위 표준의 보도자료를 생성해줘.
```

**끝! 이게 전부입니다.**

---

## 🤖 자동 작업 프로세스

### Step 0: 정보 자동 수집

#### ISBN 메타데이터
```bash
grep -A 30 "'표준명'" /var/www/wiabooks/wp-content/plugins/wia-isbn-manager/includes/class-wia-isbn-data.php
```
추출: emoji, name_ko, name_en, bundle_title, ko_title, en_title

#### 랜딩페이지 파싱
```bash
cat /var/www/wiastandards/표준명/index.html
```

#### Phase 스펙 확인
```bash
for i in 1 2 3 4; do
  head -50 /var/www/wiastandards/표준명/spec/PHASE-$i-*.md
done
```

#### 카테고리 확인
```bash
grep -B 5 -A 5 "표준명" /var/www/wiastandards/index.html
```

---

### Step 1: 달리 프롬프트 3개 생성

#### 프롬프트 1: 메인 히어로
```
Reserve top 20% with pure black (#000000) background for title text area,
{cover_prompt에서 가져온 이미지 설명},
with {테마색} glowing accents,
dark navy background (#0f172a), 
no text, no letters, no words,
professional tech aesthetic, 4K quality
```

#### 프롬프트 2: 문제점 시각화
```
A conceptual illustration showing the problem of {문제점 핵심},
depicting {피해 상황},
muted colors with red (#ef4444) highlights,
editorial illustration style, 4K quality
```

#### 프롬프트 3: 솔루션/미래 비전
```
An optimistic illustration of {해결된 미래 상태},
showing {수혜자} benefiting from {혜택},
bright colors with {테마색} accents,
warm inclusive atmosphere, 8K quality
```

---

### Step 2-3: HTML 생성

- **TEMPLATE-KO-v2.1.html** 기반 한글 기사
- **TEMPLATE-EN-v2.1.html** 기반 영문 기사
- **CEO-QUOTES-LIBRARY.md** 패턴으로 코멘트 3개
- **CATEGORY-STATS.md** 통계 활용

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

## ✅ 필수 체크리스트

```
□ ISBN 데이터 존재 확인
□ 랜딩페이지 파일 존재 확인
□ spec/PHASE-*.md 파일 4개 존재 확인
□ 테마 색상 카테고리 매핑 완료
□ CEO 코멘트 3개 패턴 적용
□ 弘益人間 섹션 포함
□ "대한민국" 사용 ("한국" 아님)
□ 날짜 표기 제거
□ 이미지 플레이스홀더 3개
□ Phase 테이블 완성
□ 비교 테이블 완성
□ 도서 정보 3종 상세 표시
□ WIA Book 구매 링크 강조
□ 관련 서비스 (시뮬/인증/컨설팅)
□ WIA 소개 (Rust 언급)
□ 대표 소개 (1인 표준화 아키텍트)
□ 연락처
```

---

## ⚠️ 필수 준수 사항

### 절대 지키기
- ✅ "한국" 대신 **"대한민국"** 사용
- ✅ 날짜 표기 **제거**
- ✅ 이미지는 플레이스홀더: `[IMAGE_PLACEHOLDER_1]` 형식
- ✅ 모든 스타일은 **인라인**으로
- ✅ CEO 코멘트는 **3개**
- ✅ 弘益人間 섹션 **필수**
- ✅ 대표 소개: "1인 표준화 아키텍트" 문구 포함

### 피하기
- ❌ "세계 최고" (자화자찬)
- ❌ 경쟁사 직접 비난
- ❌ 과장된 수치 (출처 없는 통계)

---

## 📚 참조 파일

| 파일 | 용도 |
|------|------|
| TEMPLATE-KO-v2.1.html | 한글 보도자료 구조 |
| TEMPLATE-EN-v2.1.html | 영문 보도자료 구조 |
| CEO-QUOTES-LIBRARY.md | CEO 코멘트 패턴 |
| DALLE-PROMPTS-LIBRARY.md | 이미지 프롬프트 패턴 |
| CATEGORY-STATS.md | 카테고리별 통계 |
| TERMINOLOGY-KO-EN.md | 용어 일관성 가이드 |

---

**弘益人間 · Benefit All Humanity**

*Master Prompt v4.1 | 2025-12-25*
