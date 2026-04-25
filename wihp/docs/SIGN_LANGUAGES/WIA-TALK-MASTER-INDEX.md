# WIA Talk Master Index
## 세계 수어 대백과사전 마스터 인덱스

---

## 📖 이 파일의 용도

**다른 AI(Claude 등)가 이 프로젝트를 이해하고 작업을 이어받으려면 이 파일을 먼저 읽어야 합니다.**

```
이 디렉토리: /var/www/wia.live/docs/SIGN_LANGUAGES/
총 파일: 510+ 배치 파일
총 용량: 3.4MB+
```

---

## 🎯 프로젝트 개요

| 항목 | 내용 |
|------|------|
| **프로젝트명** | WIA Talk World Sign Language Documentation |
| **철학** | 홍익인간 (弘益人間) - 널리 인간을 이롭게 |
| **제작** | SmileStory Inc. |
| **목표** | 전 세계 모든 수어의 체계적 문서화 |
| **위상** | 세계 최대 수어 백과사전 (위키피디아보다 방대) |

---

## 📁 파일 구조

### 배치 번호 체계
```
batch-001 ~ batch-100  : 주요 국가별 수어 (ASL, BSL, KSL, JSL 등)
batch-101 ~ batch-200  : 추가 국가별 수어 + 마을 수어
batch-201 ~ batch-300  : 마을 수어 심화 + 지역별 수어
batch-301 ~ batch-400  : 카리브해, 태평양, 미소국가 + 특수 주제
batch-401 ~ batch-450  : 언어학, 문화, 역사 심화
batch-451 ~ batch-510  : 전문분야, 심화 주제, 미래 방향
```

### 주요 요약 파일
| 파일 | 내용 |
|------|------|
| `batch-400-wia-talk-project-summary.md` | 400개 시점 요약 |
| `batch-450-wia-talk-complete-guide.md` | 450개 시점 가이드 |
| `batch-500-wia-talk-final-summary.md` | 500개 최종 요약 |
| `batch-411-sign-language-geographic-index.md` | 지역별 인덱스 |
| `batch-412-sign-language-iso-codes.md` | ISO 코드 목록 |

---

## 📚 카테고리별 문서 분류

### 1. 국가별 수어 (National Sign Languages) - 약 200개
- **아시아**: batch-03(KSL), batch-04(JSL), batch-05(CSL), batch-14(TSL), batch-09(ISL-인도) 등
- **유럽**: batch-02(BSL), batch-06(LSF), batch-08(DGS), batch-10(LSE) 등
- **아메리카**: batch-01(ASL), batch-13(Libras), batch-15(LSM) 등
- **아프리카**: batch-115~125 (가나, 케냐, 탄자니아, 나이지리아 등)
- **오세아니아**: batch-07(Auslan), batch-12(NZSL), batch-261~262 등

### 2. 마을 수어 (Village Sign Languages) - 약 30개
- batch-126(Kata Kolok), batch-127(Ban Khor), batch-128(ABSL)
- batch-129(Alipur), batch-130(Adamorobe), batch-134(PISL) 등

### 3. 역사적/소멸 수어 - 약 20개
- batch-142(Martha's Vineyard), batch-359(Old Kent), batch-360(Old French) 등

### 4. 수어 가족 (Sign Language Families) - 약 15개
- batch-362~373: BSL계열, LSF계열, DGS계열, 러시아계열, 스칸디나비아 등

### 5. 언어학 심화 - 약 30개
- batch-387(일반 언어학), batch-404(분류사), batch-405(유형론)
- batch-441~449: 부정, 주제-평언, 동사일치, 상, 합성어, 관용어 등
- batch-477~483: 음운론, 형태론, 통사론, 담화분석, 화용론 심화

### 6. 농문화/역사 - 약 40개
- batch-394(농문화), batch-409(역사적 인물), batch-421(농연극)
- batch-427(농예술), batch-447(농유머), batch-448(스토리텔링)
- batch-484~488: 대륙별 농역사 (유럽, 아메리카, 아시아, 아프리카, 오세아니아)

### 7. 교육/권리 - 약 30개
- batch-393(교육), batch-422(교육 방법론), batch-423(농학교)
- batch-389(법적 인정), batch-408(권리 운동), batch-498(CRPD)
- batch-497(WFD), batch-499(미래 방향)

### 8. 기술/미래 - 약 20개
- batch-392(기술), batch-425(VRS), batch-432(앱/게임)
- batch-491(AI), batch-505(웨어러블)

### 9. 전문분야별 수어 - 약 20개
- batch-451(의료), batch-452(법률), batch-453(종교)
- batch-454(STEM), batch-455(스포츠), batch-456(비즈니스)

### 10. 특수 주제 - 약 30개
- batch-381(농맹), batch-383(국제수어), batch-384(베이비사인)
- batch-429(LGBTQ+), batch-430(원주민), batch-431(난민)
- batch-434(CODA), batch-435(노인), batch-489(페미니즘)

---

## 📝 문서 작성 규칙 (표준화용)

### 파일명 규칙
```
batch-[번호]-[주제]-detailed.md
batch-[번호]-[주제].md
```

### 문서 구조 표준
```markdown
# 제목 (영어)
## 부제목 (한국어 또는 현지어)

### Overview
| Attribute | Detail |
|-----------|--------|
| **Topic** | ... |
| **Region** | ... |
| **ISO 639-3** | ... |
| **Population** | ... |
| **Status** | ... |

### Introduction
(소개 문단)

### (주제별 섹션들)
(테이블 + 설명)

### References
1. ...
2. ...

---
*WIA Talk Project - Hongik Ingan*
*Documenting the world's sign languages for universal accessibility*
```

---

## 🔑 핵심 용어

| 올바른 표현 | 피해야 할 표현 |
|------------|---------------|
| 수어 (手語) | 수화 (手話) |
| 농인 (聾人) | 청각장애인 (단독 사용시) |
| 농문화 | 장애 문화 |

---

## 🚀 향후 확장 방향

1. **더 많은 국가별 수어 상세화**
2. **언어학 연구 업데이트**
3. **신기술 (AI, 번역) 추적**
4. **각국 법적 인정 현황 업데이트**
5. **다국어 번역 (영어, 일본어, 중국어 등)**

---

## ⚠️ AI 작업자를 위한 주의사항

1. **기존 파일 수정 시**: 백업 후 작업
2. **새 파일 추가 시**: 번호 체계 유지 (batch-511, 512...)
3. **용어 통일**: "수어" 사용, "수화" 사용 금지
4. **출처 명시**: References 섹션 유지
5. **홍익인간 정신**: 모든 인류를 위한 무료 자료

---

## 📞 관련 파일 빠른 참조

- **전체 요약**: `batch-500-wia-talk-final-summary.md`
- **지역별 인덱스**: `batch-411-sign-language-geographic-index.md`
- **ISO 코드**: `batch-412-sign-language-iso-codes.md`
- **언어학 개요**: `batch-387-sign-language-linguistics.md`
- **농문화 개요**: `batch-394-deaf-culture-identity.md`

---

**© 2024 SmileStory Inc. / WIA Talk Project**
**홍익인간 - 널리 인간을 이롭게 하라**
