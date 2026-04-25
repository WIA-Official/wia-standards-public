# WIA 표준 도서 인용·사실성 정책 (Citation & Veracity Policy)

> **Version**: 1.0
> **Effective**: 2026-04-25
> **Owner**: WIA (World Certification Industry Association)
> **Audience**: 모든 표준 집필자(Claude Code 세션 포함), 검토자, 출판 담당자
> **Binding scope**: `wia-standards` (private) · `wia-standards-public` 양쪽 모든 표준
> **Supersedes**: 없음 (최초 정책)

---

## 0. 정책 한 줄 요약

> **"가공된 연구·통계·인용은 그 어떤 형태로도 결과의 근거로 쓸 수 없다."**

WIA 표준은 출판 도서로 발행되며 국내외 보건의료·산업 규제 영역에 직접 인용될 수 있다. 따라서 본문에 등장하는 모든 인용·통계·기관명·법률명·표준 번호는 **검증 가능한 1차 출처**에서 유래해야 하며, 그렇지 못한 경우 본문에서 제거하거나 일반 진술로 다운그레이드한다.

---

## 1. 정책의 배경

### 1.1 법률 위험 영역

| 위험 | 한국법 근거 | 발현 시 결과 |
|------|-------------|-------------|
| 허위 광고·표시 | 표시·광고의 공정화에 관한 법률 §3 | 시정명령·과징금 |
| 명예훼손 (가공 인용 → 실재 기관 피해) | 형법 §307, 정통망법 §70 | 형사 처벌 |
| 저작권 침해 (가공 인용으로 실재 논문 제목·저자명 도용) | 저작권법 §136 | 민형사 책임 |
| 의료기기 허위 임상 근거 | 의료기기법 §24, 식약처 광고 가이드 | 행정 처분·허가 취소 |
| 출판물 허위 인용 표시 | 출판문화산업 진흥법 §28 | 등록 취소 |

### 1.2 형님 핵심 원칙 (지속가능 도서)

- **"기술에는 만드는 사람의 온기가 담긴다"** — 가공된 사실은 온기가 아니라 거짓.
- **"Claude가 실제 전문가 정확도 담보"** (memory: `feedback_wia_standards_book_strategy.md`)
- **"불확실 기술 포인트는 웹서치 필수"**
- **"코드는 복붙 작동 보장"** — 인용도 같은 기준: 펴 들어 검증 가능해야 함.

---

## 2. 인용 등급 분류 (allow / generalize / forbid)

### 2.1 ✅ ALLOW — 본문/스펙에 자유롭게 사용

| 종류 | 형식 예시 |
|------|----------|
| 국제 표준 (ISO/IEC/IEEE/W3C/IETF/HL7) | `ISO 22523:2006 §5`, `RFC 9457`, `IEC 60601-1:2005+AMD2:2020` |
| 한국 KS·KS B 채택 표준 | `KS B ISO 13482:2019` |
| 정부 공식 법률·고시 (출처 명시) | `「나노기술개발 촉진법」(법률 제6812호, 2002-12 제정)` |
| 정부 공식 자료 (출처 URL/기관) | `식품의약품안전처 의료기기 허가 통계`, `보건복지부 장애인실태조사` |
| 공식 표준 위원회·산업단체 | `SENIAM Recommendations`, `OMG RTPS 2.5`, `OASIS CAP 1.2` |
| 코드 docstring·함수 명칭 | `Hudgins TD4 (MAV/ZC/SSC/WL)` — 명칭만 |
| 도구·제품의 공식 사양 (제조사 문서) | `Delsys Trigno IM (16 channels, 2 kHz)` |
| 자체 표준 사양 참조 | `WIA-MYO-DATA-001 §3.1` |

### 2.2 ⚠️ GENERALIZE — 일반 진술로 다운그레이드

| 원래 (금지) | 다운그레이드 (허용) |
|-------------|--------------------|
| `Hargrove et al. 2007 *IEEE TBME*` | `AR(4) 특징은 자기상관 구조를 인코딩하여 분류 정확도를 향상시키는 것으로 알려져 있다` |
| `KAIST 2024 IEEE TNSRE GCT 42% 단축` | `패턴 인식 제어는 단순 진폭 제어 대비 GCT 단축이 일반적으로 보고된다` |
| `서울대병원 2024 RCT n=42 VAS 38% 감소` | `Mirror visual feedback과 결합한 바이오피드백은 PLP 강도 감소에 활용된다` |
| `Bonawitz 2017 secure aggregation` | `secure-aggregation 프로토콜 (Shamir 비밀 분할 + DH 페어와이즈 마스킹)` |

### 2.3 🚫 FORBID — 본문/스펙에 절대 사용 금지

| 패턴 | 사유 |
|------|------|
| `Author et al. (Year)` 학술 논문 인용 | 검증 책임·도용 위험 |
| 검증 불가한 한국 임상 연구 ("○○대학교 ○○과 ○○년") | 명예훼손·허위 표시 |
| 가공된 정확한 수치 ("정확도 94.7%", "VAS 38% 감소", "처방 35% 증가") | 허위 광고 |
| 가공된 RCT/n수 ("RCT n=42", "n=87") | 의료기기 허위 임상 근거 |
| 검증 불가한 회사 부서명 ("동부팜한농 의료기기사업부") | 명예훼손 |
| 가공된 학회 발표 ("2025 ICRA", "2024 IROS") | 저작권·도용 위험 |

---

## 3. 자동 검출 룰 (machine-enforceable)

다음 grep 패턴 중 어느 하나라도 매칭되면 commit/Deep 승급 차단:

### 3.1 학술 인용 패턴 (R1)

```regex
et al\.? \(?(199[0-9]|20[0-2][0-9])
```

### 3.2 한국 기관 + 연도 인용 (R2 — 가공 의심)

```regex
(KAIST|서울대|연세|성모|아산|삼성|대한|ETRI|KIRO|KITECH|분당|국립)[^<]*\(20[12][0-9]\)
```

### 3.3 학회 발표 패턴 (R3)

```regex
(20[12][0-9])[ ]+(ICRA|IROS|NeurIPS|CVPR|ICCV|ECCV|CHI|UIST|SIGGRAPH|TBME|TNSRE|JNE|NEJM|JAMA|Lancet|Nature|Science)\b
```

### 3.4 가공 임상 수치 (R4)

```regex
(RCT|무작위 대조 시험|이중 맹검)[^<]*n=[0-9]+
```

### 3.5 면제 영역 (excluded paths)

자동 검출에서 제외되는 디렉토리:

```
api/   cli/   hardware/   integrations/   mobile-app/   models/   prompts/   schemas/
```

코드·구성·메타데이터 영역은 docstring 보존 가능. 단 문자열 내부 인용은 별도 점검.

---

## 4. 작업 워크플로우 (write → check → commit)

### 4.1 집필 단계

1. 새 챕터/스펙 작성 시 §2.1 ALLOW 영역만 사용.
2. 망설여지면 §2.2 GENERALIZE — 출처 명시 없이 일반 진술로.
3. 절대 §2.3 FORBID 패턴 사용 금지.

### 4.2 commit 전 사실성 게이트

```bash
# Required before every commit on standards/ content:
bash /var/www/wiastandards/handoff-scripts/check-citations.sh standards/<std>/

# Output:
#   ✅ PASS: 0 forbidden patterns found
#   ❌ FAIL: N forbidden patterns at lines ...
```

FAIL 시 commit 차단. §2.2 가이드대로 라인별 다운그레이드 후 재검사.

### 4.3 Deep 승급 게이트

```bash
bash /var/www/wiastandards/handoff-scripts/validate-published-v3.sh <std>

# v3 = v2(17개 사이즈/구조) + 사실성 4개 룰 = 21개 체크
# 21/21 통과해야 Deep Published 인정
```

### 4.4 Public sync 전제 조건

`Deep Published (v3 21/21)` 통과 표준만 `wia-standards-public` 반영. 위반 시 sync 거절.

---

## 5. 면책·예외 처리

### 5.1 표준 위원회 RFC/표준 문서 인용

ISO/IEC/IETF/W3C/HL7 같은 **공식 표준 발행기관 문서**는 §2.1 ALLOW. References 섹션에 다음 형식 사용:

```markdown
1. ISO 22523:2006 — External limb prostheses — Requirements and test methods.
2. IEC 60601-1:2005+AMD2:2020 — Medical electrical equipment, Part 1.
3. RFC 9457 — Problem Details for HTTP APIs.
```

### 5.2 정부 공개 통계 인용

원본 URL 또는 기관명·연도 명시 + 검증 가능한 형태:

```markdown
2022년 건강보험심사평가원 보조기기 청구 통계
(출처: HIRA 공개 자료실 https://opendata.hira.or.kr ...)
```

URL 또는 검색 가능한 보고서명 없이 "HIRA 통계 35% 증가" 만 적으면 §2.3 FORBID.

### 5.3 코드 주석 docstring

함수의 알고리즘 명칭은 보존 가능:

```python
def hudgins_td4(segment):
    """Hudgins TD4 — 4 time-domain features (MAV/ZC/SSC/WL)."""
```

연도·저자 인용 형태는 금지:

```python
# ❌ """Hudgins et al. 1993 4 time-domain features."""
# ✅ """Hudgins TD4 — MAV/ZC/SSC/WL."""
```

---

## 6. 거버넌스 및 변경 이력

### 6.1 정책 변경 절차

본 문서 변경은 다음 조건을 모두 만족해야 함:
1. WIA 운영진 승인 (형님 결정).
2. `pm-log.md`에 변경 요약 기록.
3. 변경 후 자동 검증기(`validate-published-v3.sh`) 룰 동시 업데이트.
4. 영향 받는 표준 전수 재검증.

### 6.2 변경 이력

| 버전 | 일자 | 변경 |
|------|------|------|
| 1.0 | 2026-04-25 | 최초 발행 (v3 deep QA 결과 반영) |

---

## 7. 부록 — 다운그레이드 치환 사전

| 원래 표현 | 다운그레이드 |
|----------|-------------|
| "Author et al. (YYYY)에 따르면 X 정확도가 N.N%" | "X는 일반적으로 ~ 수준의 정확도가 보고된다" |
| "2024 IEEE TNSRE 발표에 의하면..." | "최근 IEEE 학술 영역의 연구들은 ~를 시사한다" |
| "○○대학교 ○○과 (YYYY)" | "관련 분야 선행 연구" 또는 직접 제거 |
| "RCT n=42에서 N% 감소" | "임상 환경에서 ~ 효과가 활용된다" |
| "(저자 YYYY)" 본문 괄호 인용 | 괄호 통째 제거 + 일반 진술 |

---

*WIA 표준은 출판 도서이자 산업 표준 문서. 인용은 자유의 표현이 아니라 책임의 영역이다.*
*— WIA, 弘益人間 (홍익인간) Benefit All Humanity*
