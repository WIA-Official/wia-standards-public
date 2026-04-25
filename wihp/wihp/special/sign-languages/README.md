# WIHP Sign Language Mappings

## 개요

수어(Sign Language)는 음성 언어가 아닌 **시각-공간 언어**이므로, IPA(국제음성기호)가 아닌
별도의 표기 체계를 사용합니다.

## 표기 체계

WIHP 수어 매핑은 다음 체계를 활용합니다:

### 1. HamNoSys (Hamburg Notation System)
- 수어의 음운론적 요소를 ASCII로 표기
- 손 모양, 위치, 움직임, 표정 등 인코딩

### 2. SignWriting
- 시각적 아이콘 기반 표기
- 손 모양과 움직임을 그림으로 표현

### 3. WIHP 수어 코드 체계

```
HS##-LO##-MV##-OR##-NM##
```

| 코드 | 의미 | 설명 |
|------|------|------|
| HS## | Handshape | 손 모양 (01-99) |
| LO## | Location | 수어 공간 위치 |
| MV## | Movement | 움직임 유형 |
| OR## | Orientation | 손바닥 방향 |
| NM## | Non-manual | 비수지 신호 (표정 등) |

## 수어 목록

### 동아시아
- **KSL** (한국수어) - `ksl-korean-sign.md`
- **JSL** (일본수어) - `jsl-japanese-sign.md`
- **CSL** (중국수어) - 예정

### 서양
- **ASL** (미국수어) - `asl-american-sign.md`
- **BSL** (영국수어) - 예정
- **LSF** (프랑스수어) - 예정

## 한글 전사

수어를 한글로 전사할 때는 다음 규칙을 따릅니다:

1. **손 모양**: 한국 수어 지문자 참조
2. **움직임**: 화살표 기호 또는 설명
3. **위치**: 신체 부위 한글명

예시:
```
ASL "HELLO" → [B손-이마-밖으로]→ 헬로
KSL "안녕" → [펼친손-턱-앞으로흔들기] → 안녕
```

## 참고 자료

- 국립국어원 한국수어사전
- ASL Signbank
- Spread the Sign (다국어 수어 사전)
