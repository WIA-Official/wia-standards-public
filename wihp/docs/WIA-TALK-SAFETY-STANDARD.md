# WIA Talk 安全 基準書
## 지문 보호 & 문화적 안전 표준

**문서번호**: WIA-TALK-SAFETY-001
**버전**: 1.0
**작성일**: 2025-12-13
**저자**: 세계인증산업협회(WIA) 연삼흠 박사

---

## 제1장: 지문 보호 원칙 (Fingerprint Protection)

### 1.1 배경

현대 기술로 사진/영상에서 지문 추출이 가능함.
- 고해상도 카메라로 촬영된 손바닥에서 지문 패턴 추출 가능
- 셀카, SNS 사진에서도 지문 도용 위험 존재
- WIA Talk은 전 세계 사용자의 개인정보를 보호해야 함

### 1.2 핵심 원칙

**모든 WIA Talk 제스처는 지문이 보이지 않는 방향으로 설계한다.**

### 1.3 허용 방향

| 방향 | 허용 | 설명 |
|------|------|------|
| 손등 방향 (Back of hand) | ✅ | 기본 권장 |
| 측면 45도 (Side angle) | ✅ | 손등 일부 보임 |
| 손끝만 보임 (Fingertips only) | ✅ | 지문 영역 아님 |
| 손바닥 전체 (Full palm) | ❌ | 지문 노출 |
| 손바닥 정면 (Palm forward) | ❌ | 지문 노출 |

### 1.4 예외 처리

손바닥이 반드시 필요한 제스처 (예: 받는 손, 합장):
- **라인아트 스타일**로 지문 텍스처 생략
- **단순화된 선**으로 손금만 최소 표현
- 프롬프트에 명시: "no fingerprint details, simplified palm lines only"

---

## 제2장: 문화적 안전 원칙 (Cultural Safety)

### 2.1 배경

동일한 제스처가 문화권에 따라 다른 의미를 가짐.
WIA Talk은 211개 언어, 전 세계 사용자를 대상으로 함.

### 2.2 위험 제스처 목록 및 대응

| 제스처 | 위험 지역 | 위험 의미 | WIA Talk 대응 |
|--------|-----------|-----------|---------------|
| V-Sign (손등 방향) | 영국, 호주, 뉴질랜드 | 심한 욕설 | **손바닥 방향으로 변경** |
| OK Sign (👌) | 브라질 | 성적 모욕 | **"완성/동그라미"로 중립 명명** |
| OK Sign (👌) | 일부 서양 | 혐오 상징 오용 | **학술적 맥락 강조** |
| 엄지척 (👍) | 중동, 서아프리카 | 모욕 | **사용 자제, 대안 제시** |
| 검지 포인팅 | 중동, 아시아 일부 | 무례함 | **위쪽 방향으로 제한** |
| 주먹 (✊) | 전세계 | 공격/저항 | **"자연스러운 휴식" 강조** |
| 손바닥 앞으로 | 그리스 | 모욕 (무츠) | **측면 각도로 변경** |

### 2.3 안전한 제스처 설계 원칙

1. **중립적 명명**: 문화적 의미 대신 형태 기반 명명
   - ❌ "OK Sign" → ✅ "동그라미/Circle"
   - ❌ "Peace Sign" → ✅ "V자/V-Shape"

2. **방향 표준화**: 가장 안전한 방향 선택
   - V-Sign: 손바닥 방향 (손등 ❌)
   - 정지: 측면 45도 (정면 손바닥 ❌)

3. **맥락 설명**: 교재에 학술적/언어학적 목적 명시

---

## 제3장: 이미지 생성 표준

### 3.1 DALL-E 프롬프트 필수 문구

모든 프롬프트에 다음 문구 포함:

```
no fingerprint details visible,
simplified line art style,
back of hand or side angle preferred,
cultural-neutral gesture representation
```

### 3.2 손 방향별 프롬프트 템플릿

**손등 방향 (기본)**:
```
back of hand facing viewer at slight angle,
fingers clearly visible from behind,
no palm or fingerprint details shown
```

**측면 방향 (대안)**:
```
hand viewed from 45-degree side angle,
edge of hand toward viewer,
fingertips visible but palm hidden
```

**손바닥 필수 시 (예외)**:
```
palm facing upward in stylized illustration,
no fingerprint texture or palm line details,
minimalist clean line art only
```

---

## 제4장: 적용 범위

이 기준은 다음 모든 파일에 적용:

1. **교재**: WIA-Talk-Jeongeum.md
2. **언어별 배치**: batch-01 ~ batch-14
3. **DALL-E 프롬프트**: DALLE-PROMPTS-WIA-TALK.md
4. **이미지 파일**: /docs/images/*.png
5. **웹 페이지**: /wia-talk/
6. **학습 게임**: (향후 개발)

---

## 제5장: 버전 관리

| 버전 | 날짜 | 변경 내용 |
|------|------|-----------|
| 1.0 | 2025-12-13 | 최초 작성 |

---

*홍익인간 (弘益人間) - 널리 인간을 이롭게*
*세계인증산업협회(WIA) 연삼흠 박사*
