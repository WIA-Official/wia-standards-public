# Phase 4 사전 조사 결과
# WIA Ecosystem Integration Research

---

**버전**: 1.0.0
**작성일**: 2025-12-13
**작성자**: WIA / SmileStory Inc.

---

## 개요

Phase 4에서는 AAC 센서 입력을 통해 생성된 텍스트를 다양한 출력 방식으로 변환하여
WIA 생태계(TTS, ISP/WIA Talk, WIA Braille)와 연동하는 표준을 정의합니다.

이 문서는 해당 기술들에 대한 조사 결과를 정리합니다.

---

## 1. TTS (Text-to-Speech) 기술 조사

### 1.1 Web Speech API (브라우저 내장)

#### 개요
- **표준**: W3C Web Speech API Specification (Draft)
- **지원**: Chrome, Edge, Safari, Firefox (부분)
- **구성**: SpeechSynthesis + SpeechRecognition

#### 주요 인터페이스
```typescript
// 브라우저 내장 TTS
const synth = window.speechSynthesis;
const utterance = new SpeechSynthesisUtterance(text);

utterance.lang = 'ko-KR';       // 언어
utterance.rate = 1.0;           // 속도 (0.1 ~ 10)
utterance.pitch = 1.0;          // 음높이 (0 ~ 2)
utterance.volume = 1.0;         // 볼륨 (0 ~ 1)
utterance.voice = voices[0];    // 음성 선택

synth.speak(utterance);
```

#### 장점
- 무료, 오프라인 지원
- 추가 라이브러리 불필요
- 모든 주요 브라우저 지원 (2025 기준)

#### 단점
- 음성 품질이 OS/브라우저에 따라 다름
- 고급 음성(Neural TTS) 미지원
- 음성 커스터마이징 제한

#### AAC 적용
- **권장**: 기본 TTS 옵션으로 사용
- 오프라인 동작 필수인 AAC 기기에 적합
- 추가 비용 없음

#### 참고
- [MDN Web Docs: SpeechSynthesis](https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesis)
- [Web Speech API Specification](https://webaudio.github.io/web-speech-api/)

---

### 1.2 Google Cloud Text-to-Speech

#### 개요
- **서비스**: Google Cloud Platform
- **음성 종류**: Standard, WaveNet, Neural2
- **지원 언어**: 50개 이상

#### 주요 기능
- SSML (Speech Synthesis Markup Language) 지원
- 다양한 출력 형식 (MP3, LINEAR16, OGG)
- 음성 속도, 피치 조절 가능

#### Node.js 예제
```javascript
const textToSpeech = require('@google-cloud/text-to-speech');
const client = new textToSpeech.TextToSpeechClient();

const request = {
  input: { text: '안녕하세요' },
  voice: { languageCode: 'ko-KR', ssmlGender: 'NEUTRAL' },
  audioConfig: { audioEncoding: 'MP3' },
};

const [response] = await client.synthesizeSpeech(request);
```

#### 장점
- 고품질 Neural 음성
- 다양한 언어 및 음성 옵션
- SSML로 세밀한 제어 가능

#### 단점
- 유료 (월 무료 한도 존재)
- 인터넷 연결 필수
- 지연 시간 발생

#### AAC 적용
- **선택적**: 고품질 음성이 필요한 경우
- 실시간 소통보다 녹음/저장 용도에 적합

#### 참고
- [Google Cloud TTS Documentation](https://cloud.google.com/text-to-speech/docs/libraries)
- [NPM: @google-cloud/text-to-speech](https://www.npmjs.com/package/@google-cloud/text-to-speech)

---

### 1.3 기타 클라우드 TTS

#### Amazon Polly
- AWS 제공
- Neural TTS 지원
- SSML 지원

#### Microsoft Azure Speech Service
- Azure Cognitive Services
- Custom Neural Voice 가능
- 다국어 지원

#### 비교 요약

| 서비스 | 품질 | 비용 | 오프라인 | 지연시간 |
|-------|------|------|---------|---------|
| Web Speech API | 중 | 무료 | 가능 | 최소 |
| Google Cloud TTS | 상 | 유료 | 불가 | 100-300ms |
| Amazon Polly | 상 | 유료 | 불가 | 100-300ms |
| Azure Speech | 상 | 유료 | 불가 | 100-300ms |

---

## 2. 수어 아바타 기술 조사

### 2.1 오픈소스 프로젝트

#### MMS Player
- **개발**: DFKI (German Research Center for AI)
- **라이선스**: GPL-3.0
- **기술**: Blender 기반 Python 스크립트
- **입력**: MMS (MultiModal Signstream) 형식

주요 특징:
- 병렬 수어 실행 지원
- 타이밍 및 굴절 정보 포함
- HTTP API 또는 CLI 호출
- 비디오 렌더링 또는 3D 포맷 내보내기

#### SignAvatars Dataset
- **발표**: ECCV 2024
- **규모**: 70,000 비디오, 153명 서명자, 834만 프레임
- **특징**: 고립 수어 + 연속 수어 포함
- **GitHub**: [ZhengdiYu/SignAvatars](https://github.com/ZhengdiYu/SignAvatars)

#### Sign-Kit (Indian Sign Language)
- 웹 기반 툴킷
- 음성 → ISL 제스처 변환
- 3D 아바타 애니메이션
- 정확도: 4.87/5.0 (사용자 설문)

#### Sign Language Mocap Archive
- Blender 기반 (무료)
- FBX, GLTF, USD 포맷 지원
- 모션 캡처 데이터 아카이브

### 2.2 상용 솔루션

#### SiMAX
- 3D 애니메이션 아바타 시스템
- 다양한 수어 지원
- 텍스트 → 수어 변환

### 2.3 기술적 고려사항

#### GAN 기반 vs 3D 아바타

| 방식 | 장점 | 단점 |
|------|------|------|
| GAN 비디오 | 현실적 외형 | 손가락 환각, 수정 불가 |
| 3D 아바타 | 정확한 제어, 수정 가능 | 렌더링 비용 |

3D 아바타가 AAC 용도에 더 적합:
- 손가락 움직임 정확도 중요
- 수정/커스터마이징 필요
- AR/VR 환경 확장 가능

### 2.4 ISP 기반 구현 방향

WIA Talk 문서 기반으로 다음 구현:

```
텍스트 입력 → ISP 코드 매핑 → 3D 아바타 애니메이션

ISP 코드 형식:
HS##-LC##-MV##-OR##-NM##

예시:
"안녕" → HS01-LC01-MV01-OR01-NM01
```

#### 구현 단계
1. 텍스트 → 단어 분리
2. 단어 → ISP 코드 매핑 (사전 활용)
3. ISP 코드 → 제스처 파라미터 변환
4. 파라미터 → 3D 아바타 애니메이션

#### 참고
- [MMS Player GitHub](https://github.com/DFKI-SignLanguage/MMS-Player)
- [SignAvatars Project](https://signavatars.github.io/)

---

## 3. 점자 출력 기술 조사

### 3.1 Liblouis

#### 개요
- **목적**: 오픈소스 점자 번역기
- **명명**: Louis Braille 경의
- **라이선스**: LGPL

#### 주요 기능
- 컴퓨터 점자 및 문학 점자 지원
- 축약/비축약 점자 변환
- 다국어 지원 (규칙/사전 기반)
- 수학 점자 지원 (Nemeth, Marburg)

#### 사용처
- NVDA (Windows 스크린 리더)
- Orca (Linux 스크린 리더)
- BrailleBack (Android)
- JAWS (상용 스크린 리더)
- Bookshare (35만+ 도서)

#### Python 바인딩
```python
import louis

# 텍스트 → 점자 변환
text = "Hello World"
braille = louis.translateString(
    ["en-us-g2.ctb"],  # 번역 테이블
    text
)
print(braille)  # ⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙
```

#### 장점
- 무료, 오픈소스
- 다양한 언어/점자 체계 지원
- 검증된 번역 품질

#### 단점
- C 라이브러리 (바인딩 필요)
- JavaScript 공식 지원 없음 (WASM 필요)

#### 참고
- [Liblouis 공식 사이트](https://liblouis.io/)
- [GitHub: liblouis/liblouis](https://github.com/liblouis/liblouis)

---

### 3.2 BRLTTY / BrlAPI

#### 개요
- **BRLTTY**: 점자 디스플레이 데몬
- **BrlAPI**: 프로그래밍 인터페이스

#### 지원 기기
- 대부분의 상용 점자 디스플레이
- USB, Bluetooth, Serial 연결

#### 기능
- 화면 내용 → 점자 변환
- 점자 디스플레이 입력 처리
- 커서 라우팅

#### 프로그래밍 예제
```c
#include <brlapi.h>

brlapi_handle_t *handle;
handle = brlapi_openConnection(NULL, NULL);
brlapi_writeText(0, "Hello");
brlapi_closeConnection(handle);
```

#### 참고
- [BRLTTY 공식 사이트](https://brltty.app/)

---

### 3.3 WIA Braille 연동 방향

#### IPA 기반 변환 흐름
```
텍스트 "안녕"
    ↓
발음 변환 (IPA): /annjʌŋ/
    ↓
WIA Braille 매핑: ⠁⠝⠚⠪⠝
    ↓
점자 디스플레이 출력
```

#### 구현 전략
1. **텍스트 → IPA 변환**: G2P (Grapheme-to-Phoneme) 라이브러리
2. **IPA → 점자 변환**: WIA Braille 매핑 테이블
3. **점자 → 디스플레이 출력**: BrlAPI 또는 Mock

---

## 4. 결론 및 권장 사항

### 4.1 TTS 권장
```
Primary: Web Speech API
- 무료, 오프라인 지원
- AAC 기기 필수 요구사항 충족

Secondary: Google Cloud TTS (선택적)
- 고품질 음성 필요 시
- 인터페이스 확장성 고려
```

### 4.2 ISP/수어 아바타 권장
```
구현 방식: Mock Adapter + Interface 정의
- 실제 3D 아바타는 별도 프로젝트로 분리
- ISP 코드 변환 로직만 구현
- 향후 MMS Player 또는 자체 아바타 연동

ISP 코드 체계: WIA Talk 문서 참조
- 93개 핵심 제스처
- 5대 요소 (HS, LC, MV, OR, NM)
```

### 4.3 WIA Braille 권장
```
구현 방식: Mock Adapter + IPA 변환
- 실제 점자 디스플레이는 BrlAPI로 확장
- IPA 기반 점자 매핑 테이블 구현

변환 체인:
텍스트 → IPA → 점자 유니코드 → 출력
```

### 4.4 전체 아키텍처

```
┌─────────────────────────────────────┐
│           OutputManager              │
│   (통합 출력 관리자)                 │
├─────────┬─────────┬─────────────────┤
│   TTS   │   ISP   │    Braille      │
│ Adapter │ Adapter │    Adapter      │
├─────────┼─────────┼─────────────────┤
│ Web     │ ISP     │ IPA → 점자      │
│ Speech  │ Code    │ 매핑            │
│ API     │ 매핑    │                 │
└─────────┴─────────┴─────────────────┘
```

---

## 5. 참고문헌

### TTS
1. MDN Web Docs: Web Speech API
   - https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API
2. Google Cloud Text-to-Speech Documentation
   - https://cloud.google.com/text-to-speech/docs

### 수어 아바타
3. MMS Player: Open Source Sign Language Animation
   - https://github.com/DFKI-SignLanguage/MMS-Player
4. SignAvatars: ECCV 2024
   - https://signavatars.github.io/
5. Sign-Kit: ISL Toolkit
   - https://github.com/spectre900/Sign-Kit-An-Avatar-based-ISL-Toolkit

### 점자
6. Liblouis: Open Source Braille Translator
   - https://liblouis.io/
7. BRLTTY: Braille TTY
   - https://brltty.app/

### WIA 생태계
8. ISP Scientific Foundation - 내부 문서
9. WIA Braille Scientific Foundation - 내부 문서
10. WIA Talk Scientific Foundation - 내부 문서

---

<div align="center">

**Phase 4 Research Complete**

다음 단계: `/spec/PHASE-4-INTEGRATION.md` 작성

</div>
