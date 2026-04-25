# WIA Talk 테스트 시나리오
**Part 7: 성공 기준 (테스트 시나리오)**

> 제미나이도 인정하는 완벽한 테스트 커버리지

---

## 🎯 **테스트 목표**

### **기능 테스트:** 모든 기능이 정상 작동
### **성능 테스트:** 목표 성능 달성
### **접근성 테스트:** WCAG 2.1 AAA 준수
### **사용성 테스트:** 실제 사용자 시나리오

---

## 📋 **Test Suite 1: 농인 → 맹인 통신**

### **TS-1.1: 제스처 인식 → 음성 출력**

**전제 조건:**
- 카메라 연결됨
- 제스처 인식 시작됨

**테스트 단계:**
1. "안녕하세요" 제스처 수행
2. 제스처 인식 확인 (gestureResult 표시)
3. 텍스트 변환 확인 (textOutput: "안녕하세요")
4. IPA 변환 확인 (ipaOutput: "/annjʌŋhasɛjo/")
5. 점자 변환 확인 (brailleOutput: "⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕")
6. 음성 출력 확인 (TTS 재생)

**예상 결과:**
- ✅ 제스처 인식: <50ms
- ✅ 전체 변환: <200ms
- ✅ 음성 재생: 즉시
- ✅ 정확도: 95%+

**성공 기준:**
```javascript
assert(recognition.gesture.korean === '안녕하세요');
assert(recognition.confidence > 0.7);
assert(result.braille.includes('⠁⠝⠝'));
assert(speechSynthesis.speaking === true);
```

---

### **TS-1.2: 연속 제스처 인식**

**테스트 단계:**
1. "안녕하세요" 제스처
2. 2초 대기 (cooldown)
3. "감사합니다" 제스처
4. 2초 대기
5. "미안합니다" 제스처

**예상 결과:**
- ✅ 3개 제스처 모두 인식
- ✅ 각 제스처마다 음성 출력
- ✅ UI 업데이트 정상

**성공 기준:**
```javascript
assert(metrics.totalGestures === 3);
assert(metrics.totalConversions === 3);
```

---

## 📋 **Test Suite 2: 맹인 → 농인 통신**

### **TS-2.1: 음성 인식 → 텍스트 표시**

**전제 조건:**
- 마이크 연결됨
- 음성 인식 시작됨

**테스트 단계:**
1. "안녕하세요" 말하기
2. 중간 결과 확인 (speechTranscript)
3. 최종 결과 확인 (confidence > 0.8)
4. 텍스트 표시 확인 (textOutput)
5. 점자 표시 확인 (brailleOutput)

**예상 결과:**
- ✅ 음성 인식: <500ms
- ✅ 정확도: 90%+
- ✅ 텍스트 표시: 즉시
- ✅ 점자 변환: <50ms

**성공 기준:**
```javascript
assert(result.text === '안녕하세요');
assert(result.confidence > 0.8);
assert(state.output.braille !== '');
```

---

### **TS-2.2: 연속 음성 인식**

**테스트 단계:**
1. "안녕하세요" (pause)
2. "오늘 날씨가 좋네요" (pause)
3. "고맙습니다"

**예상 결과:**
- ✅ 3개 문장 모두 인식
- ✅ 각 문장마다 점자 변환
- ✅ Auto-restart 작동

**성공 기준:**
```javascript
assert(metrics.totalSpeechRecognitions === 3);
assert(state.input.speech.listening === true);
```

---

## 📋 **Test Suite 3: 양방향 동시 사용**

### **TS-3.1: 제스처 + 음성 동시 활성화**

**테스트 단계:**
1. 제스처 인식 시작
2. 음성 인식 시작
3. 농인: "안녕하세요" 제스처
4. 맹인: "반갑습니다" 말하기
5. 농인: "감사합니다" 제스처

**예상 결과:**
- ✅ 두 입력 모두 정상 작동
- ✅ 충돌 없음
- ✅ UI 업데이트 정확

**성공 기준:**
```javascript
assert(state.input.gesture.active === true);
assert(state.input.speech.listening === true);
assert(metrics.totalGestures > 0);
assert(metrics.totalSpeechRecognitions > 0);
```

---

## 📋 **Test Suite 4: 점자 역변환**

### **TS-4.1: 점자 → 텍스트 변환**

**전제 조건:**
- BrailleReverseConverter 초기화됨

**테스트 단계:**
1. 점자 입력: "⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕"
2. IPA 변환 확인
3. 한글 변환 확인

**예상 결과:**
- ✅ IPA: "/annjʌŋhasɛjo/"
- ✅ 한글: "안녕하세요"
- ✅ 변환 시간: <100ms

**성공 기준:**
```javascript
const result = converter.convert('⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕');
assert(result.text === '안녕하세요');
assert(result.duration < 100);
assert(result.confidence > 0.95);
```

---

## 📋 **Test Suite 5: UI/UX**

### **TS-5.1: 테마 변경**

**테스트 단계:**
1. High Contrast 선택
2. Dark 선택
3. Light 선택

**예상 결과:**
- ✅ 각 테마 즉시 적용
- ✅ CSS 변수 변경 확인
- ✅ 대비 비율 4.5:1 이상

**성공 기준:**
```javascript
assert(document.documentElement.getAttribute('data-theme') === 'dark');
const bgColor = getComputedStyle(document.body).backgroundColor;
assert(contrastRatio(bgColor, textColor) >= 4.5);
```

---

### **TS-5.2: 글꼴 크기 조절**

**테스트 단계:**
1. Small (16px) 선택
2. Medium (20px) 선택
3. Large (28px) 선택

**예상 결과:**
- ✅ 글꼴 크기 즉시 변경
- ✅ 레이아웃 깨짐 없음

**성공 기준:**
```javascript
assert(document.documentElement.getAttribute('data-font-size') === 'large');
const fontSize = getComputedStyle(document.body).fontSize;
assert(fontSize === '28px');
```

---

### **TS-5.3: 음성 속도 조절**

**테스트 단계:**
1. 슬라이더를 0.7로 조정
2. 텍스트 재생
3. 슬라이더를 1.5로 조정
4. 텍스트 재생

**예상 결과:**
- ✅ 속도 변경 즉시 반영
- ✅ UI 표시 동기화

**성공 기준:**
```javascript
app.state.ui.speechRate = 1.5;
app.speak();
// TTS utterance.rate === 1.5
```

---

## 📋 **Test Suite 6: 키보드 단축키**

### **TS-6.1: Space - 음성 인식 토글**

**테스트 단계:**
1. Space 키 누름 (시작)
2. 음성 인식 상태 확인
3. Space 키 다시 누름 (중지)

**예상 결과:**
- ✅ 토글 정상 작동
- ✅ 상태 표시 업데이트

---

### **TS-6.2: Ctrl+R - 마지막 텍스트 재생**

**테스트 단계:**
1. 텍스트 변환 완료
2. Ctrl+R 누름
3. TTS 재생 확인

**예상 결과:**
- ✅ 마지막 텍스트 재생
- ✅ 음성 출력 정상

---

### **TS-6.3: Ctrl+↑/↓ - 음성 속도 조절**

**테스트 단계:**
1. 초기 속도: 1.0x
2. Ctrl+↑ 3번 누름
3. 속도 확인: 1.3x
4. Ctrl+↓ 5번 누름
5. 속도 확인: 0.8x (최소 0.5)

**예상 결과:**
- ✅ 속도 조절 정상
- ✅ 범위 제한 (0.5 ~ 2.0)

---

### **TS-6.4: Esc - 전체 중지**

**테스트 단계:**
1. 제스처 + 음성 모두 활성화
2. TTS 재생 중
3. Esc 누름

**예상 결과:**
- ✅ 모든 입력 중지
- ✅ TTS 중지
- ✅ UI 상태 초기화

---

## 📋 **Test Suite 7: 접근성 (Accessibility)**

### **TS-7.1: 스크린 리더 테스트 (NVDA/JAWS)**

**테스트 항목:**
- [ ] Skip links 작동
- [ ] ARIA 라벨 읽기
- [ ] Form controls 접근
- [ ] Live regions 업데이트 알림
- [ ] 모든 버튼 label 읽기

**도구:** NVDA (Windows), JAWS, VoiceOver (Mac)

---

### **TS-7.2: 키보드 탐색 테스트**

**테스트 항목:**
- [ ] Tab으로 모든 요소 접근 가능
- [ ] 포커스 순서 논리적
- [ ] 포커스 표시 명확 (outline)
- [ ] Enter/Space로 버튼 활성화
- [ ] Esc로 모달 닫기

---

### **TS-7.3: Lighthouse 접근성 점수**

**목표:** 100점

**체크리스트:**
- [ ] Contrast ratio ≥ 4.5:1
- [ ] All images have alt text
- [ ] Form elements have labels
- [ ] ARIA attributes valid
- [ ] Page structure semantic

---

## 📋 **Test Suite 8: 성능 (Performance)**

### **TS-8.1: 변환 속도 벤치마크**

**테스트:**
```javascript
// 100번 반복 테스트
for (let i = 0; i < 100; i++) {
  const start = performance.now();
  await app._convertTextToBraille('안녕하세요');
  const duration = performance.now() - start;

  assert(duration < 50, `Too slow: ${duration}ms`);
}
```

**목표:**
- ✅ 평균: <50ms
- ✅ 최대: <100ms
- ✅ 최소: <10ms

---

### **TS-8.2: 메모리 사용량**

**테스트:**
1. 1시간 연속 사용
2. Chrome DevTools Memory 프로파일링
3. 메모리 누수 확인

**목표:**
- ✅ 메모리 증가 <100MB/hour
- ✅ GC 정상 작동
- ✅ 누수 없음

---

### **TS-8.3: FPS (60 FPS 유지)**

**테스트:**
1. 제스처 인식 중 FPS 측정
2. Chrome DevTools Performance 탭
3. 60 FPS 유지 확인

**목표:**
- ✅ 평균 FPS: 60
- ✅ 최소 FPS: 55+

---

## 📋 **Test Suite 9: 에러 처리**

### **TS-9.1: 카메라 없음**

**시나리오:** 카메라가 연결되지 않았거나 권한 거부

**예상 동작:**
- ✅ 에러 메시지 표시
- ✅ Fallback: 수동 입력 모드 제안
- ✅ 앱 크래시 없음

---

### **TS-9.2: 마이크 없음**

**시나리오:** 마이크 없거나 권한 거부

**예상 동작:**
- ✅ 에러 메시지 표시
- ✅ Fallback: 키보드 입력 제안
- ✅ 앱 크래시 없음

---

### **TS-9.3: 네트워크 오류**

**시나리오:** MediaPipe CDN 로드 실패

**예상 동작:**
- ✅ 에러 메시지 표시
- ✅ 재시도 버튼 제공
- ✅ 앱 크래시 없음

---

### **TS-9.4: 음성 인식 실패**

**시나리오:** "no-speech" 에러 (5초간 침묵)

**예상 동작:**
- ✅ 에러 메시지 표시
- ✅ Auto-restart (1초 후)
- ✅ 진동 피드백

---

## 📋 **Test Suite 10: 크로스 브라우저**

### **TS-10.1: Chrome (권장)**
- [ ] 제스처 인식
- [ ] 음성 인식 (Web Speech API)
- [ ] TTS
- [ ] UI/UX

### **TS-10.2: Edge**
- [ ] 제스처 인식
- [ ] 음성 인식
- [ ] TTS
- [ ] UI/UX

### **TS-10.3: Firefox**
- [ ] 제스처 인식
- [ ] ⚠️ 음성 인식 (미지원)
- [ ] TTS
- [ ] UI/UX

### **TS-10.4: Safari**
- [ ] 제스처 인식
- [ ] ⚠️ 음성 인식 (미지원)
- [ ] TTS (부분 지원)
- [ ] UI/UX

---

## ✅ **성공 기준 요약**

### **기능:**
- ✅ 모든 Test Suite 통과
- ✅ 에러율 <1%
- ✅ 정확도 >90%

### **성능:**
- ✅ 제스처 인식: <50ms
- ✅ 음성 인식: <500ms
- ✅ 텍스트→점자: <50ms
- ✅ 점자→텍스트: <100ms
- ✅ 전체 E2E: <1s

### **접근성:**
- ✅ Lighthouse 접근성 점수: 100
- ✅ WCAG 2.1 AAA 준수
- ✅ 스크린 리더 100% 호환

### **사용성:**
- ✅ 실제 사용자 테스트 통과
- ✅ SUS (System Usability Scale) >80
- ✅ 피드백 반영

---

## 🎯 **최종 검수 체크리스트**

- [ ] 모든 Test Suite (1-10) 통과
- [ ] 문서화 완료
- [ ] README.md 업데이트
- [ ] Git commit 정리
- [ ] wia.live 도메인 배포 준비
- [ ] 형님 최종 승인 ✅

---

**"제미나이도 GPT도 무시 못하는 완벽한 시스템!"**

Philosophy: 弘益人間 - 2000% 완성도 달성!
