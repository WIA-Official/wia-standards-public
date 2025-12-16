# WIA-AIR-POWER

> "충전"이라는 단어가 사라지는 세상
>
> 삼촌처럼 힘을 나눠주는 표준
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## 가족 관계

```
아버지 (WIA-INTENT): 의도를 표현해
어머니 (WIA-OMNI-API): 내가 다 품어줄게
삼촌 (WIA-AIR-POWER): 내가 힘 나눠줄게 💪
```

## 비전

```
현재: "배터리 몇 %야?" "충전해야 해" "케이블 어딨어?"
     ↓
미래: 항상 만땅. 그런 걱정 없음.
     "충전"이라는 단어가 사라짐.
```

## 작동 원리

```
WiFi 라우터 ─────→ 📱 자동 충전
(TX, 삼촌)         💻 자동 충전
                  ⌚ 자동 충전
                  🎧 자동 충전
                  (RX, 조카들)
```

## 사용법

```typescript
import { startCharging, getBatteryStatus } from '@wia/air-power';

// 충전 시작 (주변 TX 자동 탐색)
await startCharging();

// 상태 확인
const status = getBatteryStatus();
console.log(`배터리: ${status.level}%, 충전 중: ${status.charging}`);
```

## Power Classes

| Class | 전력 | 기기 |
|-------|------|------|
| A | 0-100mW | 센서, 태그 |
| B | 100mW-2W | 워치, 이어버드 |
| C | 2W-15W | 스마트폰, 태블릿 |
| D | 15W-100W | 노트북 |
| E | 100W+ | 가전 |

## 로드맵

```
2025-2026: 표준 확립, 파일럿
2027-2028: 상용 제품 출시
2029-2030: "충전" 개념 소멸
```

---

**WIA - World Certification Industry Association**
**삼촌처럼 힘을 나눠주는 세상**
