# WIA-HOME

## 가족의 집 - "건물주 없는 내 집" 🏠

WIA 대가족의 든든한 울타리

---

## WIA Family

```
WIA 대가족이 사는 집:

┌─────────────────────────────────────────┐
│              🏠 WIA-HOME                │
│         가족 모두가 사는 집              │
├─────────────────────────────────────────┤
│  👨 아버지 (INTENT)    - 집 설계        │
│  👩 어머니 (OMNI-API)  - 손님 맞이      │
│  💪 삼촌 (AIR-POWER)   - 전기 공급      │
│  🛡️ 이모 (AIR-SHIELD)  - 집 보안        │
│  🌐 조카 (SOCIAL)      - 이웃 연결      │
│  🏠 집 (HOME)          - 모두의 보금자리 │
└─────────────────────────────────────────┘
```

---

## Why?

```
홈페이지 하나 만들려면...

💸 도메인: $10-50/년
💸 호스팅: $5-100/월
💸 SSL: $0-200/년
💸 개발: ???

소상공인?
→ 네이버, 카카오에 종속
→ 플랫폼이 문 닫으면 끝
→ 월세 내면서 디지털 세상 살기

이제 그만!

🏠 WIA-HOME이 있으면:
→ 내 PC가 서버 (0원)
→ 도메인 무료 (yourname.wia.home)
→ 영원히 내 것
```

---

## Installation

```bash
npm install @anthropic/wia-home
```

---

## Quick Start (5분만에 홈페이지!)

```typescript
import { WIAHome } from '@anthropic/wia-home';

// 1. 경쟁 사이트 벤치마킹으로 빠르게 시작
const mySite = await WIAHome.quickStart({
  benchmark: 'https://nice-bakery-site.com',
  myInfo: {
    name: '행복한 빵집',
    phone: '010-1234-5678',
    address: '서울시 강남구 123',
    hours: '09:00-21:00'
  }
});

await mySite.start();
// 🏠 happy-bakery.wia.shop 오픈!
```

---

## Features

### 1. Clone & Customize (벤치마킹)

```typescript
// 마음에 드는 사이트 분석
const analysis = await WIAHome.analyze('https://competitor.com');
console.log('기능:', analysis.features);
console.log('구조:', analysis.structure);

// 내 버전으로 클론
const mySite = await WIAHome.clone('https://competitor.com', {
  customize: {
    name: '내 가게',
    logo: myLogo,
    colors: { primary: '#FF6B6B' }
  }
});
```

### 2. Intent-Based (말로 만들기)

```typescript
// 아버지(INTENT) 연동 - 말만 하면 됨
const mySite = await WIAHome.fromIntent(`
  나는 동네 빵집을 운영해.
  빵 종류랑 가격 보여주고
  예약 받을 수 있게 해줘.
`);

await mySite.start();
// 👨 아버지가 설계해서 집 완성!
```

### 3. Template (템플릿)

```typescript
// 업종별 템플릿
const mySite = await WIAHome.fromTemplate('bakery');
// 또는: 'restaurant', 'cafe', 'hairSalon', 'clinic', 'academy'...

await mySite.start();
```

### 4. Migration (기존 사이트 이전)

```typescript
// WordPress에서 탈출!
const mySite = await WIAHome.migrate({
  from: 'wordpress',
  url: 'https://my-expensive-site.com'
});

// 이제 호스팅비 0원!
await mySite.start();
```

---

## P2P = No Landlord (건물주 없음)

```
기존 방식:
사용자 → AWS/클라우드 → 웹사이트
         ↑
      월세 지불 💸

WIA-HOME 방식:
방문자 → 내 PC/노트북 ← 내 집! 🏠
         ↑
      P2P Network
      건물주 없음 ✓
```

```typescript
// 내 노트북이 서버!
const home = new WIAHome({ name: '내 가게' });
await home.start();

// 상태 확인
console.log(`🏠 ${home.domain} 실행 중`);
console.log(`👥 오늘 방문자: ${home.stats.visitors.today}명`);
console.log(`💻 CPU: ${home.stats.resources.cpu}%`);
```

---

## 소상공인 특화 기능

### Shop (쇼핑몰)

```typescript
// 상품 추가
await home.shop.addProduct({
  name: '수제 초코 케이크',
  price: 35000,
  description: '신선한 재료로...',
  images: [cakeImage],
  inventory: 10
});

// 결제 설정
home.shop.setPayment({
  kakaoPay: true,
  naverPay: true,
  bankTransfer: {
    enabled: true,
    accounts: [{ bank: '신한', number: '110-xxx', holder: '홍길동' }]
  }
});
```

### Booking (예약)

```typescript
// 미용실 예약 시스템
home.booking.setup({
  availability: {
    days: ['mon', 'tue', 'wed', 'thu', 'fri', 'sat'],
    hours: { start: '10:00', end: '20:00' },
    slotDuration: 60
  },
  services: [
    { name: '커트', duration: 30, price: 15000 },
    { name: '펌', duration: 120, price: 80000 }
  ],
  notifications: {
    newBooking: ['kakao', 'sms']
  }
});
```

### Analytics (통계)

```typescript
// 방문자 통계
const stats = await home.analytics.get({ period: 'last_30_days' });
console.log(`총 방문자: ${stats.visitors.total}`);
console.log(`인기 페이지:`, stats.topPages);

// 실시간
home.analytics.realtime((data) => {
  console.log(`현재 접속자: ${data.activeVisitors}`);
});
```

---

## Platform Integration (플랫폼 연동)

```typescript
// 네이버, 카카오 연동
await home.integrate({
  naver: {
    place: true,
    placeId: 'xxxxx'
  },
  kakao: {
    channel: true,
    channelId: '@myshop'
  }
});
```

---

## Low Spec Mode (구형 PC도 OK)

```typescript
// 구형 노트북에서도 실행!
const home = new WIAHome({
  name: '내 가게',
  lowSpecMode: {
    limits: {
      maxMemory: 128,  // 128MB
      maxCpu: 10       // 10%
    },
    powerSaving: {
      enabled: true,
      sleepAfterInactive: 5  // 5분 후 절전
    }
  }
});
```

---

## Security (이모가 지켜줌)

```typescript
// 보안은 기본!
// - TLS-LITE 내장
// - DDoS 방어
// - WAF (SQL Injection, XSS 차단)
// - AIR-SHIELD 연동

// 이모(AIR-SHIELD)가 자동으로 지켜줌 🛡️
```

---

## Events

```typescript
home.on('server_started', (event) => {
  console.log('🏠 집 오픈!', event.data);
});

home.on('visitor_connected', (event) => {
  console.log('👋 손님 왔어요!', event.data);
});

home.on('order_received', (event) => {
  console.log('📦 주문 들어왔어요!', event.data);
});

home.on('booking_received', (event) => {
  console.log('📅 예약 들어왔어요!', event.data);
});
```

---

## Industry Templates

| Template | Features |
|----------|----------|
| `restaurant` | 메뉴, 예약, 배달 연동 |
| `cafe` | 메뉴, 위치, 인스타 피드 |
| `bakery` | 상품, 주문, 결제 |
| `hairSalon` | 포트폴리오, 예약, 직원 소개 |
| `clinic` | 진료과, 의료진, 예약 |
| `academy` | 강좌, 시간표, 수강신청 |
| `photographer` | 포트폴리오, 견적 문의 |
| `designer` | 포트폴리오, 블로그 |

---

## Philosophy

```
홍익인간 (弘益人間) - Benefit All Humanity

집은 인권이다.
디지털 세상에서도 마찬가지다.

플랫폼에 종속되지 않을 권리,
내 공간을 소유할 권리,
영원히 내 것으로 가질 권리.

소상공인도, 프리랜서도, 학생도, 누구나.

건물주 없는 세상,
월세 없는 세상,
영원히 내 것인 세상.

- WIA (World Certification Industry Association) -
```

---

## License

MIT License

---

## 집의 약속

```
WIA-HOME의 약속:

1. 건물주가 없어요 - 내 PC가 서버
2. 영원히 내 것이에요 - 플랫폼 망해도 OK
3. 돈 안 들어요 - 호스팅비 0원
4. 쉬워요 - URL 넣으면 5분 완성
5. 안전해요 - 이모가 지켜줘요

WIA 대가족의 든든한 울타리,
비바람이 와도 흔들리지 않는 집.

- 🏠 WIA-HOME -
```
