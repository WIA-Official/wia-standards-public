# WIA-SOCIAL

## 조카의 연결 - "나 통해서 다 연결돼!" 🌐

모든 SNS를 하나로 연결하는 표준

---

## WIA Family

```
├── 아버지 (WIA-INTENT): "의도를 표현해"
├── 어머니 (WIA-OMNI-API): "내가 다 품어줄게"
├── 삼촌 (WIA-AIR-POWER): "내가 힘 나눠줄게" 💪
├── 이모 (WIA-AIR-SHIELD): "내가 지켜줄게" 🛡️
└── 조카 (WIA-SOCIAL): "나 통해서 다 연결돼!" 🌐 ← HERE
```

---

## Why?

```
Instagram, Twitter, TikTok, Threads, Bluesky...
SNS가 너무 많아요!

각각 따로 글 올리고
각각 따로 친구 맺고
각각 따로 피드 확인하고...

플랫폼에 갇혀 사는 건 싫어요!

조카가 다 연결해드릴게요~ 📱
```

---

## Installation

```bash
npm install @anthropic/wia-social
```

---

## Quick Start

```typescript
import { WIASocial } from '@anthropic/wia-social';

// 초기화
const social = new WIASocial({
  platforms: ['instagram', 'twitter', 'tiktok'],
  enableAirShield: true // 이모 연동 (개인정보 보호)
});

// 플랫폼 연결
await social.connectPlatform('instagram', 'your_instagram_token');
await social.connectPlatform('twitter', 'your_twitter_token');

// 한 번에 모든 SNS에 포스팅!
await social.post({
  content: {
    text: '오늘 맛있는 거 먹었다! 🍜',
    media: [myPhoto]
  },
  targets: ['instagram', 'twitter']
});

// 통합 피드 확인
const feed = await social.getFeed();
```

---

## Features

### 1. Cross-Platform Posting
```typescript
// 한 번 쓰면 모든 SNS에!
await social.post({
  content: { text: 'Hello World!' },
  targets: ['instagram', 'twitter', 'tiktok', 'threads']
});
```

### 2. Unified Feed
```typescript
// 모든 SNS 피드를 하나로
const feed = await social.getFeed({
  sorting: 'chronological',
  filters: {
    platforms: ['instagram', 'twitter'],
    hideReposts: true
  }
});

// 실시간 구독
social.subscribeFeed((post) => {
  console.log('새 포스트:', post);
});
```

### 3. Universal Identity
```typescript
// 하나의 ID로 모든 SNS
const identity = await WIASocial.createIdentity({
  displayName: '홍길동',
  email: 'hong@example.com'
});

// 연결된 플랫폼 확인
console.log(identity.connectedPlatforms);
```

### 4. Cross-Platform Friends
```typescript
// 친구의 모든 SNS 찾기
const friend = await social.findFriendAcrossPlatforms('홍길동');
console.log(friend.connectedPlatforms);

// 한 번에 모든 플랫폼에서 팔로우
await social.follow(friend);
```

### 5. Data Portability
```typescript
// 내 데이터 전체 내보내기
const backup = await social.exportAll();

// 특정 플랫폼만 내보내기
const twitterData = await social.exportPlatform('twitter');

// 데이터 삭제 (GDPR 준수)
await social.deleteData('facebook');
```

---

## Privacy Protection (이모 연동)

```typescript
// 포스팅 전 자동 검사
const scanResult = await social.scanContent(myContent);

if (!scanResult.safe) {
  console.log('위험 요소:', scanResult.risks);
  // 위치 정보, 전화번호, 얼굴 인식 등
}

// 메타데이터 자동 제거
const safeMedia = social.stripMediaMetadata(photo);
```

---

## Supported Platforms

| Platform | Posting | Feed | DM | Status |
|----------|---------|------|-----|--------|
| Instagram | ✅ | ✅ | ✅ | Ready |
| Twitter/X | ✅ | ✅ | ✅ | Ready |
| TikTok | ✅ | ✅ | ✅ | Ready |
| Facebook | ✅ | ✅ | ✅ | Ready |
| Threads | ✅ | ✅ | ❌ | Ready |
| LinkedIn | ✅ | ✅ | ✅ | Ready |
| Mastodon | ✅ | ✅ | ✅ | Ready |
| Bluesky | ✅ | ✅ | ✅ | Ready |
| YouTube | ✅ | ✅ | ❌ | Beta |
| Discord | ❌ | ✅ | ✅ | Beta |
| Telegram | ❌ | ✅ | ✅ | Beta |

---

## Family Integration

```typescript
import { WIASocial, familyIntegration } from '@anthropic/wia-social';

const social = new WIASocial();

// 아버지(INTENT) 연동 - 의도 기반 포스팅
await familyIntegration.postWithIntent(
  social,
  '친구들한테 오늘 좋은 일 있었다고 알려줘'
);

// 이모(AIR-SHIELD) 연동 - 보안 포스팅
await familyIntegration.securePost(
  social,
  myContent,
  ['instagram', 'twitter']
);
```

---

## Events

```typescript
// 이벤트 구독
social.on('new_post', (event) => {
  console.log('새 포스트:', event.data);
});

social.on('new_follower', (event) => {
  console.log('새 팔로워:', event.data);
});

social.on('privacy_alert', (event) => {
  console.log('이모의 경고:', event.data);
});

social.on('platform_connected', (event) => {
  console.log('플랫폼 연결됨:', event.data);
});
```

---

## Philosophy

```
홍익인간 (弘益人間) - Benefit All Humanity

디지털 시대에 연결은 인권이다.

플랫폼은 오고 가지만,
인간의 관계는 영원해야 한다.

모든 길은 로마로 통했듯이,
모든 SNS는 WIA로 통한다.

- WIA (World Certification Industry Association) -
```

---

## License

MIT License

---

## 조카의 약속

```
삼촌 이모 다 좋은데,
요즘 세상은 연결이 힘이에요!

1. 어디서든 연결해드릴게요
2. 내 데이터는 내 것, 지켜드릴게요
3. 플랫폼에 갇히지 않게 해드릴게요
4. 새로운 SNS가 나와도 바로 연결해드릴게요
5. 이모한테 잘 배워서 안전하게!

- 조카 (WIA-SOCIAL) -
```
