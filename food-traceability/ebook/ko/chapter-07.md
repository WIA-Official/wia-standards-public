# 제7장: 소비자 참여 및 투명성

**WIA-AGRI-016 전자책 시리즈**

---

## 투명성 혁명

현대 소비자는 식품의 출처를 알고자 합니다. 한때 순수한 운영 도구였던 이력 추적 시스템은 이제 강력한 소비자 참여 플랫폼이 되었습니다.

**소비자 트렌드:**
- 73%가 완전한 투명성을 위해 더 많은 비용을 지불할 의향
- 86%가 제품 원산지와 여정을 알고 싶어함
- 94%가 투명한 브랜드에 더 충성함
- 2020년 이후 QR 코드 스캔 96% 증가

---

## QR 코드 구현

### GS1 디지털 링크

```javascript
// GS1 디지털 링크 QR 코드 생성
function generateProductQR(batchData) {
  const digitalLink = `https://id.wia.org/01/${batchData.gtin}/10/${batchData.lot}`;

  return {
    url: digitalLink,
    qrCode: generateQRCode(digitalLink, {
      size: 300,
      errorCorrection: 'M'
    }),
    landingPage: `https://trace.wia.org/product/${batchData.gtin}/${batchData.lot}`
  };
}

// 소비자가 QR 스캔 → 랜딩 페이지로 리디렉션
app.get('/product/:gtin/:lot', async (req, res) => {
  const { gtin, lot } = req.params;
  const batchId = `${gtin}.${lot}`;

  const traceData = await getTraceabilityData(batchId);
  const certification = await getVerifiedCertifications(batchId);

  res.render('product-story', {
    product: traceData.product,
    origin: traceData.origin,
    journey: traceData.journey,
    certifications: certification,
    sustainability: traceData.sustainability
  });
});
```

### 소비자 대면 랜딩 페이지

```html
<!DOCTYPE html>
<html lang="ko">
<head>
    <title>유기농 사과 - 제품 스토리</title>
</head>
<body>
    <header>
        <h1>🍎 유기농 사과</h1>
        <p>배치: LOT2025001</p>
    </header>

    <section class="origin">
        <h2>농장에서</h2>
        <img src="{{origin.photoUrl}}" alt="ABC 유기농 농장">
        <h3>{{origin.farmName}}</h3>
        <p>{{origin.location}}</p>
        <p>수확일: {{origin.harvestDate}}</p>
        <p>농부: {{origin.farmerName}}</p>
        <blockquote>{{origin.farmerStory}}</blockquote>
    </section>

    <section class="journey">
        <h2>당신에게 오기까지</h2>
        <div class="timeline">
            {{#each journey}}
            <div class="event">
                <div class="date">{{this.date}}</div>
                <div class="icon">{{this.icon}}</div>
                <div class="description">
                    <strong>{{this.location}}</strong>
                    <p>{{this.event}}</p>
                </div>
            </div>
            {{/each}}
        </div>
    </section>

    <section class="certifications">
        <h2>인증</h2>
        <div class="cert-grid">
            {{#each certifications}}
            <div class="cert">
                <img src="{{this.iconUrl}}" alt="{{this.name}}">
                <p>{{this.name}}</p>
                <span class="verified">✓ 블록체인 검증됨</span>
            </div>
            {{/each}}
        </div>
    </section>

    <section class="sustainability">
        <h2>환경 영향</h2>
        <div class="metrics">
            <div class="metric">
                <span class="value">{{sustainability.co2kg}}</span>
                <span class="label">kg CO₂</span>
            </div>
            <div class="metric">
                <span class="value">{{sustainability.waterLiters}}</span>
                <span class="label">L 물</span>
            </div>
            <div class="metric">
                <span class="value">{{sustainability.localPercent}}%</span>
                <span class="label">지역 조달</span>
            </div>
        </div>
        <div class="rating">
            전체 지속가능성: {{sustainability.rating}}/5 ⭐
        </div>
    </section>
</body>
</html>
```

---

## 모바일 앱 통합

### React Native 예시

```javascript
import React, { useState } from 'react';
import { Camera } from 'expo-camera';
import { BarCodeScanner } from 'expo-barcode-scanner';

function ProductScannerScreen() {
  const [scanned, setScanned] = useState(false);

  const handleBarCodeScanned = async ({ type, data }) => {
    setScanned(true);

    // GS1 디지털 링크 파싱
    const batchInfo = parseGS1DigitalLink(data);

    // 제품 스토리 가져오기
    const story = await fetch(
      `https://api.wia.org/trace/${batchInfo.gtin}/${batchInfo.lot}`
    ).then(r => r.json());

    // 제품 스토리 화면으로 이동
    navigation.navigate('ProductStory', { story });
  };

  return (
    <Camera
      onBarCodeScanned={scanned ? undefined : handleBarCodeScanned}
      style={{ flex: 1 }}
    />
  );
}

function ProductStoryScreen({ route }) {
  const { story } = route.params;

  return (
    <ScrollView>
      <HeroImage source={{ uri: story.product.imageUrl }} />

      <OriginSection farm={story.origin} />

      <JourneyTimeline events={story.journey} />

      <CertificationBadges certifications={story.certifications} />

      <SustainabilityMetrics data={story.sustainability} />

      <BlockchainVerification txHash={story.blockchainTx} />

      <ReviewSection batchId={story.batchId} />
    </ScrollView>
  );
}
```

---

## 대화형 기능

### 소비자 리뷰

```javascript
async function submitProductReview(batchId, userId, review) {
  // 구매 확인 (선택 사항, 로열티 프로그램 통해)
  const purchaseVerified = await verifyPurchase(userId, batchId);

  const reviewData = {
    batchId,
    userId,
    rating: review.rating, // 1-5 별점
    comment: review.comment,
    categories: {
      freshness: review.freshness,
      taste: review.taste,
      packaging: review.packaging
    },
    verified: purchaseVerified,
    timestamp: new Date().toISOString()
  };

  // 리뷰 저장
  await database.collection('reviews').insert(reviewData);

  // 총합 평점 업데이트
  await updateAggregateRating(batchId);

  // 생산자에게 통보 (옵트인한 경우)
  if (review.rating <= 3) {
    await notifyProducer({
      batchId,
      issue: '낮은 평점 수신',
      rating: review.rating,
      comment: review.comment
    });
  }

  return { success: true, reviewId: reviewData.id };
}
```

### 품질 문제 신고

```javascript
async function reportQualityIssue(report) {
  const issueData = {
    batchId: report.batchId,
    userId: report.userId,
    issueType: report.type, // 품질, 안전, 포장, 기타
    severity: report.severity, // 낮음, 중간, 높음, 긴급
    description: report.description,
    photos: await uploadPhotos(report.photos),
    location: report.location,
    timestamp: new Date().toISOString()
  };

  // 안전 문제에 대한 즉시 에스컬레이션
  if (report.severity === 'critical') {
    await escalateToFoodSafety({
      ...issueData,
      escalatedAt: new Date().toISOString(),
      priority: '긴급'
    });

    // 동일 배치의 여러 보고가 있는 경우 자동 샘플링
    await checkForMultipleReports(report.batchId);
  }

  await database.collection('quality_reports').insert(issueData);

  return {
    success: true,
    reportId: issueData.id,
    message: '보고해 주셔서 감사합니다. 우리는 품질을 진지하게 받아들입니다.',
    trackingUrl: `https://app.wia.org/reports/${issueData.id}`
  };
}
```

---

## 게임화

### 지속가능성 챌린지

```javascript
class SustainabilityRewards {
  async trackConsumerImpact(userId) {
    const purchases = await getUserPurchases(userId);

    const impact = {
      totalCO2Saved: 0,
      localProductsPercent: 0,
      organicProductsCount: 0,
      sustainabilityScore: 0
    };

    for (const purchase of purchases) {
      const productData = await getProductSustainability(purchase.batchId);

      impact.totalCO2Saved += productData.co2Saved;

      if (productData.local) impact.localProductsPercent++;
      if (productData.organic) impact.organicProductsCount++;
    }

    impact.localProductsPercent =
      (impact.localProductsPercent / purchases.length) * 100;

    impact.sustainabilityScore = this.calculateScore(impact);

    // 배지 수여
    const badges = await this.awardBadges(userId, impact);

    return { impact, badges };
  }

  async awardBadges(userId, impact) {
    const badges = [];

    if (impact.organicProductsCount >= 10) {
      badges.push({
        name: '유기농 챔피언',
        icon: '🌿',
        earnedAt: new Date()
      });
    }

    if (impact.totalCO2Saved >= 100) {
      badges.push({
        name: '탄소 전사',
        icon: '🌍',
        earnedAt: new Date()
      });
    }

    if (impact.localProductsPercent >= 80) {
      badges.push({
        name: '로컬 히어로',
        icon: '🏡',
        earnedAt: new Date()
      });
    }

    // 배지 저장
    await database.collection('user_badges').insertMany(
      badges.map(b => ({ ...b, userId }))
    );

    return badges;
  }
}
```

---

## 개인정보 보호 고려사항

### 데이터 최소화

```javascript
function prepareConsumerView(fullTraceData) {
  // 민감한 비즈니스 정보 제거
  return {
    product: {
      name: fullTraceData.product.name,
      imageUrl: fullTraceData.product.imageUrl,
      // 제외: 가격, 이윤, 공급업체 계약
    },
    origin: {
      farm: fullTraceData.origin.name,
      location: fullTraceData.origin.city + ', ' + fullTraceData.origin.country,
      // 제외: 정확한 GPS, 농부 연락처, 생산 비용
    },
    journey: fullTraceData.events.map(e => ({
      date: e.timestamp.split('T')[0], // 날짜만, 정확한 시간 아님
      location: e.location.name,
      event: e.eventType
      // 제외: 내부 운송 비용, 운송업체 세부 정보
    })),
    certifications: fullTraceData.certifications.map(c => ({
      name: c.type,
      verified: true
      // 제외: 인증 비용, 감사 보고서
    }))
  };
}
```

---

## 장 요약

소비자 참여는 이력 추적을 규제 준수에서 경쟁 우위로 전환합니다:

**주요 기능:**
- QR 코드 제품 스토리
- 모바일 앱 스캔
- 대화형 여정 지도
- 블록체인 검증
- 소비자 리뷰
- 품질 문제 신고
- AR 경험
- 게임화 보상

**혜택:**
- 브랜드 신뢰 구축
- 고객 충성도 증가
- 제품 차별화
- 소비자 피드백 수집
- 지속가능성 촉진

---

## 다음 장

**제8장: 구현 가이드 및 모범 사례**

조직에서 WIA-AGRI-016을 구현하기 위한 실용적인 지침.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
