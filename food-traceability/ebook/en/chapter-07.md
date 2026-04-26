# Chapter 7: Consumer Engagement and Transparency

**WIA-AGRI-016 eBook Series**

---

## The Transparency Revolution

Modern consumers demand to know where their food comes from. Traceability systems that were once purely operational tools are now powerful consumer engagement platforms.

**Consumer Trends:**
- 73% willing to pay more for complete transparency
- 86% want to know product origin and journey
- 94% more loyal to transparent brands
- QR code scans increased 96% since 2020

---

## QR Code Implementation

### GS1 Digital Link

```javascript
// Generate GS1 Digital Link QR Code
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

// Consumer scans QR → Redirects to landing page
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

### Consumer-Facing Landing Page

```html
<!DOCTYPE html>
<html>
<head>
    <title>Organic Apples - Product Story</title>
</head>
<body>
    <header>
        <h1>🍎 Organic Apples</h1>
        <p>Batch: LOT2025001</p>
    </header>

    <section class="origin">
        <h2>From the Farm</h2>
        <img src="{{origin.photoUrl}}" alt="ABC Organic Farm">
        <h3>{{origin.farmName}}</h3>
        <p>{{origin.location}}</p>
        <p>Harvested: {{origin.harvestDate}}</p>
        <p>Farmer: {{origin.farmerName}}</p>
        <blockquote>{{origin.farmerStory}}</blockquote>
    </section>

    <section class="journey">
        <h2>Journey to You</h2>
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
        <h2>Certifications</h2>
        <div class="cert-grid">
            {{#each certifications}}
            <div class="cert">
                <img src="{{this.iconUrl}}" alt="{{this.name}}">
                <p>{{this.name}}</p>
                <span class="verified">✓ Blockchain Verified</span>
            </div>
            {{/each}}
        </div>
    </section>

    <section class="sustainability">
        <h2>Environmental Impact</h2>
        <div class="metrics">
            <div class="metric">
                <span class="value">{{sustainability.co2kg}}</span>
                <span class="label">kg CO₂</span>
            </div>
            <div class="metric">
                <span class="value">{{sustainability.waterLiters}}</span>
                <span class="label">L Water</span>
            </div>
            <div class="metric">
                <span class="value">{{sustainability.localPercent}}%</span>
                <span class="label">Local Sourcing</span>
            </div>
        </div>
        <div class="rating">
            Overall Sustainability: {{sustainability.rating}}/5 ⭐
        </div>
    </section>
</body>
</html>
```

---

## Mobile App Integration

### React Native Example

```javascript
import React, { useState } from 'react';
import { Camera } from 'expo-camera';
import { BarCodeScanner } from 'expo-barcode-scanner';

function ProductScannerScreen() {
  const [scanned, setScanned] = useState(false);

  const handleBarCodeScanned = async ({ type, data }) => {
    setScanned(true);

    // Parse GS1 Digital Link
    const batchInfo = parseGS1DigitalLink(data);

    // Fetch product story
    const story = await fetch(
      `https://api.wia.org/trace/${batchInfo.gtin}/${batchInfo.lot}`
    ).then(r => r.json());

    // Navigate to product story screen
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

## Interactive Features

### Consumer Reviews

```javascript
async function submitProductReview(batchId, userId, review) {
  // Verify purchase (optional, via loyalty program)
  const purchaseVerified = await verifyPurchase(userId, batchId);

  const reviewData = {
    batchId,
    userId,
    rating: review.rating, // 1-5 stars
    comment: review.comment,
    categories: {
      freshness: review.freshness,
      taste: review.taste,
      packaging: review.packaging
    },
    verified: purchaseVerified,
    timestamp: new Date().toISOString()
  };

  // Store review
  await database.collection('reviews').insert(reviewData);

  // Update aggregate rating
  await updateAggregateRating(batchId);

  // Notify producer (if opted in)
  if (review.rating <= 3) {
    await notifyProducer({
      batchId,
      issue: 'Low rating received',
      rating: review.rating,
      comment: review.comment
    });
  }

  return { success: true, reviewId: reviewData.id };
}
```

### Quality Issue Reporting

```javascript
async function reportQualityIssue(report) {
  const issueData = {
    batchId: report.batchId,
    userId: report.userId,
    issueType: report.type, // quality, safety, packaging, other
    severity: report.severity, // low, medium, high, critical
    description: report.description,
    photos: await uploadPhotos(report.photos),
    location: report.location,
    timestamp: new Date().toISOString()
  };

  // Immediate escalation for safety issues
  if (report.severity === 'critical') {
    await escalateToFoodSafety({
      ...issueData,
      escalatedAt: new Date().toISOString(),
      priority: 'URGENT'
    });

    // Automated sampling of same batch if multiple reports
    await checkForMultipleReports(report.batchId);
  }

  await database.collection('quality_reports').insert(issueData);

  return {
    success: true,
    reportId: issueData.id,
    message: 'Thank you for your report. We take quality seriously.',
    trackingUrl: `https://app.wia.org/reports/${issueData.id}`
  };
}
```

---

## Augmented Reality Experiences

### AR Product Journey

```javascript
// AR.js implementation
class ARProductJourney {
  async initializeAR(batchId) {
    const trace = await getTraceData(batchId);

    // Create 3D scene
    const scene = new AFRAME.Scene();

    // Add journey nodes
    trace.events.forEach((event, index) => {
      const node = this.createLocationNode(event, index);
      scene.appendChild(node);
    });

    // Add connections between nodes
    this.createConnections(trace.events);

    return scene;
  }

  createLocationNode(event, index) {
    const entity = document.createElement('a-entity');

    // 3D model based on location type
    const model = this.getLocationModel(event.location.type);

    entity.setAttribute('gltf-model', model);
    entity.setAttribute('position', this.calculatePosition(index));
    entity.setAttribute('animation', {
      property: 'rotation',
      to: '0 360 0',
      loop: true,
      dur: 10000
    });

    // Add info panel on click
    entity.addEventListener('click', () => {
      this.showEventDetails(event);
    });

    return entity;
  }
}
```

---

## Gamification

### Sustainability Challenges

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

    // Award badges
    const badges = await this.awardBadges(userId, impact);

    return { impact, badges };
  }

  async awardBadges(userId, impact) {
    const badges = [];

    if (impact.organicProductsCount >= 10) {
      badges.push({
        name: 'Organic Champion',
        icon: '🌿',
        earnedAt: new Date()
      });
    }

    if (impact.totalCO2Saved >= 100) {
      badges.push({
        name: 'Carbon Warrior',
        icon: '🌍',
        earnedAt: new Date()
      });
    }

    if (impact.localProductsPercent >= 80) {
      badges.push({
        name: 'Local Hero',
        icon: '🏡',
        earnedAt: new Date()
      });
    }

    // Store badges
    await database.collection('user_badges').insertMany(
      badges.map(b => ({ ...b, userId }))
    );

    return badges;
  }
}
```

---

## Privacy Considerations

### Data Minimization

```javascript
function prepareConsumerView(fullTraceData) {
  // Remove sensitive business information
  return {
    product: {
      name: fullTraceData.product.name,
      imageUrl: fullTraceData.product.imageUrl,
      // EXCLUDE: pricing, profit margins, supplier contracts
    },
    origin: {
      farm: fullTraceData.origin.name,
      location: fullTraceData.origin.city + ', ' + fullTraceData.origin.country,
      // EXCLUDE: exact GPS, farmer contact info, production costs
    },
    journey: fullTraceData.events.map(e => ({
      date: e.timestamp.split('T')[0], // Date only, not exact time
      location: e.location.name,
      event: e.eventType
      // EXCLUDE: internal shipping costs, carrier details
    })),
    certifications: fullTraceData.certifications.map(c => ({
      name: c.type,
      verified: true
      // EXCLUDE: certification costs, audit reports
    }))
  };
}
```

---

## Chapter Summary

Consumer engagement transforms traceability from compliance to competitive advantage:

**Key Features:**
- QR code product stories
- Mobile app scanning
- Interactive journey maps
- Blockchain verification
- Consumer reviews
- Quality issue reporting
- AR experiences
- Gamification rewards

**Benefits:**
- Build brand trust
- Increase customer loyalty
- Differentiate products
- Gather consumer feedback
- Drive sustainability

---

## Next Chapter

**Chapter 8: Implementation Guide and Best Practices**

Practical guidance for implementing WIA-AGRI-016 in your organization.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
