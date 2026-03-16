# WIA-IND-001: Phase 4 - Integration Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies platform integration requirements, SDK interfaces, and implementation guidelines for the WIA-IND-001 Fashion Technology Standard. It provides concrete guidance for integrating with e-commerce platforms, building applications, and ensuring interoperability.

## 2. E-Commerce Platform Integration

### 2.1 Shopify Integration

**App Structure:**
```
wia-fashion-tech-app/
├── shopify.app.toml
├── extensions/
│   ├── theme-app-extension/
│   │   ├── blocks/
│   │   │   ├── size-recommender.liquid
│   │   │   └── virtual-try-on.liquid
│   │   └── snippets/
│   │       └── wia-integration.liquid
│   └── app-blocks/
│       └── product-page-integration.jsx
├── web/
│   ├── frontend/
│   │   └── pages/
│   │       ├── settings.jsx
│   │       └── analytics.jsx
│   └── backend/
│       └── routes/
│           ├── webhooks.js
│           └── api.js
└── package.json
```

**Theme Extension - Size Recommender:**
```liquid
{% schema %}
{
  "name": "WIA Size Recommender",
  "target": "section",
  "settings": [
    {
      "type": "checkbox",
      "id": "show_confidence",
      "label": "Show confidence score",
      "default": true
    }
  ]
}
{% endschema %}

<div class="wia-size-recommender" data-product-id="{{ product.id }}">
  <button class="wia-get-recommendation">Get Your Perfect Size</button>
  <div class="wia-recommendation-result" style="display: none;">
    <p class="wia-recommended-size"></p>
    <p class="wia-confidence"></p>
  </div>
</div>

<script>
  window.WIA = window.WIA || {};
  WIA.productId = {{ product.id | json }};
  WIA.apiKey = {{ settings.wia_api_key | json }};
</script>
<script src="{{ 'wia-integration.js' | asset_url }}" defer></script>
```

**Shopify Webhook Configuration:**
```javascript
// Register webhooks
{
  "topic": "products/create",
  "address": "https://your-app.com/webhooks/products/create"
},
{
  "topic": "orders/create",
  "address": "https://your-app.com/webhooks/orders/create"
}
```

### 2.2 WooCommerce Integration

**Plugin Structure:**
```
wia-fashion-tech-woocommerce/
├── wia-fashion-tech.php (main plugin file)
├── includes/
│   ├── class-wia-api-client.php
│   ├── class-wia-product-integration.php
│   ├── class-wia-checkout-integration.php
│   └── class-wia-admin.php
├── admin/
│   ├── settings-page.php
│   └── analytics-dashboard.php
├── public/
│   ├── js/
│   │   └── wia-frontend.js
│   └── css/
│       └── wia-styles.css
└── languages/
    ├── wia-fashion-tech-ko_KR.po
    └── wia-fashion-tech-en_US.po
```

**Product Page Integration:**
```php
<?php
// Add size recommender to product page
add_action('woocommerce_before_add_to_cart_button', 'wia_add_size_recommender');

function wia_add_size_recommender() {
    global $product;

    if (wia_is_fashion_product($product)) {
        ?>
        <div class="wia-size-recommender"
             data-product-id="<?php echo esc_attr($product->get_id()); ?>"
             data-category="<?php echo esc_attr(wia_get_category($product)); ?>">
            <button class="wia-scan-button">
                <?php _e('Scan for Perfect Fit', 'wia-fashion-tech'); ?>
            </button>
            <div class="wia-result"></div>
        </div>
        <?php
    }
}
```

### 2.3 Custom Platform Integration

**Integration Checklist:**
- [ ] OAuth 2.0 authentication implemented
- [ ] API client with retry logic and error handling
- [ ] Product data mapping to WIA schema
- [ ] User measurement profile storage
- [ ] Size recommendation display on product pages
- [ ] Virtual try-on integration (optional)
- [ ] Checkout flow integration
- [ ] Analytics tracking
- [ ] Webhook handlers for real-time updates

## 3. SDK Specifications

### 3.1 JavaScript SDK

**Installation:**
```bash
npm install @wia/fashion-tech-sdk
```

**Usage:**
```javascript
import WIAFashionTech from '@wia/fashion-tech-sdk';

const wia = new WIAFashionTech({
  apiKey: 'your-api-key',
  environment: 'production', // or 'sandbox'
  locale: 'ko-KR',
  philosophy: '弘益人間'
});

// Create measurement
const measurement = await wia.measurements.create({
  subject: { id: 'user-123', gender: 'female' },
  measurements: {
    height: { value: 165.5, unit: 'cm', method: 'smartphone-scan' },
    chest: { value: 88.0, unit: 'cm', method: 'smartphone-scan' },
    waist: { value: 68.5, unit: 'cm', method: 'smartphone-scan' },
    hip: { value: 94.0, unit: 'cm', method: 'smartphone-scan' }
  }
});

// Get size recommendation
const recommendation = await wia.recommendations.size({
  userId: 'user-123',
  productId: 'prod-67890',
  measurementId: measurement.id
});

console.log(`Recommended size: ${recommendation.primarySize.label}`);
console.log(`Confidence: ${recommendation.primarySize.confidence * 100}%`);

// Start virtual fitting session
const session = await wia.virtualFitting.createSession({
  userId: 'user-123',
  productId: 'prod-67890',
  size: recommendation.primarySize.label,
  renderQuality: 'high'
});

// Open interactive fitting
wia.virtualFitting.open(session.sessionId);
```

### 3.2 Python SDK

**Installation:**
```bash
pip install wia-fashion-tech
```

**Usage:**
```python
from wia_fashion_tech import WIAClient, Measurement

client = WIAClient(
    api_key='your-api-key',
    environment='production',
    philosophy='弘益人間'
)

# Create measurement
measurement = client.measurements.create(
    subject={'id': 'user-123', 'gender': 'female'},
    measurements={
        'height': {'value': 165.5, 'unit': 'cm', 'method': 'smartphone-scan'},
        'chest': {'value': 88.0, 'unit': 'cm', 'method': 'smartphone-scan'},
        'waist': {'value': 68.5, 'unit': 'cm', 'method': 'smartphone-scan'},
        'hip': {'value': 94.0, 'unit': 'cm', 'method': 'smartphone-scan'}
    }
)

# Get size recommendation
recommendation = client.recommendations.get_size(
    user_id='user-123',
    product_id='prod-67890',
    measurement_id=measurement.id
)

print(f"Recommended size: {recommendation.primary_size.label}")
print(f"Confidence: {recommendation.primary_size.confidence * 100}%")
```

### 3.3 Mobile SDKs

**iOS (Swift):**
```swift
import WIAFashionTech

let wia = WIAClient(apiKey: "your-api-key", environment: .production)

// Start 3D body scan using TrueDepth camera
wia.scanner.startScan { result in
    switch result {
    case .success(let measurement):
        print("Scan complete: \(measurement.id)")

        // Get size recommendation
        wia.recommendations.getSize(
            userId: "user-123",
            productId: "prod-67890",
            measurementId: measurement.id
        ) { recommendation in
            print("Recommended size: \(recommendation.primarySize.label)")
        }

    case .failure(let error):
        print("Scan failed: \(error)")
    }
}
```

**Android (Kotlin):**
```kotlin
import com.wia.fashiontech.WIAClient
import com.wia.fashiontech.scanner.BodyScanner

val wia = WIAClient(
    apiKey = "your-api-key",
    environment = Environment.PRODUCTION,
    philosophy = "弘益人間"
)

// Start 3D body scan
val scanner = BodyScanner(context)
scanner.startScan { result ->
    result.onSuccess { measurement ->
        println("Scan complete: ${measurement.id}")

        // Get size recommendation
        wia.recommendations.getSize(
            userId = "user-123",
            productId = "prod-67890",
            measurementId = measurement.id
        ) { recommendation ->
            println("Recommended size: ${recommendation.primarySize.label}")
        }
    }

    result.onFailure { error ->
        println("Scan failed: $error")
    }
}
```

## 4. Sustainability Integration

### 4.1 Carbon Footprint Tracking

**API Integration:**
```javascript
const product = await wia.products.get('prod-67890');

console.log('Sustainability Metrics:');
console.log(`Carbon Footprint: ${product.sustainability.carbonFootprint.value} kg CO2e`);
console.log(`Water Usage: ${product.sustainability.waterUsage.value} liters`);
console.log(`Recyclability: ${product.sustainability.recyclability}%`);
console.log(`Estimated Lifespan: ${product.sustainability.lifespan.estimated} years`);

// Display sustainability badge
if (product.sustainability.carbonFootprint.value < 5) {
    displayBadge('Low Carbon');
}
if (product.sustainability.certifications.includes('Organic')) {
    displayBadge('Organic');
}
```

### 4.2 Return Reduction Tracking

**Analytics Integration:**
```javascript
// Track recommendation acceptance
wia.analytics.track('size_recommendation_accepted', {
    productId: 'prod-67890',
    recommendedSize: 'M',
    confidence: 0.92,
    userId: 'user-123'
});

// Track purchase
wia.analytics.track('purchase_completed', {
    productId: 'prod-67890',
    size: 'M',
    wasRecommended: true,
    userId: 'user-123'
});

// Track return (or lack thereof)
wia.analytics.track('return_status', {
    orderId: 'order-456',
    returned: false,
    recommendationUsed: true
});

// Get aggregated return reduction metrics
const metrics = await wia.analytics.getReturnReduction({
    dateRange: { start: '2025-01-01', end: '2025-12-31' },
    groupBy: 'month'
});

console.log(`Return rate with recommendations: ${metrics.withRecommendations.returnRate}%`);
console.log(`Return rate without recommendations: ${metrics.withoutRecommendations.returnRate}%`);
console.log(`CO2 saved: ${metrics.co2Saved} kg`);
```

## 5. Analytics and Reporting Integration

### 5.1 Dashboard Integration

**Metrics to Track:**
```javascript
const dashboard = await wia.analytics.getDashboard({
    dateRange: { start: '2025-01-01', end: '2025-01-31' }
});

console.log('Conversion Metrics:');
console.log(`Total Recommendations: ${dashboard.recommendations.total}`);
console.log(`Acceptance Rate: ${dashboard.recommendations.acceptanceRate}%`);
console.log(`Conversion Rate (with rec): ${dashboard.conversion.withRecommendation}%`);
console.log(`Conversion Rate (without): ${dashboard.conversion.withoutRecommendation}%`);
console.log(`Lift: ${dashboard.conversion.lift}%`);

console.log('\nVirtual Fitting Metrics:');
console.log(`Sessions Started: ${dashboard.virtualFitting.sessionsStarted}`);
console.log(`Completion Rate: ${dashboard.virtualFitting.completionRate}%`);
console.log(`Purchase Correlation: ${dashboard.virtualFitting.purchaseCorrelation}%`);

console.log('\nReturn Metrics:');
console.log(`Overall Return Rate: ${dashboard.returns.overallRate}%`);
console.log(`Return Rate (size issues): ${dashboard.returns.sizeIssuesRate}%`);
console.log(`Return Rate (recommended sizes): ${dashboard.returns.recommendedSizesRate}%`);
console.log(`Estimated CO2 Saved: ${dashboard.sustainability.co2SavedKg} kg`);
```

### 5.2 Custom Event Tracking

**Event Schema:**
```javascript
wia.analytics.track('custom_event', {
    category: 'engagement',
    action: 'opened_size_guide',
    label: 'product_page',
    value: 1,
    properties: {
        productId: 'prod-67890',
        hasRecommendation: true,
        recommendationConfidence: 0.92
    }
});
```

## 6. Multi-Channel Integration

### 6.1 Marketplace Integration

**Amazon Integration Example:**
```javascript
// Sync product data to Amazon
const amazonProduct = wia.marketplace.amazon.syncProduct({
    sku: 'BRAND-SKU-123',
    wiaProductId: 'prod-67890',
    enableSizeRecommendation: true,
    displayBadge: true
});

// Amazon customers can use WIA recommendations
// via marketplace integration API
```

### 6.2 Social Commerce Integration

**Instagram Shopping:**
```javascript
// Tag products with size recommendation capability
wia.social.instagram.tagProduct({
    postId: 'instagram-post-123',
    productId: 'prod-67890',
    enableVirtualTryOn: true,
    ctaText: 'Try On Virtually'
});
```

## 7. Webhook Integration

### 7.1 Webhook Event Types

```javascript
// Register webhooks
await wia.webhooks.register({
    url: 'https://your-app.com/webhooks/wia',
    events: [
        'measurement.created',
        'measurement.updated',
        'recommendation.generated',
        'fitting.session_started',
        'fitting.session_completed',
        'product.updated'
    ],
    secret: 'your-webhook-secret'
});
```

### 7.2 Webhook Handler Example

```javascript
const express = require('express');
const crypto = require('crypto');

app.post('/webhooks/wia', (req, res) => {
    // Verify signature
    const signature = req.headers['x-wia-signature'];
    const payload = JSON.stringify(req.body);
    const expectedSignature = crypto
        .createHmac('sha256', webhookSecret)
        .update(payload)
        .digest('hex');

    if (signature !== `sha256=${expectedSignature}`) {
        return res.status(401).send('Invalid signature');
    }

    // Process event
    const { event, data } = req.body;

    switch (event) {
        case 'measurement.created':
            handleMeasurementCreated(data);
            break;
        case 'recommendation.generated':
            handleRecommendationGenerated(data);
            break;
        // ... handle other events
    }

    res.status(200).send('OK');
});
```

## 8. Testing and Validation

### 8.1 Test Environment

```javascript
const wiaTest = new WIAFashionTech({
    apiKey: 'test-api-key',
    environment: 'sandbox',
    testMode: true
});

// Use test data
const testMeasurement = await wiaTest.measurements.create(
    wiaTest.testData.measurements.standard_female
);

const testRecommendation = await wiaTest.recommendations.size({
    userId: 'test-user',
    productId: 'test-product-01',
    measurementId: testMeasurement.id
});

// Assertions
assert.equal(testRecommendation.primarySize.label, 'M');
assert.isAbove(testRecommendation.primarySize.confidence, 0.85);
```

### 8.2 Validation Tools

```bash
# CLI validation tool
npm install -g @wia/validation-tool

# Validate product data
wia-validate product product-data.json

# Validate measurement data
wia-validate measurement measurement-data.json

# Test API integration
wia-test --apiKey your-test-key --endpoint https://api.example.com
```

## 9. Performance Optimization

### 9.1 Caching Strategy

```javascript
// Cache size recommendations
const cache = new WIACache({
    ttl: 3600, // 1 hour
    maxSize: 1000
});

async function getCachedRecommendation(userId, productId) {
    const cacheKey = `rec:${userId}:${productId}`;

    let recommendation = await cache.get(cacheKey);

    if (!recommendation) {
        recommendation = await wia.recommendations.size({
            userId,
            productId
        });
        await cache.set(cacheKey, recommendation);
    }

    return recommendation;
}
```

### 9.2 Batch Operations

```javascript
// Batch product updates
const products = [ /* array of products */ ];
const results = await wia.products.batchUpdate(products);

// Batch size recommendations
const recommendations = await wia.recommendations.batch({
    userId: 'user-123',
    productIds: ['prod-1', 'prod-2', 'prod-3', 'prod-4']
});
```

## 10. Compliance and Certification

### 10.1 Implementation Checklist

- [ ] **Phase 1: Data Format**
  - [ ] Body measurement schema implemented
  - [ ] Garment specification schema implemented
  - [ ] Size chart format supported
  - [ ] Validation rules enforced

- [ ] **Phase 2: API Interface**
  - [ ] Authentication (OAuth 2.0) implemented
  - [ ] All required endpoints implemented
  - [ ] Rate limiting configured
  - [ ] Error handling per RFC 7807

- [ ] **Phase 3: Protocol**
  - [ ] Privacy protocol implemented
  - [ ] Data encryption (AES-256) enabled
  - [ ] Consent management functional
  - [ ] Audit logging active

- [ ] **Phase 4: Integration**
  - [ ] Platform integration complete
  - [ ] Analytics tracking configured
  - [ ] Webhooks registered and tested
  - [ ] Documentation published

### 10.2 Certification Process

1. **Self-Assessment:** Use validation tools
2. **Test Suite:** Pass all integration tests
3. **Documentation Review:** Submit implementation docs
4. **Third-Party Audit:** Optional certification audit
5. **Listing:** Add to official WIA implementation directory

---

**Copyright © 2025 SmileStory Inc. / WIA**
**License:** CC BY 4.0
**弘益人間 - Benefit All Humanity**
