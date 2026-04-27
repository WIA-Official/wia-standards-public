# PHASE 2 — API

> Smart-store API surface: checkout orchestration, computer-vision
> detection feed, recommendation engine, and the payment flow
> that closes a session. Every endpoint accepts and returns the
> Phase-1 envelopes; this phase fixes verb semantics, idempotency,
> and the algorithmic guarantees behind detection-confidence
> arbitration.

## 3. Automated Checkout System

### 3.1 Just Walk Out Technology

The automated checkout system eliminates traditional checkout lines:

**Entry Process:**
1. Customer opens mobile app or taps payment card at entry gate
2. System creates unique session ID and activates tracking
3. Entry gate opens, customer enters store
4. Computer vision begins tracking customer position

**Shopping Process:**
1. Customer browses products normally
2. Picking up item triggers detection:
   - Computer vision identifies product
   - Shelf sensor confirms removal (weight change)
   - RFID reader validates product tag
   - Item added to virtual cart
3. Putting back item triggers reverse process:
   - Vision detects replacement action
   - Shelf sensor confirms return (weight increase)
   - Item removed from virtual cart

**Exit Process:**
1. Customer walks toward exit gate
2. System finalizes virtual cart
3. Payment automatically processed
4. Exit gate opens upon successful payment
5. Digital receipt sent to customer app/email

### 3.2 Checkout Accuracy

Multi-modal verification ensures high accuracy:

```
Product Recognition Confidence Score:
- Vision confidence: 0.95+
- Shelf weight change: Confirmed
- RFID detection: Validated
→ Combined confidence: 0.995+

If confidence < 0.90:
→ Flag for manual review
→ Request customer confirmation via app
→ Activate additional cameras for verification
```

### 3.3 Session Management

```typescript
interface CheckoutSession {
  sessionId: string;
  customerId: string;
  entryTime: Date;
  exitTime?: Date;
  authMethod: 'app' | 'card' | 'biometric';
  virtualCart: CartItem[];
  totalAmount: number;
  status: 'active' | 'completed' | 'disputed' | 'cancelled';
  confidence: number;
}

interface CartItem {
  productId: string;
  productName: string;
  quantity: number;
  price: number;
  addedAt: Date;
  confidence: number;
  detectionMethod: 'vision' | 'rfid' | 'weight' | 'hybrid';
}
```

### 3.4 Dispute Resolution

Handling checkout discrepancies:

1. **Low Confidence Items**: Flagged for manual review
2. **Customer Dispute**: Review video footage (privacy-compliant)
3. **Missing Products**: Cross-reference multiple sensor data
4. **Refund Process**: Automated for verified disputes
5. **Audit Trail**: Complete session recording for 30 days

---

## 4. Computer Vision System

### 4.1 Camera Network

Dense camera deployment for complete coverage:

**Camera Specifications:**
- **Type**: RGB-D (color + depth) cameras
- **Resolution**: 4K (3840x2160) @ 60 FPS
- **Depth Sensing**: Time-of-Flight (ToF) or structured light
- **Field of View**: 110° horizontal, 70° vertical
- **Coverage**: Overlapping zones for redundancy
- **Placement**: Ceiling-mounted, angled for optimal view

**Camera Density:**
- Shopping area: 1 camera per 15-20 m²
- Hot zones: 1 camera per 8-10 m²
- Checkout area: 1 camera per 5 m²
- Total for 500m² store: 40-60 cameras

### 4.2 Product Recognition

Deep learning models for product identification:

**Model Architecture:**
```
Input: RGB-D Image (4K + Depth)
    ↓
Backbone: EfficientNet-B7 / ResNet-152
    ↓
Feature Extraction: Multi-scale features
    ↓
Detection Head: YOLO v8 / Faster R-CNN
    ↓
Classification: Product SKU identification
    ↓
Output: {product_id, confidence, bbox, depth}
```

**Training Data:**
- 100+ images per product SKU from multiple angles
- Augmentation: rotation, lighting, occlusion
- Regular retraining with new products
- Transfer learning for similar products

**Performance Metrics:**
- Detection accuracy: 99.5%+
- Processing time: <50ms per frame
- False positive rate: <0.1%
- False negative rate: <0.3%

### 4.3 Customer Tracking

Pose estimation and tracking algorithms:

**Tracking Pipeline:**
```
1. Person Detection: Identify all customers in frame
2. Pose Estimation: 17-keypoint skeleton (OpenPose/MediaPipe)
3. Re-identification: Match customer across cameras
4. Trajectory Tracking: Kalman filter for smooth paths
5. Action Recognition: Classify interactions (pick, return, examine)
```

**Privacy-Preserving Tracking:**
- Face blurring in stored footage
- Anonymous person IDs (no PII in tracking data)
- Skeleton-only representation (no facial features)
- Automatic deletion after session completion

### 4.4 Action Recognition

Detecting product interactions:

| Action | Visual Cues | Confidence Threshold |
|--------|-------------|---------------------|
| Pick up | Hand reaches, grasps, lifts | 0.95+ |
| Put back | Hand extends, releases to shelf | 0.95+ |
| Examine | Hand holds, no shelf change | 0.85+ |
| Compare | Multiple products in hands | 0.90+ |
| Move to cart | Product moves to bag/cart area | 0.92+ |

---


## 11. Personalized Recommendations

### 11.1 Recommendation Engine

AI-powered product suggestions:

**Recommendation Types:**

1. **Collaborative Filtering**: "Customers who bought X also bought Y"
2. **Content-Based**: Based on product attributes and past purchases
3. **Contextual**: Time, location, weather-based
4. **Complementary**: Items that go together (bread + butter)
5. **Upsell**: Premium version of viewed product
6. **Cross-sell**: Related category products

**Algorithm:**
```typescript
interface RecommendationRequest {
  customerId: string;
  context: {
    currentLocation?: Zone;
    cartItems?: string[]; // Product IDs
    time: Date;
    weather?: WeatherCondition;
  };
  limit: number; // Max recommendations
}

interface Recommendation {
  productId: string;
  score: number; // 0-1 relevance score
  reason: string; // Explanation
  type: 'collaborative' | 'content' | 'contextual' | 'complementary';
}

function generateRecommendations(request: RecommendationRequest): Recommendation[] {
  const customer = getCustomerProfile(request.customerId);
  const recommendations: Recommendation[] = [];

  // 1. Collaborative filtering
  const collaborative = getCollaborativeRecommendations(customer, 10);
  recommendations.push(...collaborative);

  // 2. Content-based
  const contentBased = getContentBasedRecommendations(customer.preferences, 10);
  recommendations.push(...contentBased);

  // 3. Contextual
  if (request.context.currentLocation) {
    const contextual = getContextualRecommendations(
      request.context.currentLocation,
      request.context.time,
      5
    );
    recommendations.push(...contextual);
  }

  // 4. Complementary to cart items
  if (request.context.cartItems?.length) {
    const complementary = getComplementaryProducts(request.context.cartItems, 5);
    recommendations.push(...complementary);
  }

  // Deduplicate and score
  const unique = deduplicateAndRank(recommendations);

  // Filter out items already in cart
  const filtered = unique.filter(r =>
    !request.context.cartItems?.includes(r.productId)
  );

  // Return top N
  return filtered.slice(0, request.limit);
}
```

### 11.2 Customer Profiling

Build customer preference profiles:

```typescript
interface CustomerProfile {
  customerId: string;
  demographics: {
    ageGroup?: string; // "18-24", "25-34", etc.
    location?: string;
  };
  preferences: {
    categories: Map<string, number>; // Category → affinity score
    brands: Map<string, number>;
    priceRange: { min: number; max: number; };
    dietaryRestrictions: string[]; // "vegetarian", "gluten-free"
  };
  purchaseHistory: {
    totalPurchases: number;
    avgBasketSize: number;
    avgVisitFrequency: number; // days
    favoriteProducts: string[];
    lastPurchase: Date;
  };
  behavior: {
    avgDwellTime: number; // seconds
    preferredShoppingTime: string; // "morning", "evening"
    avgPathLength: number; // meters
    visitedZones: Map<string, number>; // Zone → visit count
  };
  loyaltyTier: 'bronze' | 'silver' | 'gold' | 'platinum';
}

// Update profile based on session
function updateCustomerProfile(customerId: string, session: CheckoutSession) {
  const profile = getCustomerProfile(customerId);

  // Update purchase history
  profile.purchaseHistory.totalPurchases++;
  profile.purchaseHistory.lastPurchase = session.exitTime;
  profile.purchaseHistory.avgBasketSize =
    (profile.purchaseHistory.avgBasketSize * (profile.purchaseHistory.totalPurchases - 1) +
     session.virtualCart.length) / profile.purchaseHistory.totalPurchases;

  // Update category preferences
  for (const item of session.virtualCart) {
    const product = getProduct(item.productId);
    const currentAffinity = profile.preferences.categories.get(product.category) || 0;
    profile.preferences.categories.set(product.category, currentAffinity + 1);
  }

  // Update behavior
  const sessionDuration =
    (session.exitTime.getTime() - session.entryTime.getTime()) / 1000;
  profile.behavior.avgDwellTime =
    (profile.behavior.avgDwellTime * (profile.purchaseHistory.totalPurchases - 1) +
     sessionDuration) / profile.purchaseHistory.totalPurchases;

  saveCustomerProfile(profile);
}
```

### 11.3 Real-Time Suggestions

In-store and app notifications:

**Trigger Points:**
```
1. Zone Entry:
   Customer enters produce section
   → "Fresh organic apples on sale today! 🍎"

2. Product Proximity:
   Customer within 2m of promoted product
   → "Try our new yogurt flavor! Sample available."

3. Cart Analysis:
   Customer has pasta in cart, no sauce
   → "Don't forget pasta sauce! Aisle 7."

4. Complementary:
   Customer has burger buns in cart
   → "Burger patties on promotion, 20% off today!"

5. Abandonment Prevention:
   Customer dwells long, no items picked
   → "Need help finding something? Ask our staff!"
```

**Delivery Channels:**
- **Mobile App**: Push notifications
- **Digital Signage**: Personalized displays as customer passes
- **Smart Cart Screen**: Recommendations on cart display
- **ESL**: Highlighted pricing near customer

---


## 13. Payment Processing

### 13.1 Payment Methods

Multiple payment options:

| Method | Description | Authentication | Processing Time |
|--------|-------------|----------------|-----------------|
| App-linked | Pre-registered card/account | Biometric, PIN | <1 second |
| Tap-to-pay | NFC credit/debit card | Card chip | 2-3 seconds |
| QR Code | Mobile wallet (Alipay, WeChat Pay) | App authentication | 2-4 seconds |
| Biometric | Face, fingerprint payment | Liveness detection | 1-2 seconds |
| Traditional | Cashier checkout | Signature, PIN | 30-60 seconds |

### 13.2 Automated Payment Flow

Walk-out payment process:

```
1. Customer Identification (Entry):
   - Scan app QR code, OR
   - Tap payment card at gate, OR
   - Face recognition (if enrolled)
   → Session created, payment method linked

2. Shopping:
   - Items tracked via computer vision + sensors
   - Virtual cart updated in real-time
   - Running total calculated

3. Exit Process:
   - Customer approaches exit gate
   - System finalizes cart (grace period for last-second adds/removes)
   - Total calculated with taxes, discounts
   - Payment authorization requested

4. Payment Authorization:
   - Linked payment method charged
   - Transaction processed (tokenized, PCI-compliant)
   - Authorization received (or declined)

5. Exit Confirmation:
   - If approved → Gate opens, receipt sent
   - If declined → Gate closed, alternative payment requested
   - Random audit (10-20% of transactions)

6. Post-Transaction:
   - Digital receipt emailed/app-delivered
   - Loyalty points awarded
   - Transaction logged for audit
```

### 13.3 Security Measures

PCI-DSS compliant payment security:

```typescript
interface PaymentSecurity {
  tokenization: {
    enabled: true; // Never store raw card numbers
    provider: 'stripe' | 'braintree' | 'adyen';
  };
  encryption: {
    inTransit: 'TLS 1.3';
    atRest: 'AES-256';
  };
  authentication: {
    twoFactor: boolean;
    biometric: boolean;
    deviceFingerprinting: boolean;
  };
  fraudDetection: {
    velocityChecks: boolean; // Limit transactions per time period
    geolocation: boolean; // Verify location vs. card
    behaviorAnalysis: boolean; // AI-based anomaly detection
  };
  compliance: {
    pciDss: 'Level 1';
    gdpr: boolean;
    ccpa: boolean;
  };
}

// Fraud detection
function detectFraud(transaction: Transaction): FraudAssessment {
  let riskScore = 0;
  const flags: string[] = [];

  // Velocity check
  const recentTransactions = getRecentTransactions(
    transaction.customerId,
    3600 // Last hour
  );
  if (recentTransactions.length > 5) {
    riskScore += 30;
    flags.push('High transaction frequency');
  }

  // Amount check
  const avgTransaction = getAvgTransactionAmount(transaction.customerId);
  if (transaction.amount > avgTransaction * 3) {
    riskScore += 20;
    flags.push('Unusually high amount');
  }

  // Location check
  const customerLocation = getCustomerLocation(transaction.customerId);
  const cardLocation = getCardIssuedCountry(transaction.paymentMethodId);
  if (customerLocation !== cardLocation) {
    riskScore += 10;
    flags.push('Location mismatch');
  }

  // Behavior analysis (AI model)
  const behaviorScore = mlModel.predict(transaction);
  riskScore += behaviorScore * 40;

  return {
    riskScore: riskScore, // 0-100
    riskLevel: riskScore < 30 ? 'low' : riskScore < 70 ? 'medium' : 'high',
    flags: flags,
    action: riskScore > 70 ? 'decline' : riskScore > 40 ? 'review' : 'approve'
  };
}
```

### 13.4 Dispute Handling

Process for customer disputes:

```
1. Customer Reports Dispute:
   - Claim: "Charged for items I didn't take"
   - Evidence: "Receipt shows 3 apples, I only took 2"

2. Automated Review:
   - Pull session video footage (privacy-compliant)
   - Review sensor data (shelf weight, RFID)
   - Check computer vision logs with confidence scores

3. Evidence Analysis:
   - Vision log: 3 apples detected (confidence: 0.87, 0.92, 0.74)
   - Shelf sensor: Weight change = 3 × 180g
   - RFID: 3 tags detected
   → Conflict: 3 sensors agree vs. customer claim

4. Human Review (if needed):
   - Customer service reviews video
   - Third apple confidence low (0.74)
   - Video shows customer examined but replaced third apple

5. Resolution:
   - Refund customer for 1 apple
   - Adjust ML model (retrain with this case)
   - Thank customer for feedback

6. Continuous Improvement:
   - Use dispute cases to improve vision models
   - Adjust confidence thresholds
   - Reduce future disputes
```

---


