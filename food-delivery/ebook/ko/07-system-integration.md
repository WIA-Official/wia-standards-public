# 7장: 시스템 통합

---

## 학습 목표

이 장을 마치면 다음을 할 수 있습니다:

- 레스토랑 POS 시스템과 배달 플랫폼을 통합하기
- 결제 게이트웨이를 구현하여 안전한 거래 처리하기
- 지도 서비스 API를 활용하여 경로 최적화하기
- IoT 센서를 통합하여 실시간 온도 모니터링하기
- 여러 배달 플랫폼의 주문을 통합 관리하기
- 각 통합의 보안 및 오류 처리 구현하기

---

## 7.1 개요

이 장은 음식 배달 운영에 필수적인 외부 시스템과의 통합을 다룹니다: 레스토랑 POS 시스템, 결제 게이트웨이, 지도 서비스 및 IoT 플랫폼.

성공적인 통합은 배달 플랫폼과 파트너 시스템 간의 원활한 데이터 흐름을 가능하게 합니다.

**통합이 중요한 이유:**
- **자동화**: 수동 데이터 입력 제거
- **정확성**: 인간 오류 감소
- **효율성**: 실시간 정보 동기화
- **확장성**: 더 많은 파트너와 쉽게 연결
- **투명성**: 모든 이해관계자에게 가시성 제공

---

## 7.2 레스토랑 POS 통합

### 7.2.1 일반 POS 시스템

**주요 플랫폼:**
- Toast (30% 시장 점유율)
- Square (25%)
- Clover (15%)
- Aloha (10%)
- 기타 (20%)

**통합 방법:**

1. **REST API**: 대부분의 현대 POS 시스템
   - 장점: 표준화, 실시간, 양방향 통신
   - 단점: API 키 관리 필요

2. **웹훅**: 실시간 주문 알림
   - 장점: 즉각적인 푸시 알림
   - 단점: 웹훅 엔드포인트 관리 필요

3. **ODATA**: Open Data Protocol
   - 장점: 표준화된 쿼리 언어
   - 단점: 복잡한 설정

4. **FTP/SFTP**: 레거시 시스템 (파일 교환)
   - 장점: 간단한 구현
   - 단점: 실시간 아님, 배치 처리만 가능

### 7.2.2 주문 흐름 통합

**레스토랑 → 배달 플랫폼:**

```typescript
// POS 시스템이 배달 플랫폼에 새 주문 전송
interface POSOrder {
  posOrderId: string;
  restaurantId: string;
  items: POSItem[];
  subtotal: number;
  tax: number;
  customer: {
    name: string;
    phone: string;
    address: string;
  };
  pickupTime: Date;
  specialInstructions?: string;
}

// POS에서 배달 플랫폼으로의 웹훅 예제
app.post('/webhooks/pos/order-ready', async (req, res) => {
  const { posOrderId, restaurantId, readyTime } = req.body;

  // 해당 배달 주문 찾기
  const order = await Order.findOne({
    externalId: posOrderId,
    restaurantId: restaurantId
  });

  if (order) {
    // 주문 상태 업데이트
    await order.updateStatus('ready');

    // 배정된 배달원에게 알림
    await notifyDriver(order.driverId, {
      type: 'order_ready',
      orderId: order.id,
      restaurantName: order.restaurant.name,
      pickupAddress: order.pickupLocation.address
    });

    // 고객에게 알림 (음식 준비 완료)
    await notifyCustomer(order.customerId, {
      type: 'food_ready',
      orderId: order.id,
      estimatedPickupTime: readyTime
    });
  }

  res.json({ success: true });
});
```

**배달 플랫폼 → POS:**

```typescript
// 배달원 도착 시 POS에 알림
async function notifyPOSDriverArrived(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'driver_arrived',
    orderId: order.externalId,
    driverName: order.driver.name,
    driverPhoto: order.driver.photo,
    estimatedPickupTime: new Date(Date.now() + 5 * 60000)  // +5분
  }, {
    headers: {
      'Authorization': `Bearer ${posConfig.apiKey}`,
      'Content-Type': 'application/json'
    }
  });
}

// 주문 픽업 시 POS에 알림
async function notifyPOSPickedUp(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'order_picked_up',
    orderId: order.externalId,
    timestamp: new Date(),
    driverId: order.driverId,
    estimatedDeliveryTime: order.estimatedDelivery
  }, {
    headers: {
      'Authorization': `Bearer ${posConfig.apiKey}`
    }
  });
}

// 배달 완료 시 POS에 알림
async function notifyPOSDelivered(order: Order) {
  const posConfig = await getPOSConfig(order.restaurantId);

  await axios.post(posConfig.webhookUrl, {
    event: 'order_delivered',
    orderId: order.externalId,
    deliveryTime: order.actualDelivery,
    proofOfDelivery: {
      photo: order.photos[0].url,
      signature: order.signature,
      location: order.deliveryLocation
    }
  }, {
    headers: {
      'Authorization': `Bearer ${posConfig.apiKey}`
    }
  });
}
```

### 7.2.3 메뉴 동기화

**POS에서 레스토랑 메뉴 동기화:**

```typescript
interface MenuItem {
  id: string;
  name: string;
  description: string;
  category: string;
  price: number;
  available: boolean;
  modifiers?: MenuModifier[];
  allergens?: string[];
  temperature: 'hot' | 'cold' | 'ambient' | 'frozen';
  prepTime: number;  // 분
}

interface MenuModifier {
  id: string;
  name: string;
  options: ModifierOption[];
  required: boolean;
  maxSelections: number;
}

interface ModifierOption {
  id: string;
  name: string;
  priceAdjustment: number;  // 센트
}

async function syncMenuFromPOS(restaurantId: string) {
  const posConfig = await getPOSConfig(restaurantId);

  try {
    // POS에서 메뉴 가져오기
    const response = await axios.get(
      `${posConfig.apiUrl}/menu`,
      {
        headers: {
          'Authorization': `Bearer ${posConfig.apiKey}`
        },
        timeout: 10000  // 10초 타임아웃
      }
    );

    const posMenu = response.data.items;

    // POS 메뉴를 우리 형식으로 변환
    const menuItems = posMenu.map(item => ({
      id: generateId(),
      externalId: item.id,
      restaurantId: restaurantId,
      name: item.name,
      description: item.description,
      category: item.category,
      price: item.price,
      available: item.available && item.in_stock,
      modifiers: transformModifiers(item.modifiers),
      allergens: parseAllergens(item.allergen_info),
      temperature: inferTemperature(item.category, item.name),
      prepTime: item.prep_time || 15,  // 기본 15분
      syncedAt: new Date()
    }));

    // 메뉴 대량 업데이트
    await MenuItem.bulkWrite(
      menuItems.map(item => ({
        updateOne: {
          filter: {
            externalId: item.externalId,
            restaurantId: item.restaurantId
          },
          update: { $set: item },
          upsert: true
        }
      }))
    );

    console.log(`${restaurantId}에 대해 ${menuItems.length}개 항목 동기화 완료`);

    // 동기화 성공 기록
    await SyncLog.create({
      restaurantId: restaurantId,
      type: 'menu_sync',
      status: 'success',
      itemCount: menuItems.length,
      timestamp: new Date()
    });

  } catch (error) {
    console.error('메뉴 동기화 오류:', error);

    // 동기화 실패 기록
    await SyncLog.create({
      restaurantId: restaurantId,
      type: 'menu_sync',
      status: 'failed',
      error: error.message,
      timestamp: new Date()
    });

    throw error;
  }
}

// 온도 요구사항 추론
function inferTemperature(category: string, name: string): string {
  const hotKeywords = ['soup', 'pizza', 'hot', 'grilled', 'fried'];
  const coldKeywords = ['salad', 'ice cream', 'smoothie', 'cold'];
  const frozenKeywords = ['frozen', 'ice cream'];

  const lowerName = name.toLowerCase();
  const lowerCategory = category.toLowerCase();

  if (frozenKeywords.some(k => lowerName.includes(k))) return 'frozen';
  if (coldKeywords.some(k => lowerName.includes(k) || lowerCategory.includes(k))) return 'cold';
  if (hotKeywords.some(k => lowerName.includes(k) || lowerCategory.includes(k))) return 'hot';

  return 'ambient';
}

// 알레르겐 정보 파싱
function parseAllergens(allergenInfo: string): string[] {
  const allergenMap = {
    'milk': '유제품',
    'eggs': '계란',
    'peanuts': '땅콩',
    'tree nuts': '견과류',
    'wheat': '밀',
    'soy': '콩',
    'shellfish': '갑각류',
    'fish': '어류'
  };

  if (!allergenInfo) return [];

  return Object.keys(allergenMap)
    .filter(allergen => allergenInfo.toLowerCase().includes(allergen))
    .map(allergen => allergenMap[allergen]);
}

// 15분마다 동기화 실행
setInterval(async () => {
  try {
    const activeRestaurants = await Restaurant.find({ status: 'active' });

    for (const restaurant of activeRestaurants) {
      await syncMenuFromPOS(restaurant.id);

      // 레스토랑 간 1초 대기 (API 속도 제한 준수)
      await new Promise(resolve => setTimeout(resolve, 1000));
    }
  } catch (error) {
    console.error('메뉴 동기화 스케줄 오류:', error);
  }
}, 15 * 60 * 1000);  // 15분
```

---

## 7.3 결제 게이트웨이 통합

### 7.3.1 지원되는 게이트웨이

주요 결제 게이트웨이 비교:

| 게이트웨이 | 장점 | 단점 | 수수료 |
|---------|------|------|--------|
| **Stripe** | 최고의 API, 개발자 친화적 | 일부 국가에서 제한적 | 2.9% + $0.30 |
| **Square** | 통합 POS + 결제 | 에코시스템 종속 | 2.6% + $0.10 |
| **PayPal/Braintree** | 글로벌 도달 범위 | 복잡한 API | 2.9% + $0.30 |
| **Adyen** | 국제 지원 우수 | 복잡한 설정 | 협상 가능 |

**권장사항**: 대부분의 경우 Stripe 사용 (API 품질, 문서화, 개발자 경험 우수)

### 7.3.2 결제 흐름

**4단계 결제 프로세스:**

1. **승인 (Authorization)**: 자금 보류
2. **캡처 (Capture)**: 실제 청구
3. **분할 (Split)**: 레스토랑, 배달원, 플랫폼에 분배
4. **환불 (Refund)**: 필요시 반환

**1. 승인 (자금 보류):**

```typescript
import Stripe from 'stripe';
const stripe = new Stripe(process.env.STRIPE_SECRET_KEY);

async function authorizePayment(order: Order): Promise<string> {
  try {
    // 결제 인텐트 생성 (승인)
    const paymentIntent = await stripe.paymentIntents.create({
      amount: order.total,  // 센트 단위
      currency: 'usd',
      customer: order.customer.stripeCustomerId,
      payment_method: order.customer.defaultPaymentMethod,
      capture_method: 'manual',  // 아직 캡처하지 않음
      description: `주문 ${order.confirmationCode}`,
      metadata: {
        orderId: order.id,
        restaurantId: order.restaurantId,
        customerId: order.customerId,
        orderType: 'food_delivery'
      }
    });

    // 결제 인텐트 확인 (카드 승인)
    const confirmed = await stripe.paymentIntents.confirm(
      paymentIntent.id,
      {
        payment_method: order.customer.defaultPaymentMethod,
        return_url: `${process.env.APP_URL}/orders/${order.id}/payment-complete`
      }
    );

    if (confirmed.status === 'requires_capture') {
      // 승인 성공
      await order.update({
        paymentIntentId: confirmed.id,
        paymentStatus: 'authorized',
        authorizedAt: new Date()
      });

      console.log(`주문 ${order.id} 결제 승인 완료`);
      return confirmed.id;

    } else if (confirmed.status === 'requires_action') {
      // 3D Secure 등 추가 인증 필요
      throw new Error('REQUIRES_ACTION');

    } else {
      throw new Error(`결제 승인 실패: ${confirmed.status}`);
    }

  } catch (error) {
    console.error('결제 승인 오류:', error);

    // 주문 상태 업데이트
    await order.update({
      paymentStatus: 'failed',
      paymentError: error.message
    });

    throw error;
  }
}
```

**2. 캡처 (카드 청구):**

```typescript
// 배달 완료 후 결제 캡처
async function capturePayment(order: Order): Promise<void> {
  try {
    // 승인된 결제 캡처
    const captured = await stripe.paymentIntents.capture(
      order.paymentIntentId,
      {
        amount_to_capture: order.total  // 전체 금액 또는 부분 금액
      }
    );

    if (captured.status === 'succeeded') {
      await order.update({
        paymentStatus: 'captured',
        capturedAt: new Date(),
        paidAt: new Date()
      });

      console.log(`주문 ${order.id} 결제 캡처 완료: $${order.total / 100}`);

      // 레스토랑, 배달원, 플랫폼에 결제 분할
      await splitPayment(order, captured.id);

      // 영수증 발송
      await sendReceipt(order);

    } else {
      throw new Error(`결제 캡처 실패: ${captured.status}`);
    }

  } catch (error) {
    console.error('결제 캡처 오류:', error);

    await order.update({
      paymentStatus: 'capture_failed',
      paymentError: error.message
    });

    throw error;
  }
}
```

**3. 분할 결제 (정산):**

```typescript
async function splitPayment(order: Order, chargeId: string) {
  try {
    // 금액 계산
    const restaurantAmount = order.subtotal + order.tax;  // 음식 + 세금
    const driverAmount = order.deliveryFee + order.tip;    // 배달비 + 팁
    const platformAmount = order.serviceFee;               // 플랫폼 수수료

    console.log(`결제 분할: 레스토랑=$${restaurantAmount/100}, 배달원=$${driverAmount/100}, 플랫폼=$${platformAmount/100}`);

    // 레스토랑에 이체
    const restaurantTransfer = await stripe.transfers.create({
      amount: restaurantAmount,
      currency: 'usd',
      destination: order.restaurant.stripeAccountId,
      source_transaction: chargeId,
      description: `주문 ${order.confirmationCode}`,
      metadata: {
        orderId: order.id,
        restaurantId: order.restaurantId,
        type: 'restaurant_payment'
      }
    });

    // 배달원에 이체
    const driverTransfer = await stripe.transfers.create({
      amount: driverAmount,
      currency: 'usd',
      destination: order.driver.stripeAccountId,
      source_transaction: chargeId,
      description: `배달 ${order.confirmationCode}`,
      metadata: {
        orderId: order.id,
        driverId: order.driverId,
        type: 'driver_payment'
      }
    });

    // 플랫폼은 서비스 수수료 자동 보유

    // 정산 완료 기록
    await order.update({
      settlementStatus: 'completed',
      settledAt: new Date(),
      restaurantTransferId: restaurantTransfer.id,
      driverTransferId: driverTransfer.id
    });

    // 정산 알림 발송
    await notifySettlement(order, {
      restaurant: restaurantAmount,
      driver: driverAmount,
      platform: platformAmount
    });

    console.log(`주문 ${order.id} 정산 완료`);

  } catch (error) {
    console.error('결제 분할 오류:', error);

    await order.update({
      settlementStatus: 'failed',
      settlementError: error.message
    });

    // 정산 실패 알림
    await notifySettlementFailure(order, error);

    throw error;
  }
}
```

**4. 환불:**

```typescript
async function refundOrder(
  order: Order,
  reason: string,
  amount?: number
): Promise<void> {
  try {
    const refundAmount = amount || order.total;

    // 부분 환불인 경우 금액 검증
    if (amount && amount > order.total) {
      throw new Error('환불 금액이 주문 금액을 초과할 수 없습니다');
    }

    console.log(`주문 ${order.id} 환불 처리: $${refundAmount/100}`);

    const refund = await stripe.refunds.create({
      payment_intent: order.paymentIntentId,
      amount: refundAmount,
      reason: 'requested_by_customer',
      metadata: {
        orderId: order.id,
        reason: reason,
        refundedBy: 'system'
      }
    });

    // 환불 처리된 경우
    if (refund.status === 'succeeded') {
      await order.update({
        refundAmount: refundAmount,
        refundStatus: 'completed',
        refundReason: reason,
        refundedAt: new Date(),
        refundId: refund.id
      });

      // 고객에게 환불 알림
      await notifyCustomer(order.customerId, {
        type: 'refund_processed',
        amount: refundAmount,
        reason: reason
      });

      console.log(`주문 ${order.id} 환불 완료`);

    } else {
      throw new Error(`환불 실패: ${refund.status}`);
    }

  } catch (error) {
    console.error('환불 오류:', error);

    await order.update({
      refundStatus: 'failed',
      refundError: error.message
    });

    throw error;
  }
}
```

---

## 7.4 지도 서비스 통합

### 7.4.1 Google Maps Platform

**사용되는 서비스:**
- **Geocoding API**: 주소 → 좌표
- **Reverse Geocoding**: 좌표 → 주소
- **Directions API**: 경로 계산
- **Distance Matrix API**: 대량 거리 계산
- **Roads API**: 도로에 스냅
- **Places API**: 주소 자동완성

**구현:**

```typescript
import { Client } from '@googlemaps/google-maps-services-js';
const mapsClient = new Client({});

// 주소 지오코딩
async function geocodeAddress(address: string) {
  try {
    const response = await mapsClient.geocode({
      params: {
        address: address,
        key: process.env.GOOGLE_MAPS_API_KEY,
        language: 'ko'  // 한국어 결과
      }
    });

    if (response.data.results.length > 0) {
      const result = response.data.results[0];

      return {
        latitude: result.geometry.location.lat,
        longitude: result.geometry.location.lng,
        formattedAddress: result.formatted_address,
        placeId: result.place_id,
        verified: true,
        addressComponents: {
          street: extractComponent(result, 'route'),
          city: extractComponent(result, 'locality'),
          state: extractComponent(result, 'administrative_area_level_1'),
          postalCode: extractComponent(result, 'postal_code'),
          country: extractComponent(result, 'country')
        }
      };
    } else {
      return {
        verified: false,
        error: '주소를 찾을 수 없습니다'
      };
    }

  } catch (error) {
    console.error('지오코딩 오류:', error);
    throw error;
  }
}

// 주소 구성 요소 추출
function extractComponent(result: any, type: string): string {
  const component = result.address_components.find(
    c => c.types.includes(type)
  );
  return component?.long_name || '';
}

// 경로 계산
async function calculateRoute(
  origin: { lat: number; lng: number },
  destination: { lat: number; lng: number },
  departureTime?: Date
) {
  try {
    const response = await mapsClient.directions({
      params: {
        origin: `${origin.lat},${origin.lng}`,
        destination: `${destination.lat},${destination.lng}`,
        mode: 'bicycling',  // 자전거, 전기자전거, 스쿠터
        departure_time: departureTime || new Date(),
        traffic_model: 'best_guess',
        alternatives: true,  // 대체 경로 포함
        key: process.env.GOOGLE_MAPS_API_KEY,
        language: 'ko'
      }
    });

    if (response.data.routes.length > 0) {
      const route = response.data.routes[0];
      const leg = route.legs[0];

      return {
        distance: leg.distance.value / 1000,  // 미터 → km
        duration: leg.duration.value / 60,     // 초 → 분
        durationInTraffic: leg.duration_in_traffic?.value / 60,
        polyline: route.overview_polyline.points,
        bounds: route.bounds,
        steps: leg.steps.map(step => ({
          instruction: step.html_instructions.replace(/<[^>]*>/g, ''),
          distance: step.distance.value / 1000,
          duration: step.duration.value / 60,
          startLocation: step.start_location,
          endLocation: step.end_location,
          maneuver: step.maneuver
        })),
        alternativeRoutes: response.data.routes.slice(1).map(r => ({
          distance: r.legs[0].distance.value / 1000,
          duration: r.legs[0].duration.value / 60,
          polyline: r.overview_polyline.points
        }))
      };
    } else {
      throw new Error('경로를 찾을 수 없습니다');
    }

  } catch (error) {
    console.error('경로 계산 오류:', error);
    throw error;
  }
}

// 거리 매트릭스 (대량 계산)
async function calculateDistanceMatrix(
  origins: Location[],
  destinations: Location[]
) {
  try {
    const response = await mapsClient.distancematrix({
      params: {
        origins: origins.map(o => `${o.latitude},${o.longitude}`),
        destinations: destinations.map(d => `${d.latitude},${d.longitude}`),
        mode: 'bicycling',
        departure_time: new Date(),
        key: process.env.GOOGLE_MAPS_API_KEY
      }
    });

    // 매트릭스 파싱
    const matrix = [];
    for (let i = 0; i < origins.length; i++) {
      const row = [];
      for (let j = 0; j < destinations.length; j++) {
        const element = response.data.rows[i].elements[j];

        if (element.status === 'OK') {
          row.push({
            distance: element.distance.value / 1000,  // km
            duration: element.duration.value / 60,     // 분
            status: 'OK'
          });
        } else {
          row.push({
            distance: null,
            duration: null,
            status: element.status
          });
        }
      }
      matrix.push(row);
    }

    return matrix;

  } catch (error) {
    console.error('거리 매트릭스 오류:', error);
    throw error;
  }
}

// 주소 자동완성
async function autocompleteAddress(input: string, location?: Location) {
  try {
    const response = await mapsClient.placeAutocomplete({
      params: {
        input: input,
        key: process.env.GOOGLE_MAPS_API_KEY,
        language: 'ko',
        location: location ? `${location.latitude},${location.longitude}` : undefined,
        radius: location ? 5000 : undefined  // 5km 반경
      }
    });

    return response.data.predictions.map(pred => ({
      description: pred.description,
      placeId: pred.place_id,
      mainText: pred.structured_formatting.main_text,
      secondaryText: pred.structured_formatting.secondary_text
    }));

  } catch (error) {
    console.error('자동완성 오류:', error);
    throw error;
  }
}
```

### 7.4.2 Mapbox 통합

**Google Maps의 대안으로 유사한 기능 제공:**

```typescript
import MapboxClient from '@mapbox/mapbox-sdk/services/geocoding';
import MapboxDirections from '@mapbox/mapbox-sdk/services/directions';

const geocodingClient = MapboxClient({
  accessToken: process.env.MAPBOX_ACCESS_TOKEN
});

const directionsClient = MapboxDirections({
  accessToken: process.env.MAPBOX_ACCESS_TOKEN
});

// Mapbox로 지오코딩
async function mapboxGeocode(address: string) {
  try {
    const response = await geocodingClient
      .forwardGeocode({
        query: address,
        limit: 1,
        language: ['ko']
      })
      .send();

    if (response.body.features.length > 0) {
      const feature = response.body.features[0];

      return {
        latitude: feature.center[1],
        longitude: feature.center[0],
        formattedAddress: feature.place_name,
        context: feature.context,
        verified: true
      };
    } else {
      return {
        verified: false,
        error: '주소를 찾을 수 없습니다'
      };
    }

  } catch (error) {
    console.error('Mapbox 지오코딩 오류:', error);
    throw error;
  }
}

// Mapbox로 경로 계산
async function mapboxRoute(origin: Location, destination: Location) {
  try {
    const response = await directionsClient
      .getDirections({
        profile: 'cycling',  // 자전거 프로필
        waypoints: [
          { coordinates: [origin.longitude, origin.latitude] },
          { coordinates: [destination.longitude, destination.latitude] }
        ],
        geometries: 'geojson',
        overview: 'full',
        steps: true,
        language: 'ko',
        alternatives: true  // 대체 경로
      })
      .send();

    if (response.body.routes.length > 0) {
      const route = response.body.routes[0];

      return {
        distance: route.distance / 1000,  // 미터 → km
        duration: route.duration / 60,    // 초 → 분
        geometry: route.geometry,
        steps: route.legs[0].steps.map(step => ({
          instruction: step.maneuver.instruction,
          distance: step.distance / 1000,
          duration: step.duration / 60,
          maneuver: step.maneuver.type
        })),
        alternativeRoutes: response.body.routes.slice(1).map(r => ({
          distance: r.distance / 1000,
          duration: r.duration / 60,
          geometry: r.geometry
        }))
      };
    } else {
      throw new Error('경로를 찾을 수 없습니다');
    }

  } catch (error) {
    console.error('Mapbox 경로 오류:', error);
    throw error;
  }
}
```

---

## 7.5 IoT 센서 통합

### 7.5.1 온도 센서 설정

**지원되는 센서:**
- **Bluetooth Low Energy (BLE)** 센서
- **Wi-Fi** 지원 센서
- **셀룰러 IoT** 센서

**센서 유형별 비교:**

| 센서 유형 | 장점 | 단점 | 배터리 수명 |
|---------|------|------|-----------|
| BLE | 저전력, 저렴 | 스마트폰 필요 | 6-12개월 |
| Wi-Fi | 실시간, 독립적 | 높은 전력 소비 | 1-3개월 |
| 셀룰러 | 어디서나 작동 | 비싸고 데이터 요금 | 3-6개월 |

**예제: AWS IoT Core 통합:**

```typescript
import AWS from 'aws-sdk';
import mqtt from 'mqtt';

const iot = new AWS.Iot({ region: 'us-east-1' });
const iotData = new AWS.IotData({
  endpoint: process.env.AWS_IOT_ENDPOINT
});

// 새 센서 등록
async function registerSensor(sensorId: string, driverId: string) {
  try {
    // IoT 사물 생성
    await iot.createThing({
      thingName: sensorId,
      attributePayload: {
        attributes: {
          driverId: driverId,
          registeredAt: new Date().toISOString(),
          sensorType: 'temperature',
          manufacturer: 'TempSense Inc'
        }
      }
    }).promise();

    // 인증서 생성
    const cert = await iot.createKeysAndCertificate({
      setAsActive: true
    }).promise();

    // 정책을 인증서에 연결
    await iot.attachPolicy({
      policyName: 'TemperatureSensorPolicy',
      target: cert.certificateArn
    }).promise();

    // 인증서를 사물에 연결
    await iot.attachThingPrincipal({
      thingName: sensorId,
      principal: cert.certificateArn
    }).promise();

    console.log(`센서 ${sensorId} 등록 완료`);

    return {
      certificateArn: cert.certificateArn,
      certificatePem: cert.certificatePem,
      privateKey: cert.keyPair.PrivateKey,
      publicKey: cert.keyPair.PublicKey
    };

  } catch (error) {
    console.error('센서 등록 오류:', error);
    throw error;
  }
}

// 센서 데이터 구독
function subscribeSensorData(callback: (reading: any) => void) {
  const client = mqtt.connect(`mqtts://${process.env.AWS_IOT_ENDPOINT}`, {
    clientId: `server_${Date.now()}`,
    cert: process.env.IOT_CERT,
    key: process.env.IOT_PRIVATE_KEY,
    ca: process.env.IOT_CA,
    reconnectPeriod: 1000,
    keepalive: 60
  });

  client.on('connect', () => {
    console.log('AWS IoT에 연결됨');

    // 모든 센서 구독
    client.subscribe('sensors/+/temperature', (err) => {
      if (err) {
        console.error('구독 오류:', err);
      } else {
        console.log('센서 토픽 구독 완료');
      }
    });
  });

  client.on('message', (topic, message) => {
    try {
      // 온도 판독값 파싱
      const reading = JSON.parse(message.toString());
      const sensorId = topic.split('/')[1];

      callback({
        sensorId,
        ...reading
      });
    } catch (error) {
      console.error('메시지 파싱 오류:', error);
    }
  });

  client.on('error', (error) => {
    console.error('MQTT 클라이언트 오류:', error);
  });

  client.on('offline', () => {
    console.warn('MQTT 클라이언트 오프라인');
  });

  return client;
}

// 온도 판독값 처리
subscribeSensorData(async (reading) => {
  const {
    sensorId,
    temperature,
    humidity,
    battery,
    timestamp
  } = reading;

  console.log(`센서 ${sensorId}: ${temperature}°C, 배터리 ${battery}%`);

  // 연결된 주문 찾기
  const order = await Order.findOne({
    status: { $in: ['picked_up', 'in_transit'] },
    'driver.sensorId': sensorId
  });

  if (order) {
    // 판독값 저장
    await TemperatureReading.create({
      orderId: order.id,
      driverId: order.driverId,
      sensorId: sensorId,
      temperature: temperature,
      humidity: humidity,
      batteryLevel: battery,
      timestamp: new Date(timestamp),
      location: order.driver.location
    });

    // 경고 확인
    const monitor = new TemperatureMonitor();
    const alert = monitor.check_temperature(
      reading,
      order.temperatureRequirement
    );

    if (alert !== AlertLevel.NONE) {
      await monitor.handle_alert(alert, reading, order.id);
    }

    // 배터리 부족 경고
    if (battery < 20) {
      await notifyDriver(order.driverId, {
        type: 'low_battery',
        message: '온도 센서 배터리가 부족합니다 (20% 미만)',
        severity: 'warning'
      });
    }
  } else {
    console.warn(`센서 ${sensorId}에 대한 활성 주문 없음`);
  }
});
```

### 7.5.2 모바일 앱 센서 통합

**배달원 휴대폰에 Bluetooth로 연결되는 센서용:**

```typescript
// React Native 예제
import { BleManager, Device } from 'react-native-ble-plx';

const bleManager = new BleManager();

// 온도 센서 스캔
function scanForSensors(onSensorFound: (sensor: Device) => void) {
  console.log('온도 센서 스캔 시작...');

  bleManager.startDeviceScan(
    ['180D'],  // 온도 서비스 UUID
    { allowDuplicates: false },
    (error, device) => {
      if (error) {
        console.error('스캔 오류:', error);
        return;
      }

      if (device && device.name?.includes('TempSensor')) {
        console.log(`센서 발견: ${device.name} (${device.id})`);
        onSensorFound(device);
        bleManager.stopDeviceScan();
      }
    }
  );

  // 30초 후 스캔 중지
  setTimeout(() => {
    bleManager.stopDeviceScan();
    console.log('스캔 중지');
  }, 30000);
}

// 센서에 연결
async function connectSensor(deviceId: string) {
  try {
    console.log(`센서 ${deviceId}에 연결 중...`);

    const device = await bleManager.connectToDevice(deviceId, {
      timeout: 10000
    });

    console.log('센서 연결됨, 서비스 검색 중...');
    await device.discoverAllServicesAndCharacteristics();

    // 온도 알림 구독
    device.monitorCharacteristicForService(
      '180D',  // 서비스 UUID
      '2A1C',  // 온도 특성 UUID
      (error, characteristic) => {
        if (error) {
          console.error('모니터 오류:', error);
          return;
        }

        if (characteristic?.value) {
          const temperature = parseTemperature(characteristic.value);

          console.log(`온도: ${temperature}°C`);

          // 서버로 전송
          sendTemperatureReading({
            sensorId: deviceId,
            temperature: temperature,
            timestamp: new Date().toISOString()
          });
        }
      }
    );

    return device;

  } catch (error) {
    console.error('센서 연결 오류:', error);
    throw error;
  }
}

// BLE 온도 데이터 파싱
function parseTemperature(base64Value: string): number {
  const buffer = Buffer.from(base64Value, 'base64');

  // 16비트 부호 있는 정수 little-endian 가정
  const raw = buffer.readInt16LE(0);

  // 섭씨로 변환 (센서에 따라 다를 수 있음)
  return raw / 100;
}

// 서버로 판독값 전송
async function sendTemperatureReading(reading: any) {
  try {
    const response = await fetch(`${API_URL}/temperature-readings`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${authToken}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(reading)
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    console.log('온도 판독값 전송 완료');

  } catch (error) {
    console.error('판독값 전송 오류:', error);

    // 오프라인 저장 (나중에 동기화)
    await storeReadingOffline(reading);
  }
}

// 센서 연결 끊기
async function disconnectSensor(device: Device) {
  try {
    await bleManager.cancelDeviceConnection(device.id);
    console.log('센서 연결 해제됨');
  } catch (error) {
    console.error('연결 해제 오류:', error);
  }
}

// 사용 예제
export function TemperatureSensorScreen() {
  const [connectedSensor, setConnectedSensor] = useState<Device | null>(null);

  const handleScanPress = () => {
    scanForSensors(async (sensor) => {
      try {
        const device = await connectSensor(sensor.id);
        setConnectedSensor(device);
      } catch (error) {
        Alert.alert('오류', '센서 연결 실패');
      }
    });
  };

  return (
    <View>
      {!connectedSensor ? (
        <Button title="센서 스캔" onPress={handleScanPress} />
      ) : (
        <Text>센서 연결됨: {connectedSensor.name}</Text>
      )}
    </View>
  );
}
```

---

## 7.6 제3자 플랫폼 통합

### 7.6.1 멀티 플랫폼 통합

**여러 배달 플랫폼의 주문 통합:**

```typescript
// 통합 주문 프로세서
class OrderAggregator {
  private platforms = {
    ubereats: new UberEatsAdapter(),
    doordash: new DoorDashAdapter(),
    grubhub: new GrubHubAdapter(),
    baemin: new BaeminAdapter(),  // 배달의민족
    coupangeats: new CoupangEatsAdapter()  // 쿠팡이츠
  };

  async processIncomingOrder(
    platform: string,
    externalOrder: any
  ): Promise<Order> {
    const adapter = this.platforms[platform];

    if (!adapter) {
      throw new Error(`알 수 없는 플랫폼: ${platform}`);
    }

    console.log(`${platform}에서 주문 수신: ${externalOrder.id}`);

    // 표준 형식으로 변환
    const order = adapter.transform(externalOrder);

    // 시스템에 생성
    const created = await Order.create({
      ...order,
      source: platform,
      externalId: externalOrder.id,
      receivedAt: new Date()
    });

    console.log(`주문 ${created.id} 생성 완료 (출처: ${platform})`);

    // 레스토랑에 알림
    await notifyRestaurant(created);

    // 이용 가능한 배달원 찾기
    await findAvailableDriver(created);

    return created;
  }

  async updateExternalOrder(
    order: Order,
    status: string
  ) {
    const adapter = this.platforms[order.source];

    if (adapter) {
      try {
        await adapter.updateStatus(order.externalId, status);
        console.log(`${order.source}에 상태 업데이트: ${status}`);
      } catch (error) {
        console.error(`${order.source} 상태 업데이트 실패:`, error);
      }
    }
  }

  // 모든 플랫폼의 주문 가져오기
  async fetchAllOrders(): Promise<Order[]> {
    const allOrders = [];

    for (const [platform, adapter] of Object.entries(this.platforms)) {
      try {
        const orders = await adapter.fetchPendingOrders();
        allOrders.push(...orders);
        console.log(`${platform}에서 ${orders.length}개 주문 가져옴`);
      } catch (error) {
        console.error(`${platform} 주문 가져오기 실패:`, error);
      }
    }

    return allOrders;
  }
}

// 플랫폼별 어댑터
class UberEatsAdapter {
  private apiKey = process.env.UBER_EATS_API_KEY;
  private baseUrl = 'https://api.uber.com/v1/eats';

  transform(externalOrder: any): Partial<Order> {
    return {
      restaurantId: externalOrder.store.id,
      items: externalOrder.eater_order.items.map(item => ({
        name: item.title,
        quantity: item.quantity,
        price: item.price,
        modifiers: item.customizations
      })),
      deliveryLocation: {
        address: externalOrder.eater.delivery_address,
        latitude: externalOrder.eater.latitude,
        longitude: externalOrder.eater.longitude
      },
      customer: {
        name: externalOrder.eater.name,
        phone: externalOrder.eater.phone
      },
      subtotal: externalOrder.eater_order.subtotal,
      total: externalOrder.eater_order.total
    };
  }

  async updateStatus(externalId: string, status: string) {
    const uberStatus = this.mapStatusToUberEats(status);

    await axios.patch(
      `${this.baseUrl}/orders/${externalId}`,
      { status: uberStatus },
      {
        headers: {
          'Authorization': `Bearer ${this.apiKey}`
        }
      }
    );
  }

  async fetchPendingOrders(): Promise<any[]> {
    const response = await axios.get(
      `${this.baseUrl}/orders`,
      {
        params: { status: 'pending' },
        headers: {
          'Authorization': `Bearer ${this.apiKey}`
        }
      }
    );

    return response.data.orders;
  }

  private mapStatusToUberEats(status: string): string {
    const statusMap = {
      'confirmed': 'accepted',
      'preparing': 'preparing',
      'ready': 'ready_for_pickup',
      'picked_up': 'picked_up',
      'delivered': 'delivered'
    };

    return statusMap[status] || status;
  }
}

// 배달의민족 어댑터
class BaeminAdapter {
  private apiKey = process.env.BAEMIN_API_KEY;
  private baseUrl = 'https://api.baemin.com/v1';

  transform(externalOrder: any): Partial<Order> {
    return {
      restaurantId: externalOrder.shop_id,
      items: externalOrder.menu_list.map(item => ({
        name: item.menu_name,
        quantity: item.quantity,
        price: item.price,
        modifiers: item.options
      })),
      deliveryLocation: {
        address: externalOrder.delivery_address,
        latitude: externalOrder.latitude,
        longitude: externalOrder.longitude,
        detailedAddress: externalOrder.address_detail
      },
      customer: {
        name: externalOrder.customer_name,
        phone: externalOrder.customer_phone
      },
      subtotal: externalOrder.menu_total,
      deliveryFee: externalOrder.delivery_fee,
      total: externalOrder.total_amount
    };
  }

  async updateStatus(externalId: string, status: string) {
    const baeminStatus = this.mapStatusToBaemin(status);

    await axios.post(
      `${this.baseUrl}/orders/${externalId}/status`,
      { status: baeminStatus },
      {
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
          'Content-Type': 'application/json'
        }
      }
    );
  }

  async fetchPendingOrders(): Promise<any[]> {
    const response = await axios.get(
      `${this.baseUrl}/orders`,
      {
        params: { status: '접수대기' },
        headers: {
          'Authorization': `Bearer ${this.apiKey}`
        }
      }
    );

    return response.data.order_list;
  }

  private mapStatusToBaemin(status: string): string {
    const statusMap = {
      'confirmed': '접수완료',
      'preparing': '조리중',
      'ready': '픽업대기',
      'picked_up': '배달중',
      'delivered': '배달완료'
    };

    return statusMap[status] || status;
  }
}

// 사용 예제
const aggregator = new OrderAggregator();

// 웹훅으로 주문 수신
app.post('/webhooks/:platform/order', async (req, res) => {
  const { platform } = req.params;
  const externalOrder = req.body;

  try {
    const order = await aggregator.processIncomingOrder(platform, externalOrder);
    res.json({ success: true, orderId: order.id });
  } catch (error) {
    console.error('주문 처리 오류:', error);
    res.status(500).json({ error: error.message });
  }
});

// 주문 상태 업데이트 시 모든 플랫폼에 동기화
async function syncOrderStatus(order: Order, newStatus: string) {
  await order.updateStatus(newStatus);
  await aggregator.updateExternalOrder(order, newStatus);
}
```

---

## 7.7 오류 처리 및 재시도

### 7.7.1 재시도 전략

```typescript
// 지수 백오프로 재시도
async function retryWithBackoff<T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  baseDelay: number = 1000
): Promise<T> {
  let lastError: Error;

  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error;

      if (i < maxRetries - 1) {
        const delay = baseDelay * Math.pow(2, i);
        console.log(`재시도 ${i + 1}/${maxRetries} (${delay}ms 후)`);
        await new Promise(resolve => setTimeout(resolve, delay));
      }
    }
  }

  throw lastError;
}

// 사용 예제
const result = await retryWithBackoff(
  () => axios.get('https://api.example.com/data'),
  3,  // 최대 3회 재시도
  1000  // 기본 1초 지연
);
```

### 7.7.2 회로 차단기 패턴

```typescript
class CircuitBreaker {
  private failureCount = 0;
  private successCount = 0;
  private lastFailureTime: Date | null = null;
  private state: 'closed' | 'open' | 'half-open' = 'closed';

  constructor(
    private threshold: number = 5,
    private timeout: number = 60000,  // 1분
    private monitoringPeriod: number = 120000  // 2분
  ) {}

  async execute<T>(fn: () => Promise<T>): Promise<T> {
    if (this.state === 'open') {
      if (Date.now() - this.lastFailureTime.getTime() > this.timeout) {
        this.state = 'half-open';
        console.log('회로 차단기: half-open 상태');
      } else {
        throw new Error('회로 차단기 열림 - 서비스 일시적으로 사용 불가');
      }
    }

    try {
      const result = await fn();
      this.onSuccess();
      return result;
    } catch (error) {
      this.onFailure();
      throw error;
    }
  }

  private onSuccess() {
    this.failureCount = 0;

    if (this.state === 'half-open') {
      this.state = 'closed';
      console.log('회로 차단기: closed 상태 (복구됨)');
    }
  }

  private onFailure() {
    this.failureCount++;
    this.lastFailureTime = new Date();

    if (this.failureCount >= this.threshold) {
      this.state = 'open';
      console.log(`회로 차단기: open 상태 (${this.failureCount}회 실패)`);
    }
  }
}

// 사용 예제
const paymentsCircuitBreaker = new CircuitBreaker(5, 60000);

async function processPayment(order: Order) {
  return paymentsCircuitBreaker.execute(async () => {
    return await stripe.paymentIntents.capture(order.paymentIntentId);
  });
}
```

---

## 7.8 요약

이 장은 다음과의 통합을 다뤘습니다:

### 통합 유형:

1. **레스토랑 POS 시스템**
   - 주문 동기화 (양방향)
   - 메뉴 동기화 (실시간)
   - 상태 업데이트 (웹훅)

2. **결제 게이트웨이**
   - 승인 (자금 보류)
   - 캡처 (실제 청구)
   - 분할 결제 (정산)
   - 환불 처리

3. **지도 서비스**
   - 지오코딩 (주소 ↔ 좌표)
   - 경로 계산 (최적화)
   - 거리 매트릭스 (대량 계산)
   - 주소 자동완성

4. **IoT 센서**
   - AWS IoT Core 통합
   - Bluetooth 센서 연결
   - 실시간 온도 모니터링
   - 경고 처리

5. **제3자 플랫폼**
   - 멀티 플랫폼 주문 통합
   - 상태 동기화
   - 어댑터 패턴 사용

### 핵심 원칙:

- **표준 프로토콜**: REST API, 웹훅, MQTT
- **오류 처리**: 재시도, 회로 차단기, 폴백
- **보안**: API 키, 인증서, 암호화
- **모니터링**: 로깅, 경고, 대시보드

---

## 복습 문제

1. POS 통합의 주요 방법 4가지는 무엇인가요?
2. Stripe 결제 흐름의 4단계를 설명하세요.
3. 지오코딩과 역지오코딩의 차이는 무엇인가요?
4. BLE 온도 센서의 장단점은 무엇인가요?
5. 회로 차단기 패턴이 필요한 이유는 무엇인가요?
6. 결제 분할 시 레스토랑, 배달원, 플랫폼에 어떻게 금액을 배분하나요?
7. Google Maps와 Mapbox의 차이점은 무엇인가요?
8. 멀티 플랫폼 통합에서 어댑터 패턴을 사용하는 이유는 무엇인가요?

---

**다음 장**: [8장: 구현 가이드 →](08-implementation.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
