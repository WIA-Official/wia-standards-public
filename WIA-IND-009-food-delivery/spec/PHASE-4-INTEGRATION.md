# WIA-IND-009: PHASE 4 - INTEGRATION SPECIFICATION
## Food Delivery Platform Standard
### Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## Table of Contents
1. [Overview](#overview)
2. [Restaurant POS Integration](#restaurant-pos-integration)
3. [Payment Gateway Integration](#payment-gateway-integration)
4. [Mapping Services Integration](#mapping-services-integration)
5. [Communication Services](#communication-services)
6. [Analytics Platforms](#analytics-platforms)
7. [Third-Party Delivery](#third-party-delivery)
8. [Webhook Patterns](#webhook-patterns)

---

## Overview

Phase 4 specifies integration patterns with external systems and services. These integrations enable food delivery platforms to leverage existing infrastructure while maintaining interoperability.

**Integration Principles:**
- **Loose Coupling**: Minimize dependencies between systems
- **Fault Tolerance**: Graceful degradation when integrations fail
- **Abstraction**: Hide third-party specifics behind interfaces
- **Observability**: Comprehensive logging and monitoring
- **Security**: Secure credential management and data transmission

---

## Restaurant POS Integration

### Supported POS Systems

Common POS systems with standardized integration:
- Square POS
- Toast POS
- Clover
- Lightspeed
- Oracle Micros
- Aloha
- NCR Silver
- Custom/Generic POS

### Integration Patterns

**1. API-based Integration** (Preferred)
- Direct REST API calls
- OAuth authentication
- Webhook notifications

**2. File-based Integration**
- FTP/SFTP file exchange
- Scheduled synchronization
- CSV or JSON format

**3. Database Integration**
- Direct database access (least preferred)
- Read-only access recommended
- Requires VPN/secure connection

### Order Transmission to POS

**Send Order via API:**
```http
POST https://api.pos-system.com/v1/orders
Authorization: Bearer {pos_access_token}
Content-Type: application/json

{
  "externalOrderId": "ORD-2025-001234",
  "sourceChannel": "delivery_platform",
  "items": [
    {
      "sku": "PIZZA-MARG-L",
      "name": "Margherita Pizza Large",
      "quantity": 2,
      "price": 12.99,
      "modifiers": [...]
    }
  ],
  "customerInfo": {
    "name": "John Doe",
    "phone": "+1-555-123-4567"
  },
  "deliveryInfo": {
    "address": "123 Main St",
    "instructions": "Ring doorbell"
  },
  "totalAmount": 35.48
}
```

**POS Response:**
```json
{
  "posOrderId": "POS-789",
  "status": "accepted",
  "estimatedPrepTime": 20,
  "kitchenTicketPrinted": true,
  "confirmationNumber": "CONF-456"
}
```

### Menu Synchronization

**Pull Menu from POS:**
```http
GET https://api.pos-system.com/v1/menus/current
Authorization: Bearer {pos_access_token}
```

**Response:**
```json
{
  "menuId": "MENU-2025-12",
  "lastUpdated": "2025-12-27T10:00:00Z",
  "categories": [
    {
      "id": "CAT-PIZZA",
      "name": "Pizzas",
      "items": [
        {
          "sku": "PIZZA-MARG-L",
          "name": "Margherita Pizza",
          "price": 12.99,
          "available": true,
          "modifiers": [...]
        }
      ]
    }
  ]
}
```

**Menu Update Webhook:**
POS system notifies platform when menu changes:

```http
POST https://api.delivery-platform.com/webhooks/menu-update
X-POS-Signature: {hmac_signature}
Content-Type: application/json

{
  "event": "menu.updated",
  "restaurantId": "REST-001",
  "timestamp": "2025-12-27T11:30:00Z",
  "changes": [
    {
      "itemSku": "PIZZA-MARG-L",
      "field": "available",
      "oldValue": true,
      "newValue": false,
      "reason": "out_of_stock"
    }
  ]
}
```

### Order Status Updates

**POS notifies platform:**
```http
POST https://api.delivery-platform.com/webhooks/order-status
X-POS-Signature: {hmac_signature}
Content-Type: application/json

{
  "event": "order.status_changed",
  "externalOrderId": "ORD-2025-001234",
  "posOrderId": "POS-789",
  "status": "preparing",
  "estimatedCompletionTime": "2025-12-27T12:20:00Z"
}
```

### Adapter Pattern

For systems without native API, use adapter:

```javascript
class POSAdapter {
  async sendOrder(order) {
    // Transform WIA-IND-009 format to POS format
    const posOrder = this.transformOrder(order);
    return await this.posClient.createOrder(posOrder);
  }
  
  async getMenu() {
    const posMenu = await this.posClient.fetchMenu();
    // Transform POS format to WIA-IND-009 format
    return this.transformMenu(posMenu);
  }
}
```

---

## Payment Gateway Integration

### Supported Payment Providers

- Stripe
- PayPal
- Square Payments
- Braintree
- Adyen
- Authorize.net

### Payment Flow

**1. Create Payment Intent:**
```http
POST https://api.delivery-platform.com/v1/payments/intents
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "orderId": "ORD-2025-001234",
  "amount": 3548,
  "currency": "USD",
  "paymentMethod": "pm_card_visa",
  "capture": false
}
```

**Platform calls Stripe:**
```http
POST https://api.stripe.com/v1/payment_intents
Authorization: Bearer {stripe_secret_key}
Content-Type: application/x-www-form-urlencoded

amount=3548&
currency=usd&
payment_method=pm_card_visa&
capture_method=manual&
metadata[orderId]=ORD-2025-001234
```

**2. Authorize Payment:**
Payment intent created in `requires_confirmation` state

**3. Capture on Delivery:**
```http
POST https://api.stripe.com/v1/payment_intents/{intent_id}/capture
Authorization: Bearer {stripe_secret_key}
```

### Split Payments

**Stripe Connect for marketplace:**
```http
POST https://api.stripe.com/v1/payment_intents
Authorization: Bearer {stripe_secret_key}
Content-Type: application/x-www-form-urlencoded

amount=3548&
currency=usd&
application_fee_amount=400&
transfer_data[destination]={restaurant_stripe_account}
```

- Total: $35.48
- Platform fee: $4.00
- Restaurant receives: $31.48

### Refund Processing

```http
POST https://api.stripe.com/v1/refunds
Authorization: Bearer {stripe_secret_key}
Content-Type: application/x-www-form-urlencoded

payment_intent={intent_id}&
amount=3548&
reason=requested_by_customer&
metadata[orderId]=ORD-2025-001234
```

### Webhook Events

**Stripe notifies platform:**
```http
POST https://api.delivery-platform.com/webhooks/stripe
Stripe-Signature: {signature}
Content-Type: application/json

{
  "type": "payment_intent.succeeded",
  "data": {
    "object": {
      "id": "pi_xxx",
      "amount": 3548,
      "status": "succeeded",
      "metadata": {
        "orderId": "ORD-2025-001234"
      }
    }
  }
}
```

---

## Mapping Services Integration

### Supported Services

- Google Maps Platform
- Mapbox
- HERE Maps
- OpenStreetMap (via Nominatim)
- Apple Maps

### Geocoding

**Address to Coordinates:**
```http
GET https://maps.googleapis.com/maps/api/geocode/json?
address=123+Main+Street+New+York+NY&
key={google_api_key}
```

**Response:**
```json
{
  "results": [
    {
      "formatted_address": "123 Main St, New York, NY 10001, USA",
      "geometry": {
        "location": {
          "lat": 40.7589,
          "lng": -73.9851
        }
      },
      "place_id": "ChIJxxx"
    }
  ]
}
```

### Directions and Routing

**Calculate Route:**
```http
GET https://maps.googleapis.com/maps/api/directions/json?
origin=40.7580,-73.9855&
destination=40.7589,-73.9851&
mode=driving&
departure_time=now&
traffic_model=best_guess&
key={google_api_key}
```

**Response:**
```json
{
  "routes": [
    {
      "legs": [
        {
          "distance": {"value": 1200, "text": "1.2 km"},
          "duration": {"value": 420, "text": "7 mins"},
          "duration_in_traffic": {"value": 540, "text": "9 mins"},
          "steps": [...]
        }
      ]
    }
  ]
}
```

### Distance Matrix

**Batch Distance Calculation:**
```http
GET https://maps.googleapis.com/maps/api/distancematrix/json?
origins=40.7580,-73.9855|40.7590,-73.9860&
destinations=40.7589,-73.9851&
mode=driving&
key={google_api_key}
```

For driver assignment optimization.

### Map Display

**Static Map Image:**
```
https://maps.googleapis.com/maps/api/staticmap?
center=40.7589,-73.9851&
zoom=14&
size=600x400&
markers=color:red|40.7589,-73.9851&
key={google_api_key}
```

**Interactive Map (JavaScript):**
```javascript
const map = new google.maps.Map(document.getElementById('map'), {
  center: {lat: 40.7589, lng: -73.9851},
  zoom: 14
});

const marker = new google.maps.Marker({
  position: {lat: 40.7589, lng: -73.9851},
  map: map,
  title: 'Delivery Location'
});
```

---

## Communication Services

### SMS/Text Messages

**Twilio Integration:**
```http
POST https://api.twilio.com/2010-04-01/Accounts/{AccountSid}/Messages.json
Authorization: Basic {base64(AccountSid:AuthToken)}
Content-Type: application/x-www-form-urlencoded

To=+15551234567&
From=+15559876543&
Body=Your order ORD-2025-001234 is out for delivery!
```

### Email Services

**SendGrid Integration:**
```http
POST https://api.sendgrid.com/v3/mail/send
Authorization: Bearer {sendgrid_api_key}
Content-Type: application/json

{
  "personalizations": [
    {
      "to": [{"email": "customer@example.com"}],
      "subject": "Order Confirmation - ORD-2025-001234"
    }
  ],
  "from": {"email": "orders@delivery-platform.com"},
  "content": [
    {
      "type": "text/html",
      "value": "<html>...</html>"
    }
  ]
}
```

### Push Notifications

**Firebase Cloud Messaging (FCM):**
```http
POST https://fcm.googleapis.com/v1/projects/{project_id}/messages:send
Authorization: Bearer {fcm_access_token}
Content-Type: application/json

{
  "message": {
    "token": "{device_fcm_token}",
    "notification": {
      "title": "Order Confirmed",
      "body": "Your order from Pizza Hut has been confirmed"
    },
    "data": {
      "orderId": "ORD-2025-001234",
      "action": "view_order"
    }
  }
}
```

**Apple Push Notification Service (APNs):**
```http
POST https://api.push.apple.com/3/device/{device_token}
apns-topic: com.delivery-platform.app
Authorization: Bearer {apns_jwt_token}
Content-Type: application/json

{
  "aps": {
    "alert": {
      "title": "Order Confirmed",
      "body": "Your order from Pizza Hut has been confirmed"
    },
    "sound": "default",
    "badge": 1
  },
  "orderId": "ORD-2025-001234"
}
```

---

## Analytics Platforms

### Google Analytics Integration

**Track Order Placement:**
```javascript
gtag('event', 'purchase', {
  transaction_id: 'ORD-2025-001234',
  value: 35.48,
  currency: 'USD',
  items: [
    {
      item_id: 'PIZZA-MARG-L',
      item_name: 'Margherita Pizza',
      quantity: 2,
      price: 12.99
    }
  ]
});
```

### Segment Integration

**Track Events:**
```http
POST https://api.segment.io/v1/track
Authorization: Basic {base64(write_key:)}
Content-Type: application/json

{
  "userId": "CUST-789456",
  "event": "Order Placed",
  "properties": {
    "orderId": "ORD-2025-001234",
    "revenue": 35.48,
    "restaurant": "Pizza Hut"
  }
}
```

---

## Third-Party Delivery

### Delivery Service Integration

Platforms can outsource delivery to third-party services:

**Create Delivery Request:**
```http
POST https://api.delivery-service.com/v1/deliveries
Authorization: Bearer {api_key}
Content-Type: application/json

{
  "externalId": "ORD-2025-001234",
  "pickupLocation": {
    "address": "456 Restaurant Ave",
    "lat": 40.7580,
    "lng": -73.9855,
    "instructions": "Ring back entrance"
  },
  "dropoffLocation": {
    "address": "123 Main St",
    "lat": 40.7589,
    "lng": -73.9851,
    "instructions": "Apartment 4B"
  },
  "packageDetails": {
    "description": "Food delivery - 2 pizzas",
    "weight": 2.5
  },
  "contactInfo": {
    "name": "John Doe",
    "phone": "+1-555-123-4567"
  }
}
```

**Track Delivery:**
```http
GET https://api.delivery-service.com/v1/deliveries/{delivery_id}
Authorization: Bearer {api_key}
```

---

## Webhook Patterns

### Webhook Security

**HMAC Verification:**
```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const computed = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');
  
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(computed)
  );
}
```

### Retry Strategy

**Exponential Backoff:**
- Attempt 1: Immediate
- Attempt 2: After 1 minute
- Attempt 3: After 5 minutes
- Attempt 4: After 15 minutes
- Attempt 5: After 1 hour
- Give up after 5 attempts

### Idempotency

Use webhook event IDs to prevent duplicate processing:

```javascript
async function handleWebhook(event) {
  const processed = await checkIfProcessed(event.id);
  if (processed) {
    return {status: 'already_processed'};
  }
  
  await processEvent(event);
  await markAsProcessed(event.id);
  
  return {status: 'success'};
}
```

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (Benefit All Humanity)**
