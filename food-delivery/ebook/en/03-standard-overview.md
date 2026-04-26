# Chapter 3: WIA Standard Overview

---

## 3.1 Architecture Overview

The WIA-IND-009 standard defines a modular, scalable architecture for food delivery systems. This chapter provides a comprehensive overview of the system components, data flow, and design decisions.

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    CUSTOMER APPLICATIONS                     │
│              (Web, iOS, Android, Voice, Chat)               │
│                                                              │
│  Features: Browse, Order, Track, Pay, Rate, Support         │
└────────────────────────┬────────────────────────────────────┘
                         │ HTTPS/WSS
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   WIA-IND-009 API GATEWAY                    │
│                                                              │
│  • Authentication & Authorization (JWT)                     │
│  • Rate Limiting & Throttling                               │
│  • Request Validation                                        │
│  • API Versioning                                            │
│  • Load Balancing                                            │
└─────┬──────────┬──────────┬──────────┬──────────┬──────────┘
      │          │          │          │          │
┌─────▼────┐ ┌──▼──────┐ ┌─▼────────┐ ┌▼─────────┐ ┌▼────────┐
│  Order   │ │ Driver  │ │  Route   │ │   Temp   │ │ Payment │
│  Service │ │ Service │ │ Optimizer│ │  Monitor │ │ Service │
│          │ │         │ │          │ │          │ │         │
│ • Create │ │• Profile│ │• Single  │ │• IoT     │ │• Charge │
│ • Update │ │• Assign │ │• Multi   │ │• Alerts  │ │• Refund │
│ • Track  │ │• Track  │ │• Dynamic │ │• Logs    │ │• Split  │
│ • Cancel │ │• Metrics│ │• TSP     │ │• HACCP   │ │• Settle │
└─────┬────┘ └──┬──────┘ └─┬────────┘ └┬─────────┘ └┬────────┘
      │          │          │          │          │
┌─────▼──────────▼──────────▼──────────▼──────────▼──────────┐
│                   DATA & ANALYTICS LAYER                     │
│                                                              │
│  • PostgreSQL: Core data (orders, drivers, customers)       │
│  • Redis: Caching, sessions, real-time data                 │
│  • TimescaleDB: Time-series (temperature, location)         │
│  • Elasticsearch: Search, logs, analytics                   │
│  • Kafka: Event streaming & message queue                   │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                  EXTERNAL INTEGRATIONS                       │
│                                                              │
│  • Restaurant POS (Toast, Square, Clover)                   │
│  • Mapping Services (Google Maps, Mapbox)                   │
│  • Payment Gateways (Stripe, Square, PayPal)                │
│  • Weather & Traffic APIs                                    │
│  • IoT Platform (AWS IoT, Azure IoT Hub)                    │
│  • SMS/Email (Twilio, SendGrid)                              │
│  • Analytics (Segment, Mixpanel)                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3.2 Core Services

### 3.2.1 Order Service

**Responsibilities:**
- Order creation and validation
- Order lifecycle management
- Status transitions
- Notification orchestration
- Business logic enforcement

**Key Features:**
```typescript
interface OrderService {
  // Order Management
  createOrder(request: OrderRequest): Promise<Order>;
  getOrder(orderId: string): Promise<Order>;
  updateOrder(orderId: string, update: OrderUpdate): Promise<Order>;
  cancelOrder(orderId: string, reason: string): Promise<void>;

  // Querying
  listOrders(filters: OrderFilters): Promise<Order[]>;
  searchOrders(query: string): Promise<Order[]>;

  // Status Management
  updateStatus(orderId: string, status: OrderStatus): Promise<void>;
  getStatusHistory(orderId: string): Promise<StatusChange[]>;

  // Tracking
  getTrackingInfo(orderId: string): Promise<TrackingInfo>;
  subscribeToUpdates(orderId: string): Promise<EventStream>;
}
```

**State Machine:**
```
PENDING → CONFIRMED → PREPARING → READY → ASSIGNED
  ↓
CANCELLED

ASSIGNED → PICKED_UP → IN_TRANSIT → ARRIVING → DELIVERED
  ↓                                              ↓
CANCELLED (partial refund)              COMPLETED
  ↓
FAILED (full refund)
```

### 3.2.2 Driver Service

**Responsibilities:**
- Driver registration and verification
- Status and location management
- Performance tracking
- Assignment matching
- Earnings calculation

**Key Features:**
```typescript
interface DriverService {
  // Driver Management
  registerDriver(application: DriverApplication): Promise<Driver>;
  getDriver(driverId: string): Promise<Driver>;
  updateDriver(driverId: string, update: DriverUpdate): Promise<Driver>;
  verifyDriver(driverId: string, verification: Verification): Promise<void>;

  // Status & Location
  updateStatus(driverId: string, status: DriverStatus): Promise<void>;
  updateLocation(driverId: string, location: Location): Promise<void>;
  getLocation(driverId: string): Promise<Location>;

  // Assignment
  findAvailableDrivers(criteria: DriverCriteria): Promise<Driver[]>;
  assignOrder(driverId: string, orderId: string): Promise<Assignment>;

  // Performance
  getMetrics(driverId: string, period: TimePeriod): Promise<DriverMetrics>;
  getRating(driverId: string): Promise<number>;

  // Earnings
  calculateEarnings(driverId: string, orderId: string): Promise<Earnings>;
  getEarningsHistory(driverId: string, period: TimePeriod): Promise<Earnings[]>;
}
```

**Driver Assignment Algorithm:**
```python
def assign_driver(order: Order) -> Optional[Driver]:
    """
    Score-based driver assignment

    Factors:
    1. Distance to pickup (40% weight)
    2. Driver rating (25% weight)
    3. Completion rate (20% weight)
    4. Current batch size (10% weight)
    5. Equipment capability (5% weight)
    """

    # Get available drivers within radius
    available = find_available_drivers(
        location=order.pickup_location,
        radius=10_km,
        vehicle_compatible=order.vehicle_requirements
    )

    # Score each driver
    scores = []
    for driver in available:
        distance_score = 1.0 - (driver.distance / 10_km)  # 0-1
        rating_score = driver.rating / 5.0  # 0-1
        completion_score = driver.completion_rate  # 0-1
        batch_score = 1.0 - (len(driver.active_orders) / driver.max_orders)
        equipment_score = 1.0 if driver.has_required_equipment(order) else 0.5

        total_score = (
            0.40 * distance_score +
            0.25 * rating_score +
            0.20 * completion_score +
            0.10 * batch_score +
            0.05 * equipment_score
        )

        scores.append((driver, total_score))

    # Return highest scoring driver
    if scores:
        scores.sort(key=lambda x: x[1], reverse=True)
        return scores[0][0]

    return None  # No available drivers
```

### 3.2.3 Route Optimizer

**Responsibilities:**
- Single-stop route calculation
- Multi-stop route optimization (TSP)
- Dynamic re-routing
- ETA calculation and updates
- Traffic and weather integration

**Key Features:**
```typescript
interface RouteOptimizer {
  // Single Route
  calculateRoute(
    pickup: Location,
    delivery: Location,
    vehicle: VehicleType
  ): Promise<Route>;

  // Multi-Stop Optimization
  optimizeMultiStop(
    stops: Stop[],
    constraints: RouteConstraints
  ): Promise<Route>;

  // Dynamic Updates
  recalculateRoute(
    routeId: string,
    currentLocation: Location,
    conditions: TrafficConditions
  ): Promise<Route>;

  // ETA
  calculateETA(
    route: Route,
    currentLocation: Location
  ): Promise<Date>;

  updateETA(routeId: string): Promise<Date>;
}
```

**Optimization Algorithm (2-Opt for TSP):**
```python
def optimize_route(stops: List[Stop], constraints: Constraints) -> Route:
    """
    2-Opt algorithm for multi-stop route optimization

    Time Complexity: O(n²) per iteration
    Space Complexity: O(n)
    Typical runtime: <500ms for 10 stops
    """

    # Initial route using nearest neighbor
    route = nearest_neighbor_route(stops)

    # Enforce constraints (pickup before delivery)
    route = enforce_precedence_constraints(route)

    # 2-Opt improvement
    improved = True
    max_iterations = 1000
    iteration = 0

    while improved and iteration < max_iterations:
        improved = False
        iteration += 1

        for i in range(len(route) - 1):
            for j in range(i + 2, len(route)):
                # Try swapping edges
                new_route = two_opt_swap(route, i, j)

                # Check if valid and better
                if (satisfies_constraints(new_route, constraints) and
                    route_cost(new_route) < route_cost(route)):
                    route = new_route
                    improved = True
                    break

            if improved:
                break

    return route

def route_cost(route: List[Stop]) -> float:
    """
    Calculate total cost: travel time + penalties
    """
    total_time = 0
    current_time = now()
    penalty = 0

    for i in range(len(route) - 1):
        # Travel time
        travel = calculate_travel_time(route[i], route[i+1])
        total_time += travel
        current_time += travel

        # Service time at stop
        total_time += route[i+1].service_duration
        current_time += route[i+1].service_duration

        # Late penalty
        if current_time > route[i+1].delivery_window.end:
            delay = (current_time - route[i+1].delivery_window.end).seconds
            penalty += 1000 * delay  # 1000 seconds penalty per second late

    return total_time + penalty
```

### 3.2.4 Temperature Monitor

**Responsibilities:**
- IoT sensor data collection
- Real-time temperature analysis
- Alert generation and escalation
- Compliance logging
- HACCP reporting

**Key Features:**
```typescript
interface TemperatureMonitor {
  // Data Collection
  recordReading(reading: TemperatureReading): Promise<void>;
  getReadings(orderId: string): Promise<TemperatureReading[]>;

  // Analysis
  analyzeCompliance(orderId: string): Promise<ComplianceReport>;
  checkThresholds(reading: TemperatureReading): Promise<Alert[]>;

  // Alerts
  sendAlert(alert: TemperatureAlert): Promise<void>;
  getAlerts(orderId: string): Promise<Alert[]>;

  // Reporting
  generateComplianceReport(period: TimePeriod): Promise<Report>;
  exportHACCPLog(orderId: string): Promise<HACCPLog>;
}
```

**Temperature Monitoring Flow:**
```
IoT Sensor (every 60 sec) → Bluetooth → Driver Phone
                                          ↓
                                     Mobile App
                                          ↓
                                   HTTPS POST /temperature
                                          ↓
                              Temperature Monitor Service
                                          ↓
                        ┌─────────────────┼─────────────────┐
                        ↓                 ↓                 ↓
                  TimescaleDB      Alert Engine      Analytics
                  (storage)        (thresholds)      (trends)
                        ↓                 ↓                 ↓
                  Compliance        Driver App       Dashboard
                  Logs              Notification
```

**Alert Logic:**
```python
def check_temperature_alert(reading: TemperatureReading, order: Order) -> Optional[Alert]:
    """
    Determine if temperature reading requires alert
    """

    temp = reading.temperature
    requirements = order.temperature_requirements

    # Hot food checks
    if requirements.type == 'hot':
        if temp < 60 and temp >= 55:
            return Alert(level='WARNING', message='Temperature dropping')
        elif temp < 55:
            return Alert(level='CRITICAL', message='Below safe temperature')

    # Cold food checks
    elif requirements.type == 'cold':
        if temp > 4 and temp <= 8:
            return Alert(level='WARNING', message='Temperature rising')
        elif temp > 8:
            return Alert(level='CRITICAL', message='Above safe temperature')

    # Frozen food checks
    elif requirements.type == 'frozen':
        if temp > -15:
            return Alert(level='CRITICAL', message='Thawing detected')

    return None  # Temperature OK
```

### 3.2.5 Payment Service

**Responsibilities:**
- Payment authorization and capture
- Refund processing
- Split payments (customer → platform → restaurant/driver)
- Transaction logging
- PCI DSS compliance

**Key Features:**
```typescript
interface PaymentService {
  // Authorization
  authorizePayment(orderId: string, amount: number): Promise<Authorization>;

  // Capture
  capturePayment(authorizationId: string): Promise<Transaction>;

  // Refunds
  refundPayment(
    transactionId: string,
    amount: number,
    reason: string
  ): Promise<Refund>;

  // Settlement
  splitPayment(
    transactionId: string,
    splits: PaymentSplit[]
  ): Promise<void>;

  // History
  getTransactionHistory(orderId: string): Promise<Transaction[]>;
}
```

**Payment Flow:**
```
Order Created → Authorize Payment ($42.50)
                ↓
         [Hold on card]
                ↓
Order Delivered → Capture Payment
                ↓
         ┌──────┴──────┬──────────┐
         ↓             ↓          ↓
    Restaurant     Platform    Driver
    ($25.00)       ($8.50)     ($9.00)
    [90% of food]  [fees]      [delivery+tip]
```

---

## 3.3 Data Flow Patterns

### 3.3.1 Order Creation Flow

```
1. Customer submits order
   POST /orders
   {
     restaurantId, items, deliveryLocation, paymentMethod
   }
   ↓
2. API Gateway validates request
   - Authentication check
   - Rate limit check
   - Input validation
   ↓
3. Order Service processes
   - Restaurant validation (active, in service area)
   - Item validation (available, correct prices)
   - Delivery validation (in range, address valid)
   - Calculate costs (subtotal, fees, tax)
   ↓
4. Payment Service authorizes
   - Authorize total amount on card
   - Hold funds (not captured yet)
   ↓
5. Order Service creates order
   - Insert into database
   - Status: PENDING → CONFIRMED
   ↓
6. Notifications sent
   - Customer: Order confirmed
   - Restaurant: New order received
   ↓
7. Driver assignment triggered
   - Find available drivers
   - Calculate scores
   - Assign to best driver
   ↓
8. Driver accepts
   - Status: CONFIRMED → ASSIGNED
   - Notifications sent to all parties
```

### 3.3.2 Real-Time Tracking Flow

```
WebSocket Connection:
Customer App ←──WebSocket──→ Tracking Service

Driver Location Updates:
Driver Phone (GPS every 10 sec)
   ↓
Mobile App
   ↓
POST /drivers/{id}/location
   {latitude, longitude, heading, speed}
   ↓
Driver Service
   ↓
Redis (cache current location)
   ↓
WebSocket Server (push to subscribers)
   ↓
Customer App (update map)

ETA Updates (every 2 minutes):
Current Location + Traffic Data
   ↓
Route Optimizer
   ↓
Calculate new ETA
   ↓
Push via WebSocket
   ↓
Customer App (update display)
```

### 3.3.3 Temperature Monitoring Flow

```
IoT Sensor (BLE) → Driver Phone (every 60 sec)
                         ↓
                   Mobile App
                         ↓
        POST /temperature-readings
        {
          orderId, sensorId, temperature,
          timestamp, location
        }
                         ↓
              Temperature Monitor Service
                         ↓
        ┌────────────────┼────────────────┐
        ↓                ↓                ↓
   TimescaleDB    Alert Engine     Real-time Dashboard
   (permanent)    (thresholds)     (WebSocket push)
        ↓                ↓
   Compliance      If CRITICAL:
   Reports         - Notify driver
                   - Notify support
                   - Flag order
                   - Auto-refund if severe
```

---

## 3.4 Technology Stack

### 3.4.1 Required Components

**Backend:**
- RESTful API framework (Express.js, FastAPI, Spring Boot, etc.)
- WebSocket server (Socket.io, ws, etc.)
- Database: PostgreSQL or compatible RDBMS
- Cache: Redis or compatible
- Message Queue: Kafka, RabbitMQ, AWS SQS, etc.

**Time-Series Data (Temperature, Location):**
- TimescaleDB (PostgreSQL extension)
- InfluxDB
- AWS Timestream
- Google Cloud Bigtable

**Search & Analytics:**
- Elasticsearch (optional but recommended)
- PostgreSQL full-text search (minimum)

**Authentication:**
- JWT (JSON Web Tokens)
- OAuth 2.0 for third-party integrations

**APIs:**
- Mapping: Google Maps, Mapbox, HERE, or OpenStreetMap
- Payment: Stripe, Square, Braintree, PayPal
- Communication: Twilio (SMS), SendGrid (Email)
- Weather: OpenWeatherMap, Weather.com API

### 3.4.2 Optional Components

**Advanced Features:**
- GraphQL for flexible querying
- gRPC for service-to-service communication
- Apache Spark for big data analytics
- TensorFlow/PyTorch for ML models

**Infrastructure:**
- Kubernetes for container orchestration
- Terraform for infrastructure as code
- Docker for containerization
- CI/CD: GitHub Actions, GitLab CI, Jenkins

### 3.4.3 Deployment Architecture

**Small Scale (single city, <1000 orders/day):**
```
Single Server:
- API + WebSocket server
- PostgreSQL database
- Redis cache
All on one machine or small cluster
```

**Medium Scale (multiple cities, 10K orders/day):**
```
Load Balancer
   ↓
API Servers (3+ instances)
   ↓
Application Servers (5+ instances)
- Order Service
- Driver Service
- Route Optimizer
- Temperature Monitor
   ↓
Data Layer
- PostgreSQL (primary + replicas)
- Redis cluster
- TimescaleDB
   ↓
Message Queue (Kafka cluster)
```

**Large Scale (national/global, 100K+ orders/day):**
```
CDN + Load Balancer (Global)
   ↓
API Gateway (Geo-distributed)
   ↓
Microservices (Auto-scaling)
- Order Service (20+ instances)
- Driver Service (10+ instances)
- Route Optimizer (5+ instances)
- Payment Service (10+ instances)
- Temperature Monitor (5+ instances)
   ↓
Data Layer (Distributed)
- PostgreSQL (sharded, multi-region)
- Redis (clustered, geo-replicated)
- TimescaleDB (partitioned)
- Elasticsearch cluster
   ↓
Message Queue (Kafka multi-region)
   ↓
Analytics & ML Pipeline
- Data warehouse (Snowflake, BigQuery)
- ML models (SageMaker, Vertex AI)
```

---

## 3.5 Scalability Considerations

### 3.5.1 Vertical Scaling Limits

**Single Server Capacity:**
- 100-500 orders/hour
- 1,000-5,000 concurrent WebSocket connections
- 10,000 database queries/second

**When to Scale Horizontally:**
- Consistent >70% CPU usage
- Database connection pool exhaustion
- Slow query times (>100ms p95)
- WebSocket connection limits reached

### 3.5.2 Horizontal Scaling Strategies

**Stateless Services:**
- API servers: Scale to N instances
- Load balance with round-robin or least-connections
- Session data in Redis (shared state)

**Database Scaling:**
- Read replicas for query load
- Sharding for write load (by city/region)
- Connection pooling (PgBouncer)

**Cache Strategy:**
- Redis cluster for high availability
- Cache frequently accessed data:
  - Driver locations (60 sec TTL)
  - Restaurant menus (5 min TTL)
  - Order status (30 sec TTL)

**WebSocket Scaling:**
- Use Redis pub/sub for cross-server messaging
- Sticky sessions to maintain connections
- Scale horizontally with load balancer

### 3.5.3 Performance Targets

**Response Time SLAs:**
```
Endpoint                    P50      P95      P99
──────────────────────────────────────────────────
GET /orders/:id           <50ms    <100ms   <200ms
POST /orders              <100ms   <300ms   <500ms
GET /tracking             <50ms    <100ms   <150ms
POST /routes/optimize     <500ms   <2s      <5s
WebSocket message         <100ms   <200ms   <500ms
```

**Throughput Targets:**
```
10,000 orders/hour peak = 2.8 orders/second
× 10 API calls per order = 28 requests/second minimum
With 3x safety margin = 84 requests/second capacity needed
```

**Availability Target:**
```
99.9% uptime = 43 minutes downtime/month
- Redundant servers
- Database replication
- Multi-AZ deployment
- Automated failover
```

---

## 3.6 Security Architecture

### 3.6.1 Authentication Flow

```
1. User login with credentials
   POST /auth/login
   {email, password}
   ↓
2. Verify credentials
   - Hash password (bcrypt)
   - Check against database
   ↓
3. Generate JWT token
   {
     sub: user_id,
     role: 'customer|driver|restaurant|admin',
     permissions: [...],
     exp: timestamp + 24h
   }
   ↓
4. Return token to client
   ↓
5. Client includes in subsequent requests
   Authorization: Bearer <token>
   ↓
6. API Gateway verifies token
   - Signature valid?
   - Not expired?
   - Has required permissions?
```

### 3.6.2 Data Encryption

**In Transit:**
- TLS 1.3 for all HTTPS connections
- WSS (secure WebSocket) for real-time updates
- Certificate pinning in mobile apps

**At Rest:**
- AES-256 encryption for database
- Separate encryption keys for PII
- Payment data: PCI DSS Level 1 compliant

### 3.6.3 API Security

**Rate Limiting:**
```
Standard tier: 100 requests/minute
Premium tier: 1,000 requests/minute
Enterprise tier: 10,000 requests/minute

WebSocket: Max 10 connections per user
```

**Input Validation:**
- Whitelist valid characters
- Length limits on strings
- Range validation on numbers
- SQL injection prevention (parameterized queries)
- XSS prevention (sanitize HTML)

**Request Signing (for webhooks):**
```
X-WIA-Signature: HMAC-SHA256(secret, timestamp + body)
X-WIA-Timestamp: Unix timestamp
Reject if timestamp > 5 minutes old (replay attack prevention)
```

---

## 3.7 Monitoring and Observability

### 3.7.1 Logging

**Structured Logging (JSON format):**
```json
{
  "timestamp": "2025-01-15T14:30:00Z",
  "level": "INFO",
  "service": "order-service",
  "trace_id": "abc123",
  "user_id": "user_456",
  "order_id": "order_789",
  "message": "Order created successfully",
  "duration_ms": 145,
  "status_code": 201
}
```

**Log Aggregation:**
- Elasticsearch for storage and search
- Kibana for visualization
- Retention: 30 days hot, 90 days archive

### 3.7.2 Metrics

**Key Metrics to Track:**
```
Business Metrics:
- Orders per hour
- Revenue per hour
- Average order value
- Customer acquisition cost

Performance Metrics:
- API response time (p50, p95, p99)
- Database query time
- Cache hit rate
- Error rate

Operational Metrics:
- Active drivers
- Available drivers
- Average delivery time
- Temperature compliance rate
- On-time delivery rate

Infrastructure Metrics:
- CPU usage
- Memory usage
- Disk I/O
- Network throughput
```

**Tools:**
- Prometheus for metrics collection
- Grafana for dashboards
- PagerDuty for alerting

### 3.7.3 Tracing

**Distributed Tracing:**
- Track request across services
- Identify bottlenecks
- Debug performance issues

**Example Trace:**
```
Order Creation (total: 456ms)
├─ API Gateway (5ms)
├─ Order Service (145ms)
│  ├─ Validate restaurant (45ms)
│  ├─ Validate items (30ms)
│  ├─ Calculate costs (20ms)
│  └─ Create order (50ms)
├─ Payment Service (180ms)
│  └─ Authorize charge (175ms)
├─ Driver Service (120ms)
│  ├─ Find drivers (80ms)
│  ├─ Calculate scores (25ms)
│  └─ Assign driver (15ms)
└─ Notification Service (6ms)
```

**Tools:**
- Jaeger or Zipkin for tracing
- OpenTelemetry for instrumentation

---

## 3.8 Summary

The WIA-IND-009 architecture provides:

1. **Modular Design**: Independent services that can scale separately
2. **Real-Time Capabilities**: WebSocket for tracking, sub-second updates
3. **Scalability**: From single restaurant to global platform
4. **Reliability**: Redundancy, failover, monitoring
5. **Security**: Encryption, authentication, input validation
6. **Performance**: Caching, optimization, efficient algorithms
7. **Observability**: Logging, metrics, tracing

This architecture supports the full food delivery lifecycle while maintaining food safety, driver welfare, and customer satisfaction.

---

**Next Chapter**: [Chapter 4: Data Formats and Models →](04-data-format.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
