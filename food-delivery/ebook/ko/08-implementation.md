# 8장: 구현 가이드

---

## 학습 목표

이 장을 마치면 다음을 할 수 있습니다:

- 프로덕션 품질의 인프라를 설정하고 구성하기
- 데이터베이스 스키마를 설계하고 최적화하기
- 배달원 온보딩 프로세스를 구현하기
- Docker 및 Kubernetes를 사용하여 배포하기
- 통합 테스트 및 부하 테스트 수행하기
- 프로덕션 출시 체크리스트 완료하기
- 식품 안전 및 데이터 프라이버시 규정 준수 보장하기

---

## 8.1 개요

이 최종 장은 WIA-IND-009 준수 음식 배달 시스템 구현을 위한 단계별 가이드를 제공합니다. 인프라 설정부터 배달원 온보딩까지, 이 가이드는 프로덕션 배포에 필요한 모든 것을 다룹니다.

**구현 단계:**

1. **인프라 설정**: 개발 환경 및 도구
2. **데이터베이스 설계**: 스키마 및 관계
3. **배달원 온보딩**: 등록부터 활성화까지
4. **배포**: Docker, Kubernetes, CI/CD
5. **테스트**: 통합, 부하, 보안 테스트
6. **모니터링**: 로깅, 경고, 대시보드
7. **규정 준수**: 식품 안전, 데이터 프라이버시

---

## 8.2 인프라 설정

### 8.2.1 기술 스택 선택

**백엔드 프레임워크 (하나 선택):**

| 프레임워크 | 장점 | 단점 | 사용 사례 |
|---------|------|------|---------|
| **Node.js + Express** | 실시간 기능, 대규모 에코시스템 | 단일 스레드 | 실시간 추적, WebSocket |
| **Python + FastAPI** | ML/데이터 과학 통합 우수 | 상대적으로 느림 | 경로 최적화, 예측 |
| **Java + Spring Boot** | 엔터프라이즈급, 타입 안전성 | 무겁고 복잡 | 대규모 엔터프라이즈 |
| **Go** | 고성능, 뛰어난 동시성 | 작은 에코시스템 | 고성능 API, 마이크로서비스 |

**권장사항**: 대부분의 경우 Node.js + Express (실시간 기능, 성숙한 에코시스템, 빠른 개발)

**데이터베이스:**

```
PostgreSQL: 주 데이터베이스
  - 주문, 배달원, 고객, 레스토랑
  - ACID 준수, 신뢰성
  - JSON 지원 (유연한 메타데이터)

Redis: 캐싱 및 실시간 데이터
  - 세션 관리
  - 실시간 위치 추적
  - 속도 제한

TimescaleDB: 시계열 데이터
  - 온도 판독값
  - 위치 기록
  - 성능 메트릭
```

**메시지 큐:**

```
Apache Kafka: 높은 처리량, 이벤트 스트리밍
  - 대규모 (>10K 주문/일)
  - 복잡한 이벤트 처리

RabbitMQ: 간단한 설정, 중간 규모에 적합
  - 중간 규모 (<10K 주문/일)
  - 간단한 큐잉

AWS SQS: 관리형 서비스, 쉬운 통합
  - AWS에 이미 있는 경우
  - 운영 오버헤드 최소화
```

**클라우드 플랫폼:**

```
AWS: 가장 포괄적, 최고의 IoT 지원
  - IoT Core, Lambda, DynamoDB
  - 광범위한 서비스

Google Cloud: 최고의 ML/AI 도구, 경쟁력 있는 가격
  - BigQuery, AI Platform
  - 강력한 네트워킹

Azure: Windows/C# 환경에 최적
  - .NET 통합
  - 엔터프라이즈 지원
```

### 8.2.2 개발 환경 설정

**1단계: 필수 구성 요소 설치**

```bash
# macOS
brew install node
brew install postgresql
brew install redis
brew install docker

# Ubuntu
sudo apt-get update
sudo apt-get install nodejs npm postgresql redis-server docker.io

# 설치 확인
node --version  # v18+
psql --version  # 14+
redis-cli --version  # 7+
docker --version  # 20+
```

**2단계: 스타터 템플릿 복제**

```bash
# 템플릿 저장소 복제
git clone https://github.com/WIA-Official/food-delivery-starter.git
cd food-delivery-starter

# 의존성 설치
npm install

# 프로젝트 구조 확인
tree -L 2
# .
# ├── src/
# │   ├── api/         # REST API 엔드포인트
# │   ├── models/      # 데이터베이스 모델
# │   ├── services/    # 비즈니스 로직
# │   ├── workers/     # 백그라운드 작업
# │   └── websocket/   # 실시간 통신
# ├── tests/           # 테스트 코드
# ├── migrations/      # 데이터베이스 마이그레이션
# └── docker/          # Docker 설정
```

**3단계: 환경 구성**

```bash
# .env 파일 생성
cat > .env << EOF
# 환경
NODE_ENV=development
PORT=3000
LOG_LEVEL=debug

# 데이터베이스
DATABASE_URL=postgresql://user:pass@localhost:5432/fooddelivery
REDIS_URL=redis://localhost:6379

# JWT 시크릿 (프로덕션에서는 반드시 변경)
JWT_SECRET=your-secret-key-change-in-production
JWT_EXPIRY=7d

# 외부 API
GOOGLE_MAPS_API_KEY=your-api-key
STRIPE_SECRET_KEY=sk_test_your-stripe-key
STRIPE_PUBLISHABLE_KEY=pk_test_your-stripe-key
AWS_IOT_ENDPOINT=your-iot-endpoint.iot.us-east-1.amazonaws.com

# 기능 플래그
ENABLE_TEMPERATURE_MONITORING=true
ENABLE_ROUTE_OPTIMIZATION=true
ENABLE_BATCH_DELIVERY=false

# 알림
TWILIO_ACCOUNT_SID=your-twilio-sid
TWILIO_AUTH_TOKEN=your-twilio-token
SENDGRID_API_KEY=your-sendgrid-key

# 모니터링
SENTRY_DSN=your-sentry-dsn
DATADOG_API_KEY=your-datadog-key
EOF

# 환경 변수 로드
source .env
```

**4단계: 데이터베이스 초기화**

```bash
# PostgreSQL 데이터베이스 생성
createdb fooddelivery

# TimescaleDB 확장 추가 (선택사항)
psql fooddelivery -c "CREATE EXTENSION IF NOT EXISTS timescaledb;"

# 마이그레이션 실행
npm run db:migrate

# 예제 데이터 시드 (개발용)
npm run db:seed

# 데이터베이스 상태 확인
psql fooddelivery -c "
SELECT table_name
FROM information_schema.tables
WHERE table_schema = 'public'
ORDER BY table_name;
"
```

**5단계: 개발 서버 시작**

```bash
# 터미널 1: API 서버 시작
npm run dev
# > 서버 시작: http://localhost:3000
# > 스웨거 문서: http://localhost:3000/api-docs

# 터미널 2: 백그라운드 워커 시작
npm run workers
# > 이메일 워커 시작됨
# > 알림 워커 시작됨
# > 정산 워커 시작됨

# 터미널 3: WebSocket 서버 시작
npm run websocket
# > WebSocket 서버 시작: ws://localhost:3001
# > 실시간 추적 준비됨

# 터미널 4: Redis 모니터링 (선택사항)
redis-cli monitor
```

---

## 8.3 데이터베이스 스키마

### 8.3.1 핵심 테이블

```sql
-- 사용자 테이블
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  role VARCHAR(50) NOT NULL,  -- 'customer', 'driver', 'restaurant', 'admin'
  first_name VARCHAR(100),
  last_name VARCHAR(100),
  phone VARCHAR(20),
  phone_verified BOOLEAN DEFAULT false,
  email_verified BOOLEAN DEFAULT false,
  avatar_url TEXT,
  status VARCHAR(50) DEFAULT 'active',  -- 'active', 'suspended', 'deleted'
  last_login_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

  CONSTRAINT users_role_check CHECK (role IN ('customer', 'driver', 'restaurant', 'admin'))
);

-- 레스토랑 테이블
CREATE TABLE restaurants (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  cuisine_type VARCHAR(100),

  -- 위치
  address TEXT NOT NULL,
  latitude DECIMAL(10, 8) NOT NULL,
  longitude DECIMAL(11, 8) NOT NULL,
  delivery_radius_km DECIMAL(5, 2) DEFAULT 5.0,

  -- 연락처
  phone VARCHAR(20),
  email VARCHAR(255),

  -- 운영 시간 (JSON)
  operating_hours JSONB,

  -- 상태 및 메트릭
  status VARCHAR(50) DEFAULT 'active',  -- 'active', 'closed', 'suspended'
  avg_prep_time INTEGER DEFAULT 15,  -- 분
  rating DECIMAL(3, 2) DEFAULT 5.0,
  total_reviews INTEGER DEFAULT 0,

  -- 결제
  stripe_account_id VARCHAR(255),
  commission_rate DECIMAL(5, 4) DEFAULT 0.15,  -- 15%

  -- 메타데이터
  images JSONB,  -- [url1, url2, ...]
  tags JSONB,    -- ['vegetarian', 'halal', ...]

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 메뉴 항목 테이블
CREATE TABLE menu_items (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  restaurant_id UUID REFERENCES restaurants(id) ON DELETE CASCADE,
  external_id VARCHAR(255),  -- POS 시스템 ID

  name VARCHAR(255) NOT NULL,
  description TEXT,
  category VARCHAR(100),

  -- 가격
  price INTEGER NOT NULL,  -- 센트

  -- 가용성
  available BOOLEAN DEFAULT true,
  in_stock BOOLEAN DEFAULT true,

  -- 음식 속성
  temperature VARCHAR(20),  -- 'hot', 'cold', 'ambient', 'frozen'
  prep_time INTEGER DEFAULT 15,  -- 분
  allergens JSONB,  -- ['peanuts', 'dairy', ...]
  dietary_tags JSONB,  -- ['vegetarian', 'vegan', 'gluten-free', ...]

  -- 수정자 (JSON)
  modifiers JSONB,

  -- 이미지
  image_url TEXT,

  -- 동기화
  synced_at TIMESTAMP,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 주문 테이블
CREATE TABLE orders (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  confirmation_code VARCHAR(10) UNIQUE NOT NULL,

  -- 관계
  restaurant_id UUID REFERENCES restaurants(id),
  customer_id UUID REFERENCES users(id),
  driver_id UUID REFERENCES users(id),

  -- 상태
  status VARCHAR(50) NOT NULL,

  -- 픽업 위치
  pickup_address TEXT NOT NULL,
  pickup_lat DECIMAL(10, 8) NOT NULL,
  pickup_lng DECIMAL(11, 8) NOT NULL,
  pickup_instructions TEXT,

  -- 배달 위치
  delivery_address TEXT NOT NULL,
  delivery_lat DECIMAL(10, 8) NOT NULL,
  delivery_lng DECIMAL(11, 8) NOT NULL,
  delivery_instructions TEXT,

  -- 가격 (센트)
  subtotal INTEGER NOT NULL,
  tax INTEGER NOT NULL,
  delivery_fee INTEGER NOT NULL,
  service_fee INTEGER NOT NULL,
  tip INTEGER DEFAULT 0,
  discount INTEGER DEFAULT 0,
  total INTEGER NOT NULL,

  -- 타이밍
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  confirmed_at TIMESTAMP,
  preparing_at TIMESTAMP,
  ready_at TIMESTAMP,
  assigned_at TIMESTAMP,
  picked_up_at TIMESTAMP,
  in_transit_at TIMESTAMP,
  delivered_at TIMESTAMP,
  completed_at TIMESTAMP,
  cancelled_at TIMESTAMP,

  estimated_prep_time INTEGER,  -- 분
  estimated_delivery_time TIMESTAMP,
  actual_delivery_time TIMESTAMP,

  -- 추가 옵션
  special_instructions TEXT,
  contactless_delivery BOOLEAN DEFAULT false,
  leave_at_door BOOLEAN DEFAULT false,

  -- 온도 요구사항
  temperature_requirement VARCHAR(20),

  -- 결제
  payment_method VARCHAR(50),  -- 'card', 'cash', 'wallet'
  payment_intent_id VARCHAR(255),
  payment_status VARCHAR(50),

  -- 정산
  settlement_status VARCHAR(50),
  settled_at TIMESTAMP,

  -- 환불
  refund_amount INTEGER DEFAULT 0,
  refund_status VARCHAR(50),
  refund_reason TEXT,
  refunded_at TIMESTAMP,

  -- 평가
  customer_rating INTEGER,  -- 1-5
  customer_review TEXT,
  driver_rating INTEGER,
  driver_review TEXT,

  -- 증명
  proof_of_delivery JSONB,  -- { photo: url, signature: data, timestamp: iso }

  -- 출처 (제3자 플랫폼)
  source VARCHAR(50) DEFAULT 'direct',  -- 'direct', 'ubereats', 'doordash', ...
  external_id VARCHAR(255),

  -- 메타데이터
  metadata JSONB,
  version INTEGER DEFAULT 1,

  CONSTRAINT orders_status_check CHECK (
    status IN ('pending', 'confirmed', 'preparing', 'ready',
               'assigned', 'picked_up', 'in_transit', 'arriving',
               'delivered', 'completed', 'cancelled', 'failed')
  )
);

-- 주문 항목 테이블
CREATE TABLE order_items (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  order_id UUID REFERENCES orders(id) ON DELETE CASCADE,
  menu_item_id UUID REFERENCES menu_items(id),

  name VARCHAR(255) NOT NULL,
  quantity INTEGER NOT NULL CHECK (quantity > 0),
  unit_price INTEGER NOT NULL,
  total_price INTEGER NOT NULL,

  -- 음식 속성
  temperature VARCHAR(20),

  -- 수정자 (JSON)
  modifiers JSONB,
  special_requests TEXT,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 배달원 테이블
CREATE TABLE drivers (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),

  -- 차량
  vehicle_type VARCHAR(50) NOT NULL,  -- 'bike', 'ebike', 'scooter', 'car', 'motorcycle'
  vehicle_make VARCHAR(100),
  vehicle_model VARCHAR(100),
  vehicle_year INTEGER,
  vehicle_color VARCHAR(50),
  license_plate VARCHAR(20),

  -- 상태
  status VARCHAR(50) DEFAULT 'offline',  -- 'online', 'offline', 'busy', 'break'
  is_online BOOLEAN DEFAULT false,

  -- 위치
  current_lat DECIMAL(10, 8),
  current_lng DECIMAL(11, 8),
  heading INTEGER,  -- 0-359 도
  speed DECIMAL(5, 2),  -- km/h
  last_location_update TIMESTAMP,

  -- 성능 메트릭
  rating DECIMAL(3, 2) DEFAULT 5.0,
  total_deliveries INTEGER DEFAULT 0,
  completed_deliveries INTEGER DEFAULT 0,
  cancelled_deliveries INTEGER DEFAULT 0,
  completion_rate DECIMAL(5, 4) DEFAULT 1.0,
  on_time_rate DECIMAL(5, 4) DEFAULT 1.0,
  acceptance_rate DECIMAL(5, 4) DEFAULT 1.0,

  -- 수익
  total_earnings INTEGER DEFAULT 0,  -- 센트
  current_shift_earnings INTEGER DEFAULT 0,

  -- 장비
  has_hot_bag BOOLEAN DEFAULT false,
  has_cold_bag BOOLEAN DEFAULT false,
  has_temperature_sensor BOOLEAN DEFAULT false,
  sensor_id VARCHAR(100),

  -- 용량
  max_orders INTEGER DEFAULT 3,
  current_orders INTEGER DEFAULT 0,

  -- 검증
  background_check_status VARCHAR(50),
  background_check_date DATE,
  license_number VARCHAR(50),
  license_verified BOOLEAN DEFAULT false,
  license_expiration DATE,
  insurance_verified BOOLEAN DEFAULT false,
  insurance_expiration DATE,

  -- 교육
  training_status VARCHAR(50),
  training_completed_at TIMESTAMP,
  food_safety_certified BOOLEAN DEFAULT false,
  food_safety_cert_date DATE,
  food_safety_cert_expiry DATE,

  -- 결제
  stripe_account_id VARCHAR(255),
  bank_account_verified BOOLEAN DEFAULT false,

  -- 가용성 (JSON)
  availability_schedule JSONB,

  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 온도 판독값 테이블 (TimescaleDB 하이퍼테이블)
CREATE TABLE temperature_readings (
  time TIMESTAMPTZ NOT NULL,
  order_id UUID NOT NULL,
  driver_id UUID NOT NULL,
  sensor_id VARCHAR(100) NOT NULL,

  temperature DECIMAL(5, 2) NOT NULL,
  humidity DECIMAL(5, 2),
  battery_level INTEGER,

  -- 위치
  latitude DECIMAL(10, 8),
  longitude DECIMAL(11, 8),

  -- 준수
  in_compliance BOOLEAN,
  alert_level VARCHAR(50),  -- 'none', 'warning', 'critical'

  -- 메타데이터
  metadata JSONB
);

-- TimescaleDB 하이퍼테이블로 변환
SELECT create_hypertable('temperature_readings', 'time');

-- 인덱스 생성
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_role ON users(role);
CREATE INDEX idx_users_status ON users(status);

CREATE INDEX idx_restaurants_location ON restaurants(latitude, longitude);
CREATE INDEX idx_restaurants_status ON restaurants(status);

CREATE INDEX idx_menu_items_restaurant ON menu_items(restaurant_id);
CREATE INDEX idx_menu_items_available ON menu_items(available, in_stock);

CREATE INDEX idx_orders_status ON orders(status);
CREATE INDEX idx_orders_customer ON orders(customer_id);
CREATE INDEX idx_orders_driver ON orders(driver_id);
CREATE INDEX idx_orders_restaurant ON orders(restaurant_id);
CREATE INDEX idx_orders_created_at ON orders(created_at DESC);
CREATE INDEX idx_orders_confirmation_code ON orders(confirmation_code);

CREATE INDEX idx_order_items_order ON order_items(order_id);

CREATE INDEX idx_drivers_status ON drivers(status);
CREATE INDEX idx_drivers_online ON drivers(is_online) WHERE is_online = true;
CREATE INDEX idx_drivers_location ON drivers(current_lat, current_lng);

CREATE INDEX idx_temp_readings_order_time ON temperature_readings(order_id, time DESC);
CREATE INDEX idx_temp_readings_driver_time ON temperature_readings(driver_id, time DESC);

-- 전체 텍스트 검색 인덱스
CREATE INDEX idx_restaurants_name_search ON restaurants USING gin(to_tsvector('english', name));
CREATE INDEX idx_menu_items_name_search ON menu_items USING gin(to_tsvector('english', name || ' ' || COALESCE(description, '')));
```

### 8.3.2 관계 및 제약조건

```sql
-- 외래 키 제약조건은 이미 테이블 정의에 포함됨

-- 추가 체크 제약조건
ALTER TABLE restaurants ADD CONSTRAINT restaurants_rating_check
  CHECK (rating >= 0 AND rating <= 5);

ALTER TABLE drivers ADD CONSTRAINT drivers_rating_check
  CHECK (rating >= 0 AND rating <= 5);

ALTER TABLE drivers ADD CONSTRAINT drivers_completion_rate_check
  CHECK (completion_rate >= 0 AND completion_rate <= 1);

ALTER TABLE orders ADD CONSTRAINT orders_customer_rating_check
  CHECK (customer_rating IS NULL OR (customer_rating >= 1 AND customer_rating <= 5));

-- 트리거: updated_at 자동 업데이트
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_restaurants_updated_at BEFORE UPDATE ON restaurants
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_menu_items_updated_at BEFORE UPDATE ON menu_items
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_drivers_updated_at BEFORE UPDATE ON drivers
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- 뷰: 활성 주문
CREATE VIEW active_orders AS
SELECT
  o.*,
  r.name as restaurant_name,
  u_customer.first_name || ' ' || u_customer.last_name as customer_name,
  u_driver.first_name || ' ' || u_driver.last_name as driver_name
FROM orders o
JOIN restaurants r ON o.restaurant_id = r.id
JOIN users u_customer ON o.customer_id = u_customer.id
LEFT JOIN users u_driver ON o.driver_id = u_driver.id
WHERE o.status IN ('pending', 'confirmed', 'preparing', 'ready', 'assigned', 'picked_up', 'in_transit')
ORDER BY o.created_at DESC;

-- 뷰: 온라인 배달원
CREATE VIEW online_drivers AS
SELECT
  d.*,
  u.first_name || ' ' || u.last_name as driver_name,
  u.phone,
  u.avatar_url
FROM drivers d
JOIN users u ON d.user_id = u.id
WHERE d.is_online = true AND d.status = 'online'
ORDER BY d.rating DESC;
```

---

## 8.4 배달원 온보딩

### 8.4.1 등록 프로세스

**1단계: 신청서 제출**

```typescript
// POST /drivers/apply
async function submitDriverApplication(req: Request, res: Response) {
  const {
    firstName,
    lastName,
    email,
    phone,
    password,
    dateOfBirth,
    vehicleType,
    licenseNumber,
    licenseState,
    licenseExpiration,
    ssn  // 신원 조회용
  } = req.body;

  try {
    // 연령 검증 (18세 이상이어야 함)
    const age = calculateAge(new Date(dateOfBirth));
    if (age < 18) {
      return res.status(400).json({
        error: '신청하려면 18세 이상이어야 합니다'
      });
    }

    // 이메일이 이미 존재하는지 확인
    const existingUser = await User.findOne({ email });
    if (existingUser) {
      return res.status(400).json({
        error: '이 이메일은 이미 등록되어 있습니다'
      });
    }

    // 사용자 계정 생성
    const hashedPassword = await bcrypt.hash(password, 10);
    const user = await User.create({
      email,
      password_hash: hashedPassword,
      role: 'driver',
      first_name: firstName,
      last_name: lastName,
      phone,
      status: 'pending'
    });

    // 배달원 프로필 생성
    const driver = await Driver.create({
      user_id: user.id,
      vehicle_type: vehicleType,
      license_number: licenseNumber,
      status: 'pending_verification'
    });

    // 신원 조회 시작
    await initiateBackgroundCheck(driver.id, {
      ssn,
      license_number: licenseNumber,
      license_state: licenseState,
      date_of_birth: dateOfBirth
    });

    // 확인 이메일 발송
    await sendVerificationEmail(user.email, user.id);

    res.status(201).json({
      message: '신청서가 제출되었습니다. 신원 조회가 진행 중입니다.',
      data: {
        driverId: driver.id,
        status: 'pending_verification'
      }
    });

  } catch (error) {
    console.error('배달원 신청 오류:', error);
    res.status(500).json({
      error: '신청 처리 중 오류가 발생했습니다'
    });
  }
}

// 연령 계산
function calculateAge(birthDate: Date): number {
  const today = new Date();
  let age = today.getFullYear() - birthDate.getFullYear();
  const monthDiff = today.getMonth() - birthDate.getMonth();

  if (monthDiff < 0 || (monthDiff === 0 && today.getDate() < birthDate.getDate())) {
    age--;
  }

  return age;
}
```

**2단계: 신원 조회**

```typescript
// Checkr 또는 유사 서비스와 통합
import Checkr from 'checkr';
const checkr = new Checkr(process.env.CHECKR_API_KEY);

async function initiateBackgroundCheck(
  driverId: string,
  info: {
    ssn: string;
    license_number: string;
    license_state: string;
    date_of_birth: Date;
  }
) {
  try {
    const driver = await Driver.findById(driverId).populate('user_id');

    // Checkr 후보자 생성
    const candidate = await checkr.candidates.create({
      first_name: driver.user_id.first_name,
      last_name: driver.user_id.last_name,
      email: driver.user_id.email,
      phone: driver.user_id.phone,
      ssn: info.ssn,
      dob: info.date_of_birth,
      driver_license_number: info.license_number,
      driver_license_state: info.license_state
    });

    // 신원 조회 보고서 생성
    const report = await checkr.reports.create(candidate.id, {
      package: 'driver_standard',  // 범죄 기록, 운전 기록
      candidate_id: candidate.id
    });

    // 참조 저장
    await driver.update({
      background_check_id: report.id,
      background_check_status: 'pending'
    });

    console.log(`배달원 ${driverId}에 대한 신원 조회 시작됨`);

    // 웹훅이 완료 시 업데이트합니다

  } catch (error) {
    console.error('신원 조회 시작 오류:', error);

    await driver.update({
      background_check_status: 'failed'
    });

    throw error;
  }
}

// 신원 조회 서비스의 웹훅
app.post('/webhooks/background-check', async (req, res) => {
  const { report_id, status, result, adjudication } = req.body;

  try {
    // 배달원 찾기
    const driver = await Driver.findOne({
      background_check_id: report_id
    }).populate('user_id');

    if (!driver) {
      return res.status(404).json({ error: '배달원을 찾을 수 없습니다' });
    }

    const isApproved = adjudication === 'approved' || result === 'clear';

    await driver.update({
      background_check_status: isApproved ? 'approved' : 'rejected',
      background_check_date: new Date()
    });

    if (isApproved) {
      console.log(`배달원 ${driver.id} 신원 조회 통과`);

      // 다음 단계로 진행
      await sendTrainingInvitation(driver.id);

      // 승인 이메일 발송
      await sendEmail({
        to: driver.user_id.email,
        subject: '신원 조회 통과!',
        template: 'background-check-approved',
        data: {
          driverName: driver.user_id.first_name
        }
      });

    } else {
      console.log(`배달원 ${driver.id} 신원 조회 불합격`);

      // 지원자에게 알림
      await sendRejectionEmail(
        driver.user_id.email,
        '신원 조회가 요구사항을 충족하지 못했습니다'
      );
    }

    res.json({ success: true });

  } catch (error) {
    console.error('신원 조회 웹훅 오류:', error);
    res.status(500).json({ error: error.message });
  }
});
```

**3단계: 교육 및 인증**

```typescript
// 교육 자료 발송
async function sendTrainingInvitation(driverId: string) {
  const driver = await Driver.findById(driverId).populate('user_id');

  // 고유 교육 토큰 생성
  const trainingToken = jwt.sign(
    { driverId: driver.id, purpose: 'training' },
    process.env.JWT_SECRET,
    { expiresIn: '7d' }
  );

  // 교육 포털 링크가 포함된 이메일 발송
  await sendEmail({
    to: driver.user_id.email,
    subject: '배달원 교육 완료하기',
    template: 'training-invitation',
    data: {
      driverName: driver.user_id.first_name,
      trainingLink: `${process.env.APP_URL}/training?token=${trainingToken}`,
      modules: [
        {
          title: '식품 안전 및 온도 관리',
          duration: '30분',
          required: true
        },
        {
          title: '고객 서비스 모범 사례',
          duration: '20분',
          required: true
        },
        {
          title: '앱 사용 및 내비게이션',
          duration: '25분',
          required: true
        },
        {
          title: '안전 및 비상 절차',
          duration: '15분',
          required: true
        }
      ],
      totalDuration: '90분',
      passingScore: 80
    }
  });

  await driver.update({
    training_status: 'invited',
    training_invited_at: new Date()
  });

  console.log(`배달원 ${driverId}에게 교육 초대 발송됨`);
}

// 교육 완료 표시
app.post('/drivers/:id/complete-training', async (req, res) => {
  const { id } = req.params;
  const { quizScore, completedModules, timeSpent } = req.body;

  try {
    const driver = await Driver.findById(id);

    if (!driver) {
      return res.status(404).json({ error: '배달원을 찾을 수 없습니다' });
    }

    // 점수 검증
    if (quizScore < 80) {
      return res.status(400).json({
        error: '퀴즈에서 80% 이상 득점해야 합니다',
        score: quizScore,
        required: 80
      });
    }

    // 모든 필수 모듈 확인
    const requiredModules = [
      'food_safety',
      'customer_service',
      'app_usage',
      'safety_procedures'
    ];

    const missingModules = requiredModules.filter(
      module => !completedModules.includes(module)
    );

    if (missingModules.length > 0) {
      return res.status(400).json({
        error: '모든 필수 모듈을 완료해야 합니다',
        missing: missingModules
      });
    }

    // 교육 완료 업데이트
    await driver.update({
      training_status: 'completed',
      training_completed_at: new Date(),
      training_score: quizScore,
      food_safety_certified: true,
      food_safety_cert_date: new Date(),
      food_safety_cert_expiry: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000)  // 1년
    });

    console.log(`배달원 ${id} 교육 완료 (점수: ${quizScore}%)`);

    // 배달원 계정 활성화
    await activateDriver(id);

    res.json({
      message: '교육이 완료되었습니다! 이제 주문 수락을 시작할 수 있습니다.',
      data: {
        score: quizScore,
        certificationDate: driver.food_safety_cert_date,
        expiryDate: driver.food_safety_cert_expiry
      }
    });

  } catch (error) {
    console.error('교육 완료 오류:', error);
    res.status(500).json({ error: error.message });
  }
});
```

**4단계: 장비 설정**

```typescript
// 배달원의 장비 설정 안내
async function setupDriverEquipment(driverId: string) {
  const checklist = [
    {
      id: 'hot_bag',
      name: '온열 음식 가방',
      required: true,
      minSpecification: 'R-8 단열, 40L 용량',
      estimatedCost: '$40-80',
      purchaseLinks: [
        'https://amazon.com/hot-food-bags',
        'https://restaurant-depot.com/delivery-bags'
      ]
    },
    {
      id: 'cold_bag',
      name: '냉장 음식 가방',
      required: true,
      minSpecification: 'R-10 단열, 30L 용량',
      estimatedCost: '$50-100',
      purchaseLinks: [
        'https://amazon.com/cold-food-bags'
      ]
    },
    {
      id: 'temperature_sensor',
      name: '온도 센서',
      required: false,
      recommended: true,
      minSpecification: 'Bluetooth 5.0, 6개월 이상 배터리 수명',
      estimatedCost: '$30-60',
      purchaseLinks: [
        'https://tempsense.com/drivers'
      ]
    },
    {
      id: 'phone_mount',
      name: '휴대폰 거치대',
      required: true,
      minSpecification: '안전한 그립, 360도 회전',
      estimatedCost: '$15-30'
    },
    {
      id: 'safety_gear',
      name: '안전 조끼 및 헬멧',
      required: true,  // 자전거/스쿠터의 경우
      minSpecification: '반사 조끼, DOT 승인 헬멧',
      estimatedCost: '$40-80',
      note: '자전거 및 스쿠터 배달원에게 필수'
    }
  ];

  return checklist;
}

// 배달원 장비 확인
app.post('/drivers/:id/confirm-equipment', async (req, res) => {
  const { id } = req.params;
  const { equipment, photos, purchaseReceipts } = req.body;

  try {
    const driver = await Driver.findById(id);

    if (!driver) {
      return res.status(404).json({ error: '배달원을 찾을 수 없습니다' });
    }

    // 필수 장비 검증
    const requiredEquipment = ['hot_bag', 'cold_bag', 'phone_mount'];

    if (driver.vehicle_type === 'bike' || driver.vehicle_type === 'scooter') {
      requiredEquipment.push('safety_gear');
    }

    const missingEquipment = requiredEquipment.filter(
      item => !equipment.includes(item)
    );

    if (missingEquipment.length > 0) {
      return res.status(400).json({
        error: '모든 필수 장비를 확인해야 합니다',
        missing: missingEquipment
      });
    }

    // 장비 확인 업데이트
    await driver.update({
      has_hot_bag: equipment.includes('hot_bag'),
      has_cold_bag: equipment.includes('cold_bag'),
      has_temperature_sensor: equipment.includes('temperature_sensor'),
      equipment_verified: true,
      equipment_photos: photos,
      equipment_receipts: purchaseReceipts
    });

    console.log(`배달원 ${id} 장비 확인 완료`);

    // 최종 활성화 (이미 활성화되지 않은 경우)
    if (driver.status === 'pending_equipment') {
      await activateDriver(id);
    }

    res.json({
      message: '장비가 확인되었습니다. 배달을 시작할 준비가 되었습니다!',
      data: {
        equipment: equipment,
        status: 'active'
      }
    });

  } catch (error) {
    console.error('장비 확인 오류:', error);
    res.status(500).json({ error: error.message });
  }
});

// 배달원 계정 활성화
async function activateDriver(driverId: string) {
  const driver = await Driver.findById(driverId).populate('user_id');

  await driver.update({
    status: 'offline'  // 활성화되었지만 오프라인
  });

  await User.findByIdAndUpdate(driver.user_id.id, {
    status: 'active'
  });

  // 환영 이메일 발송
  await sendEmail({
    to: driver.user_id.email,
    subject: '환영합니다! 배달을 시작할 준비가 되었습니다',
    template: 'driver-activated',
    data: {
      driverName: driver.user_id.first_name,
      appDownloadLink: process.env.DRIVER_APP_URL,
      supportEmail: process.env.SUPPORT_EMAIL,
      supportPhone: process.env.SUPPORT_PHONE
    }
  });

  console.log(`배달원 ${driverId} 활성화됨`);
}
```

---

## 8.5 배포

### 8.5.1 프로덕션 체크리스트

**보안:**
- [ ] 모든 기본 비밀번호 변경
- [ ] JWT 시크릿 회전
- [ ] SSL/TLS (HTTPS) 활성화
- [ ] CORS 올바르게 구성
- [ ] 속도 제한 활성화
- [ ] WAF (웹 애플리케이션 방화벽) 설정
- [ ] 저장 데이터베이스 암호화 활성화
- [ ] VPC/네트워크 보안 구성
- [ ] 환경 변수를 .env 파일이 아닌 비밀 관리자에 저장
- [ ] API 키에 대한 속도 제한 구현

**데이터베이스:**
- [ ] 자동 백업 설정 (최소 매일)
- [ ] 데이터베이스 복제 구성
- [ ] 연결 풀링 설정
- [ ] 확장을 위한 읽기 복제본 생성
- [ ] 모니터링 및 경고 설정
- [ ] 인덱스 최적화
- [ ] 백업 테스트 (복원 수행)
- [ ] 데이터 보존 정책 구현

**인프라:**
- [ ] 로드 밸런서 설정
- [ ] 자동 확장 구성
- [ ] 정적 자산에 대한 CDN 설정
- [ ] 페일오버를 사용한 DNS 구성
- [ ] 상태 확인 설정
- [ ] 로그 집계 구성 (ELK, CloudWatch)
- [ ] 메트릭 수집 설정 (Prometheus, Datadog)

**모니터링:**
- [ ] 애플리케이션 모니터링 설정 (Datadog, New Relic)
- [ ] 오류 추적 구성 (Sentry)
- [ ] 가동 시간 모니터링 설정 (Pingdom, UptimeRobot)
- [ ] 경고 규칙 생성
- [ ] 대기 순환 설정
- [ ] SLA 정의 및 추적

**테스트:**
- [ ] 통합 테스트 실행
- [ ] 부하 테스트 수행
- [ ] 보안 감사/침투 테스트
- [ ] 재해 복구 테스트
- [ ] 백업 검증
- [ ] 성능 테스트

### 8.5.2 Docker 배포

**Dockerfile:**

```dockerfile
# 다단계 빌드로 이미지 크기 최적화
FROM node:18-alpine AS builder

WORKDIR /app

# 의존성 설치
COPY package*.json ./
RUN npm ci --only=production && npm cache clean --force

# 소스 코드 복사 및 빌드
COPY . .
RUN npm run build

# 프로덕션 이미지
FROM node:18-alpine

# 보안을 위한 비루트 사용자 생성
RUN addgroup -g 1001 -S nodejs && \
    adduser -S nodejs -u 1001

WORKDIR /app

# 빌드 단계에서 파일 복사
COPY --from=builder --chown=nodejs:nodejs /app/node_modules ./node_modules
COPY --from=builder --chown=nodejs:nodejs /app/dist ./dist
COPY --from=builder --chown=nodejs:nodejs /app/package.json ./

# 비루트 사용자로 전환
USER nodejs

# 포트 노출
EXPOSE 3000

# 상태 확인
HEALTHCHECK --interval=30s --timeout=3s --start-period=40s --retries=3 \
  CMD node -e "require('http').get('http://localhost:3000/health', (res) => { process.exit(res.statusCode === 200 ? 0 : 1); })"

# 애플리케이션 시작
CMD ["node", "dist/server.js"]
```

**docker-compose.yml:**

```yaml
version: '3.8'

services:
  # API 서버
  api:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=production
      - DATABASE_URL=postgresql://postgres:${DB_PASSWORD}@db:5432/fooddelivery
      - REDIS_URL=redis://redis:6379
      - JWT_SECRET=${JWT_SECRET}
    depends_on:
      db:
        condition: service_healthy
      redis:
        condition: service_healthy
    restart: unless-stopped
    networks:
      - app-network
    volumes:
      - ./logs:/app/logs

  # WebSocket 서버
  websocket:
    build:
      context: .
      dockerfile: Dockerfile
    command: node dist/websocket.js
    ports:
      - "3001:3001"
    environment:
      - NODE_ENV=production
      - REDIS_URL=redis://redis:6379
    depends_on:
      redis:
        condition: service_healthy
    restart: unless-stopped
    networks:
      - app-network

  # 백그라운드 워커
  workers:
    build:
      context: .
      dockerfile: Dockerfile
    command: node dist/workers.js
    environment:
      - NODE_ENV=production
      - DATABASE_URL=postgresql://postgres:${DB_PASSWORD}@db:5432/fooddelivery
      - REDIS_URL=redis://redis:6379
    depends_on:
      db:
        condition: service_healthy
      redis:
        condition: service_healthy
    restart: unless-stopped
    networks:
      - app-network

  # PostgreSQL + TimescaleDB
  db:
    image: timescale/timescaledb:latest-pg14
    environment:
      - POSTGRES_PASSWORD=${DB_PASSWORD}
      - POSTGRES_DB=fooddelivery
      - POSTGRES_USER=postgres
    volumes:
      - db-data:/var/lib/postgresql/data
      - ./migrations:/docker-entrypoint-initdb.d
    ports:
      - "5432:5432"
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U postgres"]
      interval: 10s
      timeout: 5s
      retries: 5
    networks:
      - app-network

  # Redis
  redis:
    image: redis:7-alpine
    command: redis-server --appendonly yes
    ports:
      - "6379:6379"
    volumes:
      - redis-data:/data
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 10s
      timeout: 3s
      retries: 5
    networks:
      - app-network

  # Nginx (리버스 프록시)
  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf:ro
      - ./nginx/ssl:/etc/nginx/ssl:ro
    depends_on:
      - api
      - websocket
    restart: unless-stopped
    networks:
      - app-network

volumes:
  db-data:
  redis-data:

networks:
  app-network:
    driver: bridge
```

### 8.5.3 Kubernetes 배포

**deployment.yaml:**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: food-delivery-api
  labels:
    app: food-delivery
    component: api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: food-delivery
      component: api
  template:
    metadata:
      labels:
        app: food-delivery
        component: api
    spec:
      containers:
      - name: api
        image: your-registry.com/food-delivery-api:latest
        imagePullPolicy: Always
        ports:
        - containerPort: 3000
          name: http
        env:
        - name: NODE_ENV
          value: "production"
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secrets
              key: url
        - name: REDIS_URL
          value: redis://redis-service:6379
        - name: JWT_SECRET
          valueFrom:
            secretKeyRef:
              name: app-secrets
              key: jwt-secret
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 3000
          initialDelaySeconds: 30
          periodSeconds: 10
          timeoutSeconds: 5
          failureThreshold: 3
        readinessProbe:
          httpGet:
            path: /ready
            port: 3000
          initialDelaySeconds: 5
          periodSeconds: 5
          timeoutSeconds: 3
          failureThreshold: 3

---
apiVersion: v1
kind: Service
metadata:
  name: food-delivery-api
  labels:
    app: food-delivery
    component: api
spec:
  type: LoadBalancer
  selector:
    app: food-delivery
    component: api
  ports:
  - port: 80
    targetPort: 3000
    protocol: TCP
    name: http

---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: food-delivery-api-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: food-delivery-api
  minReplicas: 3
  maxReplicas: 20
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 50
        periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 0
      policies:
      - type: Percent
        value: 100
        periodSeconds: 30
      - type: Pods
        value: 2
        periodSeconds: 30
      selectPolicy: Max

---
apiVersion: v1
kind: ConfigMap
metadata:
  name: app-config
data:
  ENABLE_TEMPERATURE_MONITORING: "true"
  ENABLE_ROUTE_OPTIMIZATION: "true"
  LOG_LEVEL: "info"
```

---

## 8.6 테스트

### 8.6.1 통합 테스트

```typescript
import request from 'supertest';
import { app } from '../src/app';
import { setupTestDB, teardownTestDB } from './helpers/db';

describe('주문 API', () => {
  let authToken: string;
  let customerId: string;
  let restaurantId: string;
  let driverId: string;

  // 모든 테스트 전에 실행
  beforeAll(async () => {
    await setupTestDB();
  });

  // 모든 테스트 후에 실행
  afterAll(async () => {
    await teardownTestDB();
  });

  // 각 테스트 전에 실행
  beforeEach(async () => {
    // 테스트 고객 생성
    const customerResponse = await request(app)
      .post('/auth/register')
      .send({
        email: 'test@example.com',
        password: 'password123',
        first_name: '테스트',
        last_name: '고객',
        role: 'customer'
      });

    customerId = customerResponse.body.data.userId;

    // 로그인 및 토큰 획득
    const loginResponse = await request(app)
      .post('/auth/login')
      .send({
        email: 'test@example.com',
        password: 'password123'
      });

    authToken = loginResponse.body.data.token;

    // 테스트 레스토랑 생성
    restaurantId = 'rest_test_123';
    driverId = 'driver_test_123';
  });

  describe('POST /orders', () => {
    it('주문을 성공적으로 생성해야 함', async () => {
      const response = await request(app)
        .post('/orders')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          restaurantId: restaurantId,
          items: [
            {
              itemId: 'item_123',
              quantity: 2,
              modifiers: [
                { id: 'mod_1', name: '치즈 추가', price: 100 }
              ]
            }
          ],
          deliveryLocation: {
            latitude: 37.7858,
            longitude: -122.4068,
            address: '456 Mission St, San Francisco, CA 94105'
          },
          paymentMethodId: 'pm_test_123',
          specialInstructions: '문 앞에 두고 가주세요'
        });

      expect(response.status).toBe(201);
      expect(response.body.success).toBe(true);
      expect(response.body.data).toHaveProperty('id');
      expect(response.body.data).toHaveProperty('confirmationCode');
      expect(response.body.data.status).toBe('pending');
      expect(response.body.data.total).toBeGreaterThan(0);
    });

    it('유효하지 않은 위치로 주문을 거부해야 함', async () => {
      const response = await request(app)
        .post('/orders')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          restaurantId: restaurantId,
          items: [{ itemId: 'item_123', quantity: 1 }],
          deliveryLocation: {
            latitude: 200,  // 유효하지 않음
            longitude: -122.4068,
            address: '테스트'
          }
        });

      expect(response.status).toBe(400);
      expect(response.body.error).toBeDefined();
      expect(response.body.error.code).toBe('INVALID_INPUT');
    });

    it('인증 없이 주문을 거부해야 함', async () => {
      const response = await request(app)
        .post('/orders')
        .send({
          restaurantId: restaurantId,
          items: [{ itemId: 'item_123', quantity: 1 }]
        });

      expect(response.status).toBe(401);
    });

    it('배달 범위를 벗어난 주문을 거부해야 함', async () => {
      const response = await request(app)
        .post('/orders')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          restaurantId: restaurantId,
          items: [{ itemId: 'item_123', quantity: 1 }],
          deliveryLocation: {
            latitude: 40.7128,  // 뉴욕 (샌프란시스코 레스토랑에서 너무 멀다)
            longitude: -74.0060,
            address: 'New York, NY'
          }
        });

      expect(response.status).toBe(400);
      expect(response.body.error.code).toBe('OUT_OF_DELIVERY_RANGE');
    });
  });

  describe('GET /orders/:id', () => {
    it('주문 세부 정보를 반환해야 함', async () => {
      // 먼저 주문 생성
      const createResponse = await request(app)
        .post('/orders')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          restaurantId: restaurantId,
          items: [{ itemId: 'item_123', quantity: 1 }],
          deliveryLocation: {
            latitude: 37.7858,
            longitude: -122.4068,
            address: '테스트 주소'
          }
        });

      const orderId = createResponse.body.data.id;

      // 주문 가져오기
      const response = await request(app)
        .get(`/orders/${orderId}`)
        .set('Authorization', `Bearer ${authToken}`);

      expect(response.status).toBe(200);
      expect(response.body.data.id).toBe(orderId);
      expect(response.body.data).toHaveProperty('restaurant');
      expect(response.body.data).toHaveProperty('items');
      expect(response.body.data).toHaveProperty('status');
    });
  });

  describe('PATCH /orders/:id/cancel', () => {
    it('주문을 취소해야 함', async () => {
      // 주문 생성
      const createResponse = await request(app)
        .post('/orders')
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          restaurantId: restaurantId,
          items: [{ itemId: 'item_123', quantity: 1 }],
          deliveryLocation: {
            latitude: 37.7858,
            longitude: -122.4068,
            address: '테스트 주소'
          }
        });

      const orderId = createResponse.body.data.id;

      // 주문 취소
      const response = await request(app)
        .patch(`/orders/${orderId}/cancel`)
        .set('Authorization', `Bearer ${authToken}`)
        .send({
          reason: '마음이 바뀜'
        });

      expect(response.status).toBe(200);
      expect(response.body.data.status).toBe('cancelled');
    });
  });
});
```

### 8.6.2 부하 테스트

```javascript
// k6 부하 테스트
import http from 'k6/http';
import { check, sleep, group } from 'k6';
import { Rate, Trend, Counter } from 'k6/metrics';

// 사용자 정의 메트릭
const errorRate = new Rate('errors');
const orderCreationTime = new Trend('order_creation_time');
const ordersCreated = new Counter('orders_created');

// 테스트 옵션
export let options = {
  stages: [
    { duration: '2m', target: 100 },   // 100명의 사용자로 증가
    { duration: '5m', target: 100 },   // 100명의 사용자 유지
    { duration: '2m', target: 200 },   // 200명의 사용자로 증가
    { duration: '5m', target: 200 },   // 200명의 사용자 유지
    { duration: '2m', target: 0 },     // 0명의 사용자로 감소
  ],
  thresholds: {
    http_req_duration: ['p(95)<500'],  // 95% 요청이 500ms 미만
    http_req_failed: ['rate<0.01'],    // 오류율이 1% 미만
    'errors': ['rate<0.05'],           // 전체 오류율이 5% 미만
  },
};

const BASE_URL = 'https://api.example.com/v1';

// 테스트 설정
export function setup() {
  // 테스트 고객 생성
  const signupRes = http.post(`${BASE_URL}/auth/register`, JSON.stringify({
    email: `loadtest-${Date.now()}@example.com`,
    password: 'password123',
    first_name: '부하',
    last_name: '테스트'
  }), {
    headers: { 'Content-Type': 'application/json' }
  });

  const credentials = signupRes.json();
  return { email: credentials.email, password: 'password123' };
}

// 주요 테스트
export default function(data) {
  group('인증 흐름', () => {
    // 로그인
    const loginRes = http.post(`${BASE_URL}/auth/login`, JSON.stringify({
      email: data.email,
      password: data.password
    }), {
      headers: { 'Content-Type': 'application/json' }
    });

    check(loginRes, {
      '로그인 성공': (r) => r.status === 200,
      '토큰 수신': (r) => r.json('data.token') !== undefined,
    });

    if (loginRes.status !== 200) {
      errorRate.add(1);
      return;
    }

    const token = loginRes.json('data.token');

    // 주문 생성
    group('주문 생성', () => {
      const orderPayload = {
        restaurantId: 'rest_123',
        items: [
          {
            itemId: 'item_456',
            quantity: Math.floor(Math.random() * 3) + 1
          }
        ],
        deliveryLocation: {
          latitude: 37.7858 + (Math.random() - 0.5) * 0.1,
          longitude: -122.4068 + (Math.random() - 0.5) * 0.1,
          address: '테스트 주소'
        }
      };

      const orderRes = http.post(
        `${BASE_URL}/orders`,
        JSON.stringify(orderPayload),
        {
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        }
      );

      const orderSuccess = check(orderRes, {
        '주문 생성됨': (r) => r.status === 201,
        '주문 ID 수신': (r) => r.json('data.id') !== undefined,
      });

      if (orderSuccess) {
        ordersCreated.add(1);
        orderCreationTime.add(orderRes.timings.duration);
      } else {
        errorRate.add(1);
      }

      // 주문 ID 가져오기
      if (orderRes.status === 201) {
        const orderId = orderRes.json('data.id');

        // 주문 세부 정보 가져오기
        const getOrderRes = http.get(
          `${BASE_URL}/orders/${orderId}`,
          {
            headers: { 'Authorization': `Bearer ${token}` }
          }
        );

        check(getOrderRes, {
          '주문 세부 정보 가져옴': (r) => r.status === 200,
        });
      }
    });

    // 레스토랑 목록 조회
    group('레스토랑 검색', () => {
      const searchRes = http.get(
        `${BASE_URL}/restaurants?lat=37.7858&lng=-122.4068&radius=5`,
        {
          headers: { 'Authorization': `Bearer ${token}` }
        }
      );

      check(searchRes, {
        '레스토랑 검색 성공': (r) => r.status === 200,
        '결과 수신': (r) => r.json('data.results').length > 0,
      });
    });
  });

  // 요청 간 1초 대기
  sleep(1);
}

// 테스트 종료
export function teardown(data) {
  console.log('부하 테스트 완료');
}
```

---

## 8.7 출시 체크리스트

### 출시 전 주:
- [ ] 모든 테스트 완료 (통합, 부하, 보안)
- [ ] 지원 팀 교육
- [ ] 일반적인 문제에 대한 런북 준비
- [ ] 모니터링 대시보드 설정
- [ ] 경고 구성
- [ ] 재해 복구 절차 테스트
- [ ] 문서 업데이트 (API 문서, 사용자 가이드)
- [ ] 이해관계자에게 출시 계획 전달

### 출시 전날:
- [ ] 최종 데이터베이스 백업
- [ ] 모든 외부 통합 작동 확인
- [ ] 모든 중요 경로 테스트
- [ ] 지원 팀 준비 확인
- [ ] 롤백 절차 검토
- [ ] 부하 테스트 다시 실행
- [ ] SSL 인증서 확인
- [ ] DNS 설정 확인

### 출시 당일:
- [ ] 프로덕션에 배포
- [ ] 상태 확인 통과 확인
- [ ] 오류율 면밀히 모니터링
- [ ] 데이터베이스 성능 관찰
- [ ] 외부 API 쿼터 확인
- [ ] 사용자 피드백 모니터링
- [ ] 주요 흐름 테스트 (주문 생성, 배달 추적)
- [ ] 실시간 메트릭 모니터링 (지연 시간, 처리량, 오류)

### 출시 후 주:
- [ ] 오류 로그 일일 확인
- [ ] 성능 메트릭 검토
- [ ] 사용자 피드백 수집
- [ ] 최적화 기회 식별
- [ ] 다음 반복 계획
- [ ] 프로덕션 문제 문서화
- [ ] 지원 티켓 모니터링
- [ ] 주간 회고 수행

---

## 8.8 규정 준수 인증

### 8.8.1 식품 안전 규정 준수

**필수 문서:**

1. **온도 모니터링 로그** (90일 보존)
   - 모든 온도 판독값
   - 경고 및 조치 사항
   - 센서 교정 기록

2. **배달원 식품 안전 인증**
   - 교육 완료 증명서
   - 테스트 점수 (80% 이상)
   - 인증 만료 날짜

3. **장비 검사 기록**
   - 온열/냉장 가방 사양
   - 센서 교정 날짜
   - 유지 관리 로그

4. **사고 보고서**
   - 온도 위반
   - 고객 불만
   - 시정 조치

5. **시정 조치 로그**
   - 식별된 문제
   - 취한 조치
   - 결과 확인

**감사 준비:**

```typescript
// 규정 준수 보고서 생성
async function generateComplianceReport(startDate: Date, endDate: Date) {
  const report = {
    period: {
      start: startDate,
      end: endDate
    },

    // 온도 모니터링 통계
    temperatureMonitoring: {
      totalReadings: await TemperatureReading.countDocuments({
        time: { $gte: startDate, $lte: endDate }
      }),
      violations: await TemperatureReading.countDocuments({
        time: { $gte: startDate, $lte: endDate },
        in_compliance: false
      }),
      complianceRate: 0  // 아래에서 계산
    },

    // 배달원 인증
    driverCertifications: {
      totalDrivers: await Driver.countDocuments({ status: { $ne: 'deleted' } }),
      certified: await Driver.countDocuments({
        food_safety_certified: true,
        food_safety_cert_expiry: { $gte: new Date() }
      }),
      expiringSoon: await Driver.find({
        food_safety_cert_expiry: {
          $gte: new Date(),
          $lte: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000)  // 30일
        }
      }).select('user_id food_safety_cert_expiry')
    },

    // 장비 검증
    equipmentVerification: {
      withHotBag: await Driver.countDocuments({ has_hot_bag: true }),
      withColdBag: await Driver.countDocuments({ has_cold_bag: true }),
      withSensor: await Driver.countDocuments({ has_temperature_sensor: true })
    },

    // 사고 및 시정 조치
    incidents: await Incident.find({
      created_at: { $gte: startDate, $lte: endDate }
    }).populate('order_id driver_id'),

    correctiveActions: await CorrectiveAction.find({
      created_at: { $gte: startDate, $lte: endDate }
    })
  };

  // 준수율 계산
  if (report.temperatureMonitoring.totalReadings > 0) {
    report.temperatureMonitoring.complianceRate =
      1 - (report.temperatureMonitoring.violations / report.temperatureMonitoring.totalReadings);
  }

  return report;
}

// 감사 내보내기 생성
app.get('/admin/compliance/export', async (req, res) => {
  const { startDate, endDate } = req.query;

  const report = await generateComplianceReport(
    new Date(startDate),
    new Date(endDate)
  );

  // PDF 생성
  const pdf = await generatePDF({
    template: 'compliance-report',
    data: report
  });

  res.setHeader('Content-Type', 'application/pdf');
  res.setHeader('Content-Disposition', 'attachment; filename=compliance-report.pdf');
  res.send(pdf);
});
```

### 8.8.2 데이터 프라이버시 규정 준수

**GDPR/CCPA 요구사항:**

```typescript
// 개인 정보 보호 정책
app.get('/privacy-policy', (req, res) => {
  res.render('privacy-policy', {
    lastUpdated: '2025-01-01',
    dataCollected: [
      '이름, 이메일, 전화번호',
      '배달 주소',
      '주문 내역',
      '위치 데이터 (배달원)',
      '결제 정보',
      '온도 센서 데이터'
    ],
    dataUsage: [
      '주문 처리',
      '배달 조정',
      '식품 안전 모니터링',
      '서비스 개선'
    ],
    dataRetention: '계정 삭제 후 90일',
    userRights: [
      '데이터 액세스',
      '데이터 내보내기',
      '데이터 삭제',
      '마케팅 거부'
    ]
  });
});

// 사용자 데이터 내보내기
app.get('/users/:id/export-data', async (req, res) => {
  const { id } = req.params;

  // 권한 확인
  if (req.user.id !== id && req.user.role !== 'admin') {
    return res.status(403).json({ error: '권한 없음' });
  }

  // 모든 사용자 데이터 수집
  const userData = {
    profile: await User.findById(id).select('-password_hash'),
    orders: await Order.find({ customer_id: id }),
    deliveries: await Order.find({ driver_id: id }),
    reviews: await Review.find({ user_id: id }),
    temperatureReadings: await TemperatureReading.find({ driver_id: id })
  };

  // JSON 파일로 내보내기
  res.setHeader('Content-Type', 'application/json');
  res.setHeader('Content-Disposition', 'attachment; filename=my-data.json');
  res.json(userData);
});

// 계정 삭제
app.delete('/users/:id', async (req, res) => {
  const { id } = req.params;

  // 권한 확인
  if (req.user.id !== id && req.user.role !== 'admin') {
    return res.status(403).json({ error: '권한 없음' });
  }

  // 소프트 삭제 (90일 후 영구 삭제)
  await User.findByIdAndUpdate(id, {
    status: 'deleted',
    email: `deleted_${Date.now()}@example.com`,  // 이메일 익명화
    phone: null,
    deleted_at: new Date(),
    permanent_deletion_date: new Date(Date.now() + 90 * 24 * 60 * 60 * 1000)
  });

  // 관련 데이터 익명화
  await Order.updateMany(
    { customer_id: id },
    {
      customer_name: '삭제된 사용자',
      customer_phone: null,
      customer_email: null
    }
  );

  res.json({
    message: '계정이 삭제되었습니다. 데이터는 90일 후 영구적으로 제거됩니다.'
  });
});

// 쿠키 동의
app.post('/consent/cookies', async (req, res) => {
  const { userId, consent } = req.body;

  await UserConsent.create({
    user_id: userId,
    consent_type: 'cookies',
    consent_given: consent,
    timestamp: new Date(),
    ip_address: req.ip,
    user_agent: req.headers['user-agent']
  });

  res.json({ success: true });
});
```

---

## 8.9 요약

이 구현 가이드는 다음을 다뤘습니다:

### 주요 주제:

1. **인프라 설정**
   - 기술 스택 선택 (Node.js, PostgreSQL, Redis)
   - 개발 환경 구성
   - 환경 변수 관리

2. **데이터베이스 설계**
   - 핵심 테이블 스키마
   - 관계 및 제약조건
   - 인덱스 최적화
   - 뷰 및 트리거

3. **배달원 온보딩**
   - 신청서 제출
   - 신원 조회 (Checkr)
   - 교육 및 인증
   - 장비 검증

4. **배포**
   - Docker 컨테이너화
   - Kubernetes 오케스트레이션
   - 프로덕션 체크리스트
   - CI/CD 파이프라인

5. **테스트**
   - 통합 테스트 (Jest, Supertest)
   - 부하 테스트 (k6)
   - 보안 테스트

6. **규정 준수**
   - 식품 안전 문서화
   - 데이터 프라이버시 (GDPR/CCPA)
   - 감사 준비

이 가이드를 따르면 프로덕션 준비가 완료되고 규정을 준수하며 확장 가능한 WIA-IND-009 구현을 보장할 수 있습니다.

---

## 축하합니다!

WIA 음식 배달 표준 (WIA-IND-009) 전자책을 완료했습니다. 이제 다음을 우선시하는 완전하고 규정을 준수하는 음식 배달 시스템을 구축할 수 있는 지식을 갖췄습니다:

### 핵심 가치:

- **식품 안전**: 온도 모니터링을 통한 품질 보장
- **배달원 복지**: 공정한 보상 및 지원
- **고객 투명성**: 실시간 추적 및 업데이트
- **운영 효율성**: 경로 최적화 및 자동화
- **시스템 상호 운용성**: 표준 API 및 통합

### 다음 단계:

1. **소규모 파일럿으로 시작**
   - 단일 레스토랑
   - 소수의 배달원 (3-5명)
   - 제한된 배달 지역
   - 1-2주 동안 테스트

2. **피드백을 기반으로 반복**
   - 배달원 피드백 수집
   - 고객 만족도 측정
   - 성능 메트릭 분석
   - 프로세스 개선

3. **점진적으로 확장**
   - 더 많은 레스토랑 추가
   - 배달원 팀 확장
   - 배달 지역 확대
   - 새로운 기능 구현

4. **WIA 커뮤니티 가입**
   - 경험 공유
   - 모범 사례 학습
   - 표준 개선에 기여
   - 네트워킹 및 지원

### 리소스:

- **GitHub**: https://github.com/WIA-Official/wia-standards
- **문서**: https://wiastandards.com/ind-009
- **커뮤니티**: https://forum.wiastandards.com
- **지원**: support@wiastandards.com

### 기억하세요:

> "완벽은 선의 적이다." - 볼테르

소규모로 시작하고, 빠르게 반복하며, 계속 개선하세요. 가장 성공적인 플랫폼은 하루아침에 만들어지지 않았습니다.

---

## 복습 문제

1. 음식 배달 플랫폼에 권장되는 기술 스택은 무엇이며 그 이유는 무엇인가요?
2. TimescaleDB가 온도 판독값에 사용되는 이유는 무엇인가요?
3. 배달원 온보딩의 4단계는 무엇인가요?
4. Docker와 Kubernetes의 차이점은 무엇인가요?
5. 부하 테스트에서 모니터링해야 할 주요 메트릭은 무엇인가요?
6. GDPR 규정 준수를 위한 주요 요구사항은 무엇인가요?
7. 식품 안전 감사를 위해 어떤 문서를 준비해야 하나요?
8. 프로덕션 배포 전 체크리스트의 주요 항목은 무엇인가요?

---

**弘益人間 (홍익인간) · Benefit All Humanity**

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
