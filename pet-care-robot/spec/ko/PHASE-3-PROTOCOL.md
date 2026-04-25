# Phase 3: Pet Care Robot 프로토콜 사양

## WIA-PET-CARE-ROBOT 프로토콜 표준

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE3-001
**Primary Color**: #F59E0B (Amber)

---

## 1. 개요

### 1.1 목적

이 사양은 펫 케어 로봇 생태계를 위한 통신 프로토콜, 보안 메커니즘 및 시스템 통합 표준을 정의합니다. 로봇, 클라우드 서비스, 엣지 장치 및 클라이언트 애플리케이션 간의 안정적이고 안전하며 상호 운용 가능한 통신을 보장합니다.

**프로토콜 목표**:
- 실시간 통신 채널 정의
- 보안 및 암호화 표준 설정
- 안정적인 명령 및 제어 활성화
- 오프라인 작동 및 동기화 지원
- 다중 로봇 조정 촉진
- 데이터 무결성 및 개인정보 보호 보장
- 크로스 플랫폼 호환성 활성화

### 1.2 프로토콜 스택

| 계층 | 프로토콜 | 목적 |
|------|----------|------|
| **애플리케이션** | WIA-PET-CARE-ROBOT | 펫 케어 작업 및 데이터 |
| **메시징** | MQTT 5.0, WebSocket | 실시간 양방향 통신 |
| **전송** | TLS 1.3, DTLS 1.3 | 안전한 데이터 전송 |
| **네트워크** | IPv4, IPv6 | 네트워크 연결 |
| **물리** | WiFi, Ethernet, LTE | 장치 연결 |

### 1.3 통신 패턴

| 패턴 | 사용 사례 | 프로토콜 | 지연 시간 |
|------|----------|----------|-----------|
| **요청-응답** | API 작업 | HTTPS/REST | <500ms |
| **게시-구독** | 상태 업데이트 | MQTT | <100ms |
| **스트리밍** | 비디오/오디오 | WebRTC | <50ms |
| **명령-제어** | 로봇 작업 | MQTT/CoAP | <200ms |
| **배치 동기화** | 과거 데이터 | HTTPS | 가변 |

---

## 2. MQTT 프로토콜 구현

### 2.1 브로커 구성

```yaml
broker:
  version: "5.0"
  endpoints:
    - mqtt://mqtt.petcare.wia.org:1883
    - mqtts://mqtt.petcare.wia.org:8883
    - wss://mqtt.petcare.wia.org:443/mqtt

  authentication:
    method: "username_password"
    tls_client_cert: true
    oauth2: true

  qos_levels:
    - 0  # 최대 한 번 (상태 업데이트)
    - 1  # 최소 한 번 (명령)
    - 2  # 정확히 한 번 (중요 작업)

  persistence:
    enabled: true
    type: "disk"
    retention: "7d"

  limits:
    max_packet_size: 268435456  # 256MB
    max_connections: 10000
    max_subscriptions_per_client: 100
```

### 2.2 토픽 구조

```
wia/pet-care/{organizationId}/{robotId}/{category}/{action}
```

**토픽 계층:**

| 토픽 패턴 | 설명 | QoS | 보존 |
|-----------|------|-----|------|
| `wia/pet-care/{orgId}/{robotId}/status` | 로봇 상태 업데이트 | 0 | 예 |
| `wia/pet-care/{orgId}/{robotId}/command` | 로봇 명령 | 1 | 아니오 |
| `wia/pet-care/{orgId}/{robotId}/telemetry` | 센서 원격 측정 | 0 | 아니오 |
| `wia/pet-care/{orgId}/{robotId}/feeding/event` | 급식 이벤트 | 1 | 아니오 |
| `wia/pet-care/{orgId}/{robotId}/play/event` | 놀이 이벤트 | 1 | 아니오 |
| `wia/pet-care/{orgId}/{robotId}/health/alert` | 건강 경고 | 2 | 예 |
| `wia/pet-care/{orgId}/{robotId}/error` | 오류 알림 | 2 | 예 |
| `wia/pet-care/{orgId}/broadcast` | 조직 전체 메시지 | 1 | 아니오 |

### 2.3 MQTT 메시지 형식

```json
{
  "version": "1.0",
  "messageId": "MSG-ABC123456789",
  "timestamp": "2025-12-18T10:00:00Z",
  "source": {
    "robotId": "PCR-ABC123456789",
    "deviceType": "multi_function",
    "firmwareVersion": "2.5.1"
  },
  "payload": {
    "type": "status_update",
    "data": {
      "operational": "online",
      "batteryLevel": 85,
      "foodLevel": 65,
      "waterLevel": 80
    }
  },
  "metadata": {
    "priority": "normal",
    "ttl": 300,
    "correlationId": "CORR-XYZ789"
  }
}
```

### 2.4 명령 메시지 구조

```json
{
  "version": "1.0",
  "commandId": "CMD-20251218-001",
  "timestamp": "2025-12-18T10:05:00Z",
  "source": {
    "clientId": "mobile-app-12345",
    "userId": "USER-ABC123"
  },
  "target": {
    "robotId": "PCR-ABC123456789"
  },
  "command": {
    "action": "dispense_food",
    "parameters": {
      "petId": "PET-DOG001",
      "portionSize": 100,
      "foodType": "dry_kibble"
    },
    "timeout": 30,
    "retryPolicy": {
      "maxRetries": 3,
      "retryDelay": 5
    }
  },
  "responseRequired": true,
  "responseTimeout": 10
}
```

### 2.5 응답 메시지 구조

```json
{
  "version": "1.0",
  "responseId": "RESP-20251218-001",
  "timestamp": "2025-12-18T10:05:08Z",
  "correlationId": "CMD-20251218-001",
  "source": {
    "robotId": "PCR-ABC123456789"
  },
  "result": {
    "status": "success",
    "code": 200,
    "message": "음식이 성공적으로 분배되었습니다",
    "data": {
      "eventId": "FEED-20251218-001",
      "dispensedAmount": 100,
      "remainingFood": 3150
    }
  },
  "executionTime": 8.2
}
```

### 2.6 MQTT 연결 예제

```python
import paho.mqtt.client as mqtt
import json
import ssl
from datetime import datetime

class PetCareRobotMQTT:
    def __init__(self, broker, port, org_id, robot_id, username, password):
        self.broker = broker
        self.port = port
        self.org_id = org_id
        self.robot_id = robot_id
        self.client = mqtt.Client(
            client_id=f"{robot_id}",
            protocol=mqtt.MQTTv5
        )

        # 자격 증명 설정
        self.client.username_pw_set(username, password)

        # TLS 구성
        self.client.tls_set(
            ca_certs="/path/to/ca.crt",
            certfile="/path/to/client.crt",
            keyfile="/path/to/client.key",
            tls_version=ssl.PROTOCOL_TLSv1_2
        )

        # 콜백 설정
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"MQTT 브로커에 연결됨")

            # 명령 토픽 구독
            command_topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/command"
            client.subscribe(command_topic, qos=1)

            # 브로드캐스트 토픽 구독
            broadcast_topic = f"wia/pet-care/{self.org_id}/broadcast"
            client.subscribe(broadcast_topic, qos=1)

            # 온라인 상태 게시
            self.publish_status("online")
        else:
            print(f"연결 실패 코드 {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if "command" in payload:
                self.handle_command(payload)
            elif "broadcast" in payload:
                self.handle_broadcast(payload)

        except json.JSONDecodeError as e:
            print(f"메시지 디코딩 실패: {e}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print(f"예기치 않은 연결 해제. 재연결 중...")
            self.connect()

    def connect(self):
        self.client.connect(self.broker, self.port, keepalive=60)
        self.client.loop_start()

    def publish_status(self, operational_status):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/status"

        message = {
            "version": "1.0",
            "messageId": f"MSG-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "source": {
                "robotId": self.robot_id,
                "deviceType": "multi_function"
            },
            "payload": {
                "type": "status_update",
                "data": {
                    "operational": operational_status,
                    "batteryLevel": 85,
                    "foodLevel": 65,
                    "waterLevel": 80
                }
            }
        }

        self.client.publish(
            topic,
            payload=json.dumps(message),
            qos=0,
            retain=True
        )

    def publish_feeding_event(self, event_data):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/feeding/event"

        message = {
            "version": "1.0",
            "messageId": f"MSG-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "source": {"robotId": self.robot_id},
            "payload": {
                "type": "feeding_event",
                "data": event_data
            }
        }

        self.client.publish(topic, payload=json.dumps(message), qos=1)

    def handle_command(self, payload):
        command = payload.get("command", {})
        action = command.get("action")
        parameters = command.get("parameters", {})

        # 명령 실행
        if action == "dispense_food":
            result = self.dispense_food(parameters)
        elif action == "start_play":
            result = self.start_play_session(parameters)
        else:
            result = {"status": "error", "message": "알 수 없는 명령"}

        # 응답 전송
        self.send_response(payload["commandId"], result)

    def send_response(self, command_id, result):
        topic = f"wia/pet-care/{self.org_id}/{self.robot_id}/response"

        response = {
            "version": "1.0",
            "responseId": f"RESP-{datetime.now().timestamp()}",
            "timestamp": datetime.now().isoformat(),
            "correlationId": command_id,
            "source": {"robotId": self.robot_id},
            "result": result
        }

        self.client.publish(topic, payload=json.dumps(response), qos=1)

    def dispense_food(self, parameters):
        # 음식 분배 시뮬레이션
        return {
            "status": "success",
            "code": 200,
            "message": "음식이 성공적으로 분배되었습니다",
            "data": {
                "dispensedAmount": parameters.get("portionSize"),
                "remainingFood": 3150
            }
        }

    def start_play_session(self, parameters):
        # 놀이 세션 시작 시뮬레이션
        return {
            "status": "success",
            "code": 200,
            "message": "놀이 세션이 시작되었습니다",
            "data": {
                "sessionId": f"PLAY-{datetime.now().timestamp()}"
            }
        }

# 사용
robot = PetCareRobotMQTT(
    broker="mqtt.petcare.wia.org",
    port=8883,
    org_id="ORG-001",
    robot_id="PCR-ABC123456789",
    username="robot-user",
    password="secure-password"
)

robot.connect()
```

---

## 3. WebSocket 프로토콜

### 3.1 WebSocket 핸드셰이크

```http
GET /ws/v1/robots/PCR-ABC123456789 HTTP/1.1
Host: ws.petcare.wia.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**응답:**
```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
```

### 3.2 WebSocket 메시지 유형

| 메시지 유형 | 방향 | 목적 | 빈도 |
|-------------|------|------|------|
| `ping` | 양방향 | 연결 유지 | 30초마다 |
| `pong` | 응답 | 연결 유지 응답 | ping 시 |
| `subscribe` | 클라이언트 → 서버 | 이벤트 구독 | 연결 시 |
| `unsubscribe` | 클라이언트 → 서버 | 이벤트 구독 취소 | 필요 시 |
| `command` | 클라이언트 → 서버 | 제어 명령 | 요청 시 |
| `event` | 서버 → 클라이언트 | 실시간 이벤트 | 발생 시 |
| `status` | 서버 → 클라이언트 | 상태 업데이트 | 5초마다 |
| `error` | 서버 → 클라이언트 | 오류 알림 | 오류 시 |

### 3.3 WebSocket 프레임 형식

```json
{
  "type": "event",
  "id": "WS-MSG-001",
  "timestamp": "2025-12-18T10:10:00Z",
  "event": "feeding.completed",
  "data": {
    "eventId": "FEED-20251218-001",
    "robotId": "PCR-ABC123456789",
    "petId": "PET-DOG001",
    "portionSize": 150,
    "dispensed": true
  }
}
```

### 3.4 WebSocket 클라이언트 구현

```javascript
class PetCareWebSocket {
  constructor(url, token) {
    this.url = url;
    this.token = token;
    this.ws = null;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
    this.reconnectDelay = 1000;
    this.handlers = new Map();
  }

  connect() {
    this.ws = new WebSocket(`${this.url}?token=${this.token}`);

    this.ws.onopen = () => {
      console.log('WebSocket 연결됨');
      this.reconnectAttempts = 0;
      this.startHeartbeat();

      // 이벤트 구독
      this.subscribe(['feeding.*', 'play.*', 'health.*']);
    };

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        this.handleMessage(message);
      } catch (error) {
        console.error('메시지 파싱 실패:', error);
      }
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket 오류:', error);
    };

    this.ws.onclose = () => {
      console.log('WebSocket 연결 해제됨');
      this.stopHeartbeat();
      this.reconnect();
    };
  }

  reconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

      console.log(`${delay}ms 후 재연결 (시도 ${this.reconnectAttempts})`);

      setTimeout(() => this.connect(), delay);
    } else {
      console.error('최대 재연결 시도 횟수 도달');
    }
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.ws.readyState === WebSocket.OPEN) {
        this.send({ type: 'ping' });
      }
    }, 30000);
  }

  stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
    }
  }

  send(message) {
    if (this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.warn('WebSocket이 연결되지 않음');
    }
  }

  subscribe(events) {
    this.send({
      type: 'subscribe',
      events: events
    });
  }

  unsubscribe(events) {
    this.send({
      type: 'unsubscribe',
      events: events
    });
  }

  sendCommand(robotId, action, parameters) {
    this.send({
      type: 'command',
      id: `CMD-${Date.now()}`,
      timestamp: new Date().toISOString(),
      robotId: robotId,
      action: action,
      parameters: parameters
    });
  }

  on(eventType, handler) {
    if (!this.handlers.has(eventType)) {
      this.handlers.set(eventType, []);
    }
    this.handlers.get(eventType).push(handler);
  }

  handleMessage(message) {
    const { type, event } = message;

    // 메시지 유형별 처리
    if (this.handlers.has(type)) {
      this.handlers.get(type).forEach(handler => handler(message));
    }

    // 이벤트 유형별 처리 (이벤트 메시지용)
    if (event && this.handlers.has(event)) {
      this.handlers.get(event).forEach(handler => handler(message));
    }
  }

  disconnect() {
    if (this.ws) {
      this.stopHeartbeat();
      this.ws.close();
    }
  }
}

// 사용
const ws = new PetCareWebSocket(
  'wss://ws.petcare.wia.org/v1/robots/PCR-ABC123456789',
  'your-auth-token'
);

ws.on('feeding.completed', (message) => {
  console.log('급식 완료:', message.data);
});

ws.on('health.alert', (message) => {
  console.log('건강 경고:', message.data);
  // 사용자에게 알림 표시
});

ws.connect();

// 명령 전송
ws.sendCommand('PCR-ABC123456789', 'dispense_food', {
  petId: 'PET-DOG001',
  portionSize: 100
});
```

---

## 4. 보안 프로토콜

### 4.1 인증 방법

| 방법 | 사용 사례 | 보안 수준 | 권장 대상 |
|------|----------|----------|----------|
| **OAuth 2.0** | 사용자 애플리케이션 | 높음 | 모바일 앱, 웹 앱 |
| **API 키** | 서비스 간 | 중간 | 백엔드 통합 |
| **클라이언트 인증서** | 장치 인증 | 매우 높음 | 로봇 장치 |
| **JWT 토큰** | 세션 관리 | 높음 | 모든 API 액세스 |
| **HMAC 서명** | Webhook 확인 | 높음 | Webhook 엔드포인트 |

### 4.2 TLS 구성

```nginx
# NGINX TLS 구성
ssl_protocols TLSv1.2 TLSv1.3;
ssl_ciphers 'ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384';
ssl_prefer_server_ciphers on;
ssl_session_cache shared:SSL:10m;
ssl_session_timeout 10m;
ssl_stapling on;
ssl_stapling_verify on;

# 클라이언트 인증서 확인
ssl_client_certificate /path/to/ca.crt;
ssl_verify_client optional;
ssl_verify_depth 2;
```

### 4.3 메시지 암호화

**민감한 데이터를 위한 종단간 암호화:**

```python
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.backends import default_backend
import os
import json

class SecureMessageHandler:
    def __init__(self, private_key, peer_public_key):
        self.private_key = private_key
        self.peer_public_key = peer_public_key

    def encrypt_message(self, plaintext_data):
        """RSA 키 교환을 사용한 AES-256-GCM으로 메시지 암호화"""
        # 랜덤 AES 키 생성
        aes_key = os.urandom(32)  # 256 비트
        iv = os.urandom(12)  # GCM용 96 비트

        # AES-GCM으로 데이터 암호화
        cipher = Cipher(
            algorithms.AES(aes_key),
            modes.GCM(iv),
            backend=default_backend()
        )
        encryptor = cipher.encryptor()

        plaintext = json.dumps(plaintext_data).encode()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()

        # 수신자의 RSA 공개 키로 AES 키 암호화
        encrypted_key = self.peer_public_key.encrypt(
            aes_key,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

        return {
            "version": "1.0",
            "encrypted_key": encrypted_key.hex(),
            "iv": iv.hex(),
            "ciphertext": ciphertext.hex(),
            "tag": encryptor.tag.hex()
        }

    def decrypt_message(self, encrypted_message):
        """수신된 메시지 복호화"""
        # 개인 RSA 키로 AES 키 복호화
        encrypted_key = bytes.fromhex(encrypted_message["encrypted_key"])
        aes_key = self.private_key.decrypt(
            encrypted_key,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

        # AES-GCM으로 데이터 복호화
        iv = bytes.fromhex(encrypted_message["iv"])
        ciphertext = bytes.fromhex(encrypted_message["ciphertext"])
        tag = bytes.fromhex(encrypted_message["tag"])

        cipher = Cipher(
            algorithms.AES(aes_key),
            modes.GCM(iv, tag),
            backend=default_backend()
        )
        decryptor = cipher.decryptor()

        plaintext = decryptor.update(ciphertext) + decryptor.finalize()

        return json.loads(plaintext.decode())

# 키 생성
private_key = rsa.generate_private_key(
    public_exponent=65537,
    key_size=2048,
    backend=default_backend()
)
public_key = private_key.public_key()

# 사용
handler = SecureMessageHandler(private_key, public_key)
encrypted = handler.encrypt_message({"command": "dispense_food", "amount": 100})
decrypted = handler.decrypt_message(encrypted)
```

---

## 5. 오프라인 작동 프로토콜

### 5.1 로컬 저장소 스키마

```json
{
  "version": "1.0",
  "lastSync": "2025-12-18T10:00:00Z",
  "pendingOperations": [
    {
      "operationId": "OP-LOCAL-001",
      "timestamp": "2025-12-18T10:05:00Z",
      "type": "feeding",
      "status": "pending_sync",
      "data": {
        "eventId": "FEED-LOCAL-001",
        "petId": "PET-DOG001",
        "portionSize": 150,
        "dispensed": true
      },
      "retryCount": 0,
      "maxRetries": 3
    }
  ],
  "scheduledEvents": [
    {
      "scheduleId": "SCHED-FEED-001",
      "nextExecution": "2025-12-18T18:00:00Z",
      "action": "dispense_food",
      "parameters": {
        "petId": "PET-DOG001",
        "portionSize": 150
      }
    }
  ],
  "cachedData": {
    "petProfiles": [],
    "configuration": {}
  }
}
```

### 5.2 동기화 프로토콜

| 단계 | 작업 | 방향 | 우선순위 |
|------|------|------|----------|
| **1. 연결** | 연결 설정 | 로봇 → 클라우드 | - |
| **2. 인증** | 장치 인증 | 로봇 → 클라우드 | 긴급 |
| **3. 시간 동기화** | 시계 동기화 | 클라우드 → 로봇 | 높음 |
| **4. 구성 가져오기** | 업데이트된 구성 가져오기 | 클라우드 → 로봇 | 높음 |
| **5. 대기 중 업로드** | 오프라인 작업 업로드 | 로봇 → 클라우드 | 중간 |
| **6. 이벤트 다운로드** | 누락된 이벤트 가져오기 | 클라우드 → 로봇 | 중간 |
| **7. 충돌 해결** | 데이터 충돌 해결 | 양방향 | 높음 |
| **8. 하트비트** | 연결 유지 | 양방향 | 낮음 |

---

## 6. 다중 로봇 조정

### 6.1 조정 시나리오

| 시나리오 | 과제 | 솔루션 | 프로토콜 |
|----------|------|--------|----------|
| **공유 리소스** | 여러 로봇이 동일한 반려동물에 액세스 | 토큰 기반 잠금 | QoS 2를 사용한 MQTT |
| **페일오버** | 주 로봇 오프라인 | 백업 로봇이 인계 | 하트비트 모니터링 |
| **부하 분산** | 작업을 효율적으로 분산 | 중앙 조정자 | REST API |
| **동기화된 작업** | 조정된 다중 로봇 작업 | 이벤트 동기화 | MQTT 브로드캐스트 |
| **데이터 일관성** | 로봇 간 일관된 반려동물 데이터 | 최종 일관성 | 동기화 프로토콜 |

---

## 7. 데이터 동기화 프로토콜

### 7.1 동기화 메시지 형식

```json
{
  "syncId": "SYNC-20251218-001",
  "timestamp": "2025-12-18T10:30:00Z",
  "source": {
    "robotId": "PCR-ABC123456789",
    "lastSyncTimestamp": "2025-12-18T09:00:00Z"
  },
  "changesets": [
    {
      "entity": "feeding_event",
      "operation": "create",
      "data": {
        "eventId": "FEED-20251218-001",
        "timestamp": "2025-12-18T10:00:00Z",
        "petId": "PET-DOG001",
        "portionSize": 150,
        "dispensed": true
      },
      "checksum": "abc123def456"
    }
  ],
  "requestedData": [
    {
      "entity": "pet_profile",
      "filter": {
        "updatedAfter": "2025-12-18T09:00:00Z"
      }
    }
  ]
}
```

---

## 8. 프로토콜 준수 테스트

### 8.1 테스트 시나리오

| 테스트 사례 | 설명 | 예상 결과 |
|-------------|------|-----------|
| **MQTT 연결** | TLS로 브로커에 연결 | 성공적인 인증된 연결 |
| **토픽 구독** | 명령 토픽 구독 | 구독 확인 수신 |
| **명령 실행** | 분배 명령 전송 | 명령 실행됨, 응답 수신 |
| **오프라인 작동** | 연결 해제, 작업 수행, 재연결 | 데이터 성공적으로 동기화 |
| **페일오버** | 주 로봇 오프라인 | 백업 로봇이 60초 내 인계 |
| **암호화** | 암호화된 메시지 전송 | 메시지 성공적으로 복호화 |
| **속도 제한** | 속도 제한 초과 | 429 오류 수신 |
| **WebSocket 재연결** | 강제 연결 해제 | 자동 재연결 성공 |

### 8.2 성능 벤치마크

| 메트릭 | 목표 | 최대 | 측정 |
|--------|------|------|------|
| **명령 지연 시간** | <200ms | <500ms | 전송부터 실행까지 시간 |
| **이벤트 전달** | <100ms | <300ms | 이벤트부터 알림까지 시간 |
| **동기화 지속 시간** | <5s | <30s | 100개 레코드 동기화 시간 |
| **연결 설정** | <2s | <10s | 연결 및 인증 시간 |
| **하트비트 빈도** | 30s | 60s | 하트비트 사이 시간 |
| **오프라인 저장소** | 7일 | 30일 | 오프라인 작동 지속 시간 |

---

**弘益人間 (홍익인간)** - 인류와 모든 생명체의 이익을 위하여
© 2025 WIA
MIT License
