# 제6장: 통신 프로토콜 (Phase 3)

## 수중 통신, 네트워킹 및 실시간 데이터 전송

---

## 6.1 음향 통신 프로토콜

### 수중 채널의 특성

해양에서의 음향 통신은 지상 무선 시스템과 비교하여 고유한 도전 과제를 제시합니다. 소리는 해수에서 약 1,500 m/s로 이동하며(무선의 300,000,000 m/s와 비교), 이는 상당한 지연을 발생시킵니다. 이 채널의 특징은 다음과 같습니다:

- **다중 경로 전파**: 소리가 수면, 해저, 수온약층에서 반사
- **도플러 효과**: 이동하는 차량이 주파수를 이동시킴
- **주변 소음**: 선박, 해양 생물, 날씨
- **가변 음속**: 온도 및 염분 구배가 음파 경로를 굴절

### WIA 음향 프로토콜 스택

```
┌────────────────────────────────────────────┐
│  애플리케이션 계층 (WIA 메시지)             │
├────────────────────────────────────────────┤
│  전송 계층 (신뢰성/비신뢰성)                │
├────────────────────────────────────────────┤
│  네트워크 계층 (주소 지정, 라우팅)          │
├────────────────────────────────────────────┤
│  데이터 링크 계층 (프레이밍, ARQ)           │
├────────────────────────────────────────────┤
│  물리 계층 (음향 변조)                      │
└────────────────────────────────────────────┘
```

### 프레임 형식

**음향 프레임 구조**:

| 필드 | 크기 | 설명 |
|------|------|------|
| 프리앰블 | 16 비트 | 동기화 패턴 |
| 동기화 | 8 비트 | 프레임 구분자 |
| 소스 주소 | 8 비트 | 송신자 ID (0-255) |
| 목적지 | 8 비트 | 수신자 ID (0-255, 255=브로드캐스트) |
| 시퀀스 | 16 비트 | 프레임 시퀀스 번호 |
| 유형 | 4 비트 | 프레임 유형 (DATA, ACK, NAK, PING) |
| 우선순위 | 4 비트 | 전송 우선순위 |
| 길이 | 16 비트 | 페이로드 길이 |
| 페이로드 | 가변 | 데이터 (최대 1024 바이트) |
| CRC-32 | 32 비트 | 오류 감지 |

### 변조 방식

| 방식 | 데이터 속도 | 범위 | 견고성 |
|------|------------|------|--------|
| FSK | 100-1,000 bps | 10+ km | 높음 |
| PSK | 1-10 kbps | 1-5 km | 중간 |
| QAM | 10-100 kbps | 0.5-2 km | 낮음 |
| OFDM | 10-50 kbps | 1-5 km | 중-고 |

### 음속과 통신 지연

심해 환경에서 음향 통신의 지연 시간은 상당합니다. 예를 들어, 챌린저 해연(약 11,000m)까지의 왕복 통신 시간은 약 14.7초입니다:

```python
# 음향 통신 지연 계산
def calculate_acoustic_delay(depth_meters: float, sound_velocity: float = 1500) -> float:
    """
    왕복 음향 통신 지연 계산

    Args:
        depth_meters: 깊이 (미터)
        sound_velocity: 음속 (m/s), 기본값 1500 m/s

    Returns:
        왕복 지연 시간 (초)
    """
    one_way_time = depth_meters / sound_velocity
    round_trip_time = 2 * one_way_time
    return round_trip_time

# 예시: 마리아나 해구
challenger_deep = 10994  # 미터
delay = calculate_acoustic_delay(challenger_deep)
print(f"챌린저 해연 왕복 지연: {delay:.2f}초")  # 약 14.66초
```

### 신뢰성 있는 전송

**Stop-and-Wait ARQ**:
```
송신자                    수신자
  |                          |
  |-------- DATA[0] -------->|
  |                          | 처리
  |<-------- ACK[0] ---------|
  |                          |
  |-------- DATA[1] -------->|
  |                          | 처리
  |<-------- ACK[1] ---------|
```

**타임아웃 및 재시도**:
```python
TIMEOUT_BASE = 2 * (range_meters / 1500) + processing_time
MAX_RETRIES = 3
BACKOFF_FACTOR = 1.5

for attempt in range(MAX_RETRIES):
    send_frame(data)
    timeout = TIMEOUT_BASE * (BACKOFF_FACTOR ** attempt)
    if wait_for_ack(timeout):
        return SUCCESS
    log_retry(attempt)
return FAILURE
```

### 한국의 수중 음향 통신 연구

한국해양과학기술원(KIOST)과 국내 연구 기관들은 수중 음향 통신 분야에서 선도적인 연구를 수행하고 있습니다:

| 기관 | 연구 분야 | 주요 성과 |
|------|----------|----------|
| KIOST | OFDM 기반 수중 통신 | 10 kbps 달성 (5km) |
| KAIST | 수중 음향 네트워크 | 다중 노드 협력 통신 |
| 서울대 해양연구소 | 도플러 보상 알고리즘 | 고속 AUV 통신 |
| 한국전자통신연구원 | 저전력 음향 모뎀 | 장기 해저 관측 지원 |

---

## 6.2 광섬유 테더 사양

### 테더 아키텍처

고대역폭이 필요한 ROV 운용의 경우, 광섬유 테더가 기가비트 연결을 제공합니다:

**단일 모드 광섬유 사양**:

| 매개변수 | 값 |
|----------|-----|
| 광섬유 유형 | G.652D 단일 모드 |
| 코어 직경 | 9 μm |
| 클래딩 직경 | 125 μm |
| 파장 | 1310 nm, 1550 nm |
| 감쇠 | 0.35 dB/km @ 1310nm |
| 대역폭 | 10+ Gbps |
| 최대 길이 | 10 km (실용적) |

### 테더 데이터 채널

| 채널 | 대역폭 | 방향 | 내용 |
|------|--------|------|------|
| 비디오 1 | 50 Mbps | 상향 | HD 메인 카메라 |
| 비디오 2 | 25 Mbps | 상향 | HD 보조 카메라 |
| 원격측정 | 10 Mbps | 상향 | 센서, 항법 |
| 제어 | 1 Mbps | 하향 | 명령, 웨이포인트 |
| 음성 | 64 kbps | 양방향 | 조종사 통신 |
| 비상 | 10 kbps | 양방향 | 우선순위 채널 |

### 물리 계층 프로토콜

**프레임 구조 (이더넷 기반)**:
```
┌──────────────────────────────────────────────────┐
│ 프리앰블 │ SFD │ 목적지 │ 소스 │ 유형 │ 데이터 │ FCS │
│  7 바이트 │ 1B  │  6B   │  6B │  2B  │ 가변 │  4B │
└──────────────────────────────────────────────────┘
```

**서비스 품질(QoS) 클래스**:

| 클래스 | 우선순위 | 예시 |
|--------|----------|------|
| CRITICAL | 7 | 비상 중단 |
| HIGH | 5-6 | 제어 명령 |
| NORMAL | 3-4 | 원격측정, 비디오 |
| LOW | 1-2 | 대량 데이터 전송 |
| BACKGROUND | 0 | 긴급하지 않은 로그 |

### 테더 관리 시스템(TMS)

심해 ROV 운용에서 테더 관리 시스템은 매우 중요합니다:

```python
class TetherManagementSystem:
    """
    ROV 테더 관리 시스템 제어
    """
    def __init__(self, max_length: float = 6000):
        self.max_length = max_length  # 미터
        self.deployed_length = 0
        self.tension = 0
        self.payout_rate = 0

    def calculate_catenary(self, depth: float, horizontal_offset: float) -> float:
        """
        케이블 현수선 형상 계산

        Returns:
            필요한 케이블 길이 (미터)
        """
        import math
        # 간단화된 현수선 공식
        straight_line = math.sqrt(depth**2 + horizontal_offset**2)
        catenary_factor = 1.15  # 일반적인 여유 계수
        return straight_line * catenary_factor

    def monitor_tension(self) -> dict:
        """
        케이블 장력 모니터링 및 알림
        """
        status = {
            'tension': self.tension,
            'status': 'NORMAL',
            'action': None
        }

        if self.tension > 5000:  # kg
            status['status'] = 'CRITICAL'
            status['action'] = 'EMERGENCY_SURFACE'
        elif self.tension > 3000:
            status['status'] = 'WARNING'
            status['action'] = 'REDUCE_DEPTH'

        return status
```

---

## 6.3 다중 차량 운용을 위한 메시 네트워킹

### 네트워크 토폴로지

다중 차량 운용에는 조정된 네트워킹이 필요합니다:

```
          수상 선박
               │
     ┌─────────┼─────────┐
     │         │         │
   AUV-1    AUV-2    AUV-3
     │         │         │
     └────┬────┴────┬────┘
          │         │
     해저 트랜스폰더 네트워크
```

### 주소 지정 체계

**WIA 네트워크 주소 형식**:
```
[네트워크 ID].[차량 유형].[인스턴스]

예시:
10.1.1    - 네트워크 10, ROV 유형, 인스턴스 1
10.2.5    - 네트워크 10, AUV 유형, 인스턴스 5
10.3.1    - 네트워크 10, 센서 노드, 인스턴스 1
10.0.255  - 네트워크 10, 브로드캐스트 주소
```

### 라우팅 프로토콜

**거리 벡터 라우팅**:
```python
class AcousticRouter:
    def __init__(self, node_id):
        self.node_id = node_id
        self.routing_table = {}  # 목적지 -> (다음 홉, 비용)

    def update_route(self, destination, next_hop, cost):
        current = self.routing_table.get(destination, (None, float('inf')))
        if cost < current[1]:
            self.routing_table[destination] = (next_hop, cost)
            self.broadcast_update()

    def get_next_hop(self, destination):
        if destination in self.routing_table:
            return self.routing_table[destination][0]
        return None  # 알려진 경로 없음
```

### 시분할 다중 접속 (TDMA)

조정된 다중 차량 운용을 위해:

```
|--슬롯 1--|--슬롯 2--|--슬롯 3--|--가드--|
|  AUV-1   |  AUV-2   |  AUV-3   |  빈칸  |
|  100ms   |  100ms   |  100ms   |  50ms  |

프레임 주기: 350ms (2.86 프레임/초)
```

### 다중 AUV 협력 탐사

```python
class MultiAUVCoordinator:
    """
    다중 AUV 협력 탐사 조정기
    """
    def __init__(self, vehicle_ids: list):
        self.vehicles = {vid: {'position': None, 'status': 'IDLE'} for vid in vehicle_ids}
        self.formation = None

    def set_formation(self, formation_type: str, spacing: float):
        """
        편대 설정: LINE, V-SHAPE, GRID, CIRCLE
        """
        formations = {
            'LINE': self._create_line_formation,
            'V-SHAPE': self._create_v_formation,
            'GRID': self._create_grid_formation,
            'CIRCLE': self._create_circle_formation
        }

        self.formation = formations[formation_type](spacing)
        return self.formation

    def coordinate_survey(self, survey_area: dict, overlap: float = 0.2):
        """
        탐사 영역을 차량 간에 분배

        Args:
            survey_area: 탐사 영역 경계
            overlap: 인접 차량 간 중복 비율
        """
        num_vehicles = len(self.vehicles)
        # 영역을 스와스로 분할
        swaths = self._divide_area(survey_area, num_vehicles, overlap)

        assignments = {}
        for vid, swath in zip(self.vehicles.keys(), swaths):
            assignments[vid] = {
                'swath': swath,
                'waypoints': self._generate_lawnmower_path(swath),
                'start_time': self._calculate_start_time(vid)
            }

        return assignments
```

---

## 6.4 데이터 압축 및 우선순위 지정

### 압축 알고리즘

| 데이터 유형 | 알고리즘 | 압축 비율 |
|------------|----------|----------|
| 원격측정 (JSON) | GZIP | 5-10배 |
| 원격측정 (바이너리) | LZ4 | 2-3배 |
| 수심측량 | 델타 + GZIP | 10-20배 |
| 비디오 | H.265 | 100-200배 |
| 이미지 | JPEG/WebP | 10-30배 |

### 우선순위 큐

```python
class PriorityMessageQueue:
    def __init__(self):
        self.queues = {
            'CRITICAL': deque(maxlen=10),
            'HIGH': deque(maxlen=50),
            'NORMAL': deque(maxlen=200),
            'LOW': deque(maxlen=500)
        }

    def enqueue(self, message, priority):
        self.queues[priority].append(message)

    def dequeue(self):
        for priority in ['CRITICAL', 'HIGH', 'NORMAL', 'LOW']:
            if self.queues[priority]:
                return self.queues[priority].popleft()
        return None
```

### 적응형 전송

```python
def select_transmission_mode(channel_quality, message_priority):
    if message_priority == 'CRITICAL':
        return {'modulation': 'FSK', 'fec': 'TURBO', 'rate': 'LOW'}

    if channel_quality > 0.9:
        return {'modulation': 'QAM16', 'fec': 'LDPC', 'rate': 'HIGH'}
    elif channel_quality > 0.7:
        return {'modulation': 'PSK', 'fec': 'CONV', 'rate': 'MEDIUM'}
    else:
        return {'modulation': 'FSK', 'fec': 'TURBO', 'rate': 'LOW'}
```

### 데이터 유형별 압축 전략

```python
class DataCompressor:
    """
    데이터 유형에 따른 최적 압축 전략
    """

    def compress_telemetry(self, data: dict) -> bytes:
        """
        원격측정 데이터 압축
        - JSON을 MessagePack으로 변환 후 LZ4 압축
        """
        import msgpack
        import lz4.frame

        packed = msgpack.packb(data, use_bin_type=True)
        compressed = lz4.frame.compress(packed)

        return compressed

    def compress_bathymetry(self, grid: np.ndarray) -> bytes:
        """
        수심측량 그리드 압축
        - 델타 인코딩 후 GZIP
        """
        import gzip

        # 델타 인코딩: 인접 셀 간 차이만 저장
        delta = np.diff(grid, axis=1, prepend=grid[:, 0:1])

        # 정수로 변환 (cm 정밀도)
        quantized = (delta * 100).astype(np.int16)

        compressed = gzip.compress(quantized.tobytes())

        return compressed

    def estimate_acoustic_transmission_time(self, data_bytes: int,
                                            modulation: str = 'PSK') -> float:
        """
        음향 전송 시간 추정
        """
        rates = {
            'FSK': 500,    # bps
            'PSK': 5000,   # bps
            'QAM': 50000,  # bps
            'OFDM': 25000  # bps
        }

        bits = data_bytes * 8
        transmission_time = bits / rates.get(modulation, 1000)

        return transmission_time
```

---

## 6.5 오류 정정 및 재시도 메커니즘

### 순방향 오류 정정 (FEC)

| 코드 | 오버헤드 | 정정 능력 |
|------|----------|----------|
| 컨볼루션 (1/2) | 100% | 중간 |
| 리드-솔로몬 | 10-30% | 버스트 오류 |
| LDPC | 10-50% | 섀넌 한계 근접 |
| 터보 | 33-100% | 우수 |

### 인터리빙

시간에 걸쳐 비트를 분산시켜 버스트 오류에 대응:

```
원본:        AAAABBBBCCCCDDDD
인터리빙됨:  ABCDABCDABCDABCD

버스트 오류 발생: A_CD__CD__CD__CD
디인터리빙:       A___B___C___D___
                  (블록당 단일 오류 - 정정 가능)
```

### 확인 응답 전략

**선택적 ACK**:
```json
{
  "type": "SACK",
  "received": [1, 2, 3, 5, 6, 8],
  "missing": [4, 7],
  "timestamp": "2025-01-15T14:30:00.000Z"
}
```

**부정 확인 응답 (NAK)**:
```json
{
  "type": "NAK",
  "sequence": 4,
  "reason": "CRC_FAILURE",
  "retryRequest": true
}
```

### 하이브리드 ARQ (HARQ)

```python
class HybridARQ:
    """
    하이브리드 자동 재전송 요청
    - FEC와 ARQ 결합으로 효율성 향상
    """

    def __init__(self):
        self.soft_buffer = {}  # 소프트 결합용 버퍼

    def receive_packet(self, packet: bytes, sequence: int) -> tuple:
        """
        패킷 수신 및 처리

        Returns:
            (success, decoded_data)
        """
        # FEC 디코딩 시도
        decoded, success = self.fec_decode(packet)

        if success:
            return True, decoded

        # 실패 시 소프트 버퍼에 저장
        if sequence in self.soft_buffer:
            # 이전 수신과 소프트 결합
            combined = self.soft_combine(self.soft_buffer[sequence], packet)
            decoded, success = self.fec_decode(combined)

            if success:
                del self.soft_buffer[sequence]
                return True, decoded
            else:
                self.soft_buffer[sequence] = combined
        else:
            self.soft_buffer[sequence] = packet

        # NAK 전송 요청
        return False, None

    def soft_combine(self, stored: bytes, new: bytes) -> bytes:
        """
        소프트 비트 결합 (체이스 결합)
        """
        # 로그 우도비 결합
        combined = []
        for s, n in zip(stored, new):
            combined.append((s + n) // 2)
        return bytes(combined)
```

---

## 6.6 수면-수중 게이트웨이

### 게이트웨이 아키텍처

```
┌─────────────────────────────────────────────────────┐
│                  수면 게이트웨이                     │
├─────────────────────────────────────────────────────┤
│  ┌─────────┐  ┌──────────┐  ┌─────────────────────┐│
│  │위성 링크 │  │ 선박 LAN │  │ 음향 트랜시버      ││
│  └────┬────┘  └────┬─────┘  └──────────┬──────────┘│
│       └────────────┴───────────────────┘           │
│                     │                               │
│              프로토콜 변환기                         │
│                     │                               │
│  ┌──────────────────┴───────────────────────────┐  │
│  │          WIA 메시지 라우터                    │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### 프로토콜 변환

**HTTP/REST에서 음향으로**:
```python
def translate_http_to_acoustic(http_request):
    # 필수 명령 추출
    command = {
        'type': http_request.method,
        'path': compress_path(http_request.path),
        'body': compress_json(http_request.json) if http_request.json else None
    }

    # 음향 전송을 위한 인코딩
    acoustic_frame = encode_compact(command)
    return acoustic_frame

def compress_path(path):
    # /api/v1/commands -> 0x01 (룩업 테이블)
    path_table = {'/api/v1/commands': 0x01, '/api/v1/telemetry': 0x02}
    return path_table.get(path, path)
```

### 저장 후 전달

음향 범위를 벗어난 AUV 미션의 경우:

```python
class StoreAndForward:
    def __init__(self):
        self.message_store = []

    def store_for_delivery(self, message, destination, expires):
        self.message_store.append({
            'message': message,
            'destination': destination,
            'expires': expires,
            'stored_at': datetime.utcnow()
        })

    def on_vehicle_contact(self, vehicle_id):
        # 차량이 범위 내에 들어오면 저장된 메시지 전달
        to_deliver = [m for m in self.message_store
                      if m['destination'] == vehicle_id
                      and m['expires'] > datetime.utcnow()]
        for msg in to_deliver:
            self.transmit(msg)
            self.message_store.remove(msg)
```

### 게이트웨이 프로토콜 변환기

```python
class GatewayProtocolConverter:
    """
    수면-수중 프로토콜 변환
    """

    # REST 경로 압축 테이블
    PATH_TABLE = {
        '/api/v1/telemetry': 0x01,
        '/api/v1/commands': 0x02,
        '/api/v1/samples': 0x03,
        '/api/v1/missions': 0x04,
        '/api/v1/bathymetry': 0x05,
    }

    # 역방향 테이블
    REVERSE_PATH_TABLE = {v: k for k, v in PATH_TABLE.items()}

    def http_to_acoustic(self, request: dict) -> bytes:
        """
        HTTP 요청을 음향 프레임으로 변환
        """
        # 메서드 압축: GET=0, POST=1, PUT=2, DELETE=3
        method_map = {'GET': 0, 'POST': 1, 'PUT': 2, 'DELETE': 3}

        compact = {
            'm': method_map.get(request['method'], 0),
            'p': self.PATH_TABLE.get(request['path'], request['path']),
            'd': self._compress_body(request.get('body'))
        }

        return msgpack.packb(compact)

    def acoustic_to_http(self, frame: bytes) -> dict:
        """
        음향 프레임을 HTTP 응답으로 변환
        """
        compact = msgpack.unpackb(frame)

        method_map = {0: 'GET', 1: 'POST', 2: 'PUT', 3: 'DELETE'}

        return {
            'method': method_map.get(compact['m'], 'GET'),
            'path': self.REVERSE_PATH_TABLE.get(compact['p'], compact['p']),
            'body': self._decompress_body(compact.get('d'))
        }
```

---

## 6.7 비상 통신 절차

### 비상 메시지 유형

| 유형 | 코드 | 설명 |
|------|------|------|
| ABORT | 0x01 | 즉시 부상 |
| EMERGENCY_SURFACE | 0x02 | 제어된 상승 |
| LOST_COMMUNICATION | 0x03 | 통신 실패 |
| POWER_CRITICAL | 0x04 | 저전력 |
| ENTANGLEMENT | 0x05 | 물리적 얽힘 |
| FLOODING | 0x06 | 침수 |
| FIRE | 0x07 | 선상 화재 |

### 비상 프레임

```
┌────────────────────────────────────────────────┐
│ 비상 프레임 (최소 16 바이트)                    │
├────────────────────────────────────────────────┤
│ 매직: 0xEMER (4 바이트)                        │
│ 소스 ID (2 바이트)                             │
│ 비상 유형 (1 바이트)                           │
│ 심각도 (1 바이트): 1-5                         │
│ 위치 (8 바이트): 위도/경도/깊이 압축           │
│ CRC-16 (2 바이트)                              │
└────────────────────────────────────────────────┘
```

### 비상 대응 프로토콜

```python
def handle_emergency(emergency_frame):
    # 1. 즉시 확인 응답
    send_ack(emergency_frame.source, priority='CRITICAL')

    # 2. 모든 운영자에게 경보
    broadcast_alert(emergency_frame)

    # 3. 타임스탬프와 함께 로그
    log_emergency(emergency_frame)

    # 4. 유형에 따른 대응 시작
    if emergency_frame.type == ABORT:
        initiate_abort_sequence(emergency_frame.source)
    elif emergency_frame.type == LOST_COMMUNICATION:
        start_recovery_protocol(emergency_frame.source)
```

### 자동 복구 동작

| 조건 | 트리거 | 대응 |
|------|--------|------|
| 통신 두절 | 5분간 연락 없음 | 마지막 알려진 위치로 부상 |
| 저전력 | 배터리 <10% | 미션 중단, 부상 |
| 깊이 초과 | >최대 깊이 | 비상 상승 |
| 얽힘 | 2분간 움직임 없음 | 경보, 무게추 투하 |

### 비상 통신 시스템 구현

```python
class EmergencyCommSystem:
    """
    비상 통신 시스템
    - 최소 오버헤드
    - 최대 신뢰성
    """

    EMERGENCY_MAGIC = bytes([0x45, 0x4D, 0x45, 0x52])  # 'EMER'

    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id
        self.emergency_active = False

    def create_emergency_frame(self, emergency_type: int, severity: int,
                                position: dict) -> bytes:
        """
        최소화된 비상 프레임 생성 (16바이트)
        """
        import struct

        # 위치 압축 (8바이트)
        lat_compressed = int((position['latitude'] + 90) * 1e6)
        lon_compressed = int((position['longitude'] + 180) * 1e6)
        depth_compressed = int(position['depth'] * 10)

        frame = struct.pack(
            '!4sHBBIIH',
            self.EMERGENCY_MAGIC,       # 4 bytes
            self.vehicle_id,            # 2 bytes
            emergency_type,             # 1 byte
            severity,                   # 1 byte
            lat_compressed,             # 4 bytes (위도 일부)
            lon_compressed,             # 4 bytes (경도 일부)
            depth_compressed            # 2 bytes
        )

        # CRC-16 추가
        crc = self.calculate_crc16(frame)
        frame += struct.pack('!H', crc)

        return frame

    def send_emergency_beacon(self, emergency_type: int, position: dict):
        """
        비상 비콘 반복 전송
        """
        frame = self.create_emergency_frame(
            emergency_type,
            severity=5,
            position=position
        )

        self.emergency_active = True

        # 고신뢰성 전송: FSK, 저속, 반복
        while self.emergency_active:
            for _ in range(3):  # 3회 반복 전송
                self.acoustic_send(frame, modulation='FSK', power='MAX')
                time.sleep(0.5)
            time.sleep(5)  # 5초 대기 후 반복

    def handle_emergency_received(self, frame: bytes):
        """
        비상 프레임 수신 처리
        """
        # 파싱
        if frame[:4] != self.EMERGENCY_MAGIC:
            return None

        data = struct.unpack('!4sHBBIIHH', frame)

        emergency = {
            'vehicle_id': data[1],
            'type': data[2],
            'severity': data[3],
            'position': {
                'latitude': (data[4] / 1e6) - 90,
                'longitude': (data[5] / 1e6) - 180,
                'depth': data[6] / 10
            },
            'crc': data[7]
        }

        # 즉시 ACK 전송
        self.send_emergency_ack(emergency['vehicle_id'])

        # 모든 시스템에 알림
        self.broadcast_to_all_systems(emergency)

        return emergency
```

---

## 장 요약

WIA 심해 탐사 표준의 Phase 3는 도전적인 수중 환경을 위한 견고한 통신 프로토콜을 정의합니다. 음향 프레임 형식에서 광섬유 QoS까지, 표준은 심해 운용에서 마주치는 전체 범위의 통신 시나리오를 다룹니다.

다중 차량 네트워킹은 조정된 운용을 가능하게 하고, 우선순위 큐잉은 중요한 메시지가 먼저 전달되도록 보장합니다. 포괄적인 오류 정정 및 재시도 메커니즘은 손실이 많은 음향 채널에서 신뢰성을 최대화합니다.

비상 절차는 안전에 중요한 통신이 항상 전달되도록 보장하며, 자동 복구 동작은 통신 범위를 벗어난 차량에 대한 추가적인 보호를 제공합니다.

---

## 핵심 요점

1. **음향 통신은 지상 무선보다 10,000배 느림**
2. **광섬유 테더는 기가비트 대역폭 제공**하지만 이동성 제한
3. **다중 차량 네트워크는 TDMA 사용**으로 조정된 접근
4. **우선순위 큐잉이 중요한 메시지** 우선 전송 보장
5. **비상 프레임은 최소 오버헤드 사용**으로 신뢰성 확보

---

## 복습 질문

1. 챌린저 해연(11,000m)까지의 음향 통신 대략적인 지연 시간은 얼마입니까?
2. 수중 음향 통신을 위한 FSK와 QAM 변조를 비교하세요.
3. 음향 채널을 공유하는 5개의 AUV를 위한 TDMA 스케줄을 설계하세요.
4. 노이즈가 많은 채널에 어떤 오류 정정 코드를 선택하시겠습니까?
5. 통신 두절 시나리오의 자동 복구 동작을 설명하세요.

---

## 한국의 심해 통신 인프라

한국은 동해, 남해, 서해를 아우르는 해양 통신 인프라를 구축하고 있습니다:

| 시스템 | 위치 | 기능 |
|--------|------|------|
| 울릉분지 관측망 | 동해 | 심층수 모니터링, AUV 지원 |
| 이어도 해양과학기지 | 남해 | 기상/해양 데이터 중계 |
| 제주 해저 케이블 | 제주-육지 | 광섬유 백본 |
| SMART 케이블 | 태평양 연결 | 해저 지진 조기 경보 |

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
