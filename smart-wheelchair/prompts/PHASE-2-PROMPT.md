# WIA Smart Wheelchair - Phase 2: Sensor Interface

## 목표
휠체어에 장착되는 센서들의 표준 인터페이스를 정의합니다.

## 2.1 LiDAR 인터페이스

```typescript
interface LiDARInterface {
  // 설정
  config: {
    model: string;             // 'rplidar_a1', 'ydlidar_x4', etc.
    scanRate: number;          // Hz (5-20)
    angularResolution: number; // degrees
    maxRange: number;          // meters
    minRange: number;          // meters
  };

  // 데이터 스트림
  onScan(callback: (scan: LaserScan) => void): void;

  // 변환
  toPointCloud(scan: LaserScan): PointCloud;
}

interface LaserScan {
  header: {
    timestamp: number;
    frameId: string;           // 'lidar_frame'
  };
  angleMin: number;            // radians
  angleMax: number;            // radians
  angleIncrement: number;      // radians
  timeIncrement: number;       // seconds
  rangeMin: number;            // meters
  rangeMax: number;            // meters
  ranges: Float32Array;        // 거리 측정값
  intensities?: Float32Array;  // 반사 강도 (선택)
}
```

## 2.2 카메라 인터페이스

```typescript
interface CameraInterface {
  // 설정
  config: {
    type: 'rgb' | 'depth' | 'rgbd' | 'stereo';
    resolution: { width: number; height: number };
    fps: number;
    fov: number;               // degrees
  };

  // 스트림
  onFrame(callback: (frame: CameraFrame) => void): void;

  // 깊이 카메라 전용
  getDepthImage?(): DepthImage;
  getPointCloud?(): PointCloud;
}

interface CameraFrame {
  timestamp: number;
  format: 'rgb8' | 'bgr8' | 'mono8' | 'depth16';
  width: number;
  height: number;
  data: Uint8Array;
}

interface DepthImage {
  timestamp: number;
  width: number;
  height: number;
  data: Uint16Array;           // mm 단위
  minDepth: number;
  maxDepth: number;
}
```

## 2.3 IMU 인터페이스

```typescript
interface IMUInterface {
  config: {
    accelRange: number;        // ±g (2, 4, 8, 16)
    gyroRange: number;         // ±dps (250, 500, 1000, 2000)
    sampleRate: number;        // Hz
  };

  onData(callback: (data: IMUData) => void): void;
}

interface IMUData {
  timestamp: number;
  accel: Vector3D;             // m/s²
  gyro: Vector3D;              // rad/s
  mag?: Vector3D;              // μT
  orientation?: Quaternion;    // 센서 퓨전 결과
  temperature?: number;        // °C
}
```

## 2.4 센서 퓨전

```typescript
interface SensorFusion {
  // 입력 소스 등록
  registerSource(source: SensorSource): void;

  // 퓨전 알고리즘
  algorithm: 'ekf' | 'ukf' | 'particle_filter';

  // 출력
  getOdometry(): Odometry;
  getPose(): Pose;

  // 신뢰도
  getCovariance(): CovarianceMatrix;
}

interface Odometry {
  pose: {
    x: number;
    y: number;
    theta: number;             // radians
  };
  velocity: {
    linear: number;            // m/s
    angular: number;           // rad/s
  };
  covariance: number[];        // 6x6 matrix (flattened)
}
```

## 2.5 장애물 감지

```typescript
interface ObstacleDetector {
  // 설정
  config: {
    detectionRange: number;    // meters
    safetyMargin: number;      // meters
    updateRate: number;        // Hz
  };

  // 감지 결과
  onObstacle(callback: (obstacles: Obstacle[]) => void): void;

  // 영역별 상태
  getZoneStatus(): ZoneStatus;
}

interface Obstacle {
  id: number;
  type: 'static' | 'dynamic' | 'unknown';
  position: Vector2D;          // 휠체어 기준 상대 위치
  velocity?: Vector2D;         // 동적 장애물
  size: { width: number; depth: number };
  confidence: number;
}

interface ZoneStatus {
  front: 'clear' | 'warning' | 'danger';
  frontLeft: 'clear' | 'warning' | 'danger';
  frontRight: 'clear' | 'warning' | 'danger';
  left: 'clear' | 'warning' | 'danger';
  right: 'clear' | 'warning' | 'danger';
  rear: 'clear' | 'warning' | 'danger';
}
```

---

## 산출물

```
smart-wheelchair/
├── api/
│   ├── typescript/src/
│   │   └── sensors/
│   │       ├── lidar.ts
│   │       ├── camera.ts
│   │       ├── imu.ts
│   │       └── fusion.ts
│   └── rust/src/
│       └── sensors/
├── ros2_ws/src/
│   └── wia_wheelchair_perception/
│       ├── nodes/
│       │   ├── obstacle_detector.py
│       │   └── sensor_fusion.py
│       └── launch/
```

---

## 다음: Phase 3 (자율주행)
