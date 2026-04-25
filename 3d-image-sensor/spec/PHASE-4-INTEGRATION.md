# WIA-SEMI-013 Phase 4: Integration Specification

**Version:** 1.0  
**Date:** 2025-01-15  
**Status:** Published  

## Overview

Phase 4 provides guidelines, reference implementations, and certification procedures for integrating 3D image sensors into complete systems.

## 1. Multi-Sensor Fusion

### Fusion Architectures

**Homogeneous Fusion** (Multiple same-technology sensors):
```
ToF-1 ──┐
        ├── Fusion ──> Enhanced Depth Map
ToF-2 ──┘
```

Benefits: Wider FOV, reduced noise, occlusion handling

**Heterogeneous Fusion** (Different technologies):
```
ToF ────┐
        ├── Fusion ──> Optimal Depth Map
Stereo ─┘
```

Benefits: Complementary strengths, extended range, improved accuracy

### Fusion Algorithms

**Weighted Average:**
```python
def fuse_depth_maps(depth_maps, confidence_maps):
    total_confidence = sum(confidence_maps)
    fused = sum(d * c for d, c in zip(depth_maps, confidence_maps))
    return fused / total_confidence
```

**Confidence-Based Selection:**
```python
def select_best_depth(depth_maps, confidence_maps):
    best_idx = np.argmax(confidence_maps, axis=0)
    return np.choose(best_idx, depth_maps)
```

**Kalman Filter Fusion:**
For temporal consistency and multi-sensor fusion:

```python
class DepthKalmanFilter:
    def __init__(self, process_noise, measurement_noise):
        self.Q = process_noise  # Process noise covariance
        self.R = measurement_noise  # Measurement noise covariance
        
    def update(self, measurement, confidence):
        # Prediction
        prediction = self.state
        P_predict = self.P + self.Q
        
        # Update with measurement
        K = P_predict / (P_predict + self.R / confidence)
        self.state = prediction + K * (measurement - prediction)
        self.P = (1 - K) * P_predict
        
        return self.state
```

## 2. SLAM Integration

### Visual-Inertial Odometry (VIO)

**Sensor Suite:**
- 3D camera (depth + RGB)
- IMU (6-axis: accel + gyro)
- Optional: GPS, magnetometer

**Pipeline:**
```
Depth Frames ──┐
RGB Frames ────┼──> Feature Tracking ──> Pose Estimation ──> Map Update
IMU Data ──────┘
```

**Reference Implementation:**
```cpp
class WIASLAM {
public:
    void onDepthFrame(const DepthFrame& frame) {
        // Extract features
        auto features = extractFeatures(frame);
        
        // Match with previous frame
        auto matches = matchFeatures(features, prev_features_);
        
        // Estimate camera motion
        auto pose = estimatePose(matches);
        
        // Fuse with IMU
        pose = fuseWithIMU(pose, imu_data_);
        
        // Update map
        updateMap(frame, pose);
        
        prev_features_ = features;
    }
    
private:
    Map3D map_;
    std::vector<Feature> prev_features_;
    IMUIntegrator imu_integrator_;
};
```

### Loop Closure Detection

Recognize previously visited locations for globally consistent maps:

```python
def detect_loop_closure(current_frame, frame_database):
    # Compute frame descriptor
    descriptor = compute_global_descriptor(current_frame)
    
    # Search similar frames
    candidates = frame_database.query_similar(descriptor, top_k=10)
    
    # Geometric verification
    for candidate in candidates:
        if verify_geometric_consistency(current_frame, candidate):
            return candidate
    
    return None
```

## 3. Application Templates

### Face Authentication

**System Requirements:**
- Technology: Structured light or dToF
- Accuracy: <1mm @ 0.3-0.6m
- Frame rate: 30+ fps
- Latency: <100ms total

**Implementation:**
```python
class FaceAuthenticator:
    def __init__(self, sensor, model):
        self.sensor = sensor
        self.face_model = model
        
    def enroll(self, user_id):
        """Enroll new user."""
        depth_maps = []
        for _ in range(5):  # Capture 5 views
            frame = self.sensor.capture_frame()
            depth_map = frame.get_depth_map()
            
            # Face detection
            face_region = self.detect_face(depth_map)
            if face_region is None:
                continue
                
            depth_maps.append(face_region)
        
        # Create 3D face template
        template = self.create_template(depth_maps)
        self.face_model.enroll(user_id, template)
        
    def authenticate(self):
        """Authenticate user."""
        frame = self.sensor.capture_frame()
        depth_map = frame.get_depth_map()
        
        # Detect and extract face
        face_region = self.detect_face(depth_map)
        if face_region is None:
            return False, None
            
        # Match against enrolled templates
        match_id, confidence = self.face_model.match(face_region)
        
        return confidence > 0.95, match_id
```

### Gesture Recognition

**System Requirements:**
- Technology: iToF or stereo
- Frame rate: 60+ fps
- Range: 0.5-3m
- Latency: <50ms

**Hand Skeleton Tracking:**
```python
class HandTracker:
    def __init__(self, sensor):
        self.sensor = sensor
        self.joint_detector = HandJointDetector()
        
    def track_hand(self, depth_frame):
        # Segment hand from background
        hand_mask = self.segment_hand(depth_frame)
        
        # Extract hand point cloud
        hand_cloud = depth_frame.to_point_cloud(mask=hand_mask)
        
        # Detect 21 hand joints
        joints = self.joint_detector.detect(hand_cloud)
        
        return HandPose(joints)
```

### Robotics Navigation

**Obstacle Detection:**
```python
class ObstacleDetector:
    def __init__(self, sensor, robot_width):
        self.sensor = sensor
        self.robot_width = robot_width
        
    def detect_obstacles(self):
        depth_frame = self.sensor.capture_frame()
        point_cloud = depth_frame.to_point_cloud()
        
        # Ground plane removal
        point_cloud = self.remove_ground(point_cloud)
        
        # Cluster obstacles
        obstacles = self.cluster_obstacles(point_cloud)
        
        # Filter by robot path
        path_obstacles = [
            obs for obs in obstacles
            if self.is_in_path(obs, self.robot_width)
        ]
        
        return path_obstacles
```

## 4. Testing and Certification

### Certification Levels

**Bronze Certification:**
- Depth accuracy: ±5mm @ 2m
- Resolution: 320x240 minimum
- Frame rate: 15 fps minimum
- Indoor operation
- Basic multi-path handling

**Silver Certification:**
- Depth accuracy: ±2mm @ 2m
- Resolution: 640x480 minimum
- Frame rate: 30 fps minimum
- Indoor + outdoor operation
- Advanced multi-path correction

**Gold Certification:**
- Depth accuracy: ±1mm @ 2m
- Resolution: 1280x960 minimum
- Frame rate: 60 fps minimum
- Full sunlight operation
- AI-based multi-path handling

### Test Procedures

**Accuracy Test:**
1. Mount sensor on stable tripod
2. Position calibration plate at 0.5m, 1m, 2m, 3m, 5m
3. Capture 100 frames at each distance
4. Measure mean error and standard deviation
5. Verify ≤ specified accuracy at all distances

**Repeatability Test:**
1. Static scene capture
2. Collect 1000 frames
3. Calculate per-pixel standard deviation
4. Verify 95% of pixels within precision spec

**Multi-Path Test:**
1. Corner retroreflector setup
2. Measure depth in corner region
3. Compare to ground truth
4. Verify error ≤ 2× normal accuracy spec

**Environmental Test:**
| Condition | Requirement |
|-----------|-------------|
| Temperature | -20°C to +60°C (Silver/Gold: -40°C to +85°C) |
| Humidity | 10% to 90% RH, non-condensing |
| Ambient light | 100,000 lux (Gold level) |
| Vibration | 10-2000 Hz, 2g acceleration |
| EMI | IEC 61000-4-3 (10 V/m) |

### Compliance Documentation

Certification requires:
1. Test report from approved lab
2. Calibration certificates
3. User manual with accuracy specifications
4. Software SDK with WIA-SEMI-013 compliance
5. Declaration of conformity

## 5. Best Practices

### System Design Guidelines

**Sensor Placement:**
- Minimize occlusions
- Consider field of view overlap for fusion
- Protect optics from contamination
- Thermal management for heat-generating sensors

**Mechanical Design:**
- Rigid mounting to maintain calibration
- Vibration isolation for mobile applications
- Weatherproofing for outdoor use
- Lens protection (AR coating, scratch resistance)

**Software Architecture:**
- Abstract sensor details behind common API
- Pipeline: Acquire → Filter → Process → Apply
- Asynchronous data flow for low latency
- Error handling and graceful degradation

### Performance Optimization

**Real-Time Processing:**
```cpp
// CPU optimization
void process_point_cloud(PointCloud& cloud) {
    // Multi-threading
    #pragma omp parallel for
    for (size_t i = 0; i < cloud.size(); ++i) {
        // SIMD operations
        __m256 point = _mm256_load_ps(&cloud.points[i]);
        __m256 filtered = apply_filter_simd(point);
        _mm256_store_ps(&cloud.points[i], filtered);
    }
}

// GPU acceleration
__global__ void filter_kernel(float3* points, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        points[idx] = apply_filter(points[idx]);
    }
}
```

## 6. Future Roadmap

**Version 1.1 (Q2 2026):**
- Event-based 3D sensors
- Neural radiance field integration
- Enhanced multi-sensor calibration

**Version 2.0 (2027):**
- Compressed neural representations
- 5G edge computing integration
- AR Cloud persistent maps

## 8. Integration Standards Alignment

### 8.1 Functional Safety and Robotics

When a 3D image sensor is integrated into a system with safety implications (autonomous mobile robot, collaborative robot, surgical robot, automotive ADAS), the integration framework conforms to:

| Domain | Reference | Role |
|--------|-----------|------|
| Industrial machinery | ISO 13849-1:2023 (Performance Levels) | Sensor failure-rate budget |
| Functional safety (electrical) | IEC 61508 series | Safety Integrity Level (SIL) classification |
| Industrial robots | ISO 10218-1:2025 / ISO 10218-2:2025 | Robot integration safety |
| Collaborative robots | ISO/TS 15066:2016 | Human-robot collaboration force/pressure limits |
| Service robots | ISO 13482:2014 | Personal-care robot safety |
| Automotive | ISO 26262 series | ASIL classification, fail-operational |
| Medical | IEC 60601-1:2005+AMD2:2020 | Basic safety, essential performance |
| Medical software | IEC 62304:2006+AMD1:2015 | Software lifecycle for medical devices |

### 8.2 Quality and Process

System integrators following WIA-SEMI-013 SHOULD adopt:

- **ISO 9001:2015** quality management for the integration process.
- **ISO/IEC 27001:2022** information security management for sensor data flows.
- **IEC 62443 series** for industrial automation cybersecurity, particularly IEC 62443-3-3 (system security requirements) and IEC 62443-4-1 (secure product development lifecycle).

### 8.3 Accessibility

When the integrated system has a human user interface, accessibility conforms to:

- **ISO/IEC 40500:2012** (W3C WCAG 2.0) Level AA, with WCAG 2.1 / 2.2 forward compatibility.
- **EN 301 549 v3.2.1** for the European public-sector procurement.
- **Section 508** of the US Rehabilitation Act for US federal procurement.

3D sensor visualization simulators are subject to WCAG 2.2 with particular attention to Success Criterion 1.4.11 (Non-text Contrast) and 2.5.7 (Dragging Movements).

### 8.4 Health and Ergonomics

For systems where users interact at close range (gesture interfaces, AR/VR headsets, human pose estimation):

- **ISO 9241-11:2018** Ergonomics — Usability definitions.
- **ISO 9241-210:2019** Human-centred design for interactive systems.
- **IEC 62471:2006** Photobiological safety, when illumination uses IR or visible flood emitters.
- **IEC 60825 series** Laser product safety for ToF sensors using class-1 or class-2 laser emitters.

### 8.5 Environmental and Lifecycle

- **ISO 14040:2006 / ISO 14044:2006** for life-cycle assessment of integrated camera systems.
- **IEC 62321 series** for restricted-substance testing (RoHS compliance).
- **ISO 14064-1:2018** for greenhouse-gas reporting at facility scale, when integrators publish climate data.

### 8.6 Interoperability with Existing Robotics Frameworks

Reference bindings are provided for:

- **ROS 2 (Humble or later)** with sensor_msgs/PointCloud2 publication.
- **OpenCV 4.x** with cv::rgbd module for depth processing.
- **PCL (Point Cloud Library) 1.13+** for 3D processing pipelines.
- **Open3D 0.18+** for higher-level point cloud operations.

These bindings convert WIA frame formats to/from each ecosystem's native representation while preserving conformance with §7 of Phase 1.

### 8.7 Conformance

A complete integrated system is conformant with Phase 4 when:

1. The sensor subsystem is conformant with Phases 1–3.
2. The integration documentation declares which §8.1 functional safety reference applies.
3. The integrator has performed and recorded a hazard analysis appropriate to the application domain.
4. Accessibility conformance level is declared if a human-operated UI is included.

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**  
**弘益人間 · Benefit All Humanity**
