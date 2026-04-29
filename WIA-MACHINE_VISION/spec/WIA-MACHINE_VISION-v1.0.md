# WIA-MACHINE_VISION v1.0 Specification

**World Industry Association - Machine Vision Standard**

**Status:** Draft
**Version:** 1.0.0
**Date:** 2025-01-12
**Authors:** WIA Technical Committee on Industrial Automation
**License:** MIT

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope and Applications](#2-scope-and-applications)
3. [Machine Vision Architecture](#3-machine-vision-architecture)
4. [Image Acquisition Standards](#4-image-acquisition-standards)
5. [Object Detection and Classification](#5-object-detection-and-classification)
6. [Industrial Inspection Systems](#6-industrial-inspection-systems)
7. [3D Vision and Depth Sensing](#7-3d-vision-and-depth-sensing)
8. [Quality Control and Defect Detection](#8-quality-control-and-defect-detection)
9. [Real-time Processing](#9-real-time-processing)
10. [API Specifications](#10-api-specifications)
11. [Data Formats and Protocols](#11-data-formats-and-protocols)
12. [Integration Standards](#12-integration-standards)
13. [Security and Privacy](#13-security-and-privacy)
14. [Best Practices](#14-best-practices)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

The WIA-MACHINE_VISION standard provides a comprehensive framework for implementing machine vision systems in industrial, commercial, and research environments. This standard addresses the complete machine vision pipeline from image acquisition through analysis, decision-making, and integration with automation systems.

Machine vision has become a critical technology across industries, enabling automated inspection, quality control, robotic guidance, and intelligent monitoring. This standard establishes interoperable interfaces, data formats, and best practices that enable organizations to deploy machine vision solutions that work seamlessly across vendors and platforms.

### 1.2 Philosophy: 弘益人間 (Benefit All Humanity)

The WIA-MACHINE_VISION standard embodies the principle of 弘益人間 (Hongik Ingan) - "Benefit All Humanity." By establishing open standards for machine vision, we democratize access to advanced visual intelligence technologies. Small manufacturers can implement the same quality inspection capabilities as large enterprises. Researchers can share annotated datasets using common formats. Innovations in computer vision become accessible to all.

### 1.3 Key Features

- **Multi-modal Imaging:** Support for 2D, 3D, hyperspectral, thermal, and X-ray imaging
- **Real-time Performance:** Sub-10ms latency for high-speed production lines
- **99.9%+ Accuracy:** Micron-level precision for critical inspection tasks
- **AI-Powered:** Integration with YOLO, transformers, and custom deep learning models
- **Industry Standards:** Compliance with GigE Vision, USB3 Vision, GenICam, CoaXPress
- **Edge Computing:** Distributed processing for low-latency applications
- **Open APIs:** RESTful and gRPC interfaces for seamless integration
- **Multi-vendor Support:** Interoperable with leading camera and vision system vendors

### 1.4 Target Applications

- Manufacturing quality inspection and defect detection
- Robotic guidance and pick-and-place operations
- Automated optical inspection (AOI) for electronics
- Food and beverage sorting and grading
- Pharmaceutical tablet inspection
- Automotive assembly verification
- Semiconductor wafer inspection
- 3D measurement and metrology
- Barcode, QR code, and OCR reading
- Medical image analysis

---

## 2. Scope and Applications

### 2.1 Industrial Inspection

Industrial inspection represents the largest application domain for machine vision systems. Manufacturing facilities use vision systems to inspect products at various stages of production, detecting defects, verifying assembly correctness, and measuring critical dimensions.

**Key Requirements:**
- Inspection speeds up to 1000 parts per minute
- Defect detection with 99.9%+ accuracy
- Micron-level dimensional measurement
- Real-time feedback to production line
- Integration with manufacturing execution systems (MES)

**Common Defect Types:**
- Surface defects (scratches, dents, contamination)
- Dimensional deviations (out-of-spec measurements)
- Assembly errors (missing components, incorrect placement)
- Color variations (coating defects, inconsistent finishes)
- Structural defects (cracks, voids, porosity)

### 2.2 Robotic Guidance

Machine vision enables robots to locate, identify, and manipulate objects with precision. Vision-guided robotics has revolutionized manufacturing automation, enabling flexible production systems that can handle part variations without reprogramming.

**Capabilities:**
- 6-DOF pose estimation (<1mm accuracy)
- Bin picking with collision avoidance
- Dynamic object tracking (moving conveyor belts)
- Multi-object recognition and prioritization
- Adaptive grasp planning

### 2.3 Optical Character Recognition (OCR)

OCR systems extract text from images for traceability, inventory management, and automated data entry. Modern deep learning-based OCR achieves 99%+ accuracy on machine-printed text and 95%+ on handwritten text.

**Applications:**
- Serial number reading and verification
- Expiration date checking
- Label verification
- Document digitization
- License plate recognition

### 2.4 3D Measurement and Metrology

3D vision systems provide precise dimensional measurements for quality control and reverse engineering applications. Technologies include laser triangulation, structured light, time-of-flight, and stereo vision.

**Measurement Capabilities:**
- Dimensional accuracy: 1-50 microns (depending on technology)
- Measurement speed: up to 1 million points per second
- Object size range: 1mm to 10 meters
- Applications: flatness, roundness, volume, surface profile

### 2.5 Medical Imaging

Machine vision enhances medical diagnostics through automated analysis of X-rays, CT scans, MRIs, and microscopy images. AI-powered systems can detect anomalies, segment organs, and assist in diagnosis.

**Medical Applications:**
- Tumor detection and classification
- Retinal disease screening
- Pathology slide analysis
- Surgical guidance
- Medication verification

---

## 3. Machine Vision Architecture

### 3.1 System Components

A complete machine vision system comprises multiple interconnected components:

```
┌─────────────────────────────────────────────────────────────┐
│                    Machine Vision System                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌────────────┐      ┌──────────────┐      ┌──────────────┐│
│  │  Lighting  │─────▶│   Camera     │─────▶│ Frame        ││
│  │  System    │      │  (2D/3D)     │      │ Grabber      ││
│  └────────────┘      └──────────────┘      └──────┬───────┘│
│                                                     │         │
│                                                     ▼         │
│  ┌────────────┐      ┌──────────────┐      ┌──────────────┐│
│  │  Control   │◀─────│  Processing  │◀─────│ Calibration  ││
│  │  System    │      │    Engine    │      │   Module     ││
│  └────────────┘      └──────────────┘      └──────────────┘│
│         │                    │                               │
│         ▼                    ▼                               │
│  ┌────────────┐      ┌──────────────┐                      │
│  │  Actuators │      │   AI Models  │                      │
│  │  & I/O     │      │   (YOLO/CNN) │                      │
│  └────────────┘      └──────────────┘                      │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Lighting Systems

Proper illumination is critical for reliable machine vision performance. Lighting techniques include:

**Front Lighting:** General-purpose illumination, good for surface inspection
**Back Lighting:** Silhouette imaging, ideal for edge detection and measurement
**Dark Field:** Low-angle illumination highlighting surface defects
**Structured Light:** Projected patterns for 3D reconstruction
**Coaxial Lighting:** On-axis illumination for reflective surfaces
**Dome Lighting:** Diffuse lighting eliminating shadows

**Light Sources:**
- LED: Energy-efficient, long-life, wavelength-specific
- Laser: High-intensity, coherent light for 3D scanning
- Xenon Flash: High-speed imaging of moving objects
- Infrared: Thermal imaging and see-through applications
- UV: Fluorescence and material identification

### 3.3 Camera Technologies

Modern machine vision employs diverse camera technologies optimized for specific applications:

**Area Scan Cameras:**
- Resolution: 0.3MP to 150MP
- Frame rates: 10 fps to 1000 fps
- Sensors: CCD, CMOS, sCMOS
- Interfaces: GigE, USB3, CoaXPress, Camera Link

**Line Scan Cameras:**
- Resolution: 512 to 16,384 pixels per line
- Line rates: up to 400 kHz
- Applications: web inspection, continuous materials
- Advantages: unlimited length inspection

**3D Cameras:**
- Stereo vision: Dual-camera depth perception
- Time-of-Flight (ToF): Direct distance measurement
- Structured light: Pattern projection for 3D reconstruction
- Laser profilers: Line-by-line 3D scanning

**Specialized Cameras:**
- Hyperspectral: Spectral analysis (400-1000nm)
- Thermal: Temperature measurement and hotspot detection
- X-ray: Internal structure inspection
- Ultraviolet: Fluorescence and coating inspection

### 3.4 Processing Architecture

Machine vision processing can be deployed across multiple architectural patterns:

**Centralized Processing:**
- High-performance server processes all camera streams
- Suitable for complex AI models requiring GPU acceleration
- Typical latency: 50-200ms

**Edge Processing:**
- Processing at or near the camera
- Low latency: 5-20ms
- Limited computational resources

**Hybrid Architecture:**
- Edge devices handle real-time decisions
- Cloud/server handles model training and complex analytics
- Optimal balance of performance and cost

---

## 4. Image Acquisition Standards

### 4.1 GigE Vision

GigE Vision is the most widely adopted interface standard for industrial machine vision, using standard Gigabit Ethernet infrastructure.

**Specifications:**
- Data rate: 1 Gbps (GigE), 10 Gbps (10GigE), 25/40/100 Gbps (future)
- Cable length: up to 100m with standard Cat5e/Cat6
- Multiple cameras per network
- Bandwidth: ~110 MB/s (GigE), ~1000 MB/s (10GigE)

**Advantages:**
- Uses commodity networking infrastructure
- Long cable lengths without repeaters
- Easy multi-camera integration
- Power over Ethernet (PoE) support

**WIA Implementation:**
```json
{
  "interface": "GigE Vision",
  "version": "2.1",
  "ip_configuration": {
    "mode": "static",
    "address": "192.168.1.100",
    "subnet": "255.255.255.0",
    "gateway": "192.168.1.1"
  },
  "packet_size": 9000,
  "inter_packet_delay": 0,
  "bandwidth_throttle": 100
}
```

### 4.2 USB3 Vision

USB3 Vision leverages USB 3.0/3.1 for plug-and-play connectivity with excellent performance.

**Specifications:**
- Data rate: 5 Gbps (USB 3.0), 10 Gbps (USB 3.1)
- Cable length: up to 5m (standard), 30m+ (active cables)
- Bandwidth: ~400 MB/s practical throughput
- Bus-powered cameras supported

**Advantages:**
- Plug-and-play, no configuration required
- Single cable for data and power
- Lower cost than other interfaces
- Integrated into most PCs

### 4.3 CoaXPress

CoaXPress (CXP) is a high-speed digital interface for demanding machine vision applications.

**Specifications:**
- Data rate: 6.25 Gbps per link (CXP-6), 12.5 Gbps (CXP-12)
- Multiple links: 1, 2, or 4 links per camera
- Cable length: up to 40m (standard coax)
- Uplink for camera control

**Applications:**
- High-resolution imaging (>25MP)
- High-speed imaging (>500 fps)
- Line scan cameras with high data rates

### 4.4 GenICam Standard

GenICam (Generic Interface for Cameras) provides a uniform software interface regardless of the physical transport layer (GigE, USB3, CoaXPress, Camera Link).

**Key Components:**
- GenApi: XML-based camera description
- SFNC: Standard Feature Naming Convention
- GenTL: Transport layer interface
- CLProtocol: Consistent protocol across interfaces

**WIA GenICam Profile:**
```xml
<RegisterDescription xmlns="http://www.genicam.org/GenApi/Version_1_1">
  <Integer Name="Width">
    <Value>1920</Value>
    <Min>64</Min>
    <Max>5472</Max>
    <Inc>8</Inc>
  </Integer>
  <Integer Name="Height">
    <Value>1080</Value>
    <Min>64</Min>
    <Max>3648</Max>
    <Inc>8</Inc>
  </Integer>
  <Float Name="ExposureTime">
    <Value>10000.0</Value>
    <Min>50.0</Min>
    <Max>1000000.0</Max>
    <Unit>us</Unit>
  </Float>
  <Enumeration Name="PixelFormat">
    <EnumEntry Name="Mono8"/>
    <EnumEntry Name="RGB8"/>
    <EnumEntry Name="BayerRG8"/>
  </Enumeration>
</RegisterDescription>
```

---

## 5. Object Detection and Classification

### 5.1 Deep Learning Models

Modern machine vision relies heavily on deep learning for object detection and classification. The WIA-MACHINE_VISION standard supports multiple model architectures.

**YOLO (You Only Look Once):**
- Real-time object detection (60+ fps)
- Single-stage detector with excellent speed/accuracy trade-off
- Versions: YOLOv5, YOLOv7, YOLOv8, YOLOv9
- Typical accuracy: 50-60 mAP (COCO dataset)

**Faster R-CNN:**
- Two-stage detector with higher accuracy
- Region proposal network + classification
- Slower but more accurate than YOLO
- Typical accuracy: 55-65 mAP

**Vision Transformers (ViT):**
- Attention-based architecture
- State-of-the-art accuracy on many tasks
- Higher computational requirements
- Excellent for fine-grained classification

**EfficientDet:**
- Optimized for accuracy and efficiency
- Compound scaling for model sizing
- Good balance for edge deployment

### 5.2 Model Training Pipeline

```python
# WIA-MACHINE_VISION Model Training Interface
from wia_machine_vision import ModelTrainer, Dataset

# Load annotated training data
dataset = Dataset.load('inspection_data.wiadataset')
dataset.split(train=0.8, validation=0.1, test=0.1)

# Configure model training
trainer = ModelTrainer(
    model_type='yolov8',
    input_size=(640, 640),
    num_classes=len(dataset.classes),
    augmentation={
        'flip_horizontal': True,
        'brightness': (-0.2, 0.2),
        'rotation': (-15, 15),
        'scale': (0.8, 1.2)
    }
)

# Train model
model = trainer.train(
    dataset=dataset,
    epochs=100,
    batch_size=16,
    learning_rate=0.001,
    early_stopping_patience=10
)

# Evaluate performance
metrics = model.evaluate(dataset.test)
print(f"Precision: {metrics.precision:.3f}")
print(f"Recall: {metrics.recall:.3f}")
print(f"mAP@0.5: {metrics.map_50:.3f}")

# Export for deployment
model.export('production_model.onnx', format='onnx')
```

### 5.3 Inference Pipeline

```python
# WIA-MACHINE_VISION Inference Interface
from wia_machine_vision import VisionSystem, Model

# Load trained model
model = Model.load('production_model.onnx')

# Initialize vision system
vision = VisionSystem(camera_id='CAM_001')
vision.configure(
    exposure_time=5000,  # microseconds
    gain=1.0,
    resolution=(1920, 1080)
)

# Real-time inference
while True:
    # Acquire frame
    frame = vision.capture()

    # Run inference
    detections = model.predict(frame)

    # Filter results
    valid_detections = [
        d for d in detections
        if d.confidence > 0.85 and d.class_id in [0, 1, 2]
    ]

    # Take action based on results
    if any(d.class_name == 'defect' for d in valid_detections):
        vision.trigger_reject()

    # Log results
    vision.log_inspection(frame_id, valid_detections)
```

### 5.4 Annotation Standards

The WIA-MACHINE_VISION standard defines a unified annotation format compatible with COCO, YOLO, and Pascal VOC formats.

```json
{
  "wia_version": "1.0",
  "dataset_info": {
    "name": "Manufacturing Defect Detection",
    "version": "2024.1",
    "date_created": "2024-01-15",
    "num_images": 10000,
    "num_annotations": 45678
  },
  "categories": [
    {"id": 0, "name": "good", "supercategory": "product"},
    {"id": 1, "name": "scratch", "supercategory": "defect"},
    {"id": 2, "name": "dent", "supercategory": "defect"},
    {"id": 3, "name": "contamination", "supercategory": "defect"}
  ],
  "images": [
    {
      "id": 1,
      "file_name": "IMG_0001.png",
      "width": 1920,
      "height": 1080,
      "date_captured": "2024-01-15T10:23:45Z",
      "camera_id": "CAM_001",
      "lighting_config": "front_led_white"
    }
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [100, 200, 50, 30],
      "area": 1500,
      "segmentation": [[100,200,150,200,150,230,100,230]],
      "confidence": 0.95,
      "attributes": {
        "severity": "minor",
        "location": "surface",
        "orientation": 45.0
      }
    }
  ]
}
```

---

## 6. Industrial Inspection Systems

### 6.1 Automated Optical Inspection (AOI)

AOI systems inspect printed circuit boards (PCBs) and electronic assemblies for defects in manufacturing.

**Inspection Criteria:**
- Component presence/absence
- Component orientation and polarity
- Solder joint quality
- Pad coverage and bridging
- Component dimensions and registration

**Performance Requirements:**
- Inspection speed: 30-120 cm²/second
- False call rate: <0.1%
- Detection rate: >99.9% for critical defects
- Resolution: 10-25 microns

**WIA AOI Configuration:**
```json
{
  "inspection_type": "PCB_AOI",
  "board_size": {"width": 300, "height": 250, "unit": "mm"},
  "inspection_zones": [
    {
      "zone_id": "TOP_SIDE",
      "camera_angle": 0,
      "lighting": "ring_white_led",
      "algorithms": ["component_check", "solder_quality", "ocr"]
    },
    {
      "zone_id": "BOTTOM_SIDE",
      "camera_angle": 180,
      "lighting": "coaxial_red",
      "algorithms": ["component_check", "solder_quality"]
    }
  ],
  "defect_library": [
    {
      "defect_type": "missing_component",
      "severity": "critical",
      "action": "reject"
    },
    {
      "defect_type": "insufficient_solder",
      "severity": "major",
      "action": "alert"
    },
    {
      "defect_type": "solder_balls",
      "severity": "minor",
      "action": "log"
    }
  ]
}
```

### 6.2 Surface Inspection

Surface inspection systems detect defects on continuous materials or discrete parts.

**Applications:**
- Metal sheet inspection (scratches, dents, stains)
- Glass inspection (chips, cracks, inclusions)
- Paper/film inspection (holes, tears, wrinkles)
- Coating inspection (thickness, uniformity, defects)

**Technologies:**
- Line scan cameras for continuous web inspection
- Area scan cameras for discrete part inspection
- Laser profilers for 3D surface topology
- Hyperspectral imaging for material composition

### 6.3 Dimensional Measurement

Vision-based metrology systems provide non-contact dimensional measurement.

**Measurement Types:**
- Linear dimensions (length, width, diameter)
- Geometric features (flatness, roundness, perpendicularity)
- Profile measurement (surface roughness, contours)
- Volume measurement (fill level, material volume)

**Accuracy Specifications:**
- 2D measurement: ±0.005mm (typical)
- 3D measurement: ±0.010mm to ±0.050mm (depending on range)
- Repeatability: ±0.002mm (typical)

---

## 7. 3D Vision and Depth Sensing

### 7.1 3D Imaging Technologies

**Stereo Vision:**
- Two calibrated cameras capture images from different viewpoints
- Disparity calculation determines depth
- Advantages: Passive, works with textured surfaces
- Limitations: Requires sufficient lighting and texture

**Structured Light:**
- Projects known pattern onto scene
- Pattern deformation encodes depth
- Advantages: High accuracy (0.01-0.1mm), works on textureless surfaces
- Limitations: Sensitive to ambient light, limited range

**Time-of-Flight (ToF):**
- Measures time for light to reflect from surface
- Direct depth measurement
- Advantages: Fast (30-60 fps), works in various conditions
- Limitations: Lower resolution, limited accuracy at long range

**Laser Triangulation:**
- Laser line projected onto surface
- Camera observes line displacement
- Advantages: Very high accuracy (1-50 microns)
- Limitations: Line-by-line scanning, sensitive to surface properties

**LiDAR (Light Detection and Ranging):**
- Rotating or scanning laser measures distance
- Creates 3D point clouds
- Advantages: Long range (100+ meters), outdoor capable
- Applications: Autonomous vehicles, large-scale 3D mapping

### 7.2 Point Cloud Processing

```python
# WIA-MACHINE_VISION 3D Point Cloud Interface
from wia_machine_vision import PointCloud, Sensor3D

# Capture 3D data
sensor = Sensor3D(type='structured_light', model='SL_3000')
point_cloud = sensor.capture()

# Point cloud properties
print(f"Points: {point_cloud.num_points}")
print(f"Density: {point_cloud.density:.2f} points/mm²")
print(f"Bounding box: {point_cloud.bounds}")

# Preprocessing
point_cloud.remove_outliers(method='statistical', k=20, std_dev=2.0)
point_cloud.downsample(voxel_size=0.5)  # mm
point_cloud.estimate_normals(k_neighbors=30)

# Feature extraction
features = point_cloud.extract_features([
    'surface_roughness',
    'flatness',
    'volume',
    'dimensions'
])

# Comparison to CAD model
cad_model = PointCloud.load('reference_part.stl')
deviation = point_cloud.compare_to_reference(
    reference=cad_model,
    alignment='icp',  # Iterative Closest Point
    tolerance=0.1  # mm
)

# Results
print(f"Mean deviation: {deviation.mean:.3f}mm")
print(f"Max deviation: {deviation.max:.3f}mm")
print(f"Pass/Fail: {'PASS' if deviation.max < 0.5 else 'FAIL'}")
```

### 7.3 3D Measurement Standards

**ISO 10360:** Acceptance test and reverification test for CMM
**VDI/VDE 2634:** Optical 3D measuring systems
**ASTM E2544:** Standard terminology for 3D imaging systems

---

## 8. Quality Control and Defect Detection

### 8.1 Statistical Process Control (SPC)

Machine vision systems integrate with SPC to monitor process stability and detect trends.

**Control Charts:**
- X-bar and R charts for dimensional measurements
- p-charts for defect rates
- CUSUM charts for detecting small shifts
- EWMA charts for process monitoring

**WIA SPC Integration:**
```python
from wia_machine_vision import SPCMonitor

spc = SPCMonitor(metric='diameter', target=25.0, tolerance=0.1)

# Continuous monitoring
for measurement in vision_system.measure_stream():
    result = spc.add_measurement(measurement.value)

    if result.out_of_control:
        alert(f"Process out of control: {result.rule_violated}")

    if result.trend_detected:
        warn(f"Trend detected: {result.trend_direction}")
```

### 8.2 Defect Classification

Machine vision systems classify defects by type, severity, and location.

**Defect Taxonomy:**
- **Surface defects:** Scratches, dents, stains, discoloration
- **Dimensional defects:** Oversized, undersized, out-of-tolerance
- **Assembly defects:** Missing parts, wrong parts, misaligned
- **Functional defects:** Electrical faults, operational failures

**Severity Levels:**
- **Critical:** Product cannot function, safety hazard
- **Major:** Significant functional or aesthetic impact
- **Minor:** Cosmetic issues with minimal impact
- **Negligible:** No practical impact

### 8.3 Acceptance Criteria

```json
{
  "inspection_criteria": {
    "dimensions": {
      "length": {"target": 100.0, "tolerance": 0.2, "unit": "mm"},
      "width": {"target": 50.0, "tolerance": 0.15, "unit": "mm"},
      "thickness": {"target": 5.0, "tolerance": 0.05, "unit": "mm"}
    },
    "surface_quality": {
      "max_defects": {
        "critical": 0,
        "major": 2,
        "minor": 5
      },
      "max_defect_size": {
        "scratch": {"length": 2.0, "width": 0.1, "unit": "mm"},
        "dent": {"diameter": 1.0, "depth": 0.05, "unit": "mm"}
      }
    },
    "color": {
      "reference": "RAL_9010",
      "tolerance": {"delta_e": 2.0}
    }
  },
  "sampling_plan": {
    "type": "AQL",
    "level": "II",
    "aql_critical": 0.0,
    "aql_major": 1.5,
    "aql_minor": 4.0
  }
}
```

---

## 9. Real-time Processing

### 9.1 Latency Requirements

Different applications have varying latency requirements:

- **High-speed sorting:** <5ms (conveyor belt at 3 m/s)
- **Robotic guidance:** <10ms (pick-and-place at 120 cycles/min)
- **Process control:** <50ms (feedback to actuators)
- **Quality monitoring:** <100ms (statistical analysis)
- **Batch inspection:** <1000ms (complex AI analysis)

### 9.2 Edge Computing Architecture

```yaml
# WIA-MACHINE_VISION Edge Deployment
edge_device:
  hardware:
    cpu: "ARM Cortex-A72 @ 1.5GHz (quad-core)"
    gpu: "NVIDIA Jetson TX2 / Intel Movidius"
    memory: "8GB RAM"
    storage: "64GB eMMC"

  inference_engine:
    framework: "TensorRT / OpenVINO"
    model_format: "ONNX"
    precision: "FP16 / INT8"
    batch_size: 1
    optimization: "latency"

  performance:
    fps: 60
    latency: "8ms (camera to decision)"
    power: "15W typical"
```

### 9.3 Distributed Processing

For multi-camera systems, processing can be distributed across multiple edge devices or centralized:

```python
# Distributed processing with WIA-MACHINE_VISION
from wia_machine_vision import DistributedSystem

# Configure multi-camera system
system = DistributedSystem()

# Add edge nodes
node1 = system.add_node('edge-01', cameras=['CAM_001', 'CAM_002'])
node2 = system.add_node('edge-02', cameras=['CAM_003', 'CAM_004'])

# Configure processing pipeline
system.configure_pipeline([
    {'stage': 'acquisition', 'location': 'edge'},
    {'stage': 'preprocessing', 'location': 'edge'},
    {'stage': 'inference', 'location': 'edge'},
    {'stage': 'tracking', 'location': 'central'},
    {'stage': 'analytics', 'location': 'cloud'}
])

# Deploy and monitor
system.deploy()
metrics = system.get_metrics()
```

---

## 10. API Specifications

### 10.1 RESTful API

The WIA-MACHINE_VISION RESTful API provides HTTP-based access to vision system functionality.

**Base URL:** `https://api.vision.example.com/v1`

**Authentication:**
```http
Authorization: Bearer {jwt_token}
X-API-Key: {api_key}
```

**Core Endpoints:**

#### Capture Image
```http
POST /cameras/{camera_id}/capture

Request:
{
  "exposure_time": 5000,
  "gain": 1.5,
  "format": "jpeg",
  "quality": 95
}

Response: 200 OK
{
  "image_id": "img_20240115_103045_001",
  "timestamp": "2024-01-15T10:30:45.123Z",
  "url": "https://storage.example.com/images/img_20240115_103045_001.jpg",
  "metadata": {
    "camera_id": "CAM_001",
    "resolution": {"width": 1920, "height": 1080},
    "exposure_time": 5000,
    "gain": 1.5
  }
}
```

#### Run Inspection
```http
POST /inspect

Request:
{
  "image_id": "img_20240115_103045_001",
  "model_id": "defect_detector_v3",
  "confidence_threshold": 0.85,
  "options": {
    "nms_threshold": 0.45,
    "max_detections": 100
  }
}

Response: 200 OK
{
  "inspection_id": "insp_20240115_103046_001",
  "result": "FAIL",
  "defects": [
    {
      "defect_id": "def_001",
      "class": "scratch",
      "confidence": 0.92,
      "bbox": {"x": 450, "y": 320, "width": 80, "height": 15},
      "severity": "major",
      "attributes": {
        "length_mm": 12.5,
        "width_mm": 0.3,
        "orientation_deg": 45.2
      }
    }
  ],
  "metrics": {
    "inference_time_ms": 23,
    "total_time_ms": 67
  }
}
```

#### Query Inspection History
```http
GET /inspections?camera_id=CAM_001&start_date=2024-01-15&end_date=2024-01-16

Response: 200 OK
{
  "total": 10543,
  "pass": 10234,
  "fail": 309,
  "pass_rate": 97.07,
  "inspections": [...]
}
```

### 10.2 gRPC API

For high-performance, low-latency applications, WIA-MACHINE_VISION provides gRPC interfaces.

```protobuf
// vision_service.proto
syntax = "proto3";

package wia.machine_vision.v1;

service VisionService {
  rpc CaptureImage(CaptureRequest) returns (ImageResponse);
  rpc InspectImage(InspectRequest) returns (InspectResponse);
  rpc StreamInspection(stream InspectRequest) returns (stream InspectResponse);
}

message CaptureRequest {
  string camera_id = 1;
  int32 exposure_time_us = 2;
  float gain = 3;
  ImageFormat format = 4;
}

message ImageResponse {
  string image_id = 1;
  int64 timestamp_ns = 2;
  bytes image_data = 3;
  ImageMetadata metadata = 4;
}

message InspectRequest {
  oneof input {
    string image_id = 1;
    bytes image_data = 2;
  }
  string model_id = 3;
  float confidence_threshold = 4;
  InspectOptions options = 5;
}

message InspectResponse {
  string inspection_id = 1;
  InspectionResult result = 2;
  repeated Defect defects = 3;
  InspectionMetrics metrics = 4;
}

message Defect {
  string defect_id = 1;
  string class_name = 2;
  float confidence = 3;
  BoundingBox bbox = 4;
  Severity severity = 5;
  map<string, string> attributes = 6;
}

enum Severity {
  SEVERITY_UNSPECIFIED = 0;
  CRITICAL = 1;
  MAJOR = 2;
  MINOR = 3;
  NEGLIGIBLE = 4;
}
```

### 10.3 WebSocket API

Real-time streaming interface for live inspection monitoring.

```javascript
// WebSocket connection
const ws = new WebSocket('wss://api.vision.example.com/v1/stream');

// Subscribe to camera stream
ws.send(JSON.stringify({
  action: 'subscribe',
  camera_id: 'CAM_001',
  include_images: true,
  include_results: true
}));

// Receive inspection results
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(`Inspection: ${data.inspection_id}`);
  console.log(`Result: ${data.result}`);
  console.log(`Defects: ${data.defects.length}`);
};
```

---

## 11. Data Formats and Protocols

### 11.1 Image Formats

**Supported Formats:**
- **JPEG:** Compressed RGB, suitable for archival
- **PNG:** Lossless compression, supports alpha channel
- **TIFF:** Uncompressed, multi-page, metadata-rich
- **RAW:** Unprocessed sensor data (Bayer, etc.)
- **HDF5:** Hierarchical data format for scientific imaging

### 11.2 Metadata Standards

Every image captured includes comprehensive metadata:

```json
{
  "wia_version": "1.0",
  "image_id": "img_20240115_103045_001",
  "timestamp": "2024-01-15T10:30:45.123456Z",
  "camera": {
    "id": "CAM_001",
    "manufacturer": "FLIR",
    "model": "BFS-U3-31S4M",
    "serial": "19021234",
    "firmware": "1.16.3.0"
  },
  "acquisition": {
    "resolution": {"width": 1920, "height": 1080},
    "pixel_format": "Mono8",
    "exposure_time_us": 5000,
    "gain_db": 3.0,
    "frame_rate": 60.0,
    "trigger_mode": "hardware",
    "timestamp_ns": 1705318245123456000
  },
  "optical": {
    "lens_model": "Fujinon HF16SA-1",
    "focal_length_mm": 16.0,
    "aperture": "f/2.8",
    "field_of_view_deg": {"horizontal": 45.2, "vertical": 32.8}
  },
  "lighting": {
    "type": "ring_led",
    "wavelength_nm": 625,
    "intensity_percent": 80,
    "pulse_width_us": 100
  },
  "calibration": {
    "calibrated": true,
    "calibration_date": "2024-01-10",
    "pixels_per_mm": 12.5,
    "distortion_corrected": true
  }
}
```

---

## 12. Integration Standards

### 12.1 PLC Integration

Machine vision systems integrate with PLCs (Programmable Logic Controllers) for industrial automation.

**Communication Protocols:**
- **Modbus TCP:** Simple, widely supported
- **EtherNet/IP:** Allen-Bradley standard
- **PROFINET:** Siemens standard
- **OPC UA:** Universal interoperability standard

**Digital I/O:**
- Trigger input: Start image acquisition
- Result outputs: Pass/Fail, defect type
- Status outputs: Ready, busy, error
- Control inputs: Mode select, recipe change

### 12.2 MES Integration

Integration with Manufacturing Execution Systems for traceability and quality management.

**Data Exchange:**
- Inspection results and defect details
- Product serial numbers and batch IDs
- Process parameters and recipes
- Statistical quality metrics

**WIA MES Interface:**
```xml
<InspectionReport xmlns="http://wia-official.org/machine-vision/1.0">
  <Header>
    <InspectionID>insp_20240115_103046_001</InspectionID>
    <Timestamp>2024-01-15T10:30:46.789Z</Timestamp>
    <Station>INSPECT_01</Station>
  </Header>
  <Product>
    <SerialNumber>SN_2024_001234</SerialNumber>
    <PartNumber>PN_12345</PartNumber>
    <BatchID>BATCH_2024_W02</BatchID>
  </Product>
  <Result>FAIL</Result>
  <Defects>
    <Defect>
      <Type>scratch</Type>
      <Severity>major</Severity>
      <Location>top_surface</Location>
      <Size>12.5x0.3mm</Size>
    </Defect>
  </Defects>
  <Measurements>
    <Measurement name="length" value="100.05" unit="mm" tolerance="±0.2"/>
    <Measurement name="width" value="49.98" unit="mm" tolerance="±0.15"/>
  </Measurements>
</InspectionReport>
```

---

## 13. Security and Privacy

### 13.1 Authentication and Authorization

**Multi-factor Authentication:**
- API keys for machine-to-machine communication
- JWT tokens for user authentication
- OAuth 2.0 for third-party integration
- Certificate-based authentication for devices

**Role-Based Access Control (RBAC):**
- **Operator:** View live streams, acknowledge alerts
- **Engineer:** Configure cameras, adjust parameters
- **Quality Manager:** View reports, analyze trends
- **Administrator:** Full system access, user management

### 13.2 Data Encryption

- **At Rest:** AES-256 encryption for stored images and data
- **In Transit:** TLS 1.3 for all network communication
- **End-to-End:** Optional encryption for sensitive applications

### 13.3 Privacy Protection

For applications involving people (security, retail analytics):
- Anonymization: Face blurring, body pose only
- Retention policies: Automatic deletion after defined period
- Consent management: Opt-in/opt-out mechanisms
- Compliance: GDPR, CCPA, industry regulations

---

## 14. Best Practices

### 14.1 System Design

1. **Lighting First:** Invest in proper lighting to eliminate defects at the source
2. **Camera Selection:** Match camera resolution and frame rate to application requirements
3. **Lens Selection:** Choose appropriate focal length and aperture for field of view
4. **Processing Power:** Size compute resources for peak load plus 30% headroom
5. **Redundancy:** Critical systems should have backup cameras and processing

### 14.2 Model Development

1. **Data Collection:** Capture diverse, representative training data
2. **Annotation Quality:** Use multiple annotators, measure inter-rater agreement
3. **Class Balance:** Ensure sufficient examples of rare defect types
4. **Augmentation:** Use appropriate augmentation to improve generalization
5. **Validation:** Test on held-out data from different time periods and conditions

### 14.3 Deployment

1. **Calibration:** Regular camera calibration ensures measurement accuracy
2. **Monitoring:** Track key metrics (accuracy, throughput, uptime)
3. **Maintenance:** Clean lenses and sensors on regular schedule
4. **Updates:** Version control for models and configuration
5. **Documentation:** Maintain detailed system documentation and procedures

---

## 15. References

### 15.1 Standards

- **ISO 9001:** Quality management systems
- **ISO 10360:** Geometrical product specifications for CMM
- **ISO 13485:** Medical devices quality management
- **IEC 61000:** Electromagnetic compatibility
- **ASTM E2544:** 3D imaging systems terminology
- **VDI/VDE 2634:** Optical 3D measuring systems

### 15.2 Industry Organizations

- **AIA (Advancing Vision + Imaging):** Machine vision industry association
- **EMVA (European Machine Vision Association):** GenICam, EMVA 1288
- **JIIA (Japan Industrial Imaging Association):** GigE Vision, USB3 Vision
- **VDMA (German Engineering Federation):** OPC Vision standard

### 15.3 Technical Resources

- GenICam Standard: https://www.emva.org/standards-technology/genicam/
- GigE Vision: https://www.visiononline.org/vision-standards-details.cfm?id=120
- USB3 Vision: https://www.visiononline.org/vision-standards-details.cfm?id=203
- CoaXPress: https://www.coaxpress.org/

---

## Appendix A: Glossary

**AOI:** Automated Optical Inspection
**FOV:** Field of View
**FPS:** Frames Per Second
**GenICam:** Generic Interface for Cameras
**GigE:** Gigabit Ethernet
**mAP:** Mean Average Precision
**NMS:** Non-Maximum Suppression
**OCR:** Optical Character Recognition
**PCB:** Printed Circuit Board
**SPC:** Statistical Process Control
**ToF:** Time-of-Flight
**YOLO:** You Only Look Once

---

## Appendix B: Sample Implementations

Code examples and reference implementations available at:
https://github.com/WIA-Official/machine-vision-examples

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
License: MIT
