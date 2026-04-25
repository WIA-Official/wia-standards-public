# WIA-AI-021 Vision AI Standard - PHASE 3 Specification

**Version:** 1.0
**Status:** Official Standard
**Date:** 2025-01-15
**Organization:** World Certification Industry Association (WIA)

---

## 弘益人間 (Benefit All Humanity)

Temporal vision for understanding motion and change.

---

## 1. Executive Summary

Phase 3 introduces video analysis capabilities, enabling systems to understand temporal information, track objects across frames, recognize actions, and process continuous visual streams in real-time.

---

## 2. Video Processing

### 2.1 Video Input Requirements

**Supported Formats:**
- MP4 (H.264, H.265)
- AVI
- MOV
- WebM

**Specifications:**
- Minimum FPS: 15
- Maximum FPS: 120
- Minimum Resolution: 640x480
- Maximum Resolution: 3840x2160 (4K)
- Maximum Duration: 60 minutes (single processing)

### 2.2 Frame Extraction

**Requirements:**
- Support uniform sampling
- Support adaptive sampling
- Maintain temporal order
- Extract keyframes

**API:**
```json
{
  "video_url": "https://example.com/video.mp4",
  "sampling_strategy": "uniform",
  "target_fps": 10,
  "extract_frames": true
}
```

---

## 3. Object Tracking

### 3.1 Single Object Tracking (SOT)

**Output:**
```json
{
  "tracking_results": {
    "object_id": "track_001",
    "class": "person",
    "frames": [
      {
        "frame_number": 0,
        "timestamp_ms": 0,
        "bounding_box": {"x_min": 120, "y_min": 80, "x_max": 220, "y_max": 320},
        "confidence": 0.96
      },
      {
        "frame_number": 1,
        "timestamp_ms": 33,
        "bounding_box": {"x_min": 125, "y_min": 82, "x_max": 225, "y_max": 322},
        "confidence": 0.95
      }
    ]
  }
}
```

**Performance:**
- Success rate ≥ 70% on VOT benchmark
- Precision ≥ 80%
- Processing speed ≥ 30 FPS

### 3.2 Multi-Object Tracking (MOT)

**Requirements:**
- Track ≥ 20 objects simultaneously
- Maintain identity across occlusions
- Handle object entering/leaving scene

**Output:**
```json
{
  "tracks": [
    {
      "track_id": 1,
      "class": "person",
      "first_frame": 0,
      "last_frame": 150,
      "trajectory": [...]
    },
    {
      "track_id": 2,
      "class": "vehicle",
      "first_frame": 10,
      "last_frame": 200,
      "trajectory": [...]
    }
  ],
  "metrics": {
    "mota": 0.75,
    "motp": 0.82,
    "idf1": 0.78
  }
}
```

**Performance:**
- MOTA ≥ 65% on MOT Challenge
- IDF1 ≥ 70%
- Processing speed ≥ 20 FPS

---

## 4. Action Recognition

### 4.1 Action Classification

**Supported Actions (minimum 20 classes):**
- Walking, running, jumping
- Sitting, standing, lying down
- Waving, pointing, clapping
- Eating, drinking
- Using phone, computer
- Sports activities (throwing, kicking, etc.)

**Output:**
```json
{
  "action": "running",
  "confidence": 0.89,
  "start_time_ms": 1000,
  "end_time_ms": 3500,
  "clip_info": {
    "start_frame": 30,
    "end_frame": 105,
    "duration_frames": 75
  }
}
```

**Performance:**
- Top-1 accuracy ≥ 70% on UCF-101
- Top-1 accuracy ≥ 60% on Kinetics-400

### 4.2 Temporal Action Detection

**Requirements:**
- Detect action start and end times
- Support untrimmed videos
- Handle multiple actions per video

---

## 5. Optical Flow

### 5.1 Dense Optical Flow

**Output:**
```json
{
  "flow_field": {
    "width": 640,
    "height": 480,
    "flow_x": "base64_encoded_array",
    "flow_y": "base64_encoded_array",
    "magnitude": "base64_encoded_array",
    "angle": "base64_encoded_array"
  },
  "average_motion": {
    "magnitude": 12.5,
    "direction_degrees": 45.3
  }
}
```

**Performance:**
- End-point error ≤ 5 pixels on Sintel
- Processing speed ≥ 15 FPS

---

## 6. Video Segmentation

### 6.1 Video Object Segmentation (VOS)

**Requirements:**
- Propagate segmentation across frames
- Support semi-supervised mode (first frame annotated)
- Support unsupervised mode (automatic)

**Output:**
```json
{
  "segmentation_sequence": {
    "object_id": 1,
    "frames": [
      {
        "frame_number": 0,
        "mask": "base64_encoded_mask",
        "confidence": 0.94
      }
    ]
  }
}
```

**Performance:**
- J&F Mean ≥ 75% on DAVIS
- Processing speed ≥ 5 FPS

---

## 7. Real-Time Video Analysis

### 7.1 Live Stream Processing

**Requirements:**
- Process live video streams
- Support RTSP, RTMP, HLS
- Maintain low latency (< 500ms)
- Handle stream interruptions gracefully

**API:**
```json
{
  "stream_url": "rtsp://camera.example.com/stream",
  "tasks": ["detection", "tracking"],
  "callback_url": "https://api.example.com/results",
  "frame_skip": 2
}
```

### 7.2 Edge Deployment

**Requirements:**
- Support NVIDIA Jetson platforms
- Support Intel NUC / Movidius
- Support mobile devices (iOS, Android)
- Optimize for power efficiency

---

## 8. Video Quality Assessment

### 8.1 Quality Metrics

**Automatic checks:**
- Resolution verification
- FPS consistency
- Lighting conditions
- Motion blur detection
- Compression artifacts

**Output:**
```json
{
  "quality_assessment": {
    "overall_score": 0.85,
    "resolution_ok": true,
    "fps_stable": true,
    "lighting_adequate": true,
    "motion_blur_level": "low",
    "compression_quality": "high",
    "recommendations": [
      "Consider increasing resolution for better accuracy"
    ]
  }
}
```

---

## 9. API Endpoints (Phase 3)

### 9.1 Video Upload

```
POST /api/v3/video/upload
```

**Request:**
```json
{
  "video_url": "https://example.com/video.mp4",
  "tasks": ["detection", "tracking", "action_recognition"],
  "notify_on_complete": "https://callback.example.com"
}
```

### 9.2 Object Tracking

```
POST /api/v3/track
```

**Request:**
```json
{
  "video_id": "vid_123456",
  "initial_bbox": {"x_min": 100, "y_min": 100, "x_max": 200, "y_max": 300},
  "algorithm": "deepsort"
}
```

### 9.3 Action Recognition

```
POST /api/v3/recognize_action
```

**Request:**
```json
{
  "video_id": "vid_123456",
  "clip_start_ms": 1000,
  "clip_end_ms": 5000,
  "top_k": 5
}
```

---

## 10. Performance Benchmarks

| Task | Metric | Target | Hardware |
|------|--------|--------|----------|
| Object Detection (video) | mAP@0.5 | ≥45% | GPU |
| Multi-Object Tracking | MOTA | ≥65% | GPU |
| Action Recognition | Top-1 Acc | ≥70% | GPU |
| Optical Flow | EPE | ≤5.0 | GPU |
| Video Segmentation | J&F | ≥75% | GPU |
| Real-time Processing | FPS | ≥30 | GPU |

---

## 11. Scalability Requirements

### 11.1 Batch Processing

**Requirements:**
- Process multiple videos in parallel
- Queue management system
- Progress tracking per video
- Estimated completion time

### 11.2 Distributed Processing

**Requirements:**
- Support distributed worker nodes
- Load balancing across GPUs
- Fault tolerance (retry failed jobs)
- Result aggregation

---

## 12. Storage and Archival

### 12.1 Video Storage

**Requirements:**
- Secure storage (encrypted at rest)
- Configurable retention period
- Automatic cleanup of processed videos
- Metadata indexing for search

### 12.2 Results Storage

**Requirements:**
- Store tracking data, annotations
- Support export formats (JSON, CSV, COCO)
- Version control for results
- Query API for historical data

---

---

## 13. Wire Protocol Specification

### 13.1 Inference RPC

Vision-AI inference calls use HTTP/2 (RFC 9113) or HTTP/3 (RFC 9114) with TLS 1.3 (RFC 8446). The request body is JSON (RFC 8259) or CBOR (RFC 8949), carrying:

```json
{
  "modelId": "string",
  "modelVersion": "semver",
  "input": {
    "kind": "image|video|stream",
    "format": "image/jpeg|image/png|video/mp4|video/h265|...",
    "uri": "string",
    "inlineBytes": "<base64 if small>"
  },
  "options": {
    "confidenceThreshold": "number",
    "maxResults": "uint",
    "explainability": "bool"
  }
}
```

### 13.2 Streaming Inference

Long-lived stream inference uses HTTP/2 server push or RFC 8895-style server-sent events, or CoAP OBSERVE (RFC 7641) on constrained clients. Streamed result frames carry a monotonically increasing sequence number and the inference timestamp.

### 13.3 Result Format

Results MUST encode:

- Detected entities (boxes, polygons, masks).
- Per-entity class label and confidence.
- Optional explainability artefacts (saliency map, feature attributions).
- Provenance: model identifier, model version, weights digest (SHA-256, FIPS 180-4).

### 13.4 Result Authenticity

Where results contribute to consequential decisions (access, regulatory, safety), the response MUST be signed with COSE_Sign1 (RFC 9052) using the model-deployment key. Verifiers obtain the signing key through the discovery document.

### 13.5 RTSP Streaming

Live-camera ingestion uses RTSP 2.0 (RFC 7826) over TLS 1.3. RTSP 1.0 (RFC 2326) is supported only when the camera lacks RTSP 2.0 capability and MUST always be tunnelled through TLS regardless. RTP payload formats follow RFC 6184 (H.264/AVC) and RFC 7798 (H.265/HEVC).

---

## 14. Time and Synchronization

Inference results that bind to real-world time (e.g. forensic CCTV timestamps) MUST be slaved to NTPv4 with NTS (RFC 5905, RFC 8915). Time skew exceeding 50 ms MUST raise an audit event.

---

## 15. Cryptographic Algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / HTTP/2 / HTTP/3 | TLS 1.3 cipher suites | RFC 8446 |
| DTLS for CoAP | DTLS 1.3 | RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| Model-weight integrity | SHA-256 | FIPS 180-4 |
| Trace encryption | AES-256-GCM | ISO/IEC 18033-3 |

Implementations MUST refuse cipher suites whose IETF status is "not recommended" for new deployments.

---

## 16. Failure Modes

### 16.1 Model unavailable

If a requested model is unavailable, the surface MUST return HTTP 503 with the problem detail `vision/model-unavailable` (RFC 9457) and a `Retry-After` header indicating the next expected availability window.

### 16.2 Input rejected

If the input does not satisfy the model's pre-conditions (resolution range, codec, integrity), the surface MUST return HTTP 422 with the problem detail `vision/input-rejected` and a structured `errors` field listing the failing pre-conditions.

### 16.3 Inference timeout

Inference timeouts return HTTP 504 with the problem detail `vision/inference-timeout` and the `Retry-After` header where retry is appropriate.

---

## 17. References

1. ISO/IEC 14496-10; ISO/IEC 23008-2 — *AVC, HEVC.*
2. ISO/IEC 18033-3:2010 — *Block ciphers.*
3. ISO/IEC 19794 (all parts) — *Biometric data interchange.*
4. ISO/IEC 19795-1:2021 — *Biometric performance testing.*
5. ISO/IEC 22989:2022 — *AI concepts and terminology.*
6. ISO/IEC 23053:2022 — *AI / ML framework.*
7. ISO/IEC 27001:2022; ISO/IEC 27037:2012; ISO/IEC 27701:2019.
8. ISO/IEC 30107-3:2023 — *Biometric presentation attack detection.*
9. ISO/IEC 42001:2023 — *AI management system.*
10. ISO/IEC TR 24028:2020 — *AI trustworthiness.*
11. ISO/IEC TS 4213:2022 — *ML classification performance.*
12. RFC 2326; RFC 7826 — *RTSP 1.0 / 2.0.*
13. RFC 5905; RFC 8915 — *NTPv4, NTS.*
14. RFC 6184; RFC 7798 — *RTP payload for H.264 / H.265.*
15. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
16. RFC 8259; RFC 8615; RFC 8949 — *JSON, well-known URIs, CBOR.*
17. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
18. RFC 8613 — *OSCORE.*
19. RFC 9052; RFC 9053 — *COSE.*
20. RFC 9110; RFC 9113; RFC 9114; RFC 9457 — *HTTP family, problem details.*
21. RFC 9700 — *OAuth 2.1.*
22. FIPS 180-4 — *Secure Hash Standard.*

---

© 2025 SmileStory Inc. / World Certification Industry Association
弘益人間 · Benefit All Humanity