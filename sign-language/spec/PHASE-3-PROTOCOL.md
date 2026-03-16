# WIA Sign Language Recognition Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee  

---

## Table of Contents

1. [Introduction](#introduction)
2. [WebSocket Protocol](#websocket-protocol)
3. [gRPC Protocol](#grpc-protocol)
4. [MQTT Protocol](#mqtt-protocol)
5. [Protocol Selection Guide](#protocol-selection-guide)

---

## 1. Introduction

This specification defines communication protocols for sign language recognition.

---

## 2. WebSocket Protocol

### 2.1 Connection Handshake

```
GET /v1/stream HTTP/1.1
Upgrade: websocket
Connection: Upgrade
```

### 2.2 Message Types

- `video_frame`: Video data
- `recognition`: Recognition result
- `error`: Error message
- `ping/pong`: Keep-alive

---

## 3. gRPC Protocol

### 3.1 Service Definition

```protobuf
service SignLanguageRecognition {
  rpc Recognize(stream VideoFrame) returns (stream RecognitionResult);
  rpc Translate(TranslationRequest) returns (TranslationResponse);
}
```

---

## 4. MQTT Protocol

For IoT devices:

Topic: `wia/sign/{deviceId}/recognize`

---

## 5. Protocol Selection Guide

- **WebSocket**: Real-time web applications
- **gRPC**: Server-to-server, high performance
- **REST**: Simple integration, batch processing
- **MQTT**: IoT devices, low bandwidth

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA
