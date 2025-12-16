# WIA Education - Phase 3: Protocol

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Education의 통신 프로토콜과 LMS 통합을 정의합니다.

## LTI 1.3 Integration

### Launch Flow
1. LMS → Tool: Launch request
2. Tool: Validate JWT
3. Tool: Process accessibility profile
4. Tool → LMS: Return with accommodations

### Deep Linking
- Content selection with accessibility metadata
- Accessibility requirements propagation
- Custom parameters for accommodations

## xAPI (Experience API)

### Statement Format
```json
{
  "actor": { "mbox": "mailto:student@example.com" },
  "verb": { "id": "http://adlnet.gov/expapi/verbs/completed" },
  "object": {
    "id": "http://example.com/course/123",
    "definition": {
      "name": { "en-US": "Accessible Math Course" },
      "extensions": {
        "http://wiastandards.com/xapi/accessibility": {
          "accommodations": ["extra-time", "screen-reader"],
          "assistive_tech": ["voice-input"],
          "completion_method": "alternative-assessment"
        }
      }
    }
  }
}
```

## Event Synchronization

### Real-time Events
- **CaptionGenerated**: 자막 생성 완료
- **AudioDescriptionAdded**: 오디오 설명 추가
- **AccommodationApplied**: 조정 적용
- **AssessmentStarted**: 평가 시작 (조정 포함)

### WebSocket Protocol
```javascript
// Subscribe to events
ws.send({
  type: "subscribe",
  events: ["CaptionGenerated", "AccommodationApplied"]
});

// Receive events
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(data.type, data.payload);
};
```

## LMS Integration

### Canvas
- OAuth 2.0 authentication
- Course accessibility settings
- Assignment accommodations
- Grade sync with accommodation notes

### Moodle
- External tool integration
- Accessibility module
- Custom user fields
- Activity completion tracking

### Blackboard
- REST API integration
- Accessibility features
- Content package import
- Grade center integration

## Security

### Authentication
- OAuth 2.0
- LTI 1.3 JWT validation
- API key management

### Data Privacy
- FERPA compliance
- Student data protection
- Audit logging
- Encryption at rest and in transit

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
