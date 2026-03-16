# WIA-EDU-004 Learning Analytics Standard
## PHASE 1: Foundation

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document defines the foundational elements of the WIA-EDU-004 Learning Analytics Standard, establishing the core data models, vocabularies, and concepts that underpin all learning analytics implementations.

### 1.1 Purpose

The foundation phase establishes:
- Standard vocabulary and terminology
- Core data models for learning analytics
- Basic data types and structures
- Minimal interoperability requirements

### 1.2 Scope

This specification covers:
- **Learner data models**
- **Learning event structures**
- **Context definitions**
- **Result representations**
- **Basic metrics and KPIs**

---

## 2. Core Data Models

### 2.1 Learner Entity

The Learner represents an individual engaged in learning activities.

```json
{
  "id": "string (UUID or URI)",
  "type": "Learner",
  "identifiers": {
    "studentId": "string",
    "email": "string (optional)",
    "externalIds": {
      "lms": "string",
      "sis": "string"
    }
  },
  "profile": {
    "givenName": "string",
    "familyName": "string",
    "displayName": "string",
    "demographics": {
      "birthDate": "ISO 8601 date (optional)",
      "gender": "string (optional)",
      "primaryLanguage": "ISO 639-1 code"
    },
    "academicInfo": {
      "program": "string",
      "level": "string (undergraduate, graduate, etc.)",
      "enrollmentStatus": "string (active, inactive, graduated)"
    }
  },
  "preferences": {
    "accessibility": {},
    "privacy": {},
    "notifications": {}
  },
  "metadata": {
    "created": "ISO 8601 timestamp",
    "updated": "ISO 8601 timestamp"
  }
}
```

### 2.2 Learning Event

Learning events capture interactions between learners and learning resources.

```json
{
  "id": "string (UUID)",
  "type": "LearningEvent",
  "actor": {
    "id": "learner UUID/URI",
    "type": "Learner"
  },
  "verb": {
    "id": "URI",
    "display": {
      "en-US": "string",
      "ko-KR": "string"
    }
  },
  "object": {
    "id": "URI",
    "type": "Activity|Assessment|Resource|Course",
    "definition": {
      "name": {},
      "description": {},
      "type": "URI"
    }
  },
  "result": {
    "score": {
      "scaled": "number (0-1)",
      "raw": "number",
      "min": "number",
      "max": "number"
    },
    "success": "boolean",
    "completion": "boolean",
    "duration": "ISO 8601 duration",
    "response": "string (optional)"
  },
  "context": {
    "registration": "UUID",
    "instructor": {},
    "team": {},
    "contextActivities": {
      "parent": [],
      "grouping": [],
      "category": [],
      "other": []
    },
    "platform": "string",
    "language": "ISO 639-1",
    "extensions": {}
  },
  "timestamp": "ISO 8601 timestamp",
  "stored": "ISO 8601 timestamp"
}
```

### 2.3 Course Entity

```json
{
  "id": "string (UUID or course code)",
  "type": "Course",
  "code": "string",
  "title": {
    "en": "string",
    "ko": "string"
  },
  "description": "string",
  "subject": "string",
  "level": "string",
  "credits": "number",
  "term": {
    "id": "string",
    "name": "string",
    "startDate": "ISO 8601 date",
    "endDate": "ISO 8601 date"
  },
  "instructors": ["learner IDs"],
  "learningObjectives": [
    {
      "id": "string",
      "description": "string",
      "level": "string (Bloom's taxonomy)"
    }
  ],
  "metadata": {}
}
```

---

## 3. Standard Vocabularies

### 3.1 Verb Taxonomy

Standard verbs for learning events following xAPI patterns:

| Verb ID | Display | Definition |
|---------|---------|------------|
| `http://adlnet.gov/expapi/verbs/completed` | completed | Successfully completed an activity |
| `http://adlnet.gov/expapi/verbs/passed` | passed | Met minimum requirements |
| `http://adlnet.gov/expapi/verbs/failed` | failed | Did not meet minimum requirements |
| `http://adlnet.gov/expapi/verbs/attempted` | attempted | Started but not completed |
| `http://adlnet.gov/expapi/verbs/experienced` | experienced | Engaged with content |
| `http://adlnet.gov/expapi/verbs/attended` | attended | Participated in live session |
| `http://wia.org/verbs/mastered` | mastered | Demonstrated mastery |
| `http://wia.org/verbs/reviewed` | reviewed | Revisited previously learned content |

### 3.2 Activity Types

| Type URI | Description |
|----------|-------------|
| `http://adlnet.gov/expapi/activities/course` | Full course |
| `http://adlnet.gov/expapi/activities/module` | Course module or unit |
| `http://adlnet.gov/expapi/activities/lesson` | Individual lesson |
| `http://adlnet.gov/expapi/activities/assessment` | Quiz, test, or exam |
| `http://adlnet.gov/expapi/activities/interaction` | Interactive element |
| `http://adlnet.gov/expapi/activities/media` | Video, audio, or multimedia |
| `http://wia.org/activities/discussion` | Discussion forum or thread |
| `http://wia.org/activities/assignment` | Submitted work |

---

## 4. Core Metrics

### 4.1 Engagement Metrics

**Login Frequency**
```
login_frequency = count(login_events) / time_period_days
```

**Time on Task**
```
time_on_task = sum(event_duration) WHERE event_type IN learning_activities
```

**Interaction Depth**
```
interaction_depth = count(DISTINCT event_types) × avg_events_per_type
```

### 4.2 Performance Metrics

**Completion Rate**
```
completion_rate = (completed_activities / total_activities) × 100
```

**Average Score**
```
average_score = sum(scores) / count(assessments)
```

**Mastery Level**
```
mastery_level = CASE
  WHEN avg_score >= 90 THEN "expert"
  WHEN avg_score >= 80 THEN "proficient"
  WHEN avg_score >= 70 THEN "competent"
  ELSE "developing"
END
```

---

## 5. Data Quality Requirements

### 5.1 Completeness

All required fields MUST be populated:
- Learner: `id`, `type`
- Event: `actor`, `verb`, `object`, `timestamp`
- Result: `success` or `completion` (at least one)

### 5.2 Accuracy

- Timestamps MUST be in ISO 8601 format with timezone
- Scores MUST be within defined min/max ranges
- IDs MUST be unique within their scope
- URIs MUST be valid and resolvable where applicable

### 5.3 Consistency

- Use consistent identifiers across systems
- Follow naming conventions for all fields
- Maintain referential integrity (foreign keys)

---

## 6. Privacy and Security

### 6.1 Data Minimization

Collect only data necessary for defined analytics purposes. Avoid collecting:
- Unnecessary demographic data
- Sensitive personal information without clear justification
- Data that could enable discrimination

### 6.2 Anonymization Requirements

For aggregated analytics and research:
- Remove or hash personally identifiable information
- Ensure k-anonymity (k ≥ 5 recommended)
- Apply differential privacy where appropriate

### 6.3 Access Control

- Implement role-based access control (RBAC)
- Learners can access only their own data
- Instructors can access data for their courses
- Researchers access only anonymized data

---

## 7. Interoperability

### 7.1 Standards Alignment

WIA-EDU-004 Foundation aligns with:
- **xAPI 2.0**: Learning event structure
- **IMS Caliper 1.2**: Event profiles and metrics
- **OneRoster 1.2**: Student and course data exchange
- **LTI 1.3**: Tool integration

### 7.2 Data Exchange Formats

Supported formats for data exchange:
- **JSON**: Primary format for APIs
- **CSV**: Bulk data export/import
- **Parquet**: Analytics data warehouse
- **JSONL**: Streaming events

---

## 8. Implementation Checklist

- [ ] Define learner identifiers (student ID, email, etc.)
- [ ] Map existing data to WIA-EDU-004 learner model
- [ ] Implement learning event tracking for key activities
- [ ] Set up Learning Record Store (LRS) or equivalent
- [ ] Configure data validation and quality checks
- [ ] Establish privacy and security controls
- [ ] Test interoperability with partner systems
- [ ] Document data dictionary and mappings

---

## 9. Compliance

Implementations MUST:
1. Comply with applicable privacy regulations (GDPR, FERPA, etc.)
2. Obtain appropriate consent for data collection
3. Provide data subject rights (access, correction, deletion)
4. Maintain audit logs of data access
5. Implement encryption at rest and in transit

---

## 10. Next Steps

After completing Phase 1 Foundation:
- **Phase 2: Implementation** - Build analytics pipelines and dashboards
- **Phase 3: Integration** - Connect with external systems
- **Phase 4: Optimization** - Advanced analytics and AI/ML

---

**Document Status:** ✅ Complete
**Effective Date:** 2025-01-15
**Review Cycle:** Annual

---

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity
