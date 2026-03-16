# WIA-AI-023 NLP Standard - Phase 1: Data Format

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-25

---

## 1. Overview

Phase 1 defines standardized data formats for Natural Language Processing inputs, outputs, and intermediate representations. Consistent data formats ensure interoperability between different NLP systems and enable seamless data exchange.

### 1.1 Goals

- Define JSON schemas for all NLP tasks
- Standardize text encoding and representation
- Specify metadata requirements
- Enable language-agnostic processing
- Support multilingual applications

## 2. Core Data Schema

### 2.1 Base Request Format

All NLP requests follow this base schema:

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "task": "string",
  "input": {
    "text": "string",
    "language": "string (ISO 639-1)",
    "encoding": "UTF-8"
  },
  "config": {},
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601",
    "client_info": {}
  }
}
```

### 2.2 Base Response Format

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "task": "string",
  "output": {},
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601",
    "processing_time_ms": "number",
    "model": "string",
    "confidence": "number (0-1)"
  },
  "status": {
    "code": "number",
    "message": "string"
  }
}
```

## 3. Task-Specific Data Formats

### 3.1 Tokenization

**Request:**
```json
{
  "task": "tokenization",
  "input": {
    "text": "Natural language processing is essential.",
    "language": "en"
  },
  "config": {
    "method": "word|subword|character",
    "preserve_case": true,
    "include_offsets": true
  }
}
```

**Response:**
```json
{
  "output": {
    "tokens": ["Natural", "language", "processing", "is", "essential", "."],
    "token_count": 6,
    "offsets": [
      {"start": 0, "end": 7},
      {"start": 8, "end": 16},
      {"start": 17, "end": 27},
      {"start": 28, "end": 30},
      {"start": 31, "end": 40},
      {"start": 40, "end": 41}
    ]
  }
}
```

### 3.2 Named Entity Recognition (NER)

**Request:**
```json
{
  "task": "named_entity_recognition",
  "input": {
    "text": "Apple CEO Tim Cook visited Cupertino on January 15th.",
    "language": "en"
  },
  "config": {
    "entity_types": ["PERSON", "ORGANIZATION", "LOCATION", "DATE"],
    "aggregation": "simple"
  }
}
```

**Response:**
```json
{
  "output": {
    "entities": [
      {
        "text": "Apple",
        "type": "ORGANIZATION",
        "start": 0,
        "end": 5,
        "confidence": 0.98
      },
      {
        "text": "Tim Cook",
        "type": "PERSON",
        "start": 10,
        "end": 18,
        "confidence": 0.99
      },
      {
        "text": "Cupertino",
        "type": "LOCATION",
        "start": 27,
        "end": 36,
        "confidence": 0.97
      },
      {
        "text": "January 15th",
        "type": "DATE",
        "start": 40,
        "end": 52,
        "confidence": 0.95
      }
    ],
    "entity_count": 4
  }
}
```

### 3.3 Sentiment Analysis

**Request:**
```json
{
  "task": "sentiment_analysis",
  "input": {
    "text": "This product is absolutely amazing!",
    "language": "en"
  },
  "config": {
    "granularity": "document|sentence",
    "return_scores": true
  }
}
```

**Response:**
```json
{
  "output": {
    "sentiment": "positive",
    "confidence": 0.96,
    "scores": {
      "positive": 0.96,
      "neutral": 0.03,
      "negative": 0.01
    },
    "polarity": 0.92,
    "subjectivity": 0.85
  }
}
```

### 3.4 Text Classification

**Request:**
```json
{
  "task": "text_classification",
  "input": {
    "text": "Scientists discovered a new exoplanet orbiting a distant star.",
    "language": "en"
  },
  "config": {
    "categories": ["Technology", "Science", "Business", "Sports", "Politics"],
    "top_k": 3,
    "threshold": 0.1
  }
}
```

**Response:**
```json
{
  "output": {
    "predictions": [
      {
        "category": "Science",
        "confidence": 0.94,
        "rank": 1
      },
      {
        "category": "Technology",
        "confidence": 0.28,
        "rank": 2
      },
      {
        "category": "Business",
        "confidence": 0.05,
        "rank": 3
      }
    ],
    "primary_category": "Science"
  }
}
```

### 3.5 Text Generation

**Request:**
```json
{
  "task": "text_generation",
  "input": {
    "prompt": "Natural language processing enables",
    "language": "en"
  },
  "config": {
    "max_length": 100,
    "temperature": 0.8,
    "top_p": 0.95,
    "top_k": 50,
    "num_return_sequences": 3,
    "do_sample": true,
    "stop_sequences": ["\n\n"]
  }
}
```

**Response:**
```json
{
  "output": {
    "generated_texts": [
      "Natural language processing enables computers to understand and generate human language effectively.",
      "Natural language processing enables advanced chatbots and virtual assistants.",
      "Natural language processing enables sentiment analysis and text classification at scale."
    ],
    "generation_count": 3
  }
}
```

### 3.6 Text Summarization

**Request:**
```json
{
  "task": "summarization",
  "input": {
    "text": "Long document text here...",
    "language": "en"
  },
  "config": {
    "method": "abstractive|extractive",
    "max_length": 150,
    "min_length": 50,
    "num_sentences": null
  }
}
```

**Response:**
```json
{
  "output": {
    "summary": "Concise summary of the document...",
    "summary_length": 87,
    "compression_ratio": 0.15,
    "key_points": [
      "First key point",
      "Second key point",
      "Third key point"
    ]
  }
}
```

## 4. Language Codes

All language specifications use ISO 639-1 two-letter codes:

| Code | Language      |
|------|---------------|
| en   | English       |
| ko   | Korean        |
| es   | Spanish       |
| fr   | French        |
| de   | German        |
| zh   | Chinese       |
| ja   | Japanese      |
| ar   | Arabic        |
| hi   | Hindi         |
| pt   | Portuguese    |

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "standard": "WIA-AI-023",
  "version": "1.0",
  "status": {
    "code": 400,
    "message": "Invalid input format",
    "details": "Text field is required"
  },
  "metadata": {
    "request_id": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

### 5.2 Standard Error Codes

| Code | Meaning                  | Description                        |
|------|--------------------------|------------------------------------|
| 200  | Success                  | Request completed successfully     |
| 400  | Bad Request              | Invalid input format or parameters |
| 401  | Unauthorized             | Invalid or missing authentication  |
| 429  | Too Many Requests        | Rate limit exceeded                |
| 500  | Internal Server Error    | Server-side processing error       |
| 503  | Service Unavailable      | Service temporarily unavailable    |

## 6. Validation Rules

### 6.1 Text Input Validation

- **Encoding:** Must be valid UTF-8
- **Length:** Maximum 100,000 characters (configurable per implementation)
- **Empty text:** Returns error code 400
- **Special characters:** Preserved unless explicitly configured otherwise

### 6.2 Language Code Validation

- Must be valid ISO 639-1 code
- Case-insensitive (normalized to lowercase)
- Unknown languages return error code 400

### 6.3 Confidence Scores

- Range: 0.0 to 1.0 (inclusive)
- Precision: At least 2 decimal places
- Missing confidence defaults to null

## 7. Metadata Requirements

### 7.1 Mandatory Metadata

Every response must include:
- `request_id`: Unique identifier (UUID v4)
- `timestamp`: ISO 8601 format
- `standard`: "WIA-AI-023"
- `version`: Semantic version string

### 7.2 Optional Metadata

- `processing_time_ms`: Processing duration
- `model`: Model identifier
- `confidence`: Overall confidence score
- `warnings`: Array of warning messages
- `debug_info`: Debugging information (development only)

## 8. Compliance Checklist

- [ ] All requests include standard and version fields
- [ ] Text encoding is UTF-8
- [ ] Language codes use ISO 639-1
- [ ] Response includes request_id and timestamp
- [ ] Error responses follow standard format
- [ ] Confidence scores are in range [0, 1]
- [ ] Entity offsets are character-based (not byte-based)
- [ ] All JSON is valid and well-formed

---

**Next:** [Phase 2: API Interface](PHASE-2-API.md)

**弘益人間** · Benefit All Humanity

© 2025 WIA - World Certification Industry Association
