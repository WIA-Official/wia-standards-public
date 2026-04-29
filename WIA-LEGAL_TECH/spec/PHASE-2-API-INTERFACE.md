# WIA-LEGAL_TECH: PHASE 2 - API Interface Specification

**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Hongik Ingan - Benefit All Humanity)

## Overview

This specification defines RESTful and GraphQL API interfaces for legal technology platforms, enabling contract analysis, e-discovery, document automation, legal research, and AI-powered legal assistance.

## 1. API Architecture

### 1.1 Base Configuration

```yaml
base_url: https://api.wia-legal.tech/v1
protocols: [HTTPS]
authentication: OAuth 2.0, JWT, API Key
rate_limiting: 1000 requests/hour (standard), 10000/hour (enterprise)
response_format: JSON
api_version: v1
```

### 1.2 Authentication

```http
POST /auth/token
Content-Type: application/json

{
  "client_id": "string",
  "client_secret": "string",
  "grant_type": "client_credentials",
  "scope": "contracts.read contracts.write ediscovery.analyze"
}

Response:
{
  "access_token": "string",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "string"
}
```

## 2. Contract Analysis API

### 2.1 Upload Contract for Analysis

```http
POST /contracts/upload
Authorization: Bearer {token}
Content-Type: multipart/form-data

Parameters:
- file: binary (PDF, DOCX, TXT)
- metadata: JSON object
- analysis_type: string (full|quick|clause_extraction|risk_analysis)

Response 201:
{
  "contractId": "uuid",
  "status": "processing",
  "estimatedTime": "number (seconds)",
  "trackingUrl": "/contracts/{contractId}/status"
}
```

### 2.2 Get Contract Analysis

```http
GET /contracts/{contractId}/analysis
Authorization: Bearer {token}

Response 200:
{
  "contractId": "uuid",
  "status": "completed",
  "analysis": {
    "documentInfo": {
      "title": "string",
      "parties": ["string"],
      "effectiveDate": "ISO 8601",
      "expirationDate": "ISO 8601",
      "jurisdiction": "string"
    },
    "clauses": [{
      "id": "string",
      "type": "string",
      "text": "string",
      "riskLevel": "low|medium|high|critical",
      "favorability": "favorable|neutral|unfavorable",
      "confidence": "number (0-1)",
      "suggestions": ["string"]
    }],
    "financialTerms": [{
      "type": "string",
      "amount": "number",
      "currency": "string",
      "schedule": "string"
    }],
    "obligations": [{
      "party": "string",
      "description": "string",
      "deadline": "ISO 8601",
      "status": "string"
    }],
    "riskScore": {
      "overall": "number (0-100)",
      "categories": {
        "liability": "number",
        "compliance": "number",
        "financial": "number",
        "termination": "number"
      }
    },
    "keyDates": [{
      "type": "string",
      "date": "ISO 8601",
      "description": "string"
    }],
    "redFlags": [{
      "severity": "critical|high|medium|low",
      "description": "string",
      "location": "string",
      "recommendation": "string"
    }],
    "comparisons": {
      "industryStandard": "deviation_percentage",
      "yourTemplates": "similarity_score",
      "marketTerms": "favorability_score"
    }
  },
  "aiInsights": {
    "executiveSummary": "string",
    "negotiationPoints": ["string"],
    "missingClauses": ["string"],
    "unusualTerms": ["string"]
  }
}
```

### 2.3 Compare Contracts

```http
POST /contracts/compare
Authorization: Bearer {token}
Content-Type: application/json

{
  "contractIds": ["uuid1", "uuid2"],
  "comparisonType": "full|clauses|financial|obligations"
}

Response 200:
{
  "comparisonId": "uuid",
  "contracts": ["uuid1", "uuid2"],
  "differences": [{
    "category": "string",
    "field": "string",
    "contract1": "value",
    "contract2": "value",
    "significance": "high|medium|low"
  }],
  "similarities": [{
    "category": "string",
    "description": "string",
    "percentage": "number"
  }],
  "recommendations": ["string"]
}
```

### 2.4 Extract Clauses

```http
POST /contracts/{contractId}/extract-clauses
Authorization: Bearer {token}
Content-Type: application/json

{
  "clauseTypes": ["indemnification", "limitation_of_liability", "confidentiality"],
  "includeContext": true
}

Response 200:
{
  "contractId": "uuid",
  "clauses": [{
    "type": "string",
    "text": "string",
    "context": "string",
    "page": "number",
    "confidence": "number (0-1)"
  }]
}
```

### 2.5 Clause Recommendation

```http
POST /contracts/clauses/recommend
Authorization: Bearer {token}
Content-Type: application/json

{
  "contractType": "nda|msa|employment|lease",
  "jurisdiction": "string",
  "riskTolerance": "low|medium|high",
  "context": "string"
}

Response 200:
{
  "recommendations": [{
    "clauseType": "string",
    "priority": "required|recommended|optional",
    "templates": [{
      "id": "string",
      "text": "string",
      "riskLevel": "string",
      "popularity": "number (0-100)",
      "precedents": ["string"]
    }]
  }]
}
```

## 3. E-Discovery API

### 3.1 Create Review Project

```http
POST /ediscovery/projects
Authorization: Bearer {token}
Content-Type: application/json

{
  "name": "string",
  "caseId": "uuid",
  "reviewers": ["userId"],
  "settings": {
    "aiAssisted": true,
    "predictiveCoding": true,
    "duplicateDetection": true,
    "threadingEnabled": true
  }
}

Response 201:
{
  "projectId": "uuid",
  "status": "created",
  "settings": "object"
}
```

### 3.2 Upload Documents for Review

```http
POST /ediscovery/projects/{projectId}/documents
Authorization: Bearer {token}
Content-Type: multipart/form-data

Parameters:
- files: binary[] (max 100 files per request)
- custodian: string
- loadFile: binary (optional, DAT/OPT format)

Response 202:
{
  "uploadId": "uuid",
  "documentsQueued": "number",
  "estimatedProcessing": "number (minutes)"
}
```

### 3.3 Search Documents

```http
POST /ediscovery/projects/{projectId}/search
Authorization: Bearer {token}
Content-Type: application/json

{
  "query": "string (Boolean search)",
  "filters": {
    "dateRange": {"start": "ISO 8601", "end": "ISO 8601"},
    "custodian": ["string"],
    "fileType": ["pdf", "docx", "email"],
    "tags": ["responsive", "privileged"],
    "reviewStatus": "pending|reviewed|exported"
  },
  "sort": {
    "field": "date|relevance|custodian",
    "order": "asc|desc"
  },
  "pagination": {
    "page": 1,
    "pageSize": 50
  }
}

Response 200:
{
  "total": "number",
  "page": "number",
  "pageSize": "number",
  "documents": [{
    "documentId": "uuid",
    "filename": "string",
    "custodian": "string",
    "date": "ISO 8601",
    "fileType": "string",
    "size": "number",
    "tags": ["string"],
    "aiPredictions": {
      "responsive": "number (0-1)",
      "privileged": "number (0-1)",
      "relevance": "number (0-1)"
    },
    "snippet": "string",
    "reviewStatus": "string"
  }]
}
```

### 3.4 AI-Assisted Document Review

```http
GET /ediscovery/projects/{projectId}/documents/{documentId}/ai-analysis
Authorization: Bearer {token}

Response 200:
{
  "documentId": "uuid",
  "aiAnalysis": {
    "responsiveness": {
      "score": "number (0-1)",
      "reasoning": "string",
      "keyPhrases": ["string"]
    },
    "privilege": {
      "score": "number (0-1)",
      "indicators": ["string"],
      "confidence": "high|medium|low"
    },
    "entities": [{
      "type": "person|organization|location|date|money",
      "text": "string",
      "relevance": "number"
    }],
    "sentiment": "positive|neutral|negative",
    "topics": ["string"],
    "similarDocuments": [{
      "documentId": "uuid",
      "similarity": "number (0-1)",
      "filename": "string"
    }],
    "suggestedTags": ["string"],
    "redactionSuggestions": [{
      "text": "string",
      "reason": "pii|privileged|confidential",
      "location": "object"
    }]
  }
}
```

### 3.5 Predictive Coding Training

```http
POST /ediscovery/projects/{projectId}/predictive-coding/train
Authorization: Bearer {token}
Content-Type: application/json

{
  "trainingSet": [{
    "documentId": "uuid",
    "coding": {
      "responsive": "boolean",
      "privileged": "boolean"
    }
  }],
  "modelType": "classification|ranking"
}

Response 202:
{
  "trainingId": "uuid",
  "status": "training",
  "estimatedCompletion": "ISO 8601"
}
```

## 4. Document Automation API

### 4.1 Generate Document from Template

```http
POST /documents/generate
Authorization: Bearer {token}
Content-Type: application/json

{
  "templateId": "uuid",
  "variables": {
    "party1_name": "Acme Corp",
    "party2_name": "Beta Inc",
    "effective_date": "2026-01-15",
    "amount": 100000,
    "currency": "USD"
  },
  "format": "pdf|docx|html",
  "includeSignatureBlocks": true
}

Response 201:
{
  "documentId": "uuid",
  "downloadUrl": "string",
  "expiresAt": "ISO 8601",
  "previewUrl": "string"
}
```

### 4.2 Create Template

```http
POST /documents/templates
Authorization: Bearer {token}
Content-Type: application/json

{
  "name": "string",
  "category": "contract|pleading|letter|memo",
  "content": "string (with {{variable}} placeholders)",
  "variables": [{
    "name": "string",
    "type": "string|number|date|boolean|enum",
    "required": true,
    "default": "any",
    "validation": "string (regex)"
  }],
  "clauses": [{
    "id": "string",
    "optional": true,
    "conditions": "string (logical expression)"
  }]
}

Response 201:
{
  "templateId": "uuid",
  "version": "1.0",
  "status": "active"
}
```

### 4.3 Redline Comparison

```http
POST /documents/redline
Authorization: Bearer {token}
Content-Type: application/json

{
  "originalDocumentId": "uuid",
  "revisedDocumentId": "uuid",
  "outputFormat": "pdf|docx|html"
}

Response 200:
{
  "comparisonId": "uuid",
  "changes": {
    "additions": "number",
    "deletions": "number",
    "modifications": "number"
  },
  "downloadUrl": "string",
  "changeSummary": [{
    "type": "addition|deletion|modification",
    "section": "string",
    "description": "string",
    "significance": "major|minor"
  }]
}
```

## 5. Legal Research API

### 5.1 Search Case Law

```http
POST /research/cases/search
Authorization: Bearer {token}
Content-Type: application/json

{
  "query": "string",
  "filters": {
    "jurisdiction": ["US-CA", "US-NY"],
    "court": ["Supreme Court", "Court of Appeals"],
    "dateRange": {"start": "ISO 8601", "end": "ISO 8601"},
    "cited": "minimum_citations"
  },
  "limit": 50
}

Response 200:
{
  "results": [{
    "caseId": "uuid",
    "citation": "string",
    "title": "string",
    "court": "string",
    "date": "ISO 8601",
    "relevanceScore": "number (0-100)",
    "snippet": "string",
    "citedBy": "number",
    "shepardSignal": "red|yellow|green",
    "headnotes": ["string"]
  }],
  "total": "number",
  "relatedSearches": ["string"]
}
```

### 5.2 Cite Check

```http
POST /research/cite-check
Authorization: Bearer {token}
Content-Type: application/json

{
  "documentId": "uuid",
  "citations": ["string"]
}

Response 200:
{
  "citationAnalysis": [{
    "citation": "string",
    "valid": "boolean",
    "status": "good_law|questioned|bad_law",
    "treatment": "string",
    "citedBy": "number",
    "negativeHistory": [{
      "case": "string",
      "treatment": "overruled|reversed|distinguished",
      "date": "ISO 8601"
    }],
    "recommendation": "safe_to_cite|use_with_caution|do_not_cite"
  }]
}
```

## 6. Compliance API

### 6.1 Run Compliance Check

```http
POST /compliance/check
Authorization: Bearer {token}
Content-Type: application/json

{
  "documentId": "uuid",
  "frameworks": ["gdpr", "ccpa", "sox", "hipaa"],
  "jurisdiction": "string"
}

Response 200:
{
  "checkId": "uuid",
  "status": "compliant|non_compliant|needs_review",
  "overallScore": "number (0-100)",
  "frameworkResults": [{
    "framework": "string",
    "compliant": "boolean",
    "score": "number",
    "findings": [{
      "severity": "critical|high|medium|low",
      "category": "string",
      "description": "string",
      "recommendation": "string",
      "reference": "string"
    }]
  }],
  "requiredActions": [{
    "priority": "immediate|high|medium|low",
    "description": "string",
    "dueDate": "ISO 8601"
  }]
}
```

## 7. Workflow API

### 7.1 Create Workflow

```http
POST /workflows
Authorization: Bearer {token}
Content-Type: application/json

{
  "name": "string",
  "type": "contract_review|document_approval|matter_intake",
  "steps": [{
    "id": "string",
    "type": "review|approval|signature|notification",
    "assignee": "userId|role",
    "deadline": "number (days)",
    "conditions": "string",
    "actions": [{
      "type": "email|webhook|task",
      "config": "object"
    }]
  }],
  "autoStart": "boolean"
}

Response 201:
{
  "workflowId": "uuid",
  "status": "active",
  "version": "1.0"
}
```

### 7.2 Start Workflow Instance

```http
POST /workflows/{workflowId}/instances
Authorization: Bearer {token}
Content-Type: application/json

{
  "documentId": "uuid",
  "metadata": "object",
  "priority": "low|normal|high|urgent"
}

Response 201:
{
  "instanceId": "uuid",
  "status": "in_progress",
  "currentStep": "string",
  "assignedTo": "userId",
  "dueDate": "ISO 8601"
}
```

## 8. AI Assistant API

### 8.1 Legal Question Answering

```http
POST /ai/legal-qa
Authorization: Bearer {token}
Content-Type: application/json

{
  "question": "string",
  "context": {
    "jurisdiction": "string",
    "practiceArea": "string",
    "documentIds": ["uuid"]
  },
  "includeReferences": true
}

Response 200:
{
  "answer": "string",
  "confidence": "number (0-1)",
  "sources": [{
    "type": "case_law|statute|article",
    "citation": "string",
    "relevance": "number"
  }],
  "relatedQuestions": ["string"]
}
```

## 9. Webhook API

### 9.1 Register Webhook

```http
POST /webhooks
Authorization: Bearer {token}
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": ["contract.analyzed", "document.reviewed", "workflow.completed"],
  "secret": "string"
}

Response 201:
{
  "webhookId": "uuid",
  "status": "active"
}
```

## 10. Error Responses

```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "object",
    "requestId": "uuid",
    "timestamp": "ISO 8601"
  }
}
```

### Error Codes
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Rate Limit Exceeded
- 500: Internal Server Error
- 503: Service Unavailable

---

**Document Control**
- Created: 2026-01-12
- Version: 1.0
- Status: Official
- Next Review: 2026-07-12

**Copyright © 2025 WIA (World Certification Industry Association)**
弘益人間 · Benefit All Humanity
