# CRYO-CONSENT Phase 4: Integration Specification

## Overview

This document provides implementation guidelines, database schemas, and deployment configurations for the CRYO-CONSENT system.

## Rust Implementation

### Core Service

```rust
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sqlx::PgPool;
use uuid::Uuid;

// ============================================================================
// Domain Models
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentDocument {
    pub document_id: String,
    pub version: String,
    pub status: ConsentStatus,
    pub subject: SubjectInformation,
    pub consent_declarations: ConsentDeclarations,
    pub revival_conditions: RevivalConditions,
    pub family_agreements: FamilyAgreements,
    pub financial_arrangements: FinancialArrangements,
    pub signatures: SignatureBlock,
    pub amendments: Vec<Amendment>,
    pub created_at: DateTime<Utc>,
    pub last_modified: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, sqlx::Type)]
#[sqlx(type_name = "consent_status", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConsentStatus {
    Draft,
    PendingSignatures,
    PendingNotarization,
    Active,
    Suspended,
    Withdrawn,
    Executed,
    Completed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubjectInformation {
    pub subject_id: String,
    pub identity_ref: String,
    pub personal_info: PersonalInfo,
    pub legal_capacity: LegalCapacity,
    pub mental_capacity_assessment: MentalCapacityAssessment,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RevivalConditions {
    pub condition_logic: ConditionLogic,
    pub custom_logic: Option<String>,
    pub conditions: Vec<RevivalCondition>,
    pub minimum_duration: Option<String>,
    pub maximum_duration: Option<String>,
    pub emergency_revival_authorized: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConditionLogic {
    All,
    Any,
    Custom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RevivalCondition {
    pub condition_id: String,
    pub condition_type: ConditionType,
    pub description: String,
    pub required: bool,
    pub weight: Option<f64>,
    pub parameters: serde_json::Value,
    pub status: ConditionStatus,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConditionType {
    Medical,
    Technology,
    Time,
    Social,
    Legal,
    Financial,
    Custom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ConditionStatus {
    Pending,
    Met,
    Failed,
    Waived,
}

// ============================================================================
// Service Implementation
// ============================================================================

pub struct ConsentService {
    pool: PgPool,
    validator: ConsentValidator,
    conflict_detector: ConflictDetector,
    revival_evaluator: RevivalConditionEvaluator,
}

impl ConsentService {
    pub fn new(pool: PgPool) -> Self {
        Self {
            pool,
            validator: ConsentValidator::new(),
            conflict_detector: ConflictDetector::new(),
            revival_evaluator: RevivalConditionEvaluator::new(),
        }
    }

    /// Create a new consent document
    pub async fn create_document(
        &self,
        request: CreateDocumentRequest,
    ) -> Result<ConsentDocument, ConsentError> {
        // Generate document ID
        let document_id = format!(
            "CONSENT-{}-{:03}",
            Utc::now().format("%Y"),
            self.get_next_sequence().await?
        );

        // Load template
        let template = self.load_template(&request.template_id).await?;

        // Create initial document
        let document = ConsentDocument {
            document_id: document_id.clone(),
            version: "1.0.0".to_string(),
            status: ConsentStatus::Draft,
            subject: SubjectInformation::from_subject_id(&request.subject_id),
            consent_declarations: template.default_declarations(),
            revival_conditions: RevivalConditions::default(),
            family_agreements: FamilyAgreements::default(),
            financial_arrangements: FinancialArrangements::default(),
            signatures: SignatureBlock::default(),
            amendments: vec![],
            created_at: Utc::now(),
            last_modified: Utc::now(),
        };

        // Store in database
        self.store_document(&document).await?;

        // Emit event
        self.emit_event(ConsentEvent::DocumentCreated {
            document_id: document_id.clone(),
            subject_id: request.subject_id,
            created_at: Utc::now(),
        })
        .await?;

        Ok(document)
    }

    /// Validate a consent document
    pub async fn validate_document(
        &self,
        document_id: &str,
    ) -> Result<ValidationReport, ConsentError> {
        let document = self.get_document(document_id).await?;
        let report = self.validator.validate(&document);
        Ok(report)
    }

    /// Add signature to document
    pub async fn add_signature(
        &self,
        document_id: &str,
        signature: SignatureRequest,
    ) -> Result<SignatureRecord, ConsentError> {
        let mut document = self.get_document(document_id).await?;

        // Validate signature can be added
        self.validate_signature_request(&document, &signature)?;

        // Create signature record
        let record = SignatureRecord {
            signer_id: signature.signer_id.clone(),
            signer_name: signature.signer_name.clone(),
            signer_role: signature.signer_role.clone(),
            signature_type: signature.signature_type.clone(),
            signature_data: signature.signature_data.clone(),
            timestamp: Utc::now(),
            location: signature.location.clone(),
            verification_status: VerificationStatus::Pending,
        };

        // Add to appropriate section
        match signature.signer_role {
            SignerRole::Subject => {
                document.signatures.subject_signature = Some(record.clone());
            }
            SignerRole::Witness => {
                document.signatures.witness_signatures.push(record.clone());
            }
            SignerRole::Notary => {
                // Handle notarization
            }
            _ => {
                // Other roles
            }
        }

        // Update status if all required signatures collected
        self.check_and_update_status(&mut document).await?;

        // Store updated document
        self.update_document(&document).await?;

        // Emit event
        self.emit_event(ConsentEvent::SignatureAdded {
            document_id: document_id.to_string(),
            signer_role: signature.signer_role,
            signed_at: Utc::now(),
        })
        .await?;

        Ok(record)
    }

    /// Create an amendment
    pub async fn create_amendment(
        &self,
        document_id: &str,
        request: CreateAmendmentRequest,
    ) -> Result<Amendment, ConsentError> {
        let document = self.get_document(document_id).await?;

        // Calculate required approvals
        let guardians = &document.family_agreements.guardian_designations;
        let is_critical = self.is_critical_change(&request.changes);
        let required_approvals = if is_critical {
            (guardians.len() * 2 / 3) + 1
        } else {
            (guardians.len() / 2) + 1
        };

        let amendment = Amendment {
            amendment_id: format!(
                "AMD-{}-{:03}",
                document_id,
                document.amendments.len() + 1
            ),
            amendment_number: document.amendments.len() as i32 + 1,
            amendment_type: self.determine_amendment_type(&request.changes),
            description: self.generate_description(&request.changes),
            changes: request.changes,
            initiator: request.initiator,
            reason: request.reason,
            approvals: vec![],
            required_approvals: required_approvals as i32,
            status: AmendmentStatus::Pending,
            signatures: vec![],
            created_at: Utc::now(),
            effective_date: None,
            previous_version: document.version.clone(),
        };

        // Store amendment
        self.store_amendment(&amendment).await?;

        // Notify guardians
        self.notify_guardians_of_amendment(&document, &amendment)
            .await?;

        Ok(amendment)
    }

    /// Evaluate revival conditions
    pub async fn evaluate_revival(
        &self,
        document_id: &str,
        evidence: CurrentEvidence,
    ) -> Result<RevivalDecision, ConsentError> {
        let document = self.get_document(document_id).await?;

        // Only evaluate executed documents
        if document.status != ConsentStatus::Executed {
            return Err(ConsentError::InvalidStatus(
                "Document must be in EXECUTED status for revival evaluation".to_string(),
            ));
        }

        let decision = self
            .revival_evaluator
            .evaluate(&document.revival_conditions, &evidence);

        // Store evaluation
        self.store_revival_evaluation(document_id, &decision)
            .await?;

        // Emit event
        if decision.decision == "APPROVE" {
            self.emit_event(ConsentEvent::RevivalConditionsMet {
                document_id: document_id.to_string(),
                conditions_met: decision.conditions_met.clone(),
            })
            .await?;
        }

        Ok(decision)
    }

    /// Process withdrawal request
    pub async fn process_withdrawal(
        &self,
        document_id: &str,
        request: WithdrawalRequest,
    ) -> Result<WithdrawalResult, ConsentError> {
        let document = self.get_document(document_id).await?;

        let result = match request.withdrawal_type {
            WithdrawalType::PrePreservation => {
                self.process_pre_preservation_withdrawal(&document, &request)
                    .await?
            }
            WithdrawalType::DuringPreservation => {
                self.process_during_preservation_withdrawal(&document, &request)
                    .await?
            }
            WithdrawalType::RevivalRefusal => {
                self.process_revival_refusal(&document, &request).await?
            }
        };

        Ok(result)
    }
}

// ============================================================================
// Validators
// ============================================================================

pub struct ConsentValidator;

impl ConsentValidator {
    pub fn new() -> Self {
        Self
    }

    pub fn validate(&self, document: &ConsentDocument) -> ValidationReport {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();

        // Mental capacity validation
        self.validate_mental_capacity(document, &mut errors);

        // Signature validation
        self.validate_signatures(document, &mut errors);

        // Revival conditions validation
        self.validate_revival_conditions(document, &mut warnings);

        // Calculate scores
        let completeness = self.calculate_completeness(document);
        let compliance = self.calculate_compliance(document, &errors);

        let result = if !errors.is_empty() {
            ValidationResult::Invalid
        } else if completeness < 0.9 {
            ValidationResult::Incomplete
        } else if !warnings.is_empty() {
            ValidationResult::RequiresReview
        } else {
            ValidationResult::Valid
        };

        ValidationReport {
            document_id: document.document_id.clone(),
            validation_date: Utc::now(),
            result,
            errors,
            warnings,
            completeness_score: completeness,
            legal_compliance_score: compliance,
        }
    }

    fn validate_mental_capacity(
        &self,
        document: &ConsentDocument,
        errors: &mut Vec<ValidationError>,
    ) {
        let assessment = &document.subject.mental_capacity_assessment;

        if !assessment.understanding.preservation_process {
            errors.push(ValidationError {
                field: "mental_capacity.understanding.preservation_process".to_string(),
                error_code: "INCOMPLETE_UNDERSTANDING".to_string(),
                message: "Subject must acknowledge understanding of preservation process"
                    .to_string(),
                severity: Severity::Error,
            });
        }

        if assessment.overall_capacity == Capacity::Incapable {
            errors.push(ValidationError {
                field: "mental_capacity.overall_capacity".to_string(),
                error_code: "SUBJECT_INCAPABLE".to_string(),
                message: "Subject assessed as incapable; legal representative required"
                    .to_string(),
                severity: Severity::Error,
            });
        }
    }

    fn validate_signatures(&self, document: &ConsentDocument, errors: &mut Vec<ValidationError>) {
        if document.signatures.subject_signature.is_none() {
            errors.push(ValidationError {
                field: "signatures.subject_signature".to_string(),
                error_code: "MISSING_SUBJECT_SIGNATURE".to_string(),
                message: "Subject signature is required".to_string(),
                severity: Severity::Error,
            });
        }

        if document.signatures.witness_signatures.len() < 2 {
            errors.push(ValidationError {
                field: "signatures.witness_signatures".to_string(),
                error_code: "INSUFFICIENT_WITNESSES".to_string(),
                message: "Minimum 2 witnesses required".to_string(),
                severity: Severity::Error,
            });
        }
    }
}
```

## PostgreSQL Schema

```sql
-- ============================================================================
-- CRYO-CONSENT Database Schema
-- ============================================================================

-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- ============================================================================
-- Enum Types
-- ============================================================================

CREATE TYPE consent_status AS ENUM (
    'DRAFT',
    'PENDING_SIGNATURES',
    'PENDING_NOTARIZATION',
    'ACTIVE',
    'SUSPENDED',
    'WITHDRAWN',
    'EXECUTED',
    'COMPLETED'
);

CREATE TYPE condition_type AS ENUM (
    'MEDICAL',
    'TECHNOLOGY',
    'TIME',
    'SOCIAL',
    'LEGAL',
    'FINANCIAL',
    'CUSTOM'
);

CREATE TYPE condition_status AS ENUM (
    'PENDING',
    'MET',
    'FAILED',
    'WAIVED'
);

CREATE TYPE signer_role AS ENUM (
    'SUBJECT',
    'SPOUSE',
    'CHILD',
    'GUARDIAN',
    'WITNESS',
    'NOTARY',
    'LEGAL_COUNSEL',
    'MEDICAL_ADVISOR',
    'FACILITY_REPRESENTATIVE'
);

CREATE TYPE amendment_status AS ENUM (
    'PENDING',
    'APPROVED',
    'REJECTED',
    'SUPERSEDED'
);

CREATE TYPE withdrawal_type AS ENUM (
    'PRE_PRESERVATION',
    'DURING_PRESERVATION',
    'REVIVAL_REFUSAL'
);

-- ============================================================================
-- Core Tables
-- ============================================================================

-- Consent Documents
CREATE TABLE consent_documents (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) UNIQUE NOT NULL,
    version VARCHAR(20) NOT NULL DEFAULT '1.0.0',
    status consent_status NOT NULL DEFAULT 'DRAFT',

    subject_id VARCHAR(100) NOT NULL,
    identity_ref VARCHAR(200),

    consent_declarations JSONB NOT NULL DEFAULT '{}',
    revival_conditions JSONB NOT NULL DEFAULT '{}',
    family_agreements JSONB NOT NULL DEFAULT '{}',
    financial_arrangements JSONB NOT NULL DEFAULT '{}',
    alternative_directives JSONB NOT NULL DEFAULT '{}',

    effective_date TIMESTAMPTZ,
    expiration_date TIMESTAMPTZ,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    created_by VARCHAR(100),
    updated_by VARCHAR(100)
);

-- Subject Information
CREATE TABLE subject_information (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    full_legal_name VARCHAR(200) NOT NULL,
    date_of_birth DATE NOT NULL,
    place_of_birth VARCHAR(200),
    nationality VARCHAR(100)[],

    personal_info_encrypted BYTEA,
    contact_info_encrypted BYTEA,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Mental Capacity Assessments
CREATE TABLE mental_capacity_assessments (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    assessment_date DATE NOT NULL,
    assessor_name VARCHAR(200) NOT NULL,
    assessor_credentials VARCHAR(200),
    assessor_license VARCHAR(100),

    understanding_preservation BOOLEAN NOT NULL,
    understanding_risks BOOLEAN NOT NULL,
    understanding_no_guarantees BOOLEAN NOT NULL,
    understanding_financial BOOLEAN NOT NULL,
    understanding_family BOOLEAN NOT NULL,

    decision_voluntary BOOLEAN NOT NULL,
    decision_no_coercion BOOLEAN NOT NULL,
    decision_understood_alternatives BOOLEAN NOT NULL,

    overall_capacity VARCHAR(20) NOT NULL,
    notes TEXT,

    assessor_signature BYTEA,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Revival Conditions
CREATE TABLE revival_conditions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,
    condition_id VARCHAR(100) NOT NULL,

    condition_type condition_type NOT NULL,
    description TEXT NOT NULL,
    required BOOLEAN NOT NULL DEFAULT true,
    weight DECIMAL(5,4),

    parameters JSONB NOT NULL DEFAULT '{}',
    verification_method JSONB NOT NULL DEFAULT '{}',

    status condition_status NOT NULL DEFAULT 'PENDING',
    status_updated_at TIMESTAMPTZ,
    status_updated_by VARCHAR(100),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    UNIQUE(document_id, condition_id)
);

-- Signatures
CREATE TABLE signatures (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    signer_id VARCHAR(100) NOT NULL,
    signer_name VARCHAR(200) NOT NULL,
    signer_role signer_role NOT NULL,

    signature_type VARCHAR(20) NOT NULL,
    signature_data BYTEA NOT NULL,
    signature_hash VARCHAR(128),

    signed_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    location VARCHAR(200),
    ip_address INET,

    verification_status VARCHAR(20) NOT NULL DEFAULT 'PENDING',
    verified_at TIMESTAMPTZ,
    verified_by VARCHAR(100),

    UNIQUE(document_id, signer_id, signer_role)
);

-- Notarizations
CREATE TABLE notarizations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    notary_id VARCHAR(100) NOT NULL,
    notary_name VARCHAR(200) NOT NULL,
    notary_license VARCHAR(100) NOT NULL,
    jurisdiction VARCHAR(100) NOT NULL,

    notarization_date TIMESTAMPTZ NOT NULL,
    notarization_location VARCHAR(200),
    seal_number VARCHAR(100),

    expiration_date TIMESTAMPTZ,
    verification_url VARCHAR(500),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Amendments
CREATE TABLE amendments (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,
    amendment_id VARCHAR(100) UNIQUE NOT NULL,

    amendment_number INTEGER NOT NULL,
    amendment_type VARCHAR(50) NOT NULL,
    description TEXT,

    changes JSONB NOT NULL,
    initiator VARCHAR(100) NOT NULL,
    reason TEXT,

    required_approvals INTEGER NOT NULL,
    status amendment_status NOT NULL DEFAULT 'PENDING',

    effective_date TIMESTAMPTZ,
    previous_version VARCHAR(20),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Amendment Approvals
CREATE TABLE amendment_approvals (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    amendment_id VARCHAR(100) REFERENCES amendments(amendment_id) ON DELETE CASCADE,

    approver_id VARCHAR(100) NOT NULL,
    approver_role VARCHAR(50) NOT NULL,
    decision VARCHAR(20) NOT NULL,

    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    notes TEXT,

    signature_id UUID REFERENCES signatures(id),

    UNIQUE(amendment_id, approver_id)
);

-- Withdrawal Requests
CREATE TABLE withdrawal_requests (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    request_id VARCHAR(100) UNIQUE NOT NULL,
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    withdrawal_type withdrawal_type NOT NULL,
    initiator VARCHAR(100) NOT NULL,
    initiator_type VARCHAR(50) NOT NULL,

    reason TEXT,
    status VARCHAR(20) NOT NULL DEFAULT 'PENDING',

    refund_amount DECIMAL(15,2),
    refund_currency VARCHAR(3),

    processed_at TIMESTAMPTZ,
    processed_by VARCHAR(100),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Revival Evaluations
CREATE TABLE revival_evaluations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50) REFERENCES consent_documents(document_id) ON DELETE CASCADE,

    evaluation_date TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    evaluator VARCHAR(100) NOT NULL,

    decision VARCHAR(20) NOT NULL,
    overall_score DECIMAL(5,4),

    conditions_met VARCHAR(100)[],
    conditions_not_met VARCHAR(100)[],
    conditions_pending VARCHAR(100)[],

    evidence JSONB NOT NULL,
    recommendation TEXT,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- ============================================================================
-- Audit Log
-- ============================================================================

CREATE TABLE consent_audit_log (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    document_id VARCHAR(50),
    action VARCHAR(50) NOT NULL,
    actor VARCHAR(100) NOT NULL,

    previous_state JSONB,
    new_state JSONB,
    changes JSONB,

    ip_address INET,
    user_agent TEXT,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- ============================================================================
-- Indexes
-- ============================================================================

CREATE INDEX idx_consent_documents_subject ON consent_documents(subject_id);
CREATE INDEX idx_consent_documents_status ON consent_documents(status);
CREATE INDEX idx_consent_documents_created ON consent_documents(created_at);

CREATE INDEX idx_revival_conditions_document ON revival_conditions(document_id);
CREATE INDEX idx_revival_conditions_status ON revival_conditions(status);

CREATE INDEX idx_signatures_document ON signatures(document_id);
CREATE INDEX idx_signatures_signer ON signatures(signer_id);

CREATE INDEX idx_amendments_document ON amendments(document_id);
CREATE INDEX idx_amendments_status ON amendments(status);

CREATE INDEX idx_audit_log_document ON consent_audit_log(document_id);
CREATE INDEX idx_audit_log_created ON consent_audit_log(created_at);

-- ============================================================================
-- Triggers
-- ============================================================================

-- Update timestamp trigger
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER consent_documents_updated_at
    BEFORE UPDATE ON consent_documents
    FOR EACH ROW EXECUTE FUNCTION update_updated_at();

CREATE TRIGGER amendments_updated_at
    BEFORE UPDATE ON amendments
    FOR EACH ROW EXECUTE FUNCTION update_updated_at();

-- Audit log trigger
CREATE OR REPLACE FUNCTION log_consent_changes()
RETURNS TRIGGER AS $$
BEGIN
    INSERT INTO consent_audit_log (
        document_id, action, actor, previous_state, new_state
    ) VALUES (
        COALESCE(NEW.document_id, OLD.document_id),
        TG_OP,
        current_user,
        row_to_json(OLD),
        row_to_json(NEW)
    );
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER consent_documents_audit
    AFTER INSERT OR UPDATE OR DELETE ON consent_documents
    FOR EACH ROW EXECUTE FUNCTION log_consent_changes();
```

## Docker Deployment

### docker-compose.yml

```yaml
version: '3.8'

services:
  cryo-consent-api:
    build:
      context: .
      dockerfile: Dockerfile
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://consent:consent@postgres:5432/cryo_consent
      - REDIS_URL=redis://redis:6379
      - RUST_LOG=info
      - JWT_SECRET=${JWT_SECRET}
    depends_on:
      - postgres
      - redis
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3

  postgres:
    image: postgres:15-alpine
    environment:
      POSTGRES_USER: consent
      POSTGRES_PASSWORD: consent
      POSTGRES_DB: cryo_consent
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./init.sql:/docker-entrypoint-initdb.d/init.sql
    ports:
      - "5432:5432"

  redis:
    image: redis:7-alpine
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data

  nginx:
    image: nginx:alpine
    ports:
      - "443:443"
      - "80:80"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf:ro
      - ./certs:/etc/nginx/certs:ro
    depends_on:
      - cryo-consent-api

volumes:
  postgres_data:
  redis_data:
```

### Kubernetes Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-consent-api
  namespace: cryo-system
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-consent-api
  template:
    metadata:
      labels:
        app: cryo-consent-api
    spec:
      containers:
        - name: api
          image: wia/cryo-consent-api:1.0.0
          ports:
            - containerPort: 8080
          env:
            - name: DATABASE_URL
              valueFrom:
                secretKeyRef:
                  name: cryo-consent-secrets
                  key: database-url
            - name: JWT_SECRET
              valueFrom:
                secretKeyRef:
                  name: cryo-consent-secrets
                  key: jwt-secret
          resources:
            requests:
              memory: "256Mi"
              cpu: "250m"
            limits:
              memory: "512Mi"
              cpu: "500m"
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 30
            periodSeconds: 10
          readinessProbe:
            httpGet:
              path: /ready
              port: 8080
            initialDelaySeconds: 5
            periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: cryo-consent-api
  namespace: cryo-system
spec:
  selector:
    app: cryo-consent-api
  ports:
    - port: 80
      targetPort: 8080
  type: ClusterIP
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: cryo-consent-ingress
  namespace: cryo-system
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
    - hosts:
        - consent.wia.org
      secretName: cryo-consent-tls
  rules:
    - host: consent.wia.org
      http:
        paths:
          - path: /
            pathType: Prefix
            backend:
              service:
                name: cryo-consent-api
                port:
                  number: 80
```

---

*WIA Technical Committee - Cryopreservation Working Group*
