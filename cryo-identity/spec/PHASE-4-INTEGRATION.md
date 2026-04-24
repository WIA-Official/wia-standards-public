# CRYO-IDENTITY Phase 4: Integration Specification

## 1. System Architecture

### 1.1 High-Level Architecture

```
+------------------------------------------------------------------+
|                     CRYO-IDENTITY SYSTEM                         |
+------------------------------------------------------------------+
|                                                                  |
|  +------------------+  +------------------+  +------------------+|
|  | Identity Service |  | Vault Service    |  | Recovery Service ||
|  |                  |  |                  |  |                  ||
|  | - Create/Update  |  | - Encrypt/Store  |  | - Guardian Mgmt  ||
|  | - Link Subjects  |  | - Verify Integ.  |  | - Threshold Auth ||
|  | - Query/Search   |  | - Replicate      |  | - Execute Recov. ||
|  +--------+---------+  +--------+---------+  +--------+---------+|
|           |                     |                     |          |
|           +----------+----------+----------+----------+          |
|                      |                     |                     |
|              +-------v-------+     +-------v-------+             |
|              | Event Bus     |     | API Gateway   |             |
|              | (Kafka/NATS)  |     | (REST/WS)     |             |
|              +-------+-------+     +-------+-------+             |
|                      |                     |                     |
+----------------------|---------------------|---------------------+
                       |                     |
        +--------------+-------+     +-------+--------------+
        |                      |     |                      |
+-------v-------+      +-------v-----v-------+      +-------v-------+
| CRYO-         |      | External Services   |      | Revival       |
| PRESERVATION  |      |                     |      | Systems       |
|               |      | - Legal Registries  |      |               |
| - Subject Mgmt|      | - Biometric DBs     |      | - ID Restore  |
| - Monitoring  |      | - Credential Verify |      | - Reintegrate |
+---------------+      +---------------------+      +---------------+
```

### 1.2 Identity Service Implementation

```rust
// Rust implementation for Identity Service

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdentityService {
    db_pool: DatabasePool,
    event_publisher: EventPublisher,
    vault_service: VaultServiceClient,
}

impl IdentityService {
    pub async fn create_identity(
        &self,
        request: CreateIdentityRequest,
    ) -> Result<IdentityDocument, IdentityError> {
        // Generate identity ID
        let identity_id = self.generate_identity_id(&request);

        // Validate legal identity requirements
        self.validate_legal_identity(&request.legal_identity)?;

        // Create identity document
        let mut identity = IdentityDocument {
            id: Uuid::new_v4().to_string(),
            identity_id: identity_id.clone(),
            subject_id: request.subject_id.clone(),
            status: IdentityStatus::Active,
            legal_identity: request.legal_identity,
            personal_identity: request.personal_identity,
            digital_identity: request.digital_identity,
            biometric_identity: request.biometric_identity,
            created_at: Utc::now(),
            updated_at: Utc::now(),
        };

        // Store in database
        self.db_pool.insert_identity(&identity).await?;

        // Create vault if biometric data present
        if identity.biometric_identity.is_some() {
            let vault = self.vault_service
                .create_vault(&identity_id, &identity.biometric_identity)
                .await?;
            identity.vault_id = Some(vault.vault_id);
        }

        // Publish creation event
        self.event_publisher.publish(IdentityEvent::Created {
            identity_id: identity_id.clone(),
            subject_id: request.subject_id,
            timestamp: Utc::now(),
        }).await?;

        Ok(identity)
    }

    pub async fn link_to_subject(
        &self,
        identity_id: &str,
        subject_id: &str,
        verification: &str,
    ) -> Result<(), IdentityError> {
        // Verify identity exists
        let identity = self.db_pool.get_identity(identity_id).await?
            .ok_or(IdentityError::NotFound)?;

        // Verify subject exists in CRYO-PRESERVATION
        self.verify_subject_exists(subject_id).await?;

        // Verify linking authorization
        self.verify_link_authorization(verification).await?;

        // Update identity with subject link
        self.db_pool.update_identity_subject(identity_id, subject_id).await?;

        // Publish linking event
        self.event_publisher.publish(IdentityEvent::Linked {
            identity_id: identity_id.to_string(),
            subject_id: subject_id.to_string(),
            timestamp: Utc::now(),
        }).await?;

        Ok(())
    }

    pub async fn update_status(
        &self,
        identity_id: &str,
        new_status: IdentityStatus,
        reason: &str,
    ) -> Result<(), IdentityError> {
        let identity = self.db_pool.get_identity(identity_id).await?
            .ok_or(IdentityError::NotFound)?;

        // Validate status transition
        self.validate_status_transition(&identity.status, &new_status)?;

        // Update status
        self.db_pool.update_identity_status(identity_id, &new_status).await?;

        // Publish status change event
        self.event_publisher.publish(IdentityEvent::StatusChanged {
            identity_id: identity_id.to_string(),
            old_status: identity.status,
            new_status: new_status.clone(),
            reason: reason.to_string(),
            timestamp: Utc::now(),
        }).await?;

        Ok(())
    }

    fn generate_identity_id(&self, request: &CreateIdentityRequest) -> String {
        let year = Utc::now().format("%Y");
        let facility = request.facility_id.as_deref().unwrap_or("000");
        let sequence = self.get_next_sequence();
        format!("CRYOID-{}-{}-{:05}", year, facility, sequence)
    }

    fn validate_legal_identity(
        &self,
        legal: &LegalIdentity,
    ) -> Result<(), IdentityError> {
        // Require legal name
        if legal.legal_name.given_name.is_empty() ||
           legal.legal_name.family_name.is_empty() {
            return Err(IdentityError::ValidationFailed(
                "Legal name is required".to_string()
            ));
        }

        // Require birth record
        if legal.birth_record.is_none() {
            return Err(IdentityError::ValidationFailed(
                "Birth record is required".to_string()
            ));
        }

        Ok(())
    }

    fn validate_status_transition(
        &self,
        current: &IdentityStatus,
        new: &IdentityStatus,
    ) -> Result<(), IdentityError> {
        let valid = match (current, new) {
            (IdentityStatus::Active, IdentityStatus::Preserved) => true,
            (IdentityStatus::Preserved, IdentityStatus::Active) => false,
            (IdentityStatus::Preserved, IdentityStatus::Revived) => true,
            (IdentityStatus::Revived, IdentityStatus::Active) => true,
            _ => false,
        };

        if valid {
            Ok(())
        } else {
            Err(IdentityError::InvalidTransition)
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IdentityStatus {
    Active,
    Preserved,
    Suspended,
    Revived,
}
```

## 2. Database Schema

### 2.1 PostgreSQL Schema

```sql
-- CRYO-IDENTITY Database Schema

-- Enable required extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Identity Documents Table
CREATE TABLE identity_documents (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) UNIQUE NOT NULL,
    subject_id VARCHAR(50),
    status VARCHAR(20) NOT NULL DEFAULT 'ACTIVE',
    vault_id UUID,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_status CHECK (
        status IN ('ACTIVE', 'PRESERVED', 'SUSPENDED', 'REVIVED', 'TERMINATED')
    )
);

CREATE INDEX idx_identity_subject ON identity_documents(subject_id);
CREATE INDEX idx_identity_status ON identity_documents(status);

-- Legal Identity Table
CREATE TABLE legal_identities (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    -- Name
    given_name VARCHAR(100) NOT NULL,
    middle_name VARCHAR(100),
    family_name VARCHAR(100) NOT NULL,
    name_prefix VARCHAR(20),
    name_suffix VARCHAR(20),

    -- Birth Record
    date_of_birth DATE NOT NULL,
    place_of_birth_country CHAR(3),
    place_of_birth_region VARCHAR(100),
    place_of_birth_city VARCHAR(100),
    birth_certificate_ref VARCHAR(100),

    -- Marital Status
    marital_status VARCHAR(20),
    spouse_name VARCHAR(200),
    marriage_date DATE,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_legal_identity ON legal_identities(identity_id);

-- Government IDs Table
CREATE TABLE government_ids (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    id_type VARCHAR(30) NOT NULL,
    country CHAR(3) NOT NULL,
    number_encrypted BYTEA NOT NULL,  -- Encrypted
    issue_date DATE,
    expiry_date DATE,
    issuer VARCHAR(200),
    document_hash VARCHAR(64),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_govid_identity ON government_ids(identity_id);

-- Citizenship Table
CREATE TABLE citizenships (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    country CHAR(3) NOT NULL,
    citizenship_type VARCHAR(20) NOT NULL,
    document_number_encrypted BYTEA,
    issue_date DATE,
    expiry_date DATE,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_citizenship_identity ON citizenships(identity_id);

-- Family Relationships Table
CREATE TABLE family_relationships (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    relationship_type VARCHAR(20) NOT NULL,
    related_person_name VARCHAR(200) NOT NULL,
    related_person_cryo_id VARCHAR(50),  -- If also preserved
    related_person_dob DATE,
    is_biological BOOLEAN NOT NULL DEFAULT true,
    is_legal BOOLEAN NOT NULL DEFAULT true,
    verification_status VARCHAR(20) NOT NULL DEFAULT 'CLAIMED',
    person_status VARCHAR(20) NOT NULL DEFAULT 'UNKNOWN',

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_family_identity ON family_relationships(identity_id);

-- Personal Identity Table
CREATE TABLE personal_identities (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    values_beliefs JSONB,
    preferences JSONB,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Life Timeline Events
CREATE TABLE life_events (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    event_type VARCHAR(30) NOT NULL,
    event_date DATE,
    description TEXT NOT NULL,
    location VARCHAR(200),
    significance VARCHAR(20) NOT NULL DEFAULT 'MEDIUM',
    attachments JSONB,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_events_identity ON life_events(identity_id);

-- Skills and Expertise
CREATE TABLE skills (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    skill_name VARCHAR(100) NOT NULL,
    category VARCHAR(30) NOT NULL,
    proficiency_level SMALLINT NOT NULL CHECK (proficiency_level BETWEEN 1 AND 5),
    years_experience NUMERIC(4,1),
    certifications TEXT[],
    last_practiced DATE,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Education Records
CREATE TABLE education_records (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    institution VARCHAR(200) NOT NULL,
    degree VARCHAR(100),
    field_of_study VARCHAR(100),
    start_date DATE,
    end_date DATE,
    status VARCHAR(20) NOT NULL,
    gpa NUMERIC(3,2),
    achievements TEXT[],

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Career Records
CREATE TABLE career_records (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    employer VARCHAR(200) NOT NULL,
    position VARCHAR(100) NOT NULL,
    industry VARCHAR(100),
    start_date DATE,
    end_date DATE,
    responsibilities TEXT[],
    achievements TEXT[],

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Digital Identity Table
CREATE TABLE digital_identities (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    primary_did VARCHAR(200),
    secondary_dids TEXT[],
    recovery_dids TEXT[],
    wallet_config JSONB,
    communication_channels JSONB,
    cryptographic_keys JSONB,  -- Public keys only

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Online Accounts
CREATE TABLE online_accounts (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    platform VARCHAR(100) NOT NULL,
    username VARCHAR(200),
    email_encrypted BYTEA,
    url VARCHAR(500),
    account_type VARCHAR(30) NOT NULL,
    status VARCHAR(20) NOT NULL DEFAULT 'ACTIVE',
    created_date DATE,
    last_access DATE,
    recovery_options TEXT[],
    credential_vault_ref VARCHAR(100),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Digital Assets
CREATE TABLE digital_assets (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    asset_type VARCHAR(30) NOT NULL,
    asset_name VARCHAR(200) NOT NULL,
    description TEXT,
    platform VARCHAR(100),
    wallet_address VARCHAR(200),
    contract_address VARCHAR(200),
    token_id VARCHAR(100),
    quantity VARCHAR(50),
    estimated_value JSONB,
    access_method TEXT,
    legal_ownership VARCHAR(20) NOT NULL DEFAULT 'SOLE',

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Identity Vaults Table
CREATE TABLE identity_vaults (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vault_id VARCHAR(50) UNIQUE NOT NULL,
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    encryption_scheme VARCHAR(30) NOT NULL,
    key_management VARCHAR(30) NOT NULL,
    redundancy_level SMALLINT NOT NULL DEFAULT 3,
    access_policy JSONB NOT NULL,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Vault Storage Locations
CREATE TABLE vault_storage_locations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vault_id VARCHAR(50) NOT NULL REFERENCES identity_vaults(vault_id),

    location_type VARCHAR(20) NOT NULL,
    provider VARCHAR(100) NOT NULL,
    region VARCHAR(100),
    jurisdiction VARCHAR(100),
    status VARCHAR(20) NOT NULL DEFAULT 'ACTIVE',
    last_sync TIMESTAMPTZ,
    integrity_verified TIMESTAMPTZ,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Vault Access Log
CREATE TABLE vault_access_log (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    vault_id VARCHAR(50) NOT NULL,

    accessor_id VARCHAR(100) NOT NULL,
    accessor_type VARCHAR(30) NOT NULL,
    access_type VARCHAR(20) NOT NULL,
    section_accessed VARCHAR(50),
    access_granted BOOLEAN NOT NULL,
    denial_reason VARCHAR(200),

    accessed_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_vault_access_log ON vault_access_log(vault_id, accessed_at);

-- Guardians Table
CREATE TABLE guardians (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    guardian_id VARCHAR(50) UNIQUE NOT NULL,
    guardian_type VARCHAR(30) NOT NULL,
    name VARCHAR(200) NOT NULL,
    contact_info_encrypted BYTEA NOT NULL,
    threshold_weight SMALLINT NOT NULL DEFAULT 1,
    relationship_to_subject VARCHAR(100),
    status VARCHAR(20) NOT NULL DEFAULT 'ACTIVE',

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Recovery Requests Table
CREATE TABLE recovery_requests (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    request_id VARCHAR(50) UNIQUE NOT NULL,
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    requester VARCHAR(100) NOT NULL,
    reason TEXT NOT NULL,
    status VARCHAR(20) NOT NULL DEFAULT 'PENDING',
    threshold_required SMALLINT NOT NULL,
    initiated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    expires_at TIMESTAMPTZ NOT NULL,
    completed_at TIMESTAMPTZ
);

-- Guardian Decisions
CREATE TABLE guardian_decisions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    request_id VARCHAR(50) NOT NULL REFERENCES recovery_requests(request_id),
    guardian_id VARCHAR(50) NOT NULL REFERENCES guardians(guardian_id),

    decision VARCHAR(20) NOT NULL,
    verification_proof TEXT NOT NULL,
    notes TEXT,

    decided_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Identity Update History
CREATE TABLE identity_update_history (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identity_id VARCHAR(50) NOT NULL REFERENCES identity_documents(identity_id),

    update_type VARCHAR(30) NOT NULL,
    section_updated VARCHAR(50) NOT NULL,
    changes JSONB NOT NULL,
    reason TEXT,
    authorized_by VARCHAR(100) NOT NULL,
    authorizer_type VARCHAR(30) NOT NULL,

    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_update_history ON identity_update_history(identity_id, updated_at);

-- Create update trigger for timestamps
CREATE OR REPLACE FUNCTION update_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_identity_timestamp
    BEFORE UPDATE ON identity_documents
    FOR EACH ROW EXECUTE FUNCTION update_timestamp();

CREATE TRIGGER update_legal_identity_timestamp
    BEFORE UPDATE ON legal_identities
    FOR EACH ROW EXECUTE FUNCTION update_timestamp();

CREATE TRIGGER update_vault_timestamp
    BEFORE UPDATE ON identity_vaults
    FOR EACH ROW EXECUTE FUNCTION update_timestamp();
```

## 3. Deployment Configuration

### 3.1 Docker Compose

```yaml
version: '3.8'

services:
  cryo-identity-api:
    image: wia/cryo-identity-api:latest
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://user:pass@db:5432/cryo_identity
      - REDIS_URL=redis://redis:6379
      - KAFKA_BROKERS=kafka:9092
      - VAULT_ADDR=http://vault:8200
      - JWT_SECRET=${JWT_SECRET}
    depends_on:
      - db
      - redis
      - kafka
      - vault
    deploy:
      replicas: 3
      resources:
        limits:
          cpus: '2'
          memory: 4G

  cryo-identity-worker:
    image: wia/cryo-identity-worker:latest
    environment:
      - DATABASE_URL=postgres://user:pass@db:5432/cryo_identity
      - KAFKA_BROKERS=kafka:9092
    depends_on:
      - db
      - kafka
    deploy:
      replicas: 2

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=cryo_identity
      - POSTGRES_USER=user
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - cryo_identity_db:/var/lib/postgresql/data
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 8G

  redis:
    image: redis:7-alpine
    volumes:
      - cryo_identity_redis:/data

  vault:
    image: hashicorp/vault:latest
    cap_add:
      - IPC_LOCK
    environment:
      - VAULT_DEV_ROOT_TOKEN_ID=${VAULT_TOKEN}
    volumes:
      - cryo_identity_vault:/vault/data

volumes:
  cryo_identity_db:
  cryo_identity_redis:
  cryo_identity_vault:
```

### 3.2 Kubernetes Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-identity-api
  namespace: cryo-systems
spec:
  replicas: 3
  selector:
    matchLabels:
      app: cryo-identity-api
  template:
    metadata:
      labels:
        app: cryo-identity-api
    spec:
      containers:
      - name: api
        image: wia/cryo-identity-api:latest
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: cryo-identity-secrets
              key: database-url
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
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
  name: cryo-identity-api
  namespace: cryo-systems
spec:
  selector:
    app: cryo-identity-api
  ports:
  - port: 80
    targetPort: 8080
  type: ClusterIP
```

---
*CRYO-IDENTITY Phase 4 Specification v1.0.0*
