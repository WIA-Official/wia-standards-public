# CRYO-IDENTITY Phase 3: Protocol Specification

## 1. REST API Specification

### 1.1 OpenAPI Definition

```yaml
openapi: 3.0.3
info:
  title: CRYO-IDENTITY API
  description: Identity management for cryopreserved subjects
  version: 1.0.0
  contact:
    name: WIA Cryopreservation Working Group
    url: https://wia.org/cryo-identity

servers:
  - url: https://api.cryo-identity.wia.org/v1
    description: Production
  - url: https://api.staging.cryo-identity.wia.org/v1
    description: Staging

tags:
  - name: Identity
    description: Core identity operations
  - name: Vault
    description: Identity vault management
  - name: Verification
    description: Identity verification services
  - name: Recovery
    description: Identity recovery operations
  - name: Revival
    description: Revival identity restoration

paths:
  /identities:
    post:
      tags: [Identity]
      summary: Create new identity record
      operationId: createIdentity
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateIdentityRequest'
      responses:
        '201':
          description: Identity created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/IdentityDocument'
        '400':
          $ref: '#/components/responses/BadRequest'
        '409':
          description: Identity already exists
      security:
        - bearerAuth: []
        - facilityAuth: []

  /identities/{identityId}:
    get:
      tags: [Identity]
      summary: Get identity record
      operationId: getIdentity
      parameters:
        - name: identityId
          in: path
          required: true
          schema:
            type: string
        - name: sections
          in: query
          description: Sections to include
          schema:
            type: array
            items:
              type: string
              enum: [legal, personal, digital, biometric]
      responses:
        '200':
          description: Identity record
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/IdentityDocument'
        '403':
          $ref: '#/components/responses/Forbidden'
        '404':
          $ref: '#/components/responses/NotFound'
      security:
        - bearerAuth: []

    patch:
      tags: [Identity]
      summary: Update identity record
      operationId: updateIdentity
      parameters:
        - name: identityId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/UpdateIdentityRequest'
      responses:
        '200':
          description: Identity updated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/IdentityDocument'
        '403':
          $ref: '#/components/responses/Forbidden'
      security:
        - bearerAuth: []
        - guardianAuth: []

  /identities/{identityId}/link-subject:
    post:
      tags: [Identity]
      summary: Link identity to preservation subject
      operationId: linkSubject
      parameters:
        - name: identityId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - subjectId
              properties:
                subjectId:
                  type: string
                  description: CRYO-PRESERVATION subject ID
                linkVerification:
                  type: string
                  description: Verification proof
      responses:
        '200':
          description: Link established
        '409':
          description: Already linked
      security:
        - facilityAuth: []

  /vaults:
    post:
      tags: [Vault]
      summary: Create identity vault
      operationId: createVault
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateVaultRequest'
      responses:
        '201':
          description: Vault created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/IdentityVault'
      security:
        - bearerAuth: []
        - facilityAuth: []

  /vaults/{vaultId}:
    get:
      tags: [Vault]
      summary: Get vault metadata
      operationId: getVault
      parameters:
        - name: vaultId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Vault metadata
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VaultMetadata'
      security:
        - bearerAuth: []

  /vaults/{vaultId}/contents:
    get:
      tags: [Vault]
      summary: Access vault contents
      operationId: getVaultContents
      parameters:
        - name: vaultId
          in: path
          required: true
          schema:
            type: string
        - name: section
          in: query
          schema:
            type: string
      responses:
        '200':
          description: Vault contents
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VaultContents'
      security:
        - vaultAccessAuth: []

  /vaults/{vaultId}/integrity:
    get:
      tags: [Vault]
      summary: Verify vault integrity
      operationId: verifyVaultIntegrity
      parameters:
        - name: vaultId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Integrity report
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/IntegrityReport'
      security:
        - bearerAuth: []

  /verification/biometric:
    post:
      tags: [Verification]
      summary: Perform biometric verification
      operationId: verifyBiometric
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/BiometricVerificationRequest'
      responses:
        '200':
          description: Verification result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/BiometricVerificationResult'
      security:
        - bearerAuth: []
        - facilityAuth: []

  /verification/continuity-score:
    post:
      tags: [Verification]
      summary: Calculate identity continuity score
      operationId: calculateContinuityScore
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - identityId
              properties:
                identityId:
                  type: string
      responses:
        '200':
          description: Continuity score
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ContinuityScore'
      security:
        - bearerAuth: []

  /recovery/initiate:
    post:
      tags: [Recovery]
      summary: Initiate identity recovery
      operationId: initiateRecovery
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RecoveryInitiateRequest'
      responses:
        '201':
          description: Recovery initiated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RecoveryRequest'
      security:
        - bearerAuth: []

  /recovery/{requestId}:
    get:
      tags: [Recovery]
      summary: Get recovery request status
      operationId: getRecoveryStatus
      parameters:
        - name: requestId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Recovery status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RecoveryRequest'
      security:
        - bearerAuth: []

  /recovery/{requestId}/decide:
    post:
      tags: [Recovery]
      summary: Submit guardian decision
      operationId: submitGuardianDecision
      parameters:
        - name: requestId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/GuardianDecision'
      responses:
        '200':
          description: Decision recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RecoveryRequest'
      security:
        - guardianAuth: []

  /revival/{subjectId}/prepare:
    post:
      tags: [Revival]
      summary: Prepare identity for revival
      operationId: prepareRevivalIdentity
      parameters:
        - name: subjectId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RevivalPreparationRequest'
      responses:
        '200':
          description: Preparation complete
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalIdentityPackage'
      security:
        - facilityAuth: []
        - revivalAuth: []

  /revival/{subjectId}/restore:
    post:
      tags: [Revival]
      summary: Execute identity restoration step
      operationId: executeRestorationStep
      parameters:
        - name: subjectId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RestorationStepRequest'
      responses:
        '200':
          description: Step executed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RestorationStepResult'
      security:
        - facilityAuth: []
        - revivalAuth: []

components:
  schemas:
    CreateIdentityRequest:
      type: object
      required:
        - legalIdentity
      properties:
        subjectId:
          type: string
          description: Link to preservation subject
        legalIdentity:
          $ref: '#/components/schemas/LegalIdentity'
        personalIdentity:
          $ref: '#/components/schemas/PersonalIdentity'
        digitalIdentity:
          $ref: '#/components/schemas/DigitalIdentity'
        biometricIdentity:
          $ref: '#/components/schemas/BiometricIdentity'

    IdentityDocument:
      type: object
      properties:
        id:
          type: string
        identityId:
          type: string
        subjectId:
          type: string
        status:
          type: string
          enum: [ACTIVE, PRESERVED, SUSPENDED, REVIVED]
        legalIdentity:
          $ref: '#/components/schemas/LegalIdentity'
        personalIdentity:
          $ref: '#/components/schemas/PersonalIdentity'
        digitalIdentity:
          $ref: '#/components/schemas/DigitalIdentity'
        biometricIdentity:
          $ref: '#/components/schemas/BiometricIdentity'
        createdAt:
          type: string
          format: date-time
        updatedAt:
          type: string
          format: date-time

    LegalIdentity:
      type: object
      properties:
        legalName:
          type: object
          properties:
            givenName:
              type: string
            familyName:
              type: string
        birthRecord:
          type: object
        citizenship:
          type: array
          items:
            type: object
        governmentIds:
          type: array
          items:
            type: object

    PersonalIdentity:
      type: object
      properties:
        lifeTimeline:
          type: array
          items:
            type: object
        education:
          type: array
          items:
            type: object
        career:
          type: array
          items:
            type: object

    DigitalIdentity:
      type: object
      properties:
        dids:
          type: object
        credentialWallet:
          type: object
        onlineAccounts:
          type: array
          items:
            type: object

    BiometricIdentity:
      type: object
      properties:
        dnaProfile:
          type: object
        fingerprints:
          type: array
          items:
            type: object
        facialGeometry:
          type: object

    IdentityVault:
      type: object
      properties:
        vaultId:
          type: string
        identityId:
          type: string
        configuration:
          type: object
        status:
          type: string

    VaultMetadata:
      type: object
      properties:
        vaultId:
          type: string
        createdAt:
          type: string
          format: date-time
        lastAccessed:
          type: string
          format: date-time
        integrityStatus:
          type: string

    VaultContents:
      type: object
      properties:
        section:
          type: string
        encryptedData:
          type: string
        integrityHash:
          type: string

    IntegrityReport:
      type: object
      properties:
        vaultId:
          type: string
        timestamp:
          type: string
          format: date-time
        overallStatus:
          type: string
          enum: [VERIFIED, COMPROMISED]
        issues:
          type: array
          items:
            type: string

    BiometricVerificationRequest:
      type: object
      required:
        - identityId
        - biometricType
        - sampleData
      properties:
        identityId:
          type: string
        biometricType:
          type: string
          enum: [DNA, FINGERPRINT, FACIAL, IRIS]
        sampleData:
          type: string
          format: base64

    BiometricVerificationResult:
      type: object
      properties:
        verified:
          type: boolean
        confidence:
          type: number
        biometricType:
          type: string
        timestamp:
          type: string
          format: date-time

    ContinuityScore:
      type: object
      properties:
        overallScore:
          type: number
        componentScores:
          type: object
        riskFactors:
          type: array
          items:
            type: string
        recommendations:
          type: array
          items:
            type: string

    RecoveryInitiateRequest:
      type: object
      required:
        - identityId
        - reason
      properties:
        identityId:
          type: string
        reason:
          type: string
        requesterVerification:
          type: string

    RecoveryRequest:
      type: object
      properties:
        requestId:
          type: string
        identityId:
          type: string
        status:
          type: string
          enum: [PENDING, APPROVED, REJECTED, EXPIRED]
        threshold:
          type: integer
        approvals:
          type: array
          items:
            type: object
        expiresAt:
          type: string
          format: date-time

    GuardianDecision:
      type: object
      required:
        - decision
        - verificationProof
      properties:
        decision:
          type: string
          enum: [APPROVE, REJECT]
        verificationProof:
          type: string
        notes:
          type: string

    RevivalPreparationRequest:
      type: object
      properties:
        revivalContext:
          type: object
        prioritySections:
          type: array
          items:
            type: string

    RevivalIdentityPackage:
      type: object
      properties:
        packageId:
          type: string
        subjectId:
          type: string
        preparationStatus:
          type: string
        restorationTimeline:
          type: array
          items:
            type: object

    RestorationStepRequest:
      type: object
      required:
        - stepId
      properties:
        stepId:
          type: string
        verificationData:
          type: object

    RestorationStepResult:
      type: object
      properties:
        stepId:
          type: string
        status:
          type: string
        outcome:
          type: object

    UpdateIdentityRequest:
      type: object
      properties:
        section:
          type: string
        updates:
          type: object
        reason:
          type: string

  responses:
    BadRequest:
      description: Invalid request
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string
              details:
                type: array
                items:
                  type: string

    Forbidden:
      description: Access denied
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string

    NotFound:
      description: Resource not found
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string

  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT

    facilityAuth:
      type: http
      scheme: bearer
      description: Facility-issued credentials

    guardianAuth:
      type: http
      scheme: bearer
      description: Guardian verification token

    vaultAccessAuth:
      type: http
      scheme: bearer
      description: Vault access credentials

    revivalAuth:
      type: http
      scheme: bearer
      description: Revival team authorization
```

## 2. WebSocket Protocol

### 2.1 Real-Time Notifications

```yaml
WebSocket Protocol: CRYO-IDENTITY Notifications

Endpoint: wss://api.cryo-identity.wia.org/v1/ws

Authentication:
  Method: Query parameter or first message
  Format:
    ?token=<JWT>
    or
    {"type": "auth", "token": "<JWT>"}

Subscription Channels:
  - identity.{identityId}.updates
  - vault.{vaultId}.access
  - recovery.{requestId}.status
  - guardians.{guardianId}.requests

Message Types:

  # Client -> Server
  Subscribe:
    type: "subscribe"
    channel: "identity.CRYOID-2025-001.updates"

  Unsubscribe:
    type: "unsubscribe"
    channel: "identity.CRYOID-2025-001.updates"

  # Server -> Client
  IdentityUpdate:
    type: "identity.update"
    identityId: "CRYOID-2025-001"
    section: "legalIdentity"
    timestamp: "2025-03-15T10:00:00Z"
    updatedBy: "guardian-001"
    changeType: "MODIFICATION"

  VaultAccess:
    type: "vault.access"
    vaultId: "VAULT-001"
    accessorId: "facility-001"
    accessType: "READ"
    section: "biometricIdentity"
    timestamp: "2025-03-15T10:00:00Z"

  RecoveryRequest:
    type: "recovery.request"
    requestId: "RECOVERY-001"
    identityId: "CRYOID-2025-001"
    status: "PENDING"
    requester: "family-member-001"
    expiresAt: "2025-03-22T10:00:00Z"

  RecoveryDecision:
    type: "recovery.decision"
    requestId: "RECOVERY-001"
    guardianId: "guardian-001"
    decision: "APPROVE"
    approvalsCount: 2
    threshold: 3

  IntegrityAlert:
    type: "vault.integrity.alert"
    vaultId: "VAULT-001"
    severity: "WARNING"
    issue: "Hash mismatch detected"
    timestamp: "2025-03-15T10:00:00Z"
```

## 3. Inter-Service Communication

### 3.1 Event-Driven Architecture

```yaml
Event Bus: CRYO-IDENTITY Events

Topics:
  cryo-identity.identity.created:
    schema:
      identityId: string
      subjectId: string
      createdAt: datetime
      facility: string

  cryo-identity.identity.linked:
    schema:
      identityId: string
      subjectId: string
      linkedAt: datetime

  cryo-identity.vault.created:
    schema:
      vaultId: string
      identityId: string
      locations: array

  cryo-identity.vault.accessed:
    schema:
      vaultId: string
      accessorId: string
      accessType: string
      section: string

  cryo-identity.recovery.initiated:
    schema:
      requestId: string
      identityId: string
      requester: string
      guardians: array

  cryo-identity.recovery.completed:
    schema:
      requestId: string
      outcome: string  # APPROVED/REJECTED
      completedAt: datetime

  cryo-identity.revival.preparation.started:
    schema:
      subjectId: string
      identityId: string
      facility: string

  cryo-identity.revival.restored:
    schema:
      subjectId: string
      identityId: string
      restoredAt: datetime
      verificationMethod: string

Integration with CRYO-PRESERVATION:
  Subscribe to:
    - cryo-preservation.subject.created
    - cryo-preservation.subject.status.changed
    - cryo-preservation.revival.initiated
    - cryo-preservation.revival.completed

  Publish on:
    - cryo-identity.identity.created (after subject creation)
    - cryo-identity.revival.ready (when identity prepared)
    - cryo-identity.revival.restored (after restoration)
```

---
*CRYO-IDENTITY Phase 3 Specification v1.0.0*
