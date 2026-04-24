# CRYO-CONSENT Phase 3: Protocol Specification

## Overview

This document defines the API protocols for consent management, including REST endpoints, webhooks, and integration interfaces.

## REST API Specification

### OpenAPI 3.0

```yaml
openapi: 3.0.3
info:
  title: WIA Cryo-Consent API
  description: API for managing cryopreservation consent documents
  version: 1.0.0
  contact:
    name: WIA Technical Committee
    url: https://wia.org/cryo-consent

servers:
  - url: https://api.wia.org/cryo-consent/v1
    description: Production server
  - url: https://sandbox.wia.org/cryo-consent/v1
    description: Sandbox server

security:
  - BearerAuth: []
  - OAuth2: [consent:read, consent:write]

paths:
  /documents:
    post:
      summary: Create new consent document
      operationId: createDocument
      tags:
        - Documents
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateDocumentRequest'
      responses:
        '201':
          description: Document created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ConsentDocument'
        '400':
          $ref: '#/components/responses/BadRequest'
        '409':
          description: Document already exists for subject

    get:
      summary: List consent documents
      operationId: listDocuments
      tags:
        - Documents
      parameters:
        - name: subject_id
          in: query
          schema:
            type: string
        - name: status
          in: query
          schema:
            $ref: '#/components/schemas/ConsentStatus'
        - name: page
          in: query
          schema:
            type: integer
            default: 1
        - name: limit
          in: query
          schema:
            type: integer
            default: 20
      responses:
        '200':
          description: List of documents
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DocumentList'

  /documents/{documentId}:
    get:
      summary: Get consent document
      operationId: getDocument
      tags:
        - Documents
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      responses:
        '200':
          description: Document details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ConsentDocument'
        '404':
          $ref: '#/components/responses/NotFound'

    patch:
      summary: Update consent document
      operationId: updateDocument
      tags:
        - Documents
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/UpdateDocumentRequest'
      responses:
        '200':
          description: Document updated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ConsentDocument'

  /documents/{documentId}/validate:
    post:
      summary: Validate consent document
      operationId: validateDocument
      tags:
        - Validation
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      responses:
        '200':
          description: Validation result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ValidationReport'

  /documents/{documentId}/signatures:
    post:
      summary: Add signature to document
      operationId: addSignature
      tags:
        - Signatures
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SignatureRequest'
      responses:
        '200':
          description: Signature added
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SignatureRecord'
        '400':
          description: Invalid signature or document not ready

    get:
      summary: Get all signatures
      operationId: getSignatures
      tags:
        - Signatures
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      responses:
        '200':
          description: Signature list
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SignatureBlock'

  /documents/{documentId}/amendments:
    post:
      summary: Create amendment
      operationId: createAmendment
      tags:
        - Amendments
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateAmendmentRequest'
      responses:
        '201':
          description: Amendment created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Amendment'

    get:
      summary: List amendments
      operationId: listAmendments
      tags:
        - Amendments
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      responses:
        '200':
          description: Amendment list
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/Amendment'

  /documents/{documentId}/amendments/{amendmentId}/approve:
    post:
      summary: Approve amendment
      operationId: approveAmendment
      tags:
        - Amendments
      parameters:
        - $ref: '#/components/parameters/DocumentId'
        - $ref: '#/components/parameters/AmendmentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/AmendmentApprovalRequest'
      responses:
        '200':
          description: Approval recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Amendment'

  /documents/{documentId}/revival-evaluation:
    post:
      summary: Evaluate revival conditions
      operationId: evaluateRevival
      tags:
        - Revival
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RevivalEvaluationRequest'
      responses:
        '200':
          description: Evaluation result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalDecision'

  /documents/{documentId}/withdrawal:
    post:
      summary: Request withdrawal
      operationId: requestWithdrawal
      tags:
        - Withdrawal
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/WithdrawalRequest'
      responses:
        '200':
          description: Withdrawal request processed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/WithdrawalResult'

  /documents/{documentId}/conflicts:
    get:
      summary: Detect conflicts
      operationId: detectConflicts
      tags:
        - Validation
      parameters:
        - $ref: '#/components/parameters/DocumentId'
      responses:
        '200':
          description: Conflict analysis
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/Conflict'

  /templates:
    get:
      summary: List consent templates
      operationId: listTemplates
      tags:
        - Templates
      parameters:
        - name: jurisdiction
          in: query
          schema:
            type: string
        - name: language
          in: query
          schema:
            type: string
      responses:
        '200':
          description: Template list
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/ConsentTemplate'

  /templates/{templateId}:
    get:
      summary: Get template
      operationId: getTemplate
      tags:
        - Templates
      parameters:
        - name: templateId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Template details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ConsentTemplate'

components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
    OAuth2:
      type: oauth2
      flows:
        authorizationCode:
          authorizationUrl: https://auth.wia.org/oauth/authorize
          tokenUrl: https://auth.wia.org/oauth/token
          scopes:
            consent:read: Read consent documents
            consent:write: Modify consent documents
            consent:sign: Sign consent documents
            consent:admin: Administrative access

  parameters:
    DocumentId:
      name: documentId
      in: path
      required: true
      schema:
        type: string
        pattern: '^CONSENT-[0-9]{4}-[0-9]{3}$'
    AmendmentId:
      name: amendmentId
      in: path
      required: true
      schema:
        type: string
        pattern: '^AMD-CONSENT-[0-9]{4}-[0-9]{3}-[0-9]{3}$'

  schemas:
    ConsentStatus:
      type: string
      enum:
        - DRAFT
        - PENDING_SIGNATURES
        - PENDING_NOTARIZATION
        - ACTIVE
        - SUSPENDED
        - WITHDRAWN
        - EXECUTED
        - COMPLETED

    CreateDocumentRequest:
      type: object
      required:
        - subject_id
        - template_id
      properties:
        subject_id:
          type: string
        template_id:
          type: string
        initial_data:
          type: object

    UpdateDocumentRequest:
      type: object
      properties:
        consent_declarations:
          type: object
        revival_conditions:
          type: object
        family_agreements:
          type: object
        financial_arrangements:
          type: object

    ConsentDocument:
      type: object
      properties:
        document_id:
          type: string
        version:
          type: string
        status:
          $ref: '#/components/schemas/ConsentStatus'
        subject:
          $ref: '#/components/schemas/SubjectInformation'
        consent_declarations:
          type: object
        revival_conditions:
          type: object
        family_agreements:
          type: object
        financial_arrangements:
          type: object
        signatures:
          $ref: '#/components/schemas/SignatureBlock'
        amendments:
          type: array
          items:
            $ref: '#/components/schemas/Amendment'
        created_at:
          type: string
          format: date-time
        last_modified:
          type: string
          format: date-time

    DocumentList:
      type: object
      properties:
        documents:
          type: array
          items:
            $ref: '#/components/schemas/ConsentDocument'
        pagination:
          $ref: '#/components/schemas/Pagination'

    Pagination:
      type: object
      properties:
        total:
          type: integer
        page:
          type: integer
        limit:
          type: integer
        has_next:
          type: boolean

    ValidationReport:
      type: object
      properties:
        document_id:
          type: string
        validation_date:
          type: string
          format: date-time
        result:
          type: string
          enum: [valid, invalid, incomplete, requires_review]
        errors:
          type: array
          items:
            $ref: '#/components/schemas/ValidationError'
        warnings:
          type: array
          items:
            $ref: '#/components/schemas/ValidationError'
        completeness_score:
          type: number
        legal_compliance_score:
          type: number

    ValidationError:
      type: object
      properties:
        field:
          type: string
        error_code:
          type: string
        message:
          type: string
        severity:
          type: string
          enum: [error, warning, info]

    SignatureRequest:
      type: object
      required:
        - signer_id
        - signer_role
        - signature_type
        - signature_data
      properties:
        signer_id:
          type: string
        signer_name:
          type: string
        signer_role:
          type: string
          enum: [SUBJECT, SPOUSE, CHILD, GUARDIAN, WITNESS, NOTARY, LEGAL_COUNSEL, MEDICAL_ADVISOR]
        signature_type:
          type: string
          enum: [WET, DIGITAL, BIOMETRIC]
        signature_data:
          type: string
          format: byte
        location:
          type: string

    SignatureRecord:
      type: object
      properties:
        signer_id:
          type: string
        signer_name:
          type: string
        signer_role:
          type: string
        signature_type:
          type: string
        timestamp:
          type: string
          format: date-time
        verification_status:
          type: string
          enum: [PENDING, VERIFIED, INVALID]

    SignatureBlock:
      type: object
      properties:
        subject_signature:
          $ref: '#/components/schemas/SignatureRecord'
        witness_signatures:
          type: array
          items:
            $ref: '#/components/schemas/SignatureRecord'
        notarization:
          $ref: '#/components/schemas/NotarizationRecord'

    NotarizationRecord:
      type: object
      properties:
        notary_id:
          type: string
        notary_name:
          type: string
        notary_license:
          type: string
        jurisdiction:
          type: string
        notarization_date:
          type: string
          format: date-time

    Amendment:
      type: object
      properties:
        amendment_id:
          type: string
        amendment_number:
          type: integer
        amendment_type:
          type: string
        description:
          type: string
        changes:
          type: array
          items:
            type: object
        status:
          type: string
          enum: [PENDING, APPROVED, REJECTED, SUPERSEDED]
        approvals:
          type: array
          items:
            $ref: '#/components/schemas/AmendmentApproval'
        required_approvals:
          type: integer

    AmendmentApproval:
      type: object
      properties:
        approver_id:
          type: string
        decision:
          type: string
          enum: [APPROVE, REJECT, ABSTAIN]
        timestamp:
          type: string
          format: date-time

    CreateAmendmentRequest:
      type: object
      required:
        - changes
        - reason
      properties:
        changes:
          type: array
          items:
            type: object
            properties:
              section:
                type: string
              field:
                type: string
              new_value:
                type: object
        reason:
          type: string

    AmendmentApprovalRequest:
      type: object
      required:
        - decision
        - signature
      properties:
        decision:
          type: string
          enum: [APPROVE, REJECT, ABSTAIN]
        notes:
          type: string
        signature:
          $ref: '#/components/schemas/SignatureRequest'

    RevivalEvaluationRequest:
      type: object
      required:
        - current_evidence
      properties:
        current_evidence:
          type: object
          description: Current state of medical advances, technology, etc.

    RevivalDecision:
      type: object
      properties:
        document_id:
          type: string
        decision:
          type: string
          enum: [APPROVE, DENY, PENDING]
        conditions_met:
          type: array
          items:
            type: string
        conditions_not_met:
          type: array
          items:
            type: string
        conditions_pending:
          type: array
          items:
            type: string
        overall_score:
          type: number
        recommendation:
          type: string

    WithdrawalRequest:
      type: object
      required:
        - withdrawal_type
        - reason
      properties:
        withdrawal_type:
          type: string
          enum: [PRE_PRESERVATION, DURING_PRESERVATION, REVIVAL_REFUSAL]
        reason:
          type: string

    WithdrawalResult:
      type: object
      properties:
        request_id:
          type: string
        status:
          type: string
          enum: [APPROVED, DENIED, PENDING]
        refund_amount:
          type: number
        next_steps:
          type: array
          items:
            type: string

    Conflict:
      type: object
      properties:
        conflict_id:
          type: string
        conflict_type:
          type: string
        description:
          type: string
        parties:
          type: array
          items:
            type: string
        resolution_suggestion:
          type: string
        severity:
          type: string
          enum: [critical, major, minor]

    SubjectInformation:
      type: object
      properties:
        subject_id:
          type: string
        identity_ref:
          type: string
        personal_info:
          type: object

    ConsentTemplate:
      type: object
      properties:
        template_id:
          type: string
        name:
          type: string
        jurisdiction:
          type: string
        language:
          type: string
        version:
          type: string
        sections:
          type: array
          items:
            type: object

  responses:
    BadRequest:
      description: Bad request
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
    NotFound:
      description: Resource not found
      content:
        application/json:
          schema:
            type: object
            properties:
              error:
                type: string
```

## Webhook Events

### Event Types

```yaml
webhook_events:
  document.created:
    description: New consent document created
    payload:
      document_id: string
      subject_id: string
      created_at: datetime

  document.updated:
    description: Document updated
    payload:
      document_id: string
      updated_fields: string[]
      updated_at: datetime

  document.status_changed:
    description: Document status changed
    payload:
      document_id: string
      previous_status: string
      new_status: string
      changed_at: datetime

  signature.added:
    description: Signature added to document
    payload:
      document_id: string
      signer_role: string
      signature_type: string
      signed_at: datetime

  signature.verified:
    description: Signature verified
    payload:
      document_id: string
      signer_id: string
      verification_result: string

  amendment.created:
    description: Amendment created
    payload:
      document_id: string
      amendment_id: string
      amendment_type: string

  amendment.approved:
    description: Amendment approved
    payload:
      document_id: string
      amendment_id: string
      effective_date: datetime

  revival.evaluation_requested:
    description: Revival evaluation requested
    payload:
      document_id: string
      requested_by: string

  revival.conditions_met:
    description: Revival conditions met
    payload:
      document_id: string
      conditions_met: string[]
      decision: string

  withdrawal.requested:
    description: Withdrawal requested
    payload:
      document_id: string
      withdrawal_type: string
      requested_by: string

  withdrawal.processed:
    description: Withdrawal processed
    payload:
      document_id: string
      result: string
      refund_amount: number
```

### Webhook Registration

```
POST /webhooks
{
  "url": "https://your-service.com/webhooks/cryo-consent",
  "events": ["document.status_changed", "amendment.approved"],
  "secret": "your-webhook-secret"
}
```

---

*WIA Technical Committee - Cryopreservation Working Group*
