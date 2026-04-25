# CRYO-LEGAL Phase 3: Protocol Specification

## Overview

This document defines the REST API protocol for legal status management, document handling, jurisdiction recognition, and restoration workflows.

## OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: CRYO-LEGAL Protocol API
  description: |
    WIA Standard API for legal status management of cryopreserved individuals.
    Handles death certificates, preservation declarations, jurisdiction recognition,
    and legal restoration upon revival.
  version: 1.0.0
  contact:
    name: WIA Technical Committee
    url: https://wia.org/standards/cryo-legal
  license:
    name: WIA Open Standard License
    url: https://wia.org/licenses/open-standard

servers:
  - url: https://api.wia-legal.org/v1
    description: Production server
  - url: https://staging-api.wia-legal.org/v1
    description: Staging server

tags:
  - name: status
    description: Legal status management
  - name: documents
    description: Legal document handling
  - name: jurisdictions
    description: Multi-jurisdiction recognition
  - name: restoration
    description: Legal restoration workflow
  - name: guardianship
    description: Guardianship management

paths:
  /records:
    post:
      tags: [status]
      summary: Create a new legal status record
      operationId: createRecord
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateRecordRequest'
      responses:
        '201':
          description: Record created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalStatusRecord'
        '400':
          $ref: '#/components/responses/BadRequest'

    get:
      tags: [status]
      summary: List legal status records
      operationId: listRecords
      security:
        - bearerAuth: []
      parameters:
        - name: status
          in: query
          schema:
            $ref: '#/components/schemas/LegalStatus'
        - name: jurisdiction
          in: query
          schema:
            type: string
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
          description: List of records
          content:
            application/json:
              schema:
                type: object
                properties:
                  records:
                    type: array
                    items:
                      $ref: '#/components/schemas/LegalStatusRecordSummary'
                  pagination:
                    $ref: '#/components/schemas/Pagination'

  /records/{recordId}:
    get:
      tags: [status]
      summary: Get legal status record
      operationId: getRecord
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/recordId'
      responses:
        '200':
          description: Legal status record
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalStatusRecord'
        '404':
          $ref: '#/components/responses/NotFound'

  /records/{recordId}/transition:
    post:
      tags: [status]
      summary: Transition legal status
      operationId: transitionStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/recordId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TransitionRequest'
      responses:
        '200':
          description: Transition result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TransitionResult'
        '400':
          description: Invalid transition
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TransitionError'

  /records/{recordId}/history:
    get:
      tags: [status]
      summary: Get status transition history
      operationId: getStatusHistory
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/recordId'
      responses:
        '200':
          description: Status history
          content:
            application/json:
              schema:
                type: object
                properties:
                  current_status:
                    $ref: '#/components/schemas/LegalStatus'
                  history:
                    type: array
                    items:
                      $ref: '#/components/schemas/StatusTransition'

  /documents:
    post:
      tags: [documents]
      summary: Create a legal document
      operationId: createDocument
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          multipart/form-data:
            schema:
              $ref: '#/components/schemas/CreateDocumentRequest'
      responses:
        '201':
          description: Document created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalDocument'

    get:
      tags: [documents]
      summary: List legal documents
      operationId: listDocuments
      security:
        - bearerAuth: []
      parameters:
        - name: record_id
          in: query
          schema:
            type: string
        - name: document_type
          in: query
          schema:
            $ref: '#/components/schemas/DocumentType'
        - name: status
          in: query
          schema:
            $ref: '#/components/schemas/DocumentStatus'
      responses:
        '200':
          description: List of documents
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/LegalDocumentSummary'

  /documents/{documentId}:
    get:
      tags: [documents]
      summary: Get legal document
      operationId: getDocument
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Legal document
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalDocument'

  /documents/{documentId}/content:
    get:
      tags: [documents]
      summary: Download document content
      operationId: downloadDocument
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Document content
          content:
            application/pdf:
              schema:
                type: string
                format: binary

  /documents/{documentId}/signatures:
    post:
      tags: [documents]
      summary: Add signature to document
      operationId: addSignature
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SignatureRequest'
      responses:
        '201':
          description: Signature added
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Signature'

  /documents/{documentId}/finalize:
    post:
      tags: [documents]
      summary: Finalize document
      operationId: finalizeDocument
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Document finalized
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalDocument'

  /documents/{documentId}/notarize:
    post:
      tags: [documents]
      summary: Notarize document
      operationId: notarizeDocument
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/NotarizationRequest'
      responses:
        '200':
          description: Document notarized
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalDocument'

  /documents/{documentId}/apostille:
    post:
      tags: [documents]
      summary: Add apostille to document
      operationId: addApostille
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ApostilleRequest'
      responses:
        '200':
          description: Apostille added
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LegalDocument'

  /documents/{documentId}/verify:
    get:
      tags: [documents]
      summary: Verify document validity
      operationId: verifyDocument
      security:
        - bearerAuth: []
      parameters:
        - name: documentId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Verification result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DocumentVerification'

  /jurisdictions:
    get:
      tags: [jurisdictions]
      summary: List supported jurisdictions
      operationId: listJurisdictions
      security:
        - bearerAuth: []
      responses:
        '200':
          description: List of jurisdictions
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/Jurisdiction'

  /jurisdictions/{countryCode}:
    get:
      tags: [jurisdictions]
      summary: Get jurisdiction details
      operationId: getJurisdiction
      security:
        - bearerAuth: []
      parameters:
        - name: countryCode
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Jurisdiction details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/JurisdictionDetails'

  /records/{recordId}/jurisdictions:
    get:
      tags: [jurisdictions]
      summary: Get jurisdiction recognitions for record
      operationId: getRecordJurisdictions
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/recordId'
      responses:
        '200':
          description: Jurisdiction recognitions
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/JurisdictionRecognition'

    post:
      tags: [jurisdictions]
      summary: Register jurisdiction recognition
      operationId: registerJurisdiction
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/recordId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/JurisdictionRegistrationRequest'
      responses:
        '201':
          description: Recognition registered
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/JurisdictionRecognition'

  /jurisdictions/check-recognition:
    post:
      tags: [jurisdictions]
      summary: Check recognition between jurisdictions
      operationId: checkRecognition
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RecognitionCheckRequest'
      responses:
        '200':
          description: Recognition check result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RecognitionCheckResult'

  /restoration:
    post:
      tags: [restoration]
      summary: Initiate legal restoration case
      operationId: initiateRestoration
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/InitiateRestorationRequest'
      responses:
        '201':
          description: Restoration case initiated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RestorationCase'

    get:
      tags: [restoration]
      summary: List restoration cases
      operationId: listRestorationCases
      security:
        - bearerAuth: []
      parameters:
        - name: status
          in: query
          schema:
            $ref: '#/components/schemas/RestorationStage'
      responses:
        '200':
          description: List of restoration cases
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/RestorationCaseSummary'

  /restoration/{caseId}:
    get:
      tags: [restoration]
      summary: Get restoration case details
      operationId: getRestorationCase
      security:
        - bearerAuth: []
      parameters:
        - name: caseId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Restoration case details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RestorationCase'

  /restoration/{caseId}/tasks:
    get:
      tags: [restoration]
      summary: Get restoration tasks
      operationId: getRestorationTasks
      security:
        - bearerAuth: []
      parameters:
        - name: caseId
          in: path
          required: true
          schema:
            type: string
        - name: stage
          in: query
          schema:
            $ref: '#/components/schemas/RestorationStage'
        - name: completed
          in: query
          schema:
            type: boolean
      responses:
        '200':
          description: Restoration tasks
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/RestorationTask'

  /restoration/{caseId}/tasks/{taskId}/complete:
    post:
      tags: [restoration]
      summary: Complete a restoration task
      operationId: completeTask
      security:
        - bearerAuth: []
      parameters:
        - name: caseId
          in: path
          required: true
          schema:
            type: string
        - name: taskId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CompleteTaskRequest'
      responses:
        '200':
          description: Task completed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RestorationTask'

  /restoration/{caseId}/court-petition:
    post:
      tags: [restoration]
      summary: File court petition
      operationId: fileCourtPetition
      security:
        - bearerAuth: []
      parameters:
        - name: caseId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CourtPetitionRequest'
      responses:
        '201':
          description: Petition filed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CourtPetitionResult'

  /restoration/{caseId}/court-decision:
    post:
      tags: [restoration]
      summary: Record court decision
      operationId: recordCourtDecision
      security:
        - bearerAuth: []
      parameters:
        - name: caseId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CourtDecisionRequest'
      responses:
        '200':
          description: Decision recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RestorationCase'

  /guardianship:
    post:
      tags: [guardianship]
      summary: Create guardianship
      operationId: createGuardianship
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateGuardianshipRequest'
      responses:
        '201':
          description: Guardianship created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Guardianship'

  /guardianship/{guardianshipId}:
    get:
      tags: [guardianship]
      summary: Get guardianship details
      operationId: getGuardianship
      security:
        - bearerAuth: []
      parameters:
        - name: guardianshipId
          in: path
          required: true
          schema:
            type: string
      responses:
        '200':
          description: Guardianship details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Guardianship'

  /guardianship/{guardianshipId}/authorize:
    post:
      tags: [guardianship]
      summary: Check guardian authorization
      operationId: checkAuthorization
      security:
        - bearerAuth: []
      parameters:
        - name: guardianshipId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/AuthorizationCheckRequest'
      responses:
        '200':
          description: Authorization check result
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/AuthorizationResult'

  /guardianship/{guardianshipId}/terminate:
    post:
      tags: [guardianship]
      summary: Terminate guardianship
      operationId: terminateGuardianship
      security:
        - bearerAuth: []
      parameters:
        - name: guardianshipId
          in: path
          required: true
          schema:
            type: string
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TerminateGuardianshipRequest'
      responses:
        '200':
          description: Guardianship terminated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Guardianship'

components:
  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT

  parameters:
    recordId:
      name: recordId
      in: path
      required: true
      schema:
        type: string
        pattern: '^LEGAL-[0-9]{4}-[0-9]{6}$'

  responses:
    BadRequest:
      description: Bad request
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/Error'
    NotFound:
      description: Resource not found
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/Error'

  schemas:
    LegalStatus:
      type: string
      enum:
        - LIVING
        - LEGALLY_DECEASED
        - PRESERVED_STATUS
        - REVIVAL_PENDING
        - LEGALLY_RESTORED
        - PERMANENT_DEATH

    DocumentType:
      type: string
      enum:
        - DEATH_CERTIFICATE
        - PRESERVATION_DECLARATION
        - RESTORATION_CERTIFICATE
        - COURT_ORDER
        - GUARDIANSHIP_ORDER
        - TRUST_DOCUMENT
        - POWER_OF_ATTORNEY
        - IDENTITY_VERIFICATION

    DocumentStatus:
      type: string
      enum:
        - DRAFT
        - PENDING_SIGNATURE
        - ACTIVE
        - SUPERSEDED
        - REVOKED
        - EXPIRED

    RestorationStage:
      type: string
      enum:
        - INITIATED
        - IDENTITY_VERIFICATION
        - MEDICAL_CLEARANCE
        - LEGAL_REVIEW
        - COURT_PETITION
        - COURT_HEARING
        - RESTORATION_GRANTED
        - RIGHTS_RESTORATION
        - COMPLETED
        - DENIED

    CreateRecordRequest:
      type: object
      required:
        - subject_id
        - identity_id
        - primary_jurisdiction
      properties:
        subject_id:
          type: string
        identity_id:
          type: string
        primary_jurisdiction:
          type: string

    LegalStatusRecord:
      type: object
      properties:
        record_id:
          type: string
        subject_id:
          type: string
        identity_id:
          type: string
        current_status:
          $ref: '#/components/schemas/LegalStatus'
        primary_jurisdiction:
          $ref: '#/components/schemas/Jurisdiction'
        death_certificate:
          $ref: '#/components/schemas/DeathCertificate'
        preservation_declaration:
          $ref: '#/components/schemas/PreservationDeclaration'
        restoration_certificate:
          $ref: '#/components/schemas/RestorationCertificate'
        guardianship:
          $ref: '#/components/schemas/GuardianshipSummary'
        created_at:
          type: string
          format: date-time
        last_updated:
          type: string
          format: date-time

    LegalStatusRecordSummary:
      type: object
      properties:
        record_id:
          type: string
        subject_id:
          type: string
        current_status:
          $ref: '#/components/schemas/LegalStatus'
        primary_jurisdiction:
          type: string

    StatusTransition:
      type: object
      properties:
        transition_id:
          type: string
        from_status:
          $ref: '#/components/schemas/LegalStatus'
        to_status:
          $ref: '#/components/schemas/LegalStatus'
        transition_date:
          type: string
          format: date-time
        authority_id:
          type: string
        document_ref:
          type: string
        reason:
          type: string

    TransitionRequest:
      type: object
      required:
        - to_status
        - authority_id
        - document_ref
        - reason
      properties:
        to_status:
          $ref: '#/components/schemas/LegalStatus'
        authority_id:
          type: string
        document_ref:
          type: string
        reason:
          type: string

    TransitionResult:
      type: object
      properties:
        success:
          type: boolean
        transition:
          $ref: '#/components/schemas/StatusTransition'
        new_status:
          $ref: '#/components/schemas/LegalStatus'

    TransitionError:
      type: object
      properties:
        success:
          type: boolean
          example: false
        error:
          type: string
        current_status:
          $ref: '#/components/schemas/LegalStatus'
        valid_transitions:
          type: array
          items:
            $ref: '#/components/schemas/LegalStatus'

    Jurisdiction:
      type: object
      properties:
        country:
          type: string
        country_code:
          type: string
        region:
          type: string
        legal_system:
          type: string
        treaty_member:
          type: boolean

    JurisdictionDetails:
      type: object
      properties:
        jurisdiction:
          $ref: '#/components/schemas/Jurisdiction'
        recognition_requirements:
          type: array
          items:
            type: string
        bilateral_agreements:
          type: array
          items:
            type: string
        local_authorities:
          type: array
          items:
            type: object

    JurisdictionRecognition:
      type: object
      properties:
        jurisdiction:
          $ref: '#/components/schemas/Jurisdiction'
        recognition_status:
          type: string
        local_status_equivalent:
          type: string
        registration_id:
          type: string
        registration_date:
          type: string
          format: date-time

    DeathCertificate:
      type: object
      properties:
        certificate_id:
          type: string
        certificate_number:
          type: string
        date_of_death:
          type: string
          format: date
        cause_of_death:
          type: string
        preservation_annotated:
          type: boolean
        issue_date:
          type: string
          format: date

    PreservationDeclaration:
      type: object
      properties:
        declaration_id:
          type: string
        declaration_type:
          type: string
        facility_id:
          type: string
        preservation_date:
          type: string
          format: date
        legal_effects:
          type: object
        effective_date:
          type: string
          format: date

    RestorationCertificate:
      type: object
      properties:
        certificate_id:
          type: string
        revival_procedure_id:
          type: string
        revival_date:
          type: string
          format: date
        restored_rights:
          type: object
        issue_date:
          type: string
          format: date

    LegalDocument:
      type: object
      properties:
        document_id:
          type: string
        document_type:
          $ref: '#/components/schemas/DocumentType'
        title:
          type: string
        status:
          $ref: '#/components/schemas/DocumentStatus'
        content_hash:
          type: string
        signatures:
          type: array
          items:
            $ref: '#/components/schemas/Signature'
        notarized:
          type: boolean
        apostille:
          type: boolean
        created_at:
          type: string
          format: date-time
        effective_date:
          type: string
          format: date-time

    LegalDocumentSummary:
      type: object
      properties:
        document_id:
          type: string
        document_type:
          $ref: '#/components/schemas/DocumentType'
        title:
          type: string
        status:
          $ref: '#/components/schemas/DocumentStatus'

    Signature:
      type: object
      properties:
        signer_id:
          type: string
        signer_name:
          type: string
        signer_role:
          type: string
        signature_date:
          type: string
          format: date-time
        signature_type:
          type: string
        verified:
          type: boolean

    RestorationCase:
      type: object
      properties:
        case_id:
          type: string
        subject_id:
          type: string
        revival_procedure_id:
          type: string
        current_stage:
          $ref: '#/components/schemas/RestorationStage'
        stages_completed:
          type: array
          items:
            $ref: '#/components/schemas/RestorationStage'
        court_case_number:
          type: string
        decision:
          type: string
        created_at:
          type: string
          format: date-time

    RestorationCaseSummary:
      type: object
      properties:
        case_id:
          type: string
        subject_id:
          type: string
        current_stage:
          $ref: '#/components/schemas/RestorationStage'

    RestorationTask:
      type: object
      properties:
        task_id:
          type: string
        stage:
          $ref: '#/components/schemas/RestorationStage'
        description:
          type: string
        assigned_to:
          type: string
        due_date:
          type: string
          format: date-time
        completed:
          type: boolean
        completion_date:
          type: string
          format: date-time

    Guardianship:
      type: object
      properties:
        guardianship_id:
          type: string
        subject_id:
          type: string
        status:
          type: string
        guardians:
          type: array
          items:
            $ref: '#/components/schemas/Guardian'
        scope:
          type: object
        created_at:
          type: string
          format: date-time

    GuardianshipSummary:
      type: object
      properties:
        guardianship_id:
          type: string
        primary_guardian:
          type: string
        status:
          type: string

    Guardian:
      type: object
      properties:
        guardian_id:
          type: string
        name:
          type: string
        relationship:
          type: string
        priority:
          type: integer
        active:
          type: boolean

    CreateDocumentRequest:
      type: object
      properties:
        document_type:
          $ref: '#/components/schemas/DocumentType'
        title:
          type: string
        content:
          type: string
          format: binary
        jurisdiction:
          type: string
        effective_date:
          type: string
          format: date

    SignatureRequest:
      type: object
      required:
        - signer_id
        - signer_name
        - signer_role
        - signature_type
      properties:
        signer_id:
          type: string
        signer_name:
          type: string
        signer_role:
          type: string
        signature_type:
          type: string
        digital_signature:
          type: string

    NotarizationRequest:
      type: object
      required:
        - notary_id
        - notary_name
        - notary_jurisdiction
      properties:
        notary_id:
          type: string
        notary_name:
          type: string
        notary_jurisdiction:
          type: string

    ApostilleRequest:
      type: object
      required:
        - apostille_number
        - issuing_country
        - issuing_authority
      properties:
        apostille_number:
          type: string
        issuing_country:
          type: string
        issuing_authority:
          type: string

    DocumentVerification:
      type: object
      properties:
        valid:
          type: boolean
        document_id:
          type: string
        status:
          $ref: '#/components/schemas/DocumentStatus'
        expiration_date:
          type: string
          format: date-time
        issues:
          type: array
          items:
            type: string

    InitiateRestorationRequest:
      type: object
      required:
        - subject_id
        - revival_procedure_id
      properties:
        subject_id:
          type: string
        revival_procedure_id:
          type: string

    CompleteTaskRequest:
      type: object
      required:
        - completed_by
      properties:
        completed_by:
          type: string
        notes:
          type: string

    CourtPetitionRequest:
      type: object
      required:
        - court_name
        - jurisdiction
      properties:
        court_name:
          type: string
        jurisdiction:
          type: string

    CourtPetitionResult:
      type: object
      properties:
        success:
          type: boolean
        court_case_number:
          type: string

    CourtDecisionRequest:
      type: object
      required:
        - decision
        - judge_name
        - decision_date
      properties:
        decision:
          type: string
          enum: [GRANTED, DENIED, CONTINUED]
        judge_name:
          type: string
        decision_date:
          type: string
          format: date
        order_text:
          type: string

    CreateGuardianshipRequest:
      type: object
      required:
        - subject_id
        - guardians
        - appointment_authority
        - court_order_ref
      properties:
        subject_id:
          type: string
        guardians:
          type: array
          items:
            type: object
        scope:
          type: object
        appointment_authority:
          type: string
        court_order_ref:
          type: string

    AuthorizationCheckRequest:
      type: object
      required:
        - guardian_id
        - decision_type
      properties:
        guardian_id:
          type: string
        decision_type:
          type: string

    AuthorizationResult:
      type: object
      properties:
        authorized:
          type: boolean
        reason:
          type: string

    TerminateGuardianshipRequest:
      type: object
      required:
        - reason
        - authority
      properties:
        reason:
          type: string
        authority:
          type: string

    JurisdictionRegistrationRequest:
      type: object
      required:
        - country_code
      properties:
        country_code:
          type: string
        documents:
          type: array
          items:
            type: string

    RecognitionCheckRequest:
      type: object
      required:
        - from_jurisdiction
        - to_jurisdiction
      properties:
        from_jurisdiction:
          type: string
        to_jurisdiction:
          type: string

    RecognitionCheckResult:
      type: object
      properties:
        recognition_status:
          type: string
        requirements:
          type: array
          items:
            type: string
        bilateral_agreement:
          type: string

    Pagination:
      type: object
      properties:
        page:
          type: integer
        limit:
          type: integer
        total:
          type: integer
        total_pages:
          type: integer

    Error:
      type: object
      properties:
        code:
          type: string
        message:
          type: string
```

---

*WIA Technical Committee - Cryopreservation Working Group*
