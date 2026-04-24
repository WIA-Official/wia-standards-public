# CRYO-REVIVAL Phase 3: Protocol Specification

## Overview

This document defines the REST API protocol for revival procedure management, stage control, monitoring, and re-preservation decisions.

## OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: CRYO-REVIVAL Protocol API
  description: |
    WIA Standard API for cryopreservation revival procedures.
    Manages the complete revival workflow from preparation through post-revival care.
  version: 1.0.0
  contact:
    name: WIA Technical Committee
    url: https://wia.org/standards/cryo-revival
  license:
    name: WIA Open Standard License
    url: https://wia.org/licenses/open-standard

servers:
  - url: https://api.wia-revival.org/v1
    description: Production server
  - url: https://staging-api.wia-revival.org/v1
    description: Staging server
  - url: https://sandbox-api.wia-revival.org/v1
    description: Sandbox for testing

tags:
  - name: procedures
    description: Revival procedure management
  - name: stages
    description: Stage control and transitions
  - name: monitoring
    description: Vital signs and monitoring data
  - name: team
    description: Revival team management
  - name: alerts
    description: Alert management
  - name: re-preservation
    description: Re-preservation decisions

paths:
  /procedures:
    post:
      tags: [procedures]
      summary: Initialize a new revival procedure
      operationId: createProcedure
      security:
        - bearerAuth: []
        - apiKey: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CreateProcedureRequest'
      responses:
        '201':
          description: Procedure created successfully
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalProcedure'
        '400':
          $ref: '#/components/responses/BadRequest'
        '401':
          $ref: '#/components/responses/Unauthorized'
        '403':
          description: Consent or preparation requirements not met
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

    get:
      tags: [procedures]
      summary: List revival procedures
      operationId: listProcedures
      security:
        - bearerAuth: []
      parameters:
        - name: status
          in: query
          schema:
            $ref: '#/components/schemas/RevivalStatus'
        - name: facility_id
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
            maximum: 100
      responses:
        '200':
          description: List of procedures
          content:
            application/json:
              schema:
                type: object
                properties:
                  procedures:
                    type: array
                    items:
                      $ref: '#/components/schemas/ProcedureSummary'
                  pagination:
                    $ref: '#/components/schemas/Pagination'

  /procedures/{procedureId}:
    get:
      tags: [procedures]
      summary: Get procedure details
      operationId: getProcedure
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Procedure details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalProcedure'
        '404':
          $ref: '#/components/responses/NotFound'

    patch:
      tags: [procedures]
      summary: Update procedure status
      operationId: updateProcedure
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/UpdateProcedureRequest'
      responses:
        '200':
          description: Procedure updated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalProcedure'

  /procedures/{procedureId}/stages:
    get:
      tags: [stages]
      summary: Get all stage records for a procedure
      operationId: getStages
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Stage records
          content:
            application/json:
              schema:
                type: object
                properties:
                  current_stage:
                    $ref: '#/components/schemas/RevivalStage'
                  stages:
                    type: array
                    items:
                      $ref: '#/components/schemas/StageRecord'

  /procedures/{procedureId}/stages/current:
    get:
      tags: [stages]
      summary: Get current stage details
      operationId: getCurrentStage
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Current stage details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/StageRecord'

  /procedures/{procedureId}/stages/transition:
    post:
      tags: [stages]
      summary: Request transition to next stage
      operationId: requestStageTransition
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
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
          description: Transition criteria not met
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TransitionDenied'

  /procedures/{procedureId}/warming:
    get:
      tags: [stages]
      summary: Get warming stage status
      operationId: getWarmingStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Warming status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/WarmingStatus'

    post:
      tags: [stages]
      summary: Record warming temperature
      operationId: recordWarmingTemp
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TemperatureReading'
      responses:
        '201':
          description: Temperature recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TemperatureRecordResult'

  /procedures/{procedureId}/warming/profile:
    get:
      tags: [stages]
      summary: Get optimal warming profile
      operationId: getWarmingProfile
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Warming profile
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/WarmingProfile'

  /procedures/{procedureId}/cardiovascular:
    get:
      tags: [stages]
      summary: Get cardiovascular restart status
      operationId: getCardiovascularStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Cardiovascular status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CardiovascularStatus'

    post:
      tags: [stages]
      summary: Update cardiovascular status
      operationId: updateCardiovascularStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CardiovascularUpdate'
      responses:
        '200':
          description: Status updated with recommendations
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CardiovascularUpdateResult'

  /procedures/{procedureId}/cardiovascular/defibrillation:
    post:
      tags: [stages]
      summary: Record defibrillation attempt
      operationId: recordDefibrillation
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/DefibrillationRecord'
      responses:
        '201':
          description: Defibrillation recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DefibrillationResult'

  /procedures/{procedureId}/neurological:
    get:
      tags: [stages]
      summary: Get neurological status
      operationId: getNeurologicalStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Neurological status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/NeurologicalStatus'

  /procedures/{procedureId}/neurological/eeg:
    post:
      tags: [stages]
      summary: Record EEG reading
      operationId: recordEEG
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/EEGReading'
      responses:
        '201':
          description: EEG recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/EEGRecordResult'

  /procedures/{procedureId}/neurological/consciousness:
    post:
      tags: [stages]
      summary: Record consciousness assessment
      operationId: recordConsciousness
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ConsciousnessAssessment'
      responses:
        '201':
          description: Assessment recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ConsciousnessResult'

  /procedures/{procedureId}/monitoring/vitals:
    post:
      tags: [monitoring]
      summary: Record vital signs
      operationId: recordVitals
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/VitalSigns'
      responses:
        '201':
          description: Vitals recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VitalsRecordResult'

    get:
      tags: [monitoring]
      summary: Get vital signs history
      operationId: getVitalsHistory
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
        - name: from
          in: query
          schema:
            type: string
            format: date-time
        - name: to
          in: query
          schema:
            type: string
            format: date-time
        - name: limit
          in: query
          schema:
            type: integer
            default: 100
      responses:
        '200':
          description: Vital signs history
          content:
            application/json:
              schema:
                type: object
                properties:
                  vitals:
                    type: array
                    items:
                      $ref: '#/components/schemas/VitalSigns'
                  trends:
                    $ref: '#/components/schemas/VitalsTrends'

  /procedures/{procedureId}/monitoring/labs:
    post:
      tags: [monitoring]
      summary: Record laboratory results
      operationId: recordLabs
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/LabResults'
      responses:
        '201':
          description: Labs recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/LabRecordResult'

  /procedures/{procedureId}/complications:
    post:
      tags: [monitoring]
      summary: Record complication
      operationId: recordComplication
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ComplicationRecord'
      responses:
        '201':
          description: Complication recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Complication'

    get:
      tags: [monitoring]
      summary: Get all complications
      operationId: getComplications
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Complications list
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/Complication'

  /procedures/{procedureId}/interventions:
    post:
      tags: [monitoring]
      summary: Record intervention
      operationId: recordIntervention
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/InterventionRecord'
      responses:
        '201':
          description: Intervention recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Intervention'

  /procedures/{procedureId}/team:
    get:
      tags: [team]
      summary: Get revival team
      operationId: getTeam
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Team information
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalTeam'

    post:
      tags: [team]
      summary: Assign team member
      operationId: assignTeamMember
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TeamMemberAssignment'
      responses:
        '201':
          description: Member assigned
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/TeamMember'

  /procedures/{procedureId}/team/shifts:
    post:
      tags: [team]
      summary: Create shift schedule
      operationId: createShift
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ShiftSchedule'
      responses:
        '201':
          description: Shift created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ShiftSchedule'

  /procedures/{procedureId}/team/handover:
    post:
      tags: [team]
      summary: Record shift handover
      operationId: recordHandover
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/HandoverRecord'
      responses:
        '201':
          description: Handover recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HandoverRecord'

  /procedures/{procedureId}/alerts:
    get:
      tags: [alerts]
      summary: Get active alerts
      operationId: getAlerts
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
        - name: severity
          in: query
          schema:
            $ref: '#/components/schemas/AlertSeverity'
        - name: acknowledged
          in: query
          schema:
            type: boolean
      responses:
        '200':
          description: Active alerts
          content:
            application/json:
              schema:
                type: object
                properties:
                  alerts:
                    type: array
                    items:
                      $ref: '#/components/schemas/Alert'
                  summary:
                    $ref: '#/components/schemas/AlertSummary'

  /procedures/{procedureId}/alerts/{alertId}/acknowledge:
    post:
      tags: [alerts]
      summary: Acknowledge alert
      operationId: acknowledgeAlert
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
        - name: alertId
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
              required: [acknowledged_by]
              properties:
                acknowledged_by:
                  type: string
      responses:
        '200':
          description: Alert acknowledged
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Alert'

  /procedures/{procedureId}/alerts/{alertId}/resolve:
    post:
      tags: [alerts]
      summary: Resolve alert
      operationId: resolveAlert
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
        - name: alertId
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
              required: [resolved_by, resolution]
              properties:
                resolved_by:
                  type: string
                resolution:
                  type: string
      responses:
        '200':
          description: Alert resolved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Alert'

  /procedures/{procedureId}/re-preservation:
    get:
      tags: [re-preservation]
      summary: Get re-preservation status
      operationId: getRePreservationStatus
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Re-preservation status
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RePreservationStatus'

    post:
      tags: [re-preservation]
      summary: Initialize re-preservation consideration
      operationId: initializeRePreservation
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/RePreservationInit'
      responses:
        '201':
          description: Re-preservation initialized
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RePreservationStatus'

  /procedures/{procedureId}/re-preservation/criteria:
    post:
      tags: [re-preservation]
      summary: Evaluate re-preservation criterion
      operationId: evaluateCriterion
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/CriterionEvaluation'
      responses:
        '200':
          description: Criterion evaluated
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/CriterionResult'

  /procedures/{procedureId}/re-preservation/votes:
    post:
      tags: [re-preservation]
      summary: Record team vote
      operationId: recordVote
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/TeamVote'
      responses:
        '201':
          description: Vote recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/VoteResult'

  /procedures/{procedureId}/re-preservation/decision:
    post:
      tags: [re-preservation]
      summary: Make re-preservation decision
      operationId: makeRePreservationDecision
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Decision made
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RePreservationDecision'
        '400':
          description: Cannot make decision yet
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/DecisionBlocked'

  /procedures/{procedureId}/outcome:
    get:
      tags: [procedures]
      summary: Get procedure outcome
      operationId: getOutcome
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: Procedure outcome
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalOutcome'

    post:
      tags: [procedures]
      summary: Record procedure outcome
      operationId: recordOutcome
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/OutcomeRecord'
      responses:
        '201':
          description: Outcome recorded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/RevivalOutcome'

  /procedures/{procedureId}/stream:
    get:
      tags: [monitoring]
      summary: Stream real-time updates
      operationId: streamUpdates
      security:
        - bearerAuth: []
      parameters:
        - $ref: '#/components/parameters/procedureId'
      responses:
        '200':
          description: SSE stream of updates
          content:
            text/event-stream:
              schema:
                type: string

components:
  securitySchemes:
    bearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT
    apiKey:
      type: apiKey
      in: header
      name: X-API-Key

  parameters:
    procedureId:
      name: procedureId
      in: path
      required: true
      schema:
        type: string
        pattern: '^REVIVAL-[0-9]{4}-[0-9]{3}$'

  responses:
    BadRequest:
      description: Bad request
      content:
        application/json:
          schema:
            $ref: '#/components/schemas/Error'
    Unauthorized:
      description: Unauthorized
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
    RevivalStatus:
      type: string
      enum:
        - SCHEDULED
        - PREPARING
        - IN_PROGRESS
        - PAUSED
        - COMPLETED
        - FAILED
        - ABORTED
        - RE_PRESERVED

    RevivalStage:
      type: string
      enum:
        - PRE_REVIVAL
        - WARMING
        - CRYOPROTECTANT_REMOVAL
        - REHYDRATION
        - CARDIOVASCULAR_RESTART
        - RESPIRATORY_ACTIVATION
        - NEUROLOGICAL_RESTORATION
        - STABILIZATION
        - POST_REVIVAL_CARE

    CreateProcedureRequest:
      type: object
      required:
        - subject_id
        - identity_id
        - consent_id
        - facility_id
        - scheduled_date
      properties:
        subject_id:
          type: string
        identity_id:
          type: string
          description: CRYO-IDENTITY reference
        consent_id:
          type: string
          description: CRYO-CONSENT reference
        facility_id:
          type: string
        scheduled_date:
          type: string
          format: date-time
        notes:
          type: string

    RevivalProcedure:
      type: object
      properties:
        procedure_id:
          type: string
        subject_id:
          type: string
        identity_id:
          type: string
        consent_id:
          type: string
        status:
          $ref: '#/components/schemas/RevivalStatus'
        current_stage:
          $ref: '#/components/schemas/RevivalStage'
        preparation:
          $ref: '#/components/schemas/RevivalPreparation'
        stages:
          type: array
          items:
            $ref: '#/components/schemas/StageRecord'
        team:
          $ref: '#/components/schemas/RevivalTeam'
        facility:
          $ref: '#/components/schemas/FacilityInfo'
        started_at:
          type: string
          format: date-time
        completed_at:
          type: string
          format: date-time
        last_updated:
          type: string
          format: date-time

    ProcedureSummary:
      type: object
      properties:
        procedure_id:
          type: string
        subject_id:
          type: string
        status:
          $ref: '#/components/schemas/RevivalStatus'
        current_stage:
          $ref: '#/components/schemas/RevivalStage'
        started_at:
          type: string
          format: date-time

    UpdateProcedureRequest:
      type: object
      properties:
        status:
          $ref: '#/components/schemas/RevivalStatus'
        notes:
          type: string

    StageRecord:
      type: object
      properties:
        stage:
          $ref: '#/components/schemas/RevivalStage'
        stage_number:
          type: integer
        status:
          type: string
          enum: [PENDING, IN_PROGRESS, COMPLETED, FAILED, SKIPPED]
        started_at:
          type: string
          format: date-time
        completed_at:
          type: string
          format: date-time
        duration:
          type: string
        parameters:
          type: object
        complications_count:
          type: integer
        interventions_count:
          type: integer

    TransitionRequest:
      type: object
      required:
        - current_vitals
        - approvals
      properties:
        current_vitals:
          $ref: '#/components/schemas/VitalSigns'
        current_labs:
          $ref: '#/components/schemas/LabResults'
        approvals:
          type: array
          items:
            type: object
            properties:
              role:
                type: string
              member_id:
                type: string
              approved_at:
                type: string
                format: date-time

    TransitionResult:
      type: object
      properties:
        success:
          type: boolean
        previous_stage:
          $ref: '#/components/schemas/RevivalStage'
        new_stage:
          $ref: '#/components/schemas/RevivalStage'
        transitioned_at:
          type: string
          format: date-time

    TransitionDenied:
      type: object
      properties:
        success:
          type: boolean
          example: false
        current_stage:
          $ref: '#/components/schemas/RevivalStage'
        issues:
          type: array
          items:
            type: string

    WarmingStatus:
      type: object
      properties:
        current_temperature:
          type: number
        target_temperature:
          type: number
        phase:
          type: string
        optimal_rate:
          type: number
        is_complete:
          type: boolean
        alerts:
          type: array
          items:
            type: string

    WarmingProfile:
      type: object
      properties:
        phases:
          type: array
          items:
            type: object
            properties:
              start_temp:
                type: number
              end_temp:
                type: number
              warming_rate:
                type: number
              duration_minutes:
                type: number
              phase_name:
                type: string

    TemperatureReading:
      type: object
      required:
        - temperature
        - location
      properties:
        temperature:
          type: number
        location:
          type: string
          enum: [CORE, BRAIN, PERIPHERAL]
        source:
          type: string

    TemperatureRecordResult:
      type: object
      properties:
        recorded:
          type: boolean
        alert:
          type: string
        current_status:
          $ref: '#/components/schemas/WarmingStatus'

    CardiovascularStatus:
      type: object
      properties:
        heart_rate:
          type: integer
        rhythm:
          type: string
        systolic_bp:
          type: integer
        diastolic_bp:
          type: integer
        mean_arterial_pressure:
          type: integer
        cardiac_output:
          type: number
        restart_successful:
          type: boolean
        defibrillation_attempts:
          type: integer

    CardiovascularUpdate:
      type: object
      properties:
        heart_rate:
          type: integer
        rhythm:
          type: string
        systolic_bp:
          type: integer
        diastolic_bp:
          type: integer
        cardiac_output:
          type: number
        cvp:
          type: integer

    CardiovascularUpdateResult:
      type: object
      properties:
        status:
          $ref: '#/components/schemas/CardiovascularStatus'
        in_target:
          type: object
          additionalProperties:
            type: boolean
        recommendations:
          type: object
          properties:
            action:
              type: string
            details:
              type: array
              items:
                type: string
            medications:
              type: array
              items:
                type: string

    DefibrillationRecord:
      type: object
      required:
        - energy_joules
        - rhythm_before
        - rhythm_after
      properties:
        energy_joules:
          type: integer
        rhythm_before:
          type: string
        rhythm_after:
          type: string

    DefibrillationResult:
      type: object
      properties:
        attempt_number:
          type: integer
        success:
          type: boolean
        rhythm_achieved:
          type: string
        next_action:
          type: object

    NeurologicalStatus:
      type: object
      properties:
        consciousness_level:
          type: string
        gcs_score:
          type: integer
        eeg_pattern:
          type: string
        pupil_response:
          type: object
        reflexes_present:
          type: integer
        icp:
          type: number
        alerts:
          type: array
          items:
            type: string
        restoration_successful:
          type: boolean

    EEGReading:
      type: object
      required:
        - pattern
      properties:
        pattern:
          type: string
          enum: [FLAT, BURST_SUPPRESSION, SLOW_WAVE, NORMAL]
        frequency_hz:
          type: number
        amplitude_uv:
          type: number
        seizure_activity:
          type: boolean

    EEGRecordResult:
      type: object
      properties:
        recorded:
          type: boolean
        pattern:
          type: string
        progression:
          type: string
        alert:
          type: string

    ConsciousnessAssessment:
      type: object
      required:
        - eye_response
        - verbal_response
        - motor_response
      properties:
        eye_response:
          type: integer
          minimum: 1
          maximum: 4
        verbal_response:
          type: integer
          minimum: 1
          maximum: 5
        motor_response:
          type: integer
          minimum: 1
          maximum: 6

    ConsciousnessResult:
      type: object
      properties:
        gcs_score:
          type: integer
        consciousness_level:
          type: string
        trend:
          type: string

    VitalSigns:
      type: object
      properties:
        timestamp:
          type: string
          format: date-time
        temperature:
          type: number
        heart_rate:
          type: integer
        systolic_bp:
          type: integer
        diastolic_bp:
          type: integer
        respiratory_rate:
          type: integer
        oxygen_saturation:
          type: number
        end_tidal_co2:
          type: number
        cvp:
          type: integer

    VitalsRecordResult:
      type: object
      properties:
        recorded:
          type: boolean
        alerts:
          type: array
          items:
            $ref: '#/components/schemas/Alert'

    VitalsTrends:
      type: object
      properties:
        temperature_trend:
          type: string
        heart_rate_trend:
          type: string
        blood_pressure_trend:
          type: string

    LabResults:
      type: object
      properties:
        category:
          type: string
        tests:
          type: array
          items:
            type: object
            properties:
              name:
                type: string
              value:
                type: number
              unit:
                type: string
              reference_range:
                type: string
              flag:
                type: string

    LabRecordResult:
      type: object
      properties:
        recorded:
          type: boolean
        critical_values:
          type: array
          items:
            type: string
        alerts:
          type: array
          items:
            $ref: '#/components/schemas/Alert'

    ComplicationRecord:
      type: object
      required:
        - type
        - severity
        - description
      properties:
        type:
          type: string
        severity:
          type: string
          enum: [MILD, MODERATE, SEVERE, LIFE_THREATENING]
        description:
          type: string
        detected_by:
          type: string

    Complication:
      type: object
      properties:
        complication_id:
          type: string
        timestamp:
          type: string
          format: date-time
        stage:
          $ref: '#/components/schemas/RevivalStage'
        type:
          type: string
        severity:
          type: string
        description:
          type: string
        resolved:
          type: boolean

    InterventionRecord:
      type: object
      required:
        - type
        - indication
        - description
        - performed_by
      properties:
        type:
          type: string
        indication:
          type: string
        description:
          type: string
        performed_by:
          type: string
        assisted_by:
          type: array
          items:
            type: string

    Intervention:
      type: object
      properties:
        intervention_id:
          type: string
        timestamp:
          type: string
          format: date-time
        stage:
          $ref: '#/components/schemas/RevivalStage'
        type:
          type: string
        indication:
          type: string
        description:
          type: string
        outcome:
          type: string

    RevivalTeam:
      type: object
      properties:
        team_id:
          type: string
        director:
          $ref: '#/components/schemas/TeamMember'
        members:
          type: array
          items:
            $ref: '#/components/schemas/TeamMember'
        current_shift:
          type: string

    TeamMember:
      type: object
      properties:
        member_id:
          type: string
        name:
          type: string
        role:
          type: string
        specialization:
          type: string
        on_site:
          type: boolean
        current_assignment:
          type: string

    TeamMemberAssignment:
      type: object
      required:
        - member_id
        - role
      properties:
        member_id:
          type: string
        role:
          type: string
        responsibilities:
          type: array
          items:
            type: string

    ShiftSchedule:
      type: object
      properties:
        shift_id:
          type: string
        shift_number:
          type: integer
        start_time:
          type: string
          format: date-time
        end_time:
          type: string
          format: date-time
        team_on_duty:
          type: array
          items:
            type: object
            properties:
              member_id:
                type: string
              role:
                type: string

    HandoverRecord:
      type: object
      required:
        - from_shift
        - to_shift
        - notes
      properties:
        from_shift:
          type: string
        to_shift:
          type: string
        notes:
          type: string
        pending_issues:
          type: array
          items:
            type: string
        handover_time:
          type: string
          format: date-time

    AlertSeverity:
      type: string
      enum: [INFO, WARNING, CRITICAL, EMERGENCY]

    Alert:
      type: object
      properties:
        alert_id:
          type: string
        timestamp:
          type: string
          format: date-time
        severity:
          $ref: '#/components/schemas/AlertSeverity'
        category:
          type: string
        source:
          type: string
        message:
          type: string
        acknowledged:
          type: boolean
        acknowledged_by:
          type: string
        resolved:
          type: boolean
        resolution:
          type: string

    AlertSummary:
      type: object
      properties:
        total_alerts:
          type: integer
        active_alerts:
          type: integer
        emergency_count:
          type: integer
        critical_count:
          type: integer

    RePreservationStatus:
      type: object
      properties:
        initialized:
          type: boolean
        trigger:
          type: string
        criteria:
          type: array
          items:
            type: object
            properties:
              criterion_id:
                type: string
              description:
                type: string
              met:
                type: boolean
              evidence:
                type: string
        criteria_score:
          type: number
        votes:
          type: array
          items:
            $ref: '#/components/schemas/TeamVote'
        decision:
          type: string

    RePreservationInit:
      type: object
      required:
        - trigger
      properties:
        trigger:
          type: string
          enum:
            - IRREVERSIBLE_CARDIAC_FAILURE
            - IRREVERSIBLE_BRAIN_DAMAGE
            - MULTIPLE_ORGAN_FAILURE
            - SUBJECT_REQUEST
            - GUARDIAN_REQUEST
            - PROTOCOL_THRESHOLD

    CriterionEvaluation:
      type: object
      required:
        - criterion_id
        - met
        - evidence
      properties:
        criterion_id:
          type: string
        met:
          type: boolean
        evidence:
          type: string

    CriterionResult:
      type: object
      properties:
        criterion_id:
          type: string
        met:
          type: boolean
        current_score:
          type: number

    TeamVote:
      type: object
      required:
        - member_id
        - role
        - vote
        - rationale
      properties:
        member_id:
          type: string
        role:
          type: string
        vote:
          type: string
          enum: [PROCEED, OPPOSE, ABSTAIN]
        rationale:
          type: string

    VoteResult:
      type: object
      properties:
        recorded:
          type: boolean
        total_votes:
          type: integer
        vote_summary:
          type: object
          properties:
            PROCEED:
              type: integer
            OPPOSE:
              type: integer
            ABSTAIN:
              type: integer

    RePreservationDecision:
      type: object
      properties:
        decision:
          type: string
          enum: [REPRESERVE, CONTINUE, REVIEW]
        rationale:
          type: string
        criteria_score:
          type: number
        vote_summary:
          type: object
        timestamp:
          type: string
          format: date-time

    DecisionBlocked:
      type: object
      properties:
        can_decide:
          type: boolean
          example: false
        issues:
          type: array
          items:
            type: string

    RevivalOutcome:
      type: object
      properties:
        outcome_id:
          type: string
        procedure_id:
          type: string
        result:
          type: string
          enum:
            - FULL_SUCCESS
            - PARTIAL_SUCCESS
            - SURVIVAL_WITH_DEFICITS
            - FAILURE_REPRESERVED
            - FAILURE_DEATH
            - ABORTED
        survival_status:
          type: string
        functional_assessment:
          type: object
        complications_summary:
          type: array
          items:
            type: object
        recovery_prognosis:
          type: object
        quality_metrics:
          type: object
        documented_at:
          type: string
          format: date-time

    OutcomeRecord:
      type: object
      required:
        - result
        - survival_status
      properties:
        result:
          type: string
        survival_status:
          type: string
        functional_assessment:
          type: object
        recovery_prognosis:
          type: object
        documented_by:
          type: string

    RevivalPreparation:
      type: object
      properties:
        assessment_id:
          type: string
        scheduled_date:
          type: string
          format: date-time
        readiness_score:
          type: number
        ready_for_revival:
          type: boolean

    FacilityInfo:
      type: object
      properties:
        facility_id:
          type: string
        facility_name:
          type: string
        location:
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
        details:
          type: object
```

## WebSocket Protocol

### Real-time Monitoring Connection

```javascript
// WebSocket connection for real-time monitoring
const ws = new WebSocket('wss://api.wia-revival.org/v1/procedures/{procedureId}/ws');

// Authentication
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'AUTH',
    token: 'Bearer <jwt_token>'
  }));
};

// Subscribe to updates
ws.send(JSON.stringify({
  type: 'SUBSCRIBE',
  channels: ['vitals', 'alerts', 'stages']
}));

// Message types
// VITALS_UPDATE - New vital signs
// ALERT_NEW - New alert generated
// ALERT_ACKNOWLEDGED - Alert acknowledged
// STAGE_TRANSITION - Stage changed
// INTERVENTION - New intervention recorded
// COMPLICATION - New complication recorded
```

---

*WIA Technical Committee - Cryopreservation Working Group*
