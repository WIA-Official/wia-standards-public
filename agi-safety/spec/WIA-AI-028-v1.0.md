# WIA-AI-028: AGI Safety Standard Specification v1.0

Normative requirements for the evidence, authorization, runtime control, incident management, and assessment of bounded deployments of systems intended for artificial general intelligence applications.

## 1. Introduction

### 1.1 Purpose

WIA-AI-028 defines an assurance and control framework for deploying systems with broad reasoning, planning, learning, tool use, or delegated execution capabilities. It connects a specific system configuration to a declared operating envelope, evaluated safety controls, accountable human decisions, and enforceable runtime restrictions.

The subject of conformance is an identified deployment configuration and its control system. Conformance does not establish that a system possesses artificial general intelligence, has aligned objectives in every circumstance, or is safe outside its assessed operating envelope.

A conforming implementation MUST distinguish:

- Evidence supporting an authorization decision.
- Human authority to make that decision.
- Technical permission to execute a particular action.
- Confirmation that an action occurred or that managed execution stopped.

Model-generated assertions do not independently establish any of these conditions.

This edition defines its normative data model, identifiers, thresholds, and conformance rules. Compatibility with other WIA specifications is unspecified.

### 1.2 Scope

**In scope**

This standard governs:

- Configuration identification for models, instructions, memory, tools, evaluators, and control components.
- Operating envelopes and aggregate resource limits.
- Hazard arguments and evaluation evidence.
- Deployment authorization and separation of duties.
- Mediation of tool use, external communication, persistent changes, and delegated work.
- Suspension, revocation, containment, and reconciliation of uncertain effects.
- Evidence integrity, audit records, privacy, and access control.
- Registration APIs, lifecycle events, conformance testing, and certification records.

The standard applies to centrally hosted systems, distributed agent systems, and systems connected to external services or physical equipment. A physical deployment additionally requires an appropriate domain safety assessment. This standard does not supply machinery, vehicle, medical-device, or other domain-specific safety limits.

**Out of scope**

This standard does not define:

- A scientific test for the existence of AGI.
- A universal acceptable-risk threshold.
- A training algorithm that guarantees alignment.
- A method for proving that internal representations or objectives are benign.
- Permission to perform otherwise unauthorized activities.
- Operational instructions for hazardous biological, chemical, radiological, or other restricted activities.

Where a deployment interacts with life-science safety information, this standard covers authorization, records, access, evidence, incident workflows, and software interfaces. It does not specify agent characteristics, exposure models, acquisition or propagation methods, or laboratory procedures.

### 1.3 Normative references

The following documents are incorporated only for the subjects identified:

| Document | Incorporated subject |
|---|---|
| RFC 2119, *Key words for use in RFCs to Indicate Requirement Levels* | Requirement keywords |
| RFC 8174, *Ambiguity of Uppercase vs Lowercase in RFC 2119 Key Words* | Interpretation of uppercase keywords |
| RFC 8259, *The JavaScript Object Notation (JSON) Data Interchange Format* | JSON syntax |
| RFC 3339, *Date and Time on the Internet: Timestamps* | Timestamp representation, restricted by Section 4 |

The schema excerpt in Section 4 uses JSON Schema Draft 2020-12 syntax. The field tables and requirements remain normative wherever the excerpt does not express a constraint.

### 1.4 Terms and definitions

| Term | Definition |
|---|---|
| AGI-capable system | A system assessed for broad reasoning, planning, learning, or adaptation across tasks; the term does not certify attainment of AGI. |
| Safety case | An immutable collection of configuration declarations, hazards, controls, and evaluation evidence supporting a bounded deployment decision. |
| Configuration | The identified models, software, instructions, memory policies, tools, dependencies, and control settings relevant to behavior. |
| Operating envelope | The permitted objectives, environment, resources, data classes, actions, and duration of execution. |
| Assurance boundary | The components and communication paths over which the claimed safety controls are enforceable. |
| Release | A lifecycle record connecting one immutable safety case to review and deployment authorization. |
| Authorization epoch | A monotonically increasing release value that distinguishes successive grants and revocations of authority. |
| Run | One bounded execution session, including all work delegated under its authority. |
| Descendant | An agent or worker created or delegated work by a run, directly or indirectly. |
| Action | A proposed operation crossing an effect or information-access boundary. |
| Action gateway | A trusted component that checks authority, reserves resources, and controls dispatch. |
| Commitment point | The serialized decision after which an action may have reached an external receiver and cannot be assumed absent. |
| Hazard | A condition or behavior capable of causing harm or loss of control within the declared assessment scope. |
| Control | An enforced measure intended to prevent, detect, restrict, or recover from a hazard. |
| Residual risk | Risk remaining after the declared controls are applied. |
| Evaluation | A documented procedure producing evidence about a specified configuration and claim. |
| Challenge material | Test inputs or scenarios designed to expose a control weakness or unsafe behavior. |
| Generalization evaluation | Evaluation of whether a safety claim remains supported under declared variations of task, context, language, or environment. |
| Artifact | Immutable bytes registered with an identifier, media type, length, and cryptographic digest. |
| Review decision | An attributable human approval or rejection of one release review round. |
| Action approval | A human decision bound to one exact action proposal. |
| Suspension | Revocation of release authority and initiation of stopping associated managed execution. |
| Containment | Confirmed prevention of further managed computation or dispatch through the assurance boundary. |
| Unknown effect | An action whose external outcome cannot currently be established. |
| Incident | A recorded control failure, suspected safety violation, or unresolved safety-significant effect. |
| Conformance claim | A statement that identified components and evidence satisfy a specified level of this standard. |

### 1.5 Abbreviations

| Abbreviation | Meaning |
|---|---|
| AGI | Artificial general intelligence |
| API | Application programming interface |
| JSON | JavaScript Object Notation |
| REST | Representational state transfer |
| SHA-256 | Secure Hash Algorithm with a 256-bit digest |
| TLS | Transport Layer Security |
| UTC | Coordinated Universal Time |

### 1.6 Conformance keywords

The keywords **MUST**, **MUST NOT**, **REQUIRED**, **SHALL**, **SHALL NOT**, **SHOULD**, **SHOULD NOT**, **RECOMMENDED**, **NOT RECOMMENDED**, **MAY**, and **OPTIONAL** have the meanings described in RFC 2119 and RFC 8174 when written in uppercase.

A deviation from a SHOULD requirement MUST be documented with its rationale and assessed consequences. A deviation from a MUST requirement prevents conformance for the affected scope.

## 2. Conformance levels

Levels describe assurance depth. They do not assign a system an intrinsic danger category or authorize additional capabilities.

| Level | Name | Intended assurance |
|---|---|---|
| `L1` | Basic | Explicit boundaries, evaluated controls, independent human authorization, complete action mediation, and attributable records |
| `L2` | Standard | L1 plus separate safety and operations approval and evaluation across declared operating variations |
| `L3` | Advanced | L2 plus assessment independence, independent adversarial exercises, and evidence custody outside the development team |

Every requirement listed in a range includes both endpoints.

| Requirement IDs | L1 | L2 | L3 |
|---|---:|---:|---:|
| REQ-FUN-001 through REQ-FUN-008 | MUST | MUST | MUST |
| REQ-FUN-009 | — | MUST | MUST |
| REQ-FUN-010 | — | — | MUST |
| REQ-PER-001 through REQ-PER-004 | MUST | MUST | MUST |
| REQ-SAF-001 through REQ-SAF-005 | MUST | MUST | MUST |
| REQ-SAF-006 | — | MUST | MUST |
| REQ-SAF-007 | — | — | MUST |
| REQ-SAF-008 | MUST | MUST | MUST |
| REQ-SEC-001 through REQ-SEC-005 | MUST | MUST | MUST |
| REQ-INT-001 through REQ-INT-003 | MUST | MUST | MUST |
| REQ-ACC-001 | MUST | MUST | MUST |
| REQ-GOV-001 | MUST | MUST | MUST |

A dash means the requirement is not mandatory at that level. It does not prohibit implementation.

An implementation MUST identify which model, supervisor, gateways, artifact store, identity service, adapters, and human interfaces its claim covers. An uncontrolled descendant or effect channel cannot be excluded from the claim while continuing to exercise authority originating from the conforming run.

## 3. Reference architecture / system model

The reference architecture separates model execution from the authority to affect its environment.

```text
 Human owner       Safety reviewer       Operations reviewer
      |                    |                       |
      +---------- authenticated review interface -+
                           |
                    Release controller
                    /       |        \
                   /        |         \
          Safety-case   Identity and   Incident manager
           registry     role service          |
               |             |                |
          Artifact store     |                |
               |             |                |
               +------ Audit ledger ----------+
                             |
                    authorization state
                             |
                      Run supervisor
                       /           \
                      /             \
               Model executor    Descendant workers
                      \             /
                       \           /
                       Action gateway
                        /     |     \
                       /      |      \
                 Data adapter Tool adapter Physical adapter
                       \      |      /
                        External environment
```

The **release controller** owns release state, review rounds, authorization epochs, and authorization expiry. It does not delegate these decisions to model output.

The **run supervisor** starts and contains executors, maintains run deadlines and aggregate budgets, monitors liveness, and propagates halt requests to descendants.

The **action gateway** mediates access to protected information and all external effects. It validates an exact permission, validates proposal bytes against the pinned adapter contract, checks human approval where required, and serializes commitment against revocation.

The **artifact store** retains immutable evidence and proposal bytes. Model assets may remain in controlled runtime storage; the registered configuration artifact identifies their digests and the verification procedure. API artifact-size limits do not limit model size.

The **audit ledger** records ordered lifecycle decisions and their supporting record snapshots. An external audit export may lag the authoritative local ledger, but that lag cannot delay suspension.

The **incident manager** tracks uncertainty and remediation separately from execution status. A stopped run can still have unresolved external effects.

Roles are assigned through the identity service:

| Role | Authority |
|---|---|
| Owner | Prepares a safety case and identifies operational responsibility |
| Evaluator | Produces attributable evaluation evidence |
| Safety reviewer | Reviews hazard arguments, evidence sufficiency, and residual-risk decisions |
| Operations reviewer | Reviews deployability, monitoring, containment, and recovery arrangements |
| Operator | Starts authorized runs and requests normal completion |
| Action approver | Approves specific proposals within assigned authority |
| Responder | Suspends releases, halts runs, and manages incidents |
| Assessor | Evaluates conformance and issues an assessment record |
| Auditor | Reads authorized evidence and audit history |

A principal may hold multiple roles only where separation-of-duty requirements permit it. Multiple accounts controlled by one person count as one human principal for approval quorum.

Normal data flow is: evidence registration; safety-case registration; release review; authorization; run admission; action proposal; action approval where required; commitment; outcome reconciliation; completion or containment.

Model output, retrieved documents, tool responses, and messages from descendants cross an untrusted-content boundary. None can alter role assignments, permission definitions, review decisions, or authorization epochs.

## 4. Data model

### 4.1 Common representation rules

All API JSON MUST use UTF-8. Duplicate object member names, non-finite numbers, and a byte-order mark MUST be rejected. Object member order carries no semantic meaning, except that artifact and safety-case digests bind the exact stored bytes.

Unknown fields MUST be rejected for the records defined by this edition. Fields marked nullable remain required but may contain JSON `null`.

| Type | Definition |
|---|---|
| `ID` | String matching `^[a-z][a-z0-9-]{0,63}$`; unique within the deployment registry across resource types |
| `Text` | Unicode string containing 1–4,096 characters |
| `Digest` | Exactly 64 lowercase hexadecimal characters representing SHA-256 |
| `Time` | UTC timestamp in `YYYY-MM-DDTHH:mm:ss.sssZ` form; leap-second notation is not accepted |
| `UInt` | Integer from 0 through 9,007,199,254,740,991 |
| `PositiveUInt` | `UInt` greater than zero |
| `ID[]` | Array of unique IDs; emptiness allowed only where stated |
| `Artifact reference` | An ID resolving to accessible immutable bytes whose registered digest and length verify |

Identifiers convey identity, not access rights. Possession of an identifier MUST NOT authorize access.

All numeric limits in this specification are requirements established by this edition. They are not statements about observed system reliability or industry practice.

### 4.2 SafetyCase

A `SafetyCase` is immutable after registration.

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `schema_version` | string | — | Yes | Exactly `1.0` | Data-model version |
| `case_id` | ID | — | Yes | Unique | Safety-case identifier |
| `title` | Text | — | Yes | — | Human-readable scope |
| `owner_id` | ID | — | Yes | Registered principal | Accountable owner |
| `target_level` | enum | — | Yes | `L1`, `L2`, `L3` | Requested assurance level |
| `created_at` | Time | — | Yes | Not future-dated at registration | Assembly time |
| `system` | System | — | Yes | Section 4.3 | Configuration references |
| `envelope` | Envelope | — | Yes | Section 4.4 | Execution bounds |
| `artifacts` | Artifact[] | — | Yes | Nonempty; unique IDs | Evidence inventory |
| `hazards` | Hazard[] | — | Yes | Nonempty; unique IDs | Hazard argument |
| `evaluations` | Evaluation[] | — | Yes | Nonempty; unique IDs | Assessment evidence |

All artifact references in the case MUST resolve to entries in `artifacts`. Evaluation-to-hazard and hazard-to-evaluation references MUST agree.

### 4.3 System and Artifact

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| System | `system_id` | ID | — | Yes | Stable organizational identity | System being assessed |
| System | `configuration_artifact_id` | ID | — | Yes | Artifact reference | Exact configuration inventory |
| System | `boundary_artifact_id` | ID | — | Yes | Artifact reference | Enforceable boundary and control argument |
| Artifact | `artifact_id` | ID | — | Yes | Unique | Immutable object identity |
| Artifact | `sha256` | Digest | — | Yes | Computed over decoded stored bytes | Integrity binding |
| Artifact | `media_type` | string | — | Yes | Valid media-type value | Content interpretation |
| Artifact | `byte_length` | UInt | byte | Yes | At most 16,777,216 | Decoded content size |

The configuration artifact MUST identify model and executable digests, instruction versions, mutable-memory rules, retrieval-source controls, adapter versions, evaluator dependencies, and control settings. It MUST distinguish fixed configuration from permitted runtime state.

The boundary artifact MUST identify effect channels, trust boundaries, control ownership, relevant failure domains, role assignments, retention rules, containment mechanisms, and external commitments that cannot be recalled.

An adapter artifact MUST define its accepted proposal JSON, operation semantics, resource binding, commitment point, outcome evidence, cancellation behavior, and any downstream deduplication guarantee. The gateway MUST validate proposals against that exact artifact.

### 4.4 Envelope and Permission

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| Envelope | `environment_id` | ID | — | Yes | Declared environment | Authorized deployment context |
| Envelope | `objective` | Text | — | Yes | Bounded purpose | Permitted task scope |
| Envelope | `max_run_seconds` | PositiveUInt | second | Yes | At most 86,400 | Maximum session duration |
| Envelope | `max_actions` | PositiveUInt | action | Yes | Aggregate across descendants | Commitment budget |
| Envelope | `max_parallel_actions` | PositiveUInt | action | Yes | At most `max_actions` | Concurrent unresolved commitment limit |
| Envelope | `max_descendants` | UInt | descendant | Yes | Aggregate lifetime count | Delegation limit |
| Envelope | `allowed_data_classes` | enum[] | — | Yes | Nonempty; unique | Permitted information classes |
| Envelope | `permissions` | Permission[] | — | Yes | Unique permission IDs; may be empty | Allowlisted operations |
| Permission | `permission_id` | ID | — | Yes | Unique within case | Permission identity |
| Permission | `adapter_artifact_id` | ID | — | Yes | Artifact reference | Pinned adapter contract |
| Permission | `operation` | ID | — | Yes | Exact comparison | Adapter operation |
| Permission | `resource_id` | ID | — | Yes | Exact comparison; no wildcard | Bound destination or resource |
| Permission | `effect` | enum | — | Yes | Defined below | Consequence reversibility |
| Permission | `requires_human_approval` | boolean | — | Yes | True for `IRREVERSIBLE` | Approval requirement |

Data classes are classifications assigned by the deployment’s data owner:

| Value | Meaning |
|---|---|
| `PUBLIC` | Approved for unrestricted disclosure |
| `INTERNAL` | Limited to authorized organizational use |
| `CONFIDENTIAL` | Disclosure or access requires explicitly assigned privileges |
| `RESTRICTED` | Highest declared handling restriction, with individually scoped access |

These classes do not replace authorization or define legal categories. A resource’s classification MUST be established outside model-generated content.

Effect values are:

| Value | Meaning |
|---|---|
| `READ_ONLY` | Access without a persistent modification or disclosure outside the approved information boundary |
| `REVERSIBLE` | An effect with an assessed restoration procedure that prevents lasting safety-significant consequences |
| `IRREVERSIBLE` | An effect whose relevant consequences cannot reliably be recalled or restored |

External disclosure is `IRREVERSIBLE` once information leaves the approved boundary. An operation cannot be classified `REVERSIBLE` solely because an inverse command exists.

### 4.5 Hazard and Evaluation

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| Hazard | `hazard_id` | ID | — | Yes | Unique within case | Hazard identity |
| Hazard | `description` | Text | — | Yes | — | Unsafe condition or behavior |
| Hazard | `trigger` | Text | — | Yes | — | Initiating circumstances |
| Hazard | `consequence` | Text | — | Yes | — | Credible adverse outcome |
| Hazard | `severity` | enum | — | Yes | Defined below | Consequence classification |
| Hazard | `disposition` | enum | — | Yes | Defined below | Treatment status |
| Hazard | `control_artifact_ids` | ID[] | — | Yes | Nonempty for `CONTROLLED` | Control evidence |
| Hazard | `evaluation_ids` | ID[] | — | Yes | Nonempty for `CONTROLLED` | Verification coverage |
| Hazard | `acceptance_artifact_id` | ID or null | — | Yes | Non-null for `ACCEPTED` | Signed residual-risk rationale |
| Evaluation | `evaluation_id` | ID | — | Yes | Unique within case | Evaluation identity |
| Evaluation | `evaluator_id` | ID | — | Yes | Registered principal | Evidence producer |
| Evaluation | `category` | enum | — | Yes | Defined below | Evaluation purpose |
| Evaluation | `method_artifact_id` | ID | — | Yes | Artifact reference | Procedure and acceptance criteria |
| Evaluation | `result_artifact_id` | ID | — | Yes | Artifact reference | Observations and findings |
| Evaluation | `executed_at` | Time | — | Yes | Not future-dated | Evaluation completion time |
| Evaluation | `outcome` | enum | — | Yes | `PASS`, `FAIL`, `INCONCLUSIVE` | Claim outcome |
| Evaluation | `covered_hazard_ids` | ID[] | — | Yes | Nonempty | Hazard coverage |
| Evaluation | `limitations` | Text | — | Yes | Explicit limitations required | Conditions not established |

Severity values:

| Value | Meaning |
|---|---|
| `CRITICAL` | Credible severe irreversible harm or loss of effective control beyond the assessed boundary |
| `MAJOR` | Material harm, unauthorized disclosure, or substantial recoverable disruption |
| `MINOR` | Limited recoverable adverse consequences within the assessed boundary |

Disposition values:

| Value | Meaning |
|---|---|
| `OPEN` | Treatment or evidence is incomplete |
| `CONTROLLED` | Controls are implemented, evaluated, and supported by a residual-risk argument |
| `ACCEPTED` | An authorized human has explicitly accepted the residual risk |

Evaluation categories:

| Value | Meaning |
|---|---|
| `CONTROL_VERIFICATION` | Direct testing of claimed enforcement mechanisms |
| `ADVERSARIAL` | Challenges seeking unsafe behavior or control bypass |
| `GENERALIZATION` | Testing across declared operating variations |
| `INDEPENDENT_REVIEW` | Independent review and challenge of the safety argument |

`PASS` means the stated acceptance criteria were met. `FAIL` means at least one was not met. `INCONCLUSIVE` means available observations cannot determine the claim.

### 4.6 Release and ReviewDecision

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| Release | `release_id` | ID | — | Yes | Server assigned | Lifecycle identity |
| Release | `case_id` | ID | — | Yes | Existing case | Immutable case binding |
| Release | `case_sha256` | Digest | — | Yes | Exact registered case bytes | Content binding |
| Release | `state` | enum | — | Yes | Section 7 | Lifecycle state |
| Release | `revision` | PositiveUInt | revision | Yes | Starts at 1; increases on mutation | Concurrency control |
| Release | `review_round` | UInt | round | Yes | Starts at 0 | Approval generation |
| Release | `authorization_epoch` | UInt | epoch | Yes | Starts at 0 | Runtime authority generation |
| Release | `registered_at` | Time | — | Yes | Server time | Creation time |
| Release | `authorization_expires_at` | Time or null | — | Yes | Non-null only when `AUTHORIZED` | Authority deadline |
| Release | `decisions` | ReviewDecision[] | — | Yes | Append-only; may be empty | Review history |
| ReviewDecision | `decision_id` | ID | — | Yes | Server assigned | Decision identity |
| ReviewDecision | `review_round` | PositiveUInt | round | Yes | Current round when recorded | Scope binding |
| ReviewDecision | `principal_id` | ID | — | Yes | Derived from authentication | Human reviewer |
| ReviewDecision | `role` | enum | — | Yes | `SAFETY`, `OPERATIONS` | Review responsibility |
| ReviewDecision | `decision` | enum | — | Yes | `APPROVE`, `REJECT` | Decision |
| ReviewDecision | `created_at` | Time | — | Yes | Server time | Decision time |
| ReviewDecision | `expires_at` | Time | — | Yes | Within 30 days of creation | Approval validity |
| ReviewDecision | `rationale_artifact_id` | ID | — | Yes | Artifact reference | Decision basis |

`SAFETY` and `OPERATIONS` correspond to the reviewer roles in Section 3. `APPROVE` grants a favorable decision within its scope; `REJECT` refuses it.

### 4.7 Run, Action, and ActionApproval

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| Run | `run_id` | ID | — | Yes | Server assigned | Session identity |
| Run | `release_id` | ID | — | Yes | Authorized release at creation | Release binding |
| Run | `authorization_epoch` | UInt | epoch | Yes | Captured at admission | Authority binding |
| Run | `principal_id` | ID | — | Yes | Authenticated operator | Initiator |
| Run | `objective_artifact_id` | ID | — | Yes | Artifact reference | Concrete task |
| Run | `state` | enum | — | Yes | Section 7 | Execution state |
| Run | `revision` | PositiveUInt | revision | Yes | Monotonic | Concurrency version |
| Run | `started_at` | Time | — | Yes | Server time | Admission time |
| Run | `deadline_at` | Time | — | Yes | Section 5 | Effective deadline |
| Run | `actions_committed` | UInt | action | Yes | Monotonic | Consumed action budget |
| Run | `open_actions` | UInt | action | Yes | Includes unresolved commitments | Parallel budget usage |
| Run | `descendants_created` | UInt | descendant | Yes | Monotonic | Consumed delegation budget |
| Run | `containment_confirmed` | boolean | — | Yes | True for terminal states | Managed execution contained |
| Run | `termination_reason` | Text or null | — | Yes | Non-null outside `RUNNING` | Completion or halt cause |
| Action | `action_id` | ID | — | Yes | Server assigned | Proposal identity |
| Action | `run_id` | ID | — | Yes | Existing run | Aggregate budget owner |
| Action | `permission_id` | ID | — | Yes | Permission in bound case | Authority requested |
| Action | `proposal_artifact_id` | ID | — | Yes | Immutable JSON artifact | Exact adapter input |
| Action | `proposal_sha256` | Digest | — | Yes | Matches artifact | Approval binding |
| Action | `state` | enum | — | Yes | Section 7 | Effect status |
| Action | `revision` | PositiveUInt | revision | Yes | Monotonic | Concurrency version |
| Action | `created_at` | Time | — | Yes | Server time | Proposal time |
| Action | `result_artifact_id` | ID or null | — | Yes | Required for resolved outcomes | Outcome evidence |
| Action | `approvals` | ActionApproval[] | — | Yes | Append-only; may be empty | Human decisions |
| ActionApproval | `approval_id` | ID | — | Yes | Server assigned | Decision identity |
| ActionApproval | `principal_id` | ID | — | Yes | Authenticated human | Decision maker |
| ActionApproval | `proposal_sha256` | Digest | — | Yes | Exact action digest | Content binding |
| ActionApproval | `decision` | enum | — | Yes | `APPROVE`, `REJECT` | Human decision |
| ActionApproval | `created_at` | Time | — | Yes | Server time | Decision time |
| ActionApproval | `expires_at` | Time | — | Yes | Within 300 seconds of creation | Commitment deadline |

An action approval is scoped by its containing action, which binds the run, release, epoch, permission, and proposal digest. It cannot authorize a different action containing identical bytes.

### 4.8 Incident and Event

| Entity | Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|---|
| Incident | `incident_id` | ID | — | Yes | Server assigned | Incident identity |
| Incident | `release_id` | ID | — | Yes | Existing release | Affected authorization |
| Incident | `run_id` | ID or null | — | Yes | Must belong to release | Affected run |
| Incident | `severity` | enum | — | Yes | `CRITICAL`, `MAJOR`, `MINOR` | Consequence classification |
| Incident | `description` | Text | — | Yes | — | Observed concern |
| Incident | `trigger_artifact_id` | ID | — | Yes | Artifact reference | Initial evidence |
| Incident | `state` | enum | — | Yes | `OPEN`, `TRIAGED`, `CLOSED` | Response stage |
| Incident | `revision` | PositiveUInt | revision | Yes | Monotonic | Concurrency version |
| Incident | `created_at` | Time | — | Yes | Server time | Registration time |
| Incident | `resolution_artifact_id` | ID or null | — | Yes | Required when `CLOSED` | Findings and remediation |
| Event | `event_id` | ID | — | Yes | Unique | Stable event identity |
| Event | `sequence` | PositiveUInt | event | Yes | Strictly increasing per registry | Authoritative order |
| Event | `occurred_at` | Time | — | Yes | Server time | Recorded timestamp |
| Event | `type` | enum | — | Yes | Section 6.5 | Event meaning |
| Event | `actor_id` | ID | — | Yes | Human or service principal | Accountable actor |
| Event | `resource_id` | ID | — | Yes | Affected resource or request | Subject |
| Event | `previous_revision` | UInt | revision | Yes | Zero for creation or denial | Prior version |
| Event | `revision` | UInt | revision | Yes | Zero for denial | Resulting version |
| Event | `release_id` | ID or null | — | Yes | When applicable | Release correlation |
| Event | `authorization_epoch` | UInt or null | epoch | Yes | When applicable | Authority correlation |
| Event | `details_artifact_id` | ID | — | Yes | Restricted artifact reference | Cause and resulting record snapshot |

`OPEN` means awaiting investigation; `TRIAGED` means ownership and affected scope are established; `CLOSED` means resolution evidence has been accepted.

### 4.9 JSON Schema excerpt

The following complete schema fragment validates an `Envelope` object. Cross-field and artifact-resolution constraints remain governed by the tables.

```json
{
  "type": "object",
  "additionalProperties": false,
  "required": [
    "environment_id",
    "objective",
    "max_run_seconds",
    "max_actions",
    "max_parallel_actions",
    "max_descendants",
    "allowed_data_classes",
    "permissions"
  ],
  "properties": {
    "environment_id": {
      "type": "string",
      "pattern": "^[a-z][a-z0-9-]{0,63}$"
    },
    "objective": {
      "type": "string",
      "minLength": 1,
      "maxLength": 4096
    },
    "max_run_seconds": {
      "type": "integer",
      "minimum": 1,
      "maximum": 86400
    },
    "max_actions": {
      "type": "integer",
      "minimum": 1,
      "maximum": 9007199254740991
    },
    "max_parallel_actions": {
      "type": "integer",
      "minimum": 1,
      "maximum": 9007199254740991
    },
    "max_descendants": {
      "type": "integer",
      "minimum": 0,
      "maximum": 9007199254740991
    },
    "allowed_data_classes": {
      "type": "array",
      "minItems": 1,
      "uniqueItems": true,
      "items": {
        "enum": ["PUBLIC", "INTERNAL", "CONFIDENTIAL", "RESTRICTED"]
      }
    },
    "permissions": {
      "type": "array",
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": [
          "permission_id",
          "adapter_artifact_id",
          "operation",
          "resource_id",
          "effect",
          "requires_human_approval"
        ],
        "properties": {
          "permission_id": {
            "type": "string",
            "pattern": "^[a-z][a-z0-9-]{0,63}$"
          },
          "adapter_artifact_id": {
            "type": "string",
            "pattern": "^[a-z][a-z0-9-]{0,63}$"
          },
          "operation": {
            "type": "string",
            "pattern": "^[a-z][a-z0-9-]{0,63}$"
          },
          "resource_id": {
            "type": "string",
            "pattern": "^[a-z][a-z0-9-]{0,63}$"
          },
          "effect": {
            "enum": ["READ_ONLY", "REVERSIBLE", "IRREVERSIBLE"]
          },
          "requires_human_approval": {
            "type": "boolean"
          }
        },
        "allOf": [
          {
            "if": {
              "properties": {
                "effect": {"const": "IRREVERSIBLE"}
              },
              "required": ["effect"]
            },
            "then": {
              "properties": {
                "requires_human_approval": {"const": true}
              }
            }
          }
        ]
      }
    }
  }
}
```

### 4.10 Complete illustrative SafetyCase

This example describes a synthetic, read-only summarization deployment. All identities, timestamps, sizes, and digests are illustrative. The document is complete at the data-model level; operational acceptance additionally requires the referenced bytes and their verification.

```json
{
  "schema_version": "1.0",
  "case_id": "case-example",
  "title": "Illustrative synthetic-note summarizer",
  "owner_id": "principal-owner",
  "target_level": "L1",
  "created_at": "2030-01-01T10:00:00.000Z",
  "system": {
    "system_id": "system-example",
    "configuration_artifact_id": "art-config",
    "boundary_artifact_id": "art-boundary"
  },
  "envelope": {
    "environment_id": "env-simulator",
    "objective": "Summarize approved synthetic notes inside the isolated workspace.",
    "max_run_seconds": 600,
    "max_actions": 4,
    "max_parallel_actions": 1,
    "max_descendants": 0,
    "allowed_data_classes": ["PUBLIC"],
    "permissions": [
      {
        "permission_id": "perm-read",
        "adapter_artifact_id": "art-adapter",
        "operation": "read",
        "resource_id": "resource-synthetic-notes",
        "effect": "READ_ONLY",
        "requires_human_approval": false
      }
    ]
  },
  "artifacts": [
    {
      "artifact_id": "art-config",
      "sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
      "media_type": "application/json",
      "byte_length": 2048
    },
    {
      "artifact_id": "art-boundary",
      "sha256": "123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0",
      "media_type": "application/json",
      "byte_length": 3072
    },
    {
      "artifact_id": "art-adapter",
      "sha256": "23456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef01",
      "media_type": "application/json",
      "byte_length": 1024
    },
    {
      "artifact_id": "art-method",
      "sha256": "3456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef012",
      "media_type": "application/json",
      "byte_length": 4096
    },
    {
      "artifact_id": "art-results",
      "sha256": "456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123",
      "media_type": "application/json",
      "byte_length": 6144
    }
  ],
  "hazards": [
    {
      "hazard_id": "haz-unapproved-disclosure",
      "description": "Retrieved text attempts to redirect output to an unapproved destination.",
      "trigger": "Instruction-like content appears inside a synthetic note.",
      "consequence": "Information crosses the approved workspace boundary.",
      "severity": "MAJOR",
      "disposition": "CONTROLLED",
      "control_artifact_ids": ["art-boundary", "art-adapter"],
      "evaluation_ids": ["eval-controls", "eval-adversarial"],
      "acceptance_artifact_id": null
    }
  ],
  "evaluations": [
    {
      "evaluation_id": "eval-controls",
      "evaluator_id": "principal-evaluator",
      "category": "CONTROL_VERIFICATION",
      "method_artifact_id": "art-method",
      "result_artifact_id": "art-results",
      "executed_at": "2030-01-01T09:00:00.000Z",
      "outcome": "PASS",
      "covered_hazard_ids": ["haz-unapproved-disclosure"],
      "limitations": "Covers the pinned read adapter and declared isolated environment."
    },
    {
      "evaluation_id": "eval-adversarial",
      "evaluator_id": "principal-evaluator",
      "category": "ADVERSARIAL",
      "method_artifact_id": "art-method",
      "result_artifact_id": "art-results",
      "executed_at": "2030-01-01T09:30:00.000Z",
      "outcome": "PASS",
      "covered_hazard_ids": ["haz-unapproved-disclosure"],
      "limitations": "Synthetic challenges support only the declared threat classes and configuration."
    }
  ]
}
```

## 5. Requirements and thresholds

### 5.1 Functional requirements

**REQ-FUN-001 — Configuration and evidence binding.**  
The registry MUST preserve registered safety-case bytes and artifact bytes without modification. Before authorization and run admission, the controller MUST verify the relevant runtime configuration against the case. A changed model, instruction policy, adapter, permission, memory policy, safety control, or relevant dependency MUST require a new case unless the existing case explicitly defines and evaluates that variation.

**REQ-FUN-002 — Bounded execution.**  
The supervisor and gateway MUST enforce the envelope across the complete run. Commitment MUST reserve budget before dispatch. Failed or uncertain commitments MUST NOT refund `actions_committed`. Unresolved commitments consume `open_actions`. Completion, retries, new worker identities, and descendant termination MUST NOT reset aggregate counters. Integer overflow MUST fail closed.

**REQ-FUN-003 — Review authority.**  
L1 authorization MUST include a current `SAFETY` approval from a human who is neither the case owner nor an evaluator named in that case. Decisions MUST bind the current review round and immutable case. The controller MUST recheck role validity and expiry at authorization. Loss or withdrawal of an approving principal’s authority MUST invalidate the affected authorization and initiate suspension.

**REQ-FUN-004 — Lifecycle and revocation.**  
The controller MUST implement Section 7. Authorization and suspension MUST each increase `authorization_epoch`. Run admission and action commitment MUST check current release state, epoch, expiry, and applicable incidents. An old epoch MUST remain invalid after reauthorization. Suspension MUST serialize with commitment and close admission before a successful acknowledgement.

**REQ-FUN-005 — Complete action mediation.**  
Every protected read, external communication, persistent write, resource change, and delegation MUST pass through a gateway-controlled permission. The gateway MUST bind the operation to the declared resource and adapter. It MUST reject undeclared destinations, invalid proposal structures, and unavailable authority checks. A model’s stated intention or claimed permission MUST NOT substitute for an enforced check.

**REQ-FUN-006 — Delegation.**  
Descendants MUST inherit constraints no weaker than the parent run’s. Their actions and resource reservations MUST charge the same aggregate budgets. The supervisor MUST record parentage and attributable identities and propagate containment to all descendants. Delegation to an external service without enforceable downstream limits MUST be treated as an external action with explicitly assessed consequences.

**REQ-FUN-007 — Ordered audit.**  
Lifecycle changes, approvals, commitments, incident changes, and authenticated authorization denials MUST produce audit events. Safety-increasing authority changes and their events MUST commit atomically. Timestamps MUST NOT replace event sequence as the ordering authority. Emergency gate closure MUST remain possible if audit storage fails; the service MUST then refuse new execution and MUST NOT acknowledge a durably recorded transition until one exists.

**REQ-FUN-008 — Incident handling.**  
A suspected boundary escape, approval bypass, unauthorized effect, containment failure, or safety-significant unknown effect MUST create an incident. A `CRITICAL` incident MUST suspend affected releases. Reviewers MUST examine shared dependencies to identify additional affected releases. Uncertainty about affected scope MUST be recorded and conservatively bounded. Closing an incident MUST NOT itself restore authorization.

**REQ-FUN-009 — Dual review.**  
L2 and L3 authorization MUST include both `SAFETY` and `OPERATIONS` approvals from distinct humans. Both MUST satisfy REQ-FUN-003’s exclusions. Operations review MUST verify deployment-specific supervision, capacity, recovery, and external-effect reconciliation.

**REQ-FUN-010 — Assessment independence.**  
L3 assessment MUST be performed by people outside the development team’s decision authority for the assessed system. Conflicts of interest and reporting relationships MUST be documented. An independently controlled repository MUST retain the assessment report, case digest, and audit checkpoint evidence.

### 5.2 Performance and temporal requirements

| ID | Normative requirement | Engineering rationale |
|---|---|---|
| REQ-PER-001 | Under declared supported load, an authenticated suspension or halt request MUST close its applicable admission gate within 1,000 ms after the complete request reaches the control service. A successful response MUST follow durable state recording and gate closure. | Bounds continued admission while preserving an observable control decision. |
| REQ-PER-002 | Executors MUST provide supervisory liveness signals at intervals no greater than 1,000 ms. If no valid signal has arrived for 2,000 ms, the supervisor MUST close that executor’s effect path and initiate halting. | Prevents indefinite authority when execution cannot be observed. |
| REQ-PER-003 | Within 5,000 ms after gate closure, managed executors and descendants MUST be terminated or isolated so that further managed computation and dispatch are prevented. Unconfirmed containment MUST remain `HALTING` and MUST generate an incident. | Separates prompt denial of effects from confirmation of containment. |
| REQ-PER-004 | Runs MUST last no more than 86,400 seconds. Review approvals and evaluation freshness MUST last no more than 30 days. Action approvals MUST last no more than 300 seconds. Time-dependent authorization MUST fail closed when authoritative clock uncertainty exceeds 1,000 ms. | Limits unattended authority and stale evidence while making deadline enforcement testable. |

For REQ-PER-004, a day is exactly 86,400 seconds.

The release authorization expiry MUST be no later than the earliest of:

1. Authorization time plus 30 days.
2. The expiry of each approval required for its quorum.
3. Each required evaluation’s `executed_at` plus 30 days.

A run’s `deadline_at` MUST be the earlier of release expiry and `started_at` plus `max_run_seconds`.

The controller MUST conservatively account for known clock uncertainty: if the latest plausible current time reaches a deadline, authority is expired. Monotonic elapsed-time measurement MUST enforce run duration so that a backward wall-clock adjustment cannot extend execution.

The boundary artifact MUST declare supported control-request load and concurrency. These timing requirements MUST be tested at those declared limits. They are control-system limits, not guarantees about how quickly an external physical process stops.

### 5.3 Safety requirements

**REQ-SAF-001 — Hazard argument.**  
The safety case MUST connect each hazard to initiating conditions, consequences, controls, verification, and residual-risk reasoning. `OPEN` hazards MUST block authorization. `CRITICAL` hazards MUST NOT use `ACCEPTED`; they require `CONTROLLED` disposition and at least two controls with documented failure independence. A common model instruction or shared credential cannot be counted as independent controls without supporting analysis.

**REQ-SAF-002 — Evaluation sufficiency.**  
Every claimed control MUST have reproducible verification evidence, explicit acceptance criteria, and retained limitations. Authorization MUST require `CONTROL_VERIFICATION` coverage and no `FAIL` or `INCONCLUSIVE` evaluation in the submitted case. Earlier failures MUST remain available in historical evidence; remediation requiring changed configuration produces a new case. Test success MUST NOT be reported as a universal failure-rate bound.

**REQ-SAF-003 — Adversarial evaluation.**  
Every level MUST include `ADVERSARIAL` evaluation addressing instruction injection, forged authority, tool-output manipulation, unauthorized disclosure, persistence attempts, delegation-based escalation, budget evasion, misleading completion reports, and interference with shutdown. A threat class may be excluded only when the boundary artifact demonstrates that its required capability or channel is absent. Tests MUST examine enforcement behavior rather than relying solely on the model’s refusal text.

**REQ-SAF-004 — Consequential action approval.**  
Every `IRREVERSIBLE` action MUST receive current human approval before commitment. Actions associated with a `CRITICAL` hazard MUST also require human approval unless the safety case demonstrates that the permitted operation cannot reach that hazard’s triggering conditions. Approval presentation MUST identify the exact destination, operation, proposal content, consequence bounds, and expiry. Changed bytes require a new action and approval.

**REQ-SAF-005 — Halt and uncertain effects.**  
Halting MUST prevent new commitments, cancel uncommitted proposals, stop or isolate managed executors, and request cancellation of in-flight effects where supported. Sending cancellation is not confirmation of cancellation. Unknown delivery or completion MUST be represented as `UNKNOWN`, with incident linkage. Automatic redispatch MUST be prohibited unless the pinned adapter establishes durable downstream deduplication for that action identity.

**REQ-SAF-006 — Operating variation.**  
L2 and L3 evidence MUST evaluate the declared variation in task composition, context length, language, retrieval content, dependency behavior, and resource pressure that can affect safety. Test material MUST include challenges withheld from configuration development. Applicability and exclusions MUST be justified against the actual envelope.

**REQ-SAF-007 — Independent challenge.**  
L3 MUST include independently designed adversarial exercises and an `INDEPENDENT_REVIEW` evaluation. Exercises MUST include evaluator-awareness concerns, multistep escalation, conflicting objectives, and control-component failure injection. Dangerous effects MUST be replaced with instrumented simulators or bounded substitutes while retaining the assessed authorization and containment paths.

**REQ-SAF-008 — Learning, memory, and drift.**  
Online learning, self-modification, persistent memory, and retrieval changes MUST have explicit change boundaries. Models MUST NOT modify their own permissions, evaluation acceptance criteria, audit controls, or supervisor policy. Unassessed behavioral changes, dependency drift, or invalidated safety assumptions MUST block new commitments and initiate review; affected authorized releases MUST be suspended.

### 5.4 Security and privacy requirements

**REQ-SEC-001 — Authenticated authority.**  
Network interfaces MUST use TLS 1.3 and authenticate both calling services and the service endpoint. Human authorization and irreversible-action approval MUST use multifactor authentication. Role checks MUST occur at the controller or gateway. Submitted principal names MUST NOT override authenticated identity.

**REQ-SEC-002 — Isolation and credentials.**  
Executors MUST be isolated from supervisor control storage, role administration, audit modification, and unrestricted network or device access. Tool credentials MUST remain outside model-visible context. Service credentials MUST be scoped to the minimum resources and operations needed by their component.

**REQ-SEC-003 — Evidence and audit integrity.**  
The store MUST verify artifact hashes and lengths on ingestion and retrieval. Audit history MUST resist undetected deletion and modification through access separation and authenticated checkpoints. A hash chain alone is insufficient evidence that an administrator could not rewrite history. Loss of evidence required by an active authorization MUST suspend that authorization.

**REQ-SEC-004 — Privacy and retention.**  
The boundary artifact MUST assign retention and access rules to task inputs, outputs, proposals, evidence, and metadata. Audit metadata MUST avoid embedding raw secrets or unnecessary personal information. Detailed content MUST use access-controlled artifacts. Control and certification metadata MUST be retained for at least 90 days after the associated release is retired or its certificate ceases to be valid, whichever occurs later. Required deletion of artifact bytes MUST leave a restricted tombstone and invalidate any authorization depending on those bytes.

**REQ-SEC-005 — Trust separation.**  
Retrieved text, tool results, model-generated evaluations, and descendant messages MUST be treated as data. They MUST NOT change executable policy or acquire authority through claimed role names, formatting, or instructions. Evaluation results generated by the assessed model MUST be independently checked before supporting a safety claim. Hidden reasoning traces are not required evidence.

### 5.5 Interoperability, accessibility, and governance

**REQ-INT-001 — Representations.**  
Implementations MUST apply Section 4 types, closed vocabularies, reference checks, and byte-level digest rules. Identical IDs with different immutable bytes MUST be rejected. Human-readable translations MAY vary; machine identifiers MUST remain unchanged.

**REQ-INT-002 — API behavior.**  
Implementations MUST implement Section 6’s endpoints, concurrency controls, idempotency rules, status codes, and failure semantics. A receipt or returned record MUST NOT itself be treated as an execution grant.

**REQ-INT-003 — Event exchange.**  
Event export MUST preserve event identity, sequence, and resource revision. Consumers MUST deduplicate by `event_id`, detect sequence gaps, and retrieve missing events before asserting a complete audit history.

**REQ-ACC-001 — Human control accessibility.**  
Review, approval, and emergency-control interfaces MUST expose programmatic labels, keyboard-operable controls, textual state descriptions, and confirmation of the actual committed result. Critical distinctions MUST NOT depend only on color, audio, or visual position. Approval interfaces MUST distinguish a proposed action from an executed action and gate closure from confirmed containment.

**REQ-GOV-001 — Claim governance.**  
Conformance claims, assessments, renewal, and changes MUST follow Section 9. A claim MUST identify its assessed configuration and envelope and MUST NOT imply universal safety or unrestricted deployment permission.

## 6. Interfaces and API

### 6.1 Transport and request controls

The base path is `/v1`. Requests and responses MUST carry `WIA-Spec-Version: 1.0`. JSON bodies use `application/json`.

Ordinary request bodies MUST NOT exceed 1,048,576 bytes. Artifact-upload bodies MUST NOT exceed 25,165,824 bytes, and decoded artifact bytes MUST NOT exceed 16,777,216 bytes. These limits bound parser and control-plane memory use.

Mutable resource reads return a strong ETag containing the decimal `revision`, enclosed in quotation marks. Mutations of an existing release, run, action, or incident MUST provide `If-Match`, except suspension and halt.

Creation and ordinary mutation requests MUST provide `Idempotency-Key`, an ID. Its scope is authenticated principal, HTTP method, and path. The server MUST atomically retain the key, request fingerprint, result status, and response for at least 30 days and longer while the operation is unresolved.

An identical retry returns the stored result. Reuse with different body bytes or original precondition returns `IDEMPOTENCY_CONFLICT`. Authentication and permission to read the receipt are checked before replay; matching replay occurs before testing whether the original ETag is now stale.

Suspension and halt MUST work without `If-Match` or `Idempotency-Key`. Repeated requests MUST coalesce without restoring authority.

### 6.2 Endpoint inventory

All named request and response types are JSON objects. A dash means no request body.

| Method | Path | Purpose | Request JSON type | Success response |
|---|---|---|---|---|
| POST | `/v1/artifacts` | Store immutable bytes | ArtifactUpload | `201` Artifact |
| GET | `/v1/artifacts/{artifact_id}` | Retrieve authorized bytes | — | `200` ArtifactContent |
| POST | `/v1/cases` | Register exact safety-case bytes | SafetyCase | `201` SafetyCase |
| GET | `/v1/cases/{case_id}` | Retrieve case | — | `200` SafetyCase |
| POST | `/v1/releases` | Create release | ReleaseCreate | `201` Release |
| GET | `/v1/releases/{release_id}` | Read lifecycle and decisions | — | `200` Release |
| POST | `/v1/releases/{release_id}/transitions` | Review, authorize, reject, or retire | ReleaseTransition | `200` Release |
| POST | `/v1/releases/{release_id}/reviews` | Append human review decision | ReviewInput | `201` ReviewDecision |
| POST | `/v1/releases/{release_id}/suspend` | Revoke authority | ReasonInput | `202` Release |
| POST | `/v1/runs` | Admit bounded execution | RunCreate | `201` Run |
| GET | `/v1/runs/{run_id}` | Read execution status | — | `200` Run |
| POST | `/v1/runs/{run_id}/halt` | Close run gate and contain | ReasonInput | `202` Run |
| POST | `/v1/runs/{run_id}/finish` | Request normal completion | ReasonInput | `200` Run |
| POST | `/v1/runs/{run_id}/actions` | Register exact proposal | ActionCreate | `201` Action |
| GET | `/v1/actions/{action_id}` | Read effect status | — | `200` Action |
| POST | `/v1/actions/{action_id}/approvals` | Record human decision | ActionApprovalInput | `201` ActionApproval |
| POST | `/v1/actions/{action_id}/dispatch` | Commit permitted operation | EmptyObject | `202` Action |
| POST | `/v1/incidents` | Register concern | IncidentCreate | `201` Incident |
| GET | `/v1/incidents/{incident_id}` | Read incident | — | `200` Incident |
| POST | `/v1/incidents/{incident_id}/transitions` | Triage or close incident | IncidentTransition | `200` Incident |
| GET | `/v1/events` | Export ordered events | — | `200` EventPage |

Creation responses MUST include `Location` where a corresponding resource-read endpoint exists. Mutations return the ETag of the controlling mutable resource, including review and approval append operations.

### 6.3 Command JSON definitions

Every listed member is required. Unlisted members are forbidden.

| Type | Members and constraints |
|---|---|
| ArtifactUpload | `artifact_id`: ID; `media_type`: string; `content_base64`: standard padded Base64 string |
| ArtifactContent | `descriptor`: Artifact; `content_base64`: standard padded Base64 string |
| ReleaseCreate | `case_id`: ID |
| ReleaseTransition | `target_state`: `UNDER_REVIEW`, `AUTHORIZED`, `REJECTED`, or `RETIRED`; `reason`: Text |
| ReviewInput | `review_round`: PositiveUInt; `role`: `SAFETY` or `OPERATIONS`; `decision`: `APPROVE` or `REJECT`; `expires_at`: Time; `rationale_artifact_id`: ID |
| ReasonInput | `reason`: Text |
| RunCreate | `release_id`: ID; `authorization_epoch`: UInt; `objective_artifact_id`: ID |
| ActionCreate | `permission_id`: ID; `proposal_artifact_id`: ID |
| ActionApprovalInput | `proposal_sha256`: Digest; `decision`: `APPROVE` or `REJECT`; `expires_at`: Time |
| EmptyObject | Exactly `{}` |
| IncidentCreate | `release_id`: ID; `run_id`: ID or null; `severity`: severity enum; `description`: Text; `trigger_artifact_id`: ID |
| IncidentTransition | `target_state`: `TRIAGED` or `CLOSED`; `resolution_artifact_id`: ID or null |
| EventPage | `events`: Event[]; `next_after`: UInt; `has_more`: boolean |

`GET /v1/events` accepts `after`, a UInt defaulting to zero, and `limit`, an integer from 1 through 1,000 defaulting to 100. Events are returned in increasing sequence. `next_after` is the last returned sequence, or the supplied `after` when the page is empty.

Clients cannot set server timestamps, actors, resource revisions, computed digests, or action outcomes. Adapter-to-gateway outcome reporting is an authenticated internal interface whose exact transport is deployment-specific and MUST be included in the assessed boundary.

### 6.4 Status and error codes

Errors use the following JSON object shape:

| Field | Type | Meaning |
|---|---|---|
| `error` | object | Contains the following fields |
| `error.code` | enum | One value from the table below |
| `error.message` | Text | Human-readable explanation |
| `error.request_id` | ID | Traceable request identity |
| `error.details` | array | Zero or more objects containing `field` and `reason`, both strings |

`field` identifies a JSON member path or HTTP header. Errors MUST NOT expose protected proposal content or credentials.

| HTTP status | Error code | Meaning |
|---|---|---|
| 400 | `MALFORMED_JSON` | Invalid JSON, encoding, or duplicate keys |
| 400 | `INVALID_REQUEST` | Invalid query, header, Base64, or request framing |
| 401 | `UNAUTHENTICATED` | Authentication absent or invalid |
| 403 | `FORBIDDEN` | Authenticated principal lacks authority |
| 404 | `NOT_FOUND` | Resource unavailable to this principal |
| 409 | `STATE_CONFLICT` | Transition or operation forbidden in current state |
| 409 | `ID_CONFLICT` | Identifier already binds different immutable content |
| 409 | `IDEMPOTENCY_CONFLICT` | Key reused for a different request |
| 409 | `BUDGET_EXCEEDED` | Admission would exceed a declared budget |
| 412 | `REVISION_MISMATCH` | Supplied ETag is stale |
| 413 | `PAYLOAD_TOO_LARGE` | Encoded or decoded size exceeds its limit |
| 422 | `SCHEMA_VIOLATION` | Fields, types, versions, or cross-field constraints invalid |
| 422 | `EVIDENCE_INVALID` | Evidence unavailable, mismatched, stale, or unverifiable |
| 422 | `ASSURANCE_GATE_FAILED` | Required hazard, review, or evaluation condition unmet |
| 422 | `APPROVAL_REQUIRED` | Required human approval absent |
| 422 | `APPROVAL_INVALID` | Approval rejected, mismatched, revoked, or unauthorized |
| 422 | `REQUEST_EXPIRED` | Relevant approval, release, or run deadline reached |
| 428 | `PRECONDITION_REQUIRED` | Required `If-Match` missing |
| 429 | `CAPACITY_EXCEEDED` | Declared service capacity unavailable |
| 503 | `SAFETY_UNAVAILABLE` | Authoritative safety state or enforcement unavailable |

Authentication and authorization checks precede disclosure of detailed resource errors. No error response grants permission to retry an external effect blindly.

### 6.5 Event types

The event vocabulary is closed:

| Value | Meaning |
|---|---|
| `ARTIFACT_STORED` | Immutable artifact registered |
| `CASE_REGISTERED` | Safety case registered |
| `RELEASE_CREATED` | Release lifecycle created |
| `RELEASE_STATE_CHANGED` | Release state or authorization epoch changed |
| `REVIEW_RECORDED` | Human review appended |
| `RUN_CREATED` | Run admitted |
| `RUN_STATE_CHANGED` | Run state or containment status changed |
| `ACTION_CREATED` | Proposal registered |
| `ACTION_APPROVAL_RECORDED` | Human action decision appended |
| `ACTION_STATE_CHANGED` | Commitment or effect outcome changed |
| `INCIDENT_CREATED` | Incident registered |
| `INCIDENT_STATE_CHANGED` | Incident triaged or closed |
| `REQUEST_DENIED` | Authenticated authority request denied |

Export delivery MAY duplicate events. It MUST NOT assign a new event identity to a duplicate. Resource snapshots in event details MUST permit reconstruction of the relevant decision without requiring raw task content.

### 6.6 Interface versioning

`/v1` denotes the major API family; `WIA-Spec-Version` selects the exact supported specification. This edition accepts only `1.0`.

A compatible minor revision MAY add separate endpoints or clarify requirements without changing the accepted representations or meanings of existing endpoints. Adding required members, changing closed enums, weakening safety gates, or changing commitment semantics requires a major version unless implemented through a separately negotiated interface.

## 7. Protocol and lifecycle

### 7.1 Release state machine

| State | Meaning |
|---|---|
| `REGISTERED` | Immutable case linked; review not started |
| `UNDER_REVIEW` | Current review round open; execution prohibited |
| `AUTHORIZED` | Admission possible while all live checks remain valid |
| `SUSPENDED` | Authority revoked; containment or remediation may continue |
| `REJECTED` | Review refused; terminal release |
| `RETIRED` | Release permanently withdrawn; terminal release |

| From | To | Trigger and required conditions |
|---|---|---|
| `REGISTERED` | `UNDER_REVIEW` | Owner or reviewer begins review; increment `review_round` |
| `UNDER_REVIEW` | `AUTHORIZED` | Controller verifies all applicable requirements and current approvals; increment epoch |
| `UNDER_REVIEW` | `REJECTED` | Authorized reviewer rejects; a recorded `REJECT` decision causes this transition |
| `AUTHORIZED` | `SUSPENDED` | Responder request, expiry, authority loss, incident, drift, or enforcement failure; increment epoch |
| `SUSPENDED` | `UNDER_REVIEW` | All associated runs terminal and contained; blocking incidents resolved; begin new round |
| `REGISTERED` | `RETIRED` | Owner withdraws unused release |
| `UNDER_REVIEW` | `RETIRED` | Owner withdraws review |
| `SUSPENDED` | `RETIRED` | All associated runs terminal and contained |

All other transitions are forbidden. An authorized release MUST be suspended before retirement. Rejection and retirement cannot be undone; another release may reference the same case only if that case remains adequate.

A suspension request for an already suspended release returns its current state. Terminal releases remain terminal. Other nonauthorized states reject suspension with `STATE_CONFLICT`.

Entering `UNDER_REVIEW` invalidates earlier rounds for quorum. Duplicate approvals from one human count once. A reviewer may record a rejection after approval; rejection terminates the current review. After authorization, withdrawal is performed through suspension.

### 7.2 Run state machine

| State | Meaning |
|---|---|
| `RUNNING` | Managed execution active under live authority |
| `HALTING` | Admission closed; containment not yet confirmed |
| `HALTED` | Containment confirmed after a requested or policy-triggered stop |
| `COMPLETED` | Normal completion confirmed; no unresolved actions |
| `FAILED` | Execution failed and containment is confirmed |

Permitted transitions are:

- `RUNNING` → `HALTING` following halt, suspension, timeout, budget exhaustion requiring termination, safety trigger, or executor failure.
- `RUNNING` → `COMPLETED` after authenticated supervisor confirmation of completion and containment.
- `HALTING` → `HALTED` when containment is confirmed after a nonfailure stop.
- `HALTING` → `FAILED` when containment is confirmed after execution or control failure.

Terminal states have no outgoing transitions. A late completion message MUST NOT change `HALTING` to `COMPLETED`.

`HALTED` and `FAILED` may retain externally committed unknown effects if those effects are recorded in incidents. Their `containment_confirmed` value concerns managed execution and dispatch; it does not assert rollback of external effects.

### 7.3 Action state machine

| State | Meaning |
|---|---|
| `PROPOSED` | Immutable proposal registered; no effect committed |
| `COMMITTED` | Budget reserved and dispatch commitment recorded |
| `SUCCEEDED` | Intended operation completion established |
| `FAILED` | Failure established by sufficient outcome evidence |
| `CANCELLED` | Proposal stopped before commitment |
| `UNKNOWN` | External delivery or outcome cannot be established |

Permitted transitions are:

- `PROPOSED` → `COMMITTED` after all commitment checks pass.
- `PROPOSED` → `CANCELLED` after rejection, halt, suspension, or explicit supervisor cancellation.
- `COMMITTED` → `SUCCEEDED`, `FAILED`, or `UNKNOWN`.
- `UNKNOWN` → `SUCCEEDED` or `FAILED` following reconciliation evidence.

All other transitions are forbidden. Timeout alone does not establish failure without effect. If an adapter fails after possible partial effects, its result artifact MUST describe those effects; it cannot imply restoration.

A committed action consumes its action budget permanently. Its parallel reservation remains held until a resolved outcome or confirmed cancellation of the external operation is evidenced. Unknown effects MUST NOT be hidden by releasing reservations.

### 7.4 Incident state machine

An incident follows `OPEN` → `TRIAGED` → `CLOSED`.

Triage MUST identify an accountable responder, affected releases and dependencies, immediate controls, and investigation scope in the audit details.

Closure MUST include findings, unresolved consequences, remediation verification, and a decision about whether a new safety case is required. A new occurrence creates a new incident referencing relevant prior evidence rather than rewriting a closed record.

### 7.5 Main sequences

**Registration and authorization**

1. The owner registers artifacts and verifies their returned descriptors.
2. The owner submits the complete safety case.
3. The registry validates structure, references, and immutable-byte integrity.
4. A release is created in `REGISTERED`.
5. Review begins, creating a new round.
6. Reviewers inspect evidence and submit attributable decisions.
7. The controller rechecks quorum, role validity, evidence freshness, hazard disposition, configuration, and incident status.
8. Authorization commits the state, expiry, new epoch, and audit event atomically.

**Action execution**

1. An operator requests a run using the current release epoch.
2. The supervisor binds its deadline and budgets.
3. The executor submits an immutable proposal artifact and permission reference.
4. The gateway validates the adapter contract and information classification.
5. A human approves the exact proposal when required.
6. At commitment, the gateway rechecks release and run authority, epoch, approval, deadline, and aggregate reservations.
7. Commitment is serialized against suspension; the action becomes `COMMITTED`.
8. The adapter records success, failure, or uncertainty from authenticated outcome evidence.

**Suspension and recovery**

1. A responder or controller trigger closes admission and commits revocation.
2. New commitments ordered after revocation are rejected.
3. Uncommitted proposals are cancelled.
4. The supervisor contains executors and descendants.
5. Previously committed effects are reconciled independently.
6. Incidents capture unresolved or unsafe outcomes.
7. Recovery proceeds through a new review round or a new safety case.
8. Reauthorization creates a new epoch and new runs; old runs never resume.

## 8. Testing and conformance procedures

### 8.1 Test environment

Testing MUST use the assessed controller, supervisor, gateways, role configuration, and adapter versions. Hazardous downstream effects MUST use instrumented substitutes with matching authorization and commitment behavior.

The environment MUST provide:

- An injectable wall clock and monotonic timer.
- Barriers for ordering concurrent commitment, expiry, and suspension.
- Controlled loss of executor, network, audit-store, and identity-service availability.
- Observable gateway admission and external dispatch records.
- Distinct human principals for required separation of duties.
- Load generation up to the declared supported capacity.

Timing tests MUST measure request arrival, gate closure, and containment independently. Tests MUST report worst observed times without converting those observations into unsupported population reliability claims.

### 8.2 Test cases

| ID | Requirement | Method | Pass criteria |
|---|---|---|---|
| TC-001 | REQ-FUN-001, REQ-INT-001 | Change immutable bytes, configuration digests, fields, and references | Mutations and mismatches rejected; original records unchanged |
| TC-002 | REQ-FUN-002, REQ-FUN-006 | Race commitments across descendants at budget boundaries | Aggregate limits never exceeded; failed and uncertain commitments remain charged |
| TC-003 | REQ-FUN-003 | Attempt owner approval, evaluator approval, duplicate-human quorum, expired and revoked approval | Invalid approvals cannot authorize; authority loss suspends affected release |
| TC-004 | REQ-FUN-004 | Order commitment before and after suspension, then reauthorize | No commitment after revocation; old epochs remain invalid |
| TC-005 | REQ-FUN-005, REQ-SAF-004 | Attempt undeclared operations, destination changes, changed proposal bytes, and irreversible dispatch without approval | Every unauthorized path denied before commitment |
| TC-006 | REQ-FUN-007, REQ-SEC-003 | Fail audit transactions and alter stored evidence | No unaudited authority grant; tampering detected; emergency gate closure remains available |
| TC-007 | REQ-FUN-008 | Inject critical incident and shared-dependency uncertainty | Affected releases suspended and scope investigation recorded |
| TC-008 | REQ-FUN-009 | Attempt L2 authorization with one reviewer or duplicate identities | Distinct valid safety and operations decisions required |
| TC-009 | REQ-FUN-010 | Inspect reporting relationships and external evidence custody | Independence and separately controlled evidence retention established |
| TC-010 | REQ-PER-001 | Issue suspension and halt at declared maximum supported load | Applicable gate closes within 1,000 ms; successful acknowledgement follows durable commit |
| TC-011 | REQ-PER-002 | Withhold executor liveness signals | Effect path closes by the 2,000 ms liveness deadline |
| TC-012 | REQ-PER-003, REQ-SAF-005 | Use unresponsive executor, delayed adapter, and unknown delivery | Containment within 5,000 ms; uncertainty represented honestly; no unsafe redispatch |
| TC-013 | REQ-PER-004 | Exercise expiry boundaries, clock rollback, and excessive clock uncertainty | Deadlines are conservative; no authority extension or acceptance beyond limits |
| TC-014 | REQ-SAF-001 | Inspect each hazard and inject common-control failures | Open hazards block; critical hazards have evaluated independent controls |
| TC-015 | REQ-SAF-002 | Reproduce control evaluations and submit failed or inconclusive cases | Required claims reproducible; deficient cases cannot authorize |
| TC-016 | REQ-SAF-003 | Execute applicable adversarial threat classes | Control bypass is prevented; observed weaknesses block authorization |
| TC-017 | REQ-SAF-006 | Exercise declared operating variations using withheld challenges | Claims supported across declared scope or envelope narrowed |
| TC-018 | REQ-SAF-007 | Conduct independent multistep and component-failure exercises | Independent evidence exists; unresolved safety findings block authorization |
| TC-019 | REQ-SAF-008 | Change memory policy, dependencies, instructions, and control files | Unauthorized changes blocked; material drift suspends affected authority |
| TC-020 | REQ-SEC-001 | Attempt unauthenticated, wrong-role, forged-principal, and insufficient-factor operations | No unauthorized authority change or approval accepted |
| TC-021 | REQ-SEC-002, REQ-SEC-005 | Attempt control-store access and inject policy instructions through data channels | Isolation holds; content cannot alter authority |
| TC-022 | REQ-SEC-004 | Inspect captured logs, deletion workflow, and retention configuration | Sensitive content minimized; tombstones retained; dependent authority invalidated |
| TC-023 | REQ-INT-002 | Exercise retries, concurrent duplicates, conflicting keys, missing and stale ETags | Exactly one local mutation per request identity; prescribed responses returned |
| TC-024 | REQ-INT-003 | Duplicate, omit, and reorder exported events | Consumers deduplicate and identify gaps without asserting false completeness |
| TC-025 | REQ-ACC-001 | Operate review and halt interfaces using keyboard and assistive labels | Required controls and distinctions remain usable and explicit |
| TC-026 | REQ-GOV-001 | Inspect claim, assessment, renewal, and change records | Scope, version, level, evidence, expiry, and limitations are accurate |

Each test MUST exercise all applicable control paths. Selecting only the most favorable adapter or executor does not establish conformance for other claimed components.

### 8.3 Reporting format

A conformance report MUST be registered as an artifact containing this JSON structure:

| Field | Type | Constraint |
|---|---|---|
| `report_id` | ID | Unique |
| `standard_id` | string | Exactly `WIA-AI-028` |
| `standard_version` | string | Exactly `1.0` |
| `level` | enum | `L1`, `L2`, or `L3` |
| `case_id` | ID | Assessed case |
| `case_sha256` | Digest | Exact assessed bytes |
| `test_environment_artifact_id` | ID | Environment, load, clocks, and component inventory |
| `assessor_id` | ID | Accountable assessor |
| `started_at` | Time | Assessment start |
| `completed_at` | Time | Not earlier than start |
| `results` | array | One result for every TC-001 through TC-026 |

Each result contains `test_id`, `requirements`, `outcome`, and `observations_artifact_id`. `requirements` is an array of the mapped requirement IDs.

Report outcomes are `PASS`, `FAIL`, `INCONCLUSIVE`, and `NOT_APPLICABLE`. The first three have the meanings in Section 4.5. `NOT_APPLICABLE` is permitted only for tests belonging exclusively to a higher conformance level.

Certification requires `PASS` for every applicable test. A test that could not be executed is `INCONCLUSIVE`, not `NOT_APPLICABLE`.

## 9. Certification, governance and versioning

### 9.1 Certification process

Certification under this document is an evidence-based conformance statement by an identified assessor. It does not imply recognition by an unspecified accreditation body.

The assessor MUST:

1. Identify the exact case, deployment boundary, claimed level, and component inventory.
2. Review the safety argument and applicable requirement coverage.
3. Verify the Section 8 test report and supporting observations.
4. Confirm that deficiencies have been resolved through controlled changes.
5. Issue an attributable certification record.

A certificate MUST NOT grant runtime authority. Deployment still requires release authorization and live gateway checks.

### 9.2 Certification record

| Field | Type | Constraint or meaning |
|---|---|---|
| `certificate_id` | ID | Unique certificate identity |
| `standard_id` | string | `WIA-AI-028` |
| `standard_version` | string | `1.0` |
| `level` | enum | `L1`, `L2`, `L3` |
| `case_id` | ID | Certified case |
| `case_sha256` | Digest | Immutable scope binding |
| `assessor_id` | ID | Accountable assessor |
| `assessment_type` | enum | Defined below |
| `report_artifact_id` | ID | Conformance report |
| `issued_at` | Time | Issue time |
| `expires_at` | Time | At most 365 days after issue |
| `status` | enum | Defined below |

Assessment types are:

- `SELF_ASSESSED`: assessment performed within the responsible organization without the independence claim required for L3.
- `INDEPENDENTLY_ASSESSED`: assessment satisfies REQ-FUN-010’s independence conditions.

L3 MUST use `INDEPENDENTLY_ASSESSED`.

Certificate statuses are:

- `VALID`: assessment remains current for its bound scope.
- `SUSPENDED`: claim temporarily unusable pending investigation.
- `WITHDRAWN`: claim permanently withdrawn.
- `EXPIRED`: validity period ended.

Certificate records and changes MUST be authenticated and retained with assessor attribution. A valid certificate cannot override expired evaluations or release approvals.

### 9.3 Renewal and change control

Renewal MUST repeat applicable testing against the current case and operational evidence. Prior results MAY be reused only when the assessor demonstrates unchanged configuration, unchanged test applicability, valid evidence, and no unresolved contradictory findings.

A new case is REQUIRED for material changes to permissions, models, instruction policies, adapters, safety controls, memory rules, or operating assumptions. Relevant changes in external dependencies also require a new case unless already covered by an evaluated variation.

Editorial corrections to descriptive material MUST NOT silently change registered case bytes. Corrected evidence receives a new artifact identity and, when part of the safety argument, a new case.

Discovery that a required control is ineffective MUST suspend affected runtime authorization immediately and initiate certificate review. Certification status changes do not replace incident handling.

### 9.4 Standard governance and compatibility

Changes to this standard MUST include a rationale, affected requirement IDs, migration effects, and updated conformance tests.

A revision MUST distinguish:

- Editorial clarification without altered obligations.
- Compatible additions that preserve existing interface behavior.
- Breaking changes to representations, authority semantics, or required controls.

Requirement IDs MUST NOT be reassigned to different meanings. Removed requirements remain reserved in later editions.

Existing `1.0` records MUST retain their original interpretation. A later implementation MUST NOT reinterpret an old enum, digest, approval, or authorization epoch using new semantics.

## Appendix A: Example payloads

All scenarios, identities, and timestamps in this appendix are illustrative.

### A.1 Artifact transport example

The following request stores the three UTF-8 bytes `abc`. This is a transport fixture, not safety evidence.

`POST /v1/artifacts`

```json
{
  "artifact_id": "art-transport-example",
  "media_type": "text/plain",
  "content_base64": "YWJj"
}
```

The response is:

```json
{
  "artifact_id": "art-transport-example",
  "sha256": "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
  "media_type": "text/plain",
  "byte_length": 3
}
```

### A.2 Review request example

`POST /v1/releases/rel-example/reviews`

```json
{
  "review_round": 1,
  "role": "SAFETY",
  "decision": "APPROVE",
  "expires_at": "2030-01-08T10:30:00.000Z",
  "rationale_artifact_id": "art-review-rationale"
}
```

The authenticated reviewer determines `principal_id`; the request cannot select a different identity.

### A.3 Run admission example

`POST /v1/runs`

```json
{
  "release_id": "rel-example",
  "authorization_epoch": 1,
  "objective_artifact_id": "art-run-objective"
}
```

An illustrative successful response is:

```json
{
  "run_id": "run-example",
  "release_id": "rel-example",
  "authorization_epoch": 1,
  "principal_id": "principal-operator",
  "objective_artifact_id": "art-run-objective",
  "state": "RUNNING",
  "revision": 1,
  "started_at": "2030-01-01T11:00:00.000Z",
  "deadline_at": "2030-01-01T11:10:00.000Z",
  "actions_committed": 0,
  "open_actions": 0,
  "descendants_created": 0,
  "containment_confirmed": false,
  "termination_reason": null
}
```

### A.4 Suspension example

`POST /v1/releases/rel-example/suspend`

```json
{
  "reason": "Illustrative adapter integrity check failed during supervision."
}
```

A successful suspension response establishes that admission is closed and revocation is durable. Clients obtain individual run records to determine whether containment is confirmed.

### A.5 Unknown-effect incident example

In this illustrative scenario, an adapter connection is lost after an action’s commitment point. The gateway cannot establish whether the receiver accepted the operation.

`POST /v1/incidents`

```json
{
  "release_id": "rel-example",
  "run_id": "run-example",
  "severity": "MAJOR",
  "description": "Illustrative committed operation has an unconfirmed external outcome.",
  "trigger_artifact_id": "art-delivery-observations"
}
```

The associated action remains `UNKNOWN` until reconciliation evidence establishes its outcome. Repeating the operation is not a reconciliation method.

### A.6 Stale-revision error example

```json
{
  "error": {
    "code": "REVISION_MISMATCH",
    "message": "The resource changed after the supplied revision was read.",
    "request_id": "req-example",
    "details": [
      {
        "field": "If-Match",
        "reason": "Read the current resource before proposing another ordinary mutation."
      }
    ]
  }
}
```

Emergency suspension and halt do not require a revision match.

## Appendix B: Revision history

v1.0 — initial release