# WIA-AI-027: AI Privacy Preservation Standard Specification v1.0

A standard governing personal-data processing, privacy-preserving AI releases, privacy accounting, subject-request handling, and verifiable lifecycle controls for AI systems.

## 1. Introduction

### 1.1 Purpose

WIA-AI-027 defines technical and operational requirements for preserving privacy throughout AI data collection, preparation, training, evaluation, deployment, inference, and retirement.

The standard establishes:

- A common resource model for datasets, processing jobs, artifacts, privacy budgets, and subject requests.
- Enforceable boundaries between private computation and authorized disclosure.
- A precise, person-level differential privacy mechanism for bounded integer aggregates.
- Requirements for model lineage, retention, source-data erasure, and model remediation.
- Interoperable management interfaces and reproducible conformance procedures.

The English name is **AI Privacy Preservation**. The Korean name is **AI 프라이버시**. The standard identifier is **WIA-AI-027**, version **1.0**.

This document establishes its own normative identifiers, fields, thresholds, and interfaces. No antecedent WIA technical specification accompanies it. Compatibility with other WIA technical profiles, existing certification registries, or institutional approval procedures is therefore unspecified.

Conformance establishes satisfaction of this document’s requirements within a declared system boundary. It does not establish compliance with every applicable law, prove that data are anonymous, or demonstrate that an AI model cannot disclose personal information.

### 1.2 Scope

#### In scope

This standard governs:

- Personal information used in AI training, evaluation, retrieval, and inference.
- Derived representations, including embeddings, features, checkpoints, and model outputs.
- Registration of processing purposes, recipients, authority evidence, and retention limits.
- Access control and isolation across tenants, principals, purposes, and processing stages.
- Person-level contribution bounding and cumulative differential privacy accounting.
- Release of controlled artifacts and qualifying differentially private artifacts.
- Subject restriction and erasure requests, including their effects on dependent models.
- Privacy incident evidence, auditability, software interfaces, and conformity assessment.

A deployment MAY use federated learning, secure aggregation, encrypted computation, or trusted execution mechanisms. Such mechanisms do not replace the requirements of this standard.

Federated learning can retain source records at participants while still disclosing information through updates. Secure aggregation can conceal individual updates from an aggregator while leaving aggregate or model leakage possible. Encryption protects specified communication or storage boundaries; it does not by itself limit information disclosed by an authorized computation.

#### Out of scope

This version does not standardize:

- Legal determinations concerning the validity of a processing authority.
- A universal acceptable level of model memorization or membership-inference risk.
- A general differential privacy accountant for arbitrary training algorithms.
- Differentially private stochastic gradient descent.
- A cryptographic protocol for federated learning or secure multiparty computation.
- Hardware resistance to every physical or microarchitectural side channel.
- A universal machine-unlearning algorithm or proof of complete model forgetting.
- Recovery of copies already obtained by independent recipients.

An implementation using an additional privacy mechanism MUST identify it separately. It MUST NOT describe that mechanism as the v1.0 core mechanism unless it satisfies the exact construction in Section 5.5.

### 1.3 Normative references

The following documents apply where referenced:

- **RFC 2119**, *Key words for use in RFCs to Indicate Requirement Levels*.
- **RFC 8174**, *Ambiguity of Uppercase vs Lowercase in RFC 2119 Key Words*.
- **RFC 8259**, *The JavaScript Object Notation (JSON) Data Interchange Format*.
- **RFC 3339**, *Date and Time on the Internet: Timestamps*.
- **RFC 8446**, *The Transport Layer Security (TLS) Protocol Version 1.3*.
- **RFC 9110**, *HTTP Semantics*.
- **RFC 8785**, *JSON Canonicalization Scheme (JCS)*.
- **JSON Schema Draft 2020-12**, Core and Validation specifications.

No external document supplies additional WIA-specific thresholds or field values.

### 1.4 Terms and definitions

| Term | Definition |
|---|---|
| Personal data | Information relating to an identifiable person, including information that becomes identifying when combined with reasonably available related information. |
| Data subject | The person whose information is processed. |
| Subject reference | An opaque, protected identifier used to associate records with a person inside an accounting scope. |
| Tenant | An administrative isolation boundary whose principals, resources, and accounting records are managed together. |
| Privacy domain | The permanent privacy-accounting scope associated with one tenant under this version. |
| Dataset snapshot | An immutable collection of input bytes and associated subject assignments registered for processing. |
| Processing purpose | A registered, specific reason for processing data, represented by a stable identifier and supporting authority evidence. |
| Authority evidence | An immutable record describing why a processing activity is permitted and what restrictions apply. |
| Direct identifier | A value that directly identifies or contacts a person within its operating context. |
| Quasi-identifier | An attribute or combination of attributes that can assist identification when linked with other information. |
| Sensitive attribute | An attribute designated as requiring heightened protection by the deployment’s documented policy. |
| Embedding | A derived numerical representation of an input that can retain identifying or sensitive information. |
| Artifact | A stored result of AI processing, including a model, aggregate, prediction, embedding collection, or report. |
| Lineage | A recorded dependency relationship connecting inputs, transformations, jobs, and artifacts. |
| Controlled release | Disclosure authorized by purpose, recipient, retention, and access policies without a differential privacy claim. |
| Differential privacy | A property bounding how much an output distribution can change between specified neighboring datasets. |
| Neighboring datasets | For the core profile, two datasets differing by the addition or removal of one person’s complete contribution. |
| Contribution bound | The maximum permitted magnitude of one person’s contribution to a query. |
| Sensitivity | The maximum change in a query result between neighboring datasets under a specified distance measure. |
| Privacy budget | A limit and accounting record for cumulative privacy parameters within a declared scope. |
| Budget reservation | A temporary allocation preventing concurrent jobs from collectively exceeding a budget. |
| Budget commitment | An irreversible accounting charge associated with execution of a privacy mechanism. |
| Postprocessing | Computation using released private results and permitted independent inputs without additional access to protected data. |
| Membership inference | An attempt to determine whether a person or record participated in a model’s training data. |
| Model inversion | An attempt to recover information about inputs, attributes, or representative training content from a model. |
| Machine unlearning | A process intended to reduce or remove specified training-data influence from a model. |
| Release gate | The component that verifies release conditions before an artifact becomes accessible to recipients. |
| Retention exception | A documented, time-bounded decision to retain otherwise erasable material under restricted access. |
| Privacy incident | An event involving unauthorized processing, disclosure, linkage, retention, or impairment of required privacy controls. |

### 1.5 Abbreviations

| Abbreviation | Meaning |
|---|---|
| ACL | Access control list |
| AI | Artificial intelligence |
| API | Application programming interface |
| CSPRNG | Cryptographically secure pseudorandom number generator |
| DP | Differential privacy |
| HTTP | Hypertext Transfer Protocol |
| JSON | JavaScript Object Notation |
| RAG | Retrieval-augmented generation |
| TLS | Transport Layer Security |
| UTC | Coordinated Universal Time |

### 1.6 Conformance keywords

The keywords **MUST**, **MUST NOT**, **REQUIRED**, **SHOULD**, **SHOULD NOT**, and **MAY** are interpreted as described in RFC 2119 and RFC 8174 when written in uppercase.

A **SHOULD** deviation requires a documented engineering justification, its privacy consequences, and approval by the deployment’s accountable privacy authority.

Field constraints, state-transition rules, mathematical definitions, and interface rules are normative through the requirement IDs that reference them. Examples are informative and do not override normative provisions.

## 2. Conformance levels

Conformance applies to a named implementation, configuration, tenant scope, and declared boundary.

| Level | Name | Required requirement IDs |
|---|---|---|
| Level 1 | Controlled Processing | REQ-FUN-001–004; REQ-PERF-001–002; REQ-SAF-001 and 003; REQ-SEC-001–004; REQ-PRIV-001–003; REQ-INT-001–003; REQ-ACC-001; REQ-GOV-001 |
| Level 2 | Managed AI Privacy | Every Level 1 requirement; REQ-FUN-005–006; REQ-SAF-002 |
| Level 3 | Accounted Private Release | Every Level 2 requirement; REQ-DP-001–005 |

The ranges in this table include every intervening identifier.

Level 1 requires enforceable handling and release controls. Level 2 additionally requires automated subject-request workflows, model-impact handling, and documented privacy evaluation. Level 3 additionally requires the core DP mechanism, permanent accounting, and controlled postprocessing.

A Level 3 system MAY perform controlled processing. Its conformance statement MUST identify which release routes provide the DP guarantee. Level 3 does not make every model, inference response, or internal log differentially private.

Any implementation claiming that an output conforms to the core DP profile MUST satisfy REQ-DP-001–005 for that output, regardless of its overall level.

A deployment without human-facing interfaces MAY mark the interface-specific portion of REQ-ACC-001 not applicable. Its machine-readable explanations remain required. No other required provision may be omitted merely because the implementation lacks the necessary component.

## 3. Reference architecture / system model

### 3.1 Components

A conforming system contains the following logical components. Components MAY share a process, but their responsibilities and trust boundaries MUST remain identifiable.

| Component | Responsibility |
|---|---|
| Registration service | Registers immutable snapshots, policies, subject assignments, and transformation references. |
| Identity and policy service | Authenticates principals and evaluates purposes, recipients, authority, and expiration. |
| Subject index | Associates all records belonging to the same person within the privacy domain. |
| Job coordinator | Admits jobs, enforces state transitions, and fences obsolete workers. |
| Privacy accountant | Reserves and commits privacy budget transactionally. |
| Isolated executor | Performs approved transformations with only the inputs and capabilities granted to the job. |
| Artifact store | Stores sealed outputs and immutable content digests. |
| Release gate | Controls recipient access and enforces policy, lineage, and accounting conditions. |
| Subject-request controller | Restricts affected processing and tracks erasure and model remediation. |
| Evidence repository | Resolves immutable references to authority, transformation, verification, assessment, and exception records. |
| Audit service | Records ordered, integrity-protected management events. |

### 3.2 Roles

The identity service MUST distinguish these logical roles:

- **Administrator:** provisions the tenant and its immutable accounting scope.
- **Privacy officer:** approves authority evidence, reviews privacy assessments, and handles verified subject requests.
- **Operator:** registers approved datasets and submits or monitors jobs.
- **Executor:** accesses only the inputs and output staging area of an assigned job.
- **Recipient:** accesses only artifacts explicitly released to that recipient.
- **Auditor:** reads authorized metadata and evidence without automatically receiving source-data access.

A principal MAY hold multiple roles. Combining administrator, operator, and auditor authority MUST be recorded in the assessment because it weakens separation of duties.

An AI model is not an authorization authority. Generated text, tool suggestions, retrieved instructions, and model confidence values MUST NOT create processing authority.

### 3.3 Boundaries and flows

```text
                  Management trust boundary
  Administrator / Privacy officer / Operator / Auditor
                            |
                    Identity + policy
                            |
             +--------------+---------------+
             |              |               |
        Registration   Job coordinator   Subject requests
             |              |               |
       Dataset registry     +------ Privacy accountant
             |              |
       Subject index   Isolated executor
             |              |
             +---- private inputs
                            |
                      Sealed artifact
                            |
                    Release gate + audit
                            |
             +--------------+---------------+
             |                              |
      Authorized recipient          Approved model serving
             |
        Recipient boundary
```

Source-data connectors, external inference services, and external storage providers MUST be placed either inside the assessed processing boundary or behind a documented controlled-disclosure boundary.

A management API response is not automatically a DP release. Management metadata can reveal dataset existence, activity, or subject-request timing. Recipient credentials MUST NOT provide access to management metadata.

The DP observer boundary includes every channel exposed to the claimed DP recipient, including artifact content and relevant availability, status, timing, and selection behavior. Private management channels are outside that observer boundary only when access separation is enforced.

### 3.4 Accounting scope

Each tenant has exactly one privacy domain in v1.0. All registered snapshots, their replacements, and their DP releases use that domain.

Renaming a tenant, changing storage, migrating a deployment, deleting a dataset, or replacing an accounting service MUST NOT create a fresh budget for the same continuing scope.

The standard does not establish a global guarantee across unrelated tenants. A person participating in multiple tenants can incur cumulative privacy loss across them. A broader claim requires composition across those scopes and an explicit statement of the combined bound.

## 4. Data model

### 4.1 Encoding and common types

JSON documents MUST use UTF-8 and comply with RFC 8259. Duplicate object member names, non-finite numbers, and unknown fields in defined resource objects MUST be rejected.

| Type | Representation | Constraints |
|---|---|---|
| `Id` | JSON string | Pattern `^[a-z][a-z0-9_]{2,63}$`; contains no direct personal identifier. |
| `Timestamp` | JSON string | RFC 3339 UTC subset `YYYY-MM-DDTHH:MM:SSZ`; no fractional seconds. |
| `Digest` | JSON string | Exactly 64 lowercase hexadecimal characters representing SHA-256. |
| `Reference` | `Id` | Resolves to an immutable, integrity-verified evidence or internal-storage record. |
| `IdSet` | JSON array | Unique `Id` values; order has no meaning; maximum 128 elements. |
| `Nullable<T>` | Value or JSON `null` | The member remains present when its value is null. |

References MUST resolve through configured repositories. A supplied reference MUST NOT cause arbitrary network retrieval or filesystem access.

Every Domain, Dataset, Job, Artifact, and SubjectRequest contains these common fields:

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `id` | `Id` | — | Yes | Immutable, unique within tenant | Resource identifier. |
| `tenant_id` | `Id` | — | Yes | Matches authenticated tenant | Isolation boundary. |
| `created_at` | `Timestamp` | UTC | Yes | Server assigned | Creation time. |
| `revision` | Integer | revision | Yes | At least 1 | Increments on every resource mutation. |

References are tenant-local unless a separately assessed disclosure explicitly authorizes crossing a boundary. The core DP profile does not permit cross-tenant artifact dependencies.

### 4.2 PrivacyDomain

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `state` | `DomainState` | — | Yes | Defined below | Processing availability. |
| `epsilon_limit_micros` | Integer | epsilon × 10⁶ | Yes | 0–3,000,000 | Permanent accounting ceiling. |
| `epsilon_committed_micros` | Integer | epsilon × 10⁶ | Yes | Nonnegative | Irreversible cumulative charges. |
| `epsilon_reserved_micros` | Integer | epsilon × 10⁶ | Yes | Nonnegative | Sum of live reservations. |

The following invariant MUST hold:

```text
epsilon_committed_micros + epsilon_reserved_micros
    <= epsilon_limit_micros
```

A zero ceiling disables DP job admission. Level 3 requires a positive ceiling. The ceiling is fixed when the domain is provisioned and MUST NOT subsequently increase.

### 4.3 Dataset and DatasetPolicy

A Dataset identifies one immutable snapshot. A changed snapshot requires a new resource.

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `domain_id` | `Id` | — | Yes | Tenant’s domain | Accounting scope. |
| `state` | `DatasetState` | — | Yes | Defined below | Processing eligibility. |
| `data_ref` | `Nullable<Reference>` | — | Yes | Non-null while active | Exact snapshot bytes. |
| `subject_index_ref` | `Nullable<Reference>` | — | Yes | Non-null while active | Protected person-to-record mapping. |
| `schema_ref` | `Reference` | — | Yes | Immutable | Input schema and missing-value rules. |
| `snapshot_digest` | `Digest` | — | Yes | Hash of registered snapshot bytes | Integrity binding. |
| `categories` | Array of `DataCategory` | — | Yes | Nonempty, unique | Applicable information categories. |
| `policy` | `DatasetPolicy` | — | Yes | Table below | Processing constraints. |
| `erased_at` | `Nullable<Timestamp>` | UTC | Yes | Non-null only when erased | Verified completion time. |

When `state` is `ERASED`, both storage references MUST be null. `snapshot_digest` and minimal lineage MAY remain as restricted accounting evidence.

| DatasetPolicy field | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `purpose_ids` | `IdSet` | — | Yes | Nonempty | Permitted purposes. |
| `recipient_ids` | `IdSet` | — | Yes | Nonempty | Permitted recipients. |
| `workloads` | Array of `Workload` | — | Yes | Nonempty, unique | Permitted operations. |
| `authority_ref` | `Reference` | — | Yes | Current evidence | Basis and limits of processing authority. |
| `use_until` | `Timestamp` | UTC | Yes | After registration | Last permitted processing or derivative-use time. |
| `erase_by` | `Timestamp` | UTC | Yes | Between `use_until` and 86,400 seconds afterward | Live-storage erasure deadline. |
| `backup_erase_by` | `Timestamp` | UTC | Yes | At or after `erase_by`; no later than 2,592,000 seconds after `use_until` | Backup erasure deadline. |
| `artifact_ttl_seconds` | Integer | seconds | Yes | 1–2,147,483,647 | Maximum artifact lifetime measured from job creation. |
| `controlled_release_allowed` | Boolean | — | Yes | — | Whether non-DP disclosure is permitted. |

The lifetime integer range is an interchange constraint, not a recommended retention duration. The authority record MUST justify actual retention settings.

### 4.4 Job and DP configuration

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `domain_id` | `Id` | — | Yes | Tenant’s domain | Accounting scope. |
| `purpose_id` | `Id` | — | Yes | Allowed by every private input | Processing purpose. |
| `workload` | `Workload` | — | Yes | Defined below | AI activity. |
| `release_mode` | `ReleaseMode` | — | Yes | Defined below | Release assurance. |
| `artifact_kind` | `ArtifactKind` | — | Yes | Compatible with transformation | Expected result category. |
| `dataset_ids` | `IdSet` | — | Yes | Mode-specific | Source snapshots. |
| `input_artifact_ids` | `IdSet` | — | Yes | Mode-specific | Artifact dependencies. |
| `transform_ref` | `Reference` | — | Yes | Immutable approved manifest | Code, configuration, and coordinate semantics. |
| `recipient_ids` | `IdSet` | — | Yes | Nonempty | Intended recipients. |
| `expires_at` | `Timestamp` | UTC | Yes | Within inherited limits | Artifact-use deadline. |
| `publish_at` | `Nullable<Timestamp>` | UTC | Yes | Required for DP modes | Publicly selected earliest publication time. |
| `dp` | `Nullable<DPConfig>` | — | Yes | Mode-specific | Core mechanism parameters. |
| `state` | `JobState` | — | Yes | Defined below | Lifecycle state. |
| `budget_state` | `BudgetState` | — | Yes | Consistent with mode and state | Accounting disposition. |
| `artifact_id` | `Nullable<Id>` | — | Yes | Non-null from `SEALED` onward | Generated artifact. |
| `failure_code` | `Nullable<ErrorCode>` | — | Yes | Non-null only for `FAILED` | Sanitized failure classification. |

Mode constraints:

- `CONTROLLED` requires at least one dataset or input artifact; `dp` is null and `budget_state` is `NONE`.
- `DP_AGGREGATE` requires exactly one dataset, no input artifacts, `artifact_kind` equal to `AGGREGATE`, and a non-null `dp`.
- `DP_POSTPROCESS` requires no datasets, at least one input artifact, `dp` equal to null, and `budget_state` equal to `NONE`.
- `DP_POSTPROCESS` inputs MUST be released DP artifacts with valid origin lineage in the same domain.
- For either DP mode, `publish_at` MUST be specified before execution and precede `expires_at`.

| DPConfig field | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `mechanism` | String | — | Yes | `DISCRETE_LAPLACE_VECTOR` | Core mechanism identifier. |
| `adjacency` | String | — | Yes | `ADD_REMOVE_PERSON` | Neighbor relation. |
| `dimension` | Integer | coordinates | Yes | 1–1,024 | Fixed output length. |
| `l1_bound` | Integer | integer coordinate units | Yes | 1–1,000,000 | Per-person contribution bound. |
| `epsilon_micros` | Integer | epsilon × 10⁶ | Yes | 1,000–1,000,000 | Reserved and committed charge. |
| `delta` | Integer | probability | Yes | Exactly 0 | Ideal mechanism’s additive privacy parameter. |
| `output_min` | Integer | coordinate units | Yes | Signed 32-bit range | Public lower clamp. |
| `output_max` | Integer | coordinate units | Yes | Signed 32-bit range; greater than `output_min` | Public upper clamp. |

All DP configuration values and transformation choices MUST be fixed independently of protected values, or obtained through an already accounted DP release.

### 4.5 Artifact

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `domain_id` | `Id` | — | Yes | Matches originating job | Accounting scope. |
| `job_id` | `Id` | — | Yes | Immutable | Originating job. |
| `kind` | `ArtifactKind` | — | Yes | Matches job | Result category. |
| `state` | `ArtifactState` | — | Yes | Defined below | Access disposition. |
| `media_type` | String | — | Yes | `application/json` or `application/octet-stream` | Content encoding. |
| `sha256` | `Digest` | — | Yes | Hash of exact stored content bytes | Integrity binding. |
| `byte_length` | Integer | bytes | Yes | 0–9,007,199,254,740,991 | Exact content length. |
| `expires_at` | `Timestamp` | UTC | Yes | Matches effective job deadline | Last permitted access time. |
| `dp_origin_job_ids` | `IdSet` | — | Yes | Mode-dependent | Distinct root DP aggregate jobs. |

For a controlled artifact, `dp_origin_job_ids` is empty. For a DP aggregate it contains the originating job. For DP postprocessing it is the union of its input origins.

An implementation MUST reject a postprocessing operation whose origin set would exceed 128 entries. It MUST NOT silently truncate lineage.

A `DP_AGGREGATE` artifact has exactly this content shape:

```json
{
  "standard": "WIA-AI-027",
  "version": "1.0",
  "job_id": "job_example",
  "mechanism": "DISCRETE_LAPLACE_VECTOR",
  "values": [3, 8, 5]
}
```

This is an illustrative payload. `values` contains exactly `dimension` integers, each within the public output bounds. No actual subject count, unclipped sum, random seed, or private diagnostic is permitted in this payload.

### 4.6 SubjectRequest and outcomes

| Name | Type | Unit | Required | Constraints | Description |
|---|---|---|---|---|---|
| `subject_ref` | `Reference` | — | Yes | Protected linkage reference | Verified person scope. |
| `verification_ref` | `Reference` | — | Yes | Immutable verification evidence | Authority to make the request. |
| `action` | `SubjectAction` | — | Yes | Defined below | Requested operation. |
| `state` | `SubjectRequestState` | — | Yes | Defined below | Workflow status. |
| `restriction_due_at` | `Timestamp` | UTC | Yes | Creation plus 60 seconds | Access-restriction deadline. |
| `erasure_due_at` | `Timestamp` | UTC | Yes | Creation plus 86,400 seconds | Live-storage erasure deadline. |
| `backup_due_at` | `Timestamp` | UTC | Yes | Creation plus 2,592,000 seconds | Backup deadline. |
| `outcomes` | Array of `Outcome` | — | Yes | Complete affected-resource inventory before completion | Resource-level disposition. |
| `evidence_refs` | `IdSet` | — | Yes | Required for terminal decisions | Supporting evidence. |

`Outcome` has the following required fields:

| Name | Type | Unit | Constraints | Description |
|---|---|---|---|---|
| `target_id` | `Id` | — | Registered resource or inventory item | Affected material. |
| `target_type` | `TargetType` | — | Defined below | Storage or artifact category. |
| `disposition` | `Disposition` | — | Defined below | Verified action. |
| `evidence_ref` | `Nullable<Reference>` | — | Required when no longer pending | Verification evidence. |
| `replacement_id` | `Nullable<Id>` | — | Required only for `REPLACED` | Approved replacement. |

The request clock starts when a verified request is accepted. Verification evidence MUST preserve the original receipt time so that verification delay remains visible. This standard does not prescribe a universal identity-verification method or deadline.

### 4.7 Enumerations

The following lists are exhaustive.

| Enumeration | Values and meanings |
|---|---|
| `DomainState` | `ACTIVE`: processing permitted subject to policy; `FROZEN`: new execution and new disclosure prohibited. |
| `DatasetState` | `ACTIVE`: eligible for approved processing; `RESTRICTED`: processing prohibited; `ERASED`: registered live and backup copies verified erased. |
| `DataCategory` | `DIRECT_IDENTIFIER`: direct identity/contact data; `QUASI_IDENTIFIER`: linkage-capable attributes; `SENSITIVE_ATTRIBUTE`: policy-designated sensitive values; `CONTENT`: text, image, audio, or other source content; `EMBEDDING`: derived representations; `MODEL_OUTPUT`: prior predictions or generated content. |
| `Workload` | `TRAINING`: parameter or model construction; `INFERENCE`: applying a model; `EVALUATION`: measuring model behavior; `ANALYTICS`: other approved aggregate analysis. |
| `ReleaseMode` | `CONTROLLED`: policy-authorized disclosure; `DP_AGGREGATE`: core mechanism output; `DP_POSTPROCESS`: output derived exclusively from qualifying DP artifacts and permitted independent inputs. |
| `ArtifactKind` | `MODEL`: executable parameters or model representation; `EMBEDDING`: representation collection; `AGGREGATE`: statistical vector; `PREDICTION`: inference result; `REPORT`: analysis document. |
| `JobState` | `QUEUED`: admitted, not executing; `RUNNING`: execution started; `SEALED`: immutable output stored but not released; `RELEASED`: recipient access enabled; `FAILED`: execution or release preparation failed; `CANCELED`: stopped before release; `REVOKED`: previously released access withdrawn. |
| `BudgetState` | `NONE`: no new DP charge; `RESERVED`: budget held; `COMMITTED`: irreversible charge; `RETURNED`: unused reservation released before execution. |
| `ArtifactState` | `SEALED`: inaccessible to recipients; `AVAILABLE`: released and accessible subject to policy; `REVOKED`: access withdrawn; `ERASED`: registered content copies verified erased. |
| `SubjectAction` | `RESTRICT`: stop affected processing; `ERASE`: restrict and remove affected material subject to documented exceptions. |
| `SubjectRequestState` | `OPEN`: accepted; `RESTRICTED`: access blocked; `REMEDIATING`: erasure or model work underway; `COMPLETE`: requested action verified; `EXCEPTION`: restricted retention remains; `REJECTED`: request authority failed subsequent verification. |
| `TargetType` | `DATASET`: source or derived snapshot; `ARTIFACT`: stored result or model; `CACHE`: transient serving or processing copy; `BACKUP`: recovery copy; `SUBJECT_INDEX`: linkage material. |
| `Disposition` | `PENDING`: unfinished; `RESTRICTED`: access disabled; `ERASED`: removal verified; `REPLACED`: original withdrawn and approved replacement installed; `RETAINED`: documented restricted exception. |

### 4.8 JSON Schema excerpt

This schema is the normative structural schema for the `DPConfig` object. Cross-field and mathematical constraints elsewhere in this document remain applicable.

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "WIA-AI-027 v1.0 DPConfig",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "mechanism",
    "adjacency",
    "dimension",
    "l1_bound",
    "epsilon_micros",
    "delta",
    "output_min",
    "output_max"
  ],
  "properties": {
    "mechanism": {
      "const": "DISCRETE_LAPLACE_VECTOR"
    },
    "adjacency": {
      "const": "ADD_REMOVE_PERSON"
    },
    "dimension": {
      "type": "integer",
      "minimum": 1,
      "maximum": 1024
    },
    "l1_bound": {
      "type": "integer",
      "minimum": 1,
      "maximum": 1000000
    },
    "epsilon_micros": {
      "type": "integer",
      "minimum": 1000,
      "maximum": 1000000
    },
    "delta": {
      "const": 0
    },
    "output_min": {
      "type": "integer",
      "minimum": -2147483648,
      "maximum": 2147483647
    },
    "output_max": {
      "type": "integer",
      "minimum": -2147483648,
      "maximum": 2147483647
    }
  }
}
```

The `$schema` value identifies the referenced JSON Schema dialect; it is not an additional WIA reference or a runtime network-fetch requirement.

### 4.9 Complete illustrative Job document

The following is a complete Job resource. Its identifiers refer to synthetic, previously registered resources. All timestamps and configuration choices are illustrative.

```json
{
  "id": "job_example",
  "tenant_id": "tenant_example",
  "created_at": "2030-01-01T01:00:00Z",
  "revision": 4,
  "domain_id": "domain_example",
  "purpose_id": "purpose_model_evaluation",
  "workload": "EVALUATION",
  "release_mode": "DP_AGGREGATE",
  "artifact_kind": "AGGREGATE",
  "dataset_ids": ["dataset_example"],
  "input_artifact_ids": [],
  "transform_ref": "transform_evaluation_bins",
  "recipient_ids": ["recipient_research"],
  "expires_at": "2030-01-01T02:00:00Z",
  "publish_at": "2030-01-01T01:05:00Z",
  "dp": {
    "mechanism": "DISCRETE_LAPLACE_VECTOR",
    "adjacency": "ADD_REMOVE_PERSON",
    "dimension": 3,
    "l1_bound": 1,
    "epsilon_micros": 250000,
    "delta": 0,
    "output_min": 0,
    "output_max": 100
  },
  "state": "RELEASED",
  "budget_state": "COMMITTED",
  "artifact_id": "artifact_example",
  "failure_code": null
}
```

## 5. Requirements and thresholds

The numerical limits in this section are requirements selected for this standard. They are not statistics, universal risk boundaries, or evidence of legal adequacy.

### 5.1 Functional requirements

**REQ-FUN-001 — Registration and lineage.**  
The implementation MUST validate and persist the applicable Section 4 resources before processing. Dataset snapshots, subject assignments, transformation manifests, and artifact dependencies MUST be integrity-bound and immutable. A transformation manifest MUST identify executable code, configuration, input schema, coordinate meanings, missing-value handling, and permitted network or tool capabilities.

**REQ-FUN-002 — Purpose and recipient enforcement.**  
Admission, execution start, and release MUST independently verify current authority, purpose, workload, recipient, and expiration constraints. Combining inputs MUST apply the intersection of their permissions. A derived artifact MUST NOT acquire broader recipient access or a later deadline merely because it is derived.

**REQ-FUN-003 — Durable lifecycle.**  
Jobs and artifacts MUST follow Section 7. Every accepted mutation MUST have a durable resource identity. A process crash MUST NOT create an unregistered output, duplicate charge, or unaccounted release.

**REQ-FUN-004 — Retention enforcement.**  
Expired or restricted inputs MUST cease being available to new execution. Live copies MUST be erased by the applicable deadline, and backup copies by their separate deadline. Restore procedures MUST apply restriction and erasure records before restoring serving access. Incomplete backup erasure MUST NOT be reported as complete erasure.

**REQ-FUN-005 — Subject-request automation.**  
Level 2 and Level 3 systems MUST implement the SubjectRequest workflow. They MUST discover affected snapshots, embeddings, caches, checkpoints, models, and backups through lineage and inventory. A request MUST NOT be marked complete while an affected resource remains unexamined.

**REQ-FUN-006 — Model impact and remediation.**  
For an affected controlled model, the system MUST withdraw access until an approved disposition is established. Acceptable dispositions include verified retraining without the affected material, verified replacement, restricted retention, or an explicitly evaluated unlearning method. An unlearning claim MUST name its reference model or behavioral target, assumptions, verification method, and remaining limitations.

Deleting training rows alone MUST NOT be described as removing their influence from an existing model.

### 5.2 Performance and resource limits

**REQ-PERF-001 — Bounded management input.**  
Management JSON requests MUST be limited to 1,048,576 bytes after HTTP framing, with no content encoding other than identity. Nesting depth MUST NOT exceed 32. Event pages MUST contain no more than 100 events.

These limits bound parser memory, validation work, and audit-page processing. They do not limit artifact content transferred through the artifact-content interface.

**REQ-PERF-002 — Restriction propagation.**  
A verified restriction, explicit revocation, or domain freeze MUST prevent affected new disclosures within 60 seconds of durable acceptance. Revocable bearer credentials, if used, MUST have a maximum lifetime of 300 seconds and MUST still obey the 60-second revocation requirement through active checks or equivalent invalidation.

Subject-request live erasure has a maximum interval of 86,400 seconds after acceptance; backup erasure has a maximum interval of 2,592,000 seconds, except for explicitly recorded restricted-retention exceptions.

The short restriction interval separates access withdrawal from slower storage reclamation. The longer backup interval accommodates recovery-media lifecycle work without allowing continued processing.

No fixed completion deadline is imposed on exact DP noise generation. An implementation MUST NOT truncate a mathematically required sampler merely to satisfy an operational timeout.

### 5.3 Privacy safety requirements

**REQ-SAF-001 — Inference and retrieval isolation.**  
Prompt content, retrieved documents, and generated instructions MUST be treated as untrusted data. They MUST NOT modify recipient ACLs, authorize tools, change privacy budgets, or select unrestricted export destinations.

RAG systems MUST enforce source eligibility before retrieval and recheck authorization before disclosure. Caches MUST include tenant, authorization scope, purpose, and relevant model or artifact revision in their isolation key.

Transient prompts, retrieved context, and inference outputs MUST be deleted from ordinary execution caches within 300 seconds after job completion unless separately registered for an authorized persistent purpose. Logging or tracing does not create such a purpose.

**REQ-SAF-002 — Model privacy evaluation.**  
Before controlled model release, Level 2 and Level 3 systems MUST evaluate applicable memorization, extraction, membership-inference, inversion, and cross-user disclosure risks.

The assessment MUST specify attacker access, available auxiliary information, target populations, test data separation, metrics, acceptance criteria, and limitations before interpreting results. A deployment-specific threshold MAY be used, but its value and rationale MUST be recorded. This standard specifies no universal empirical attack-success threshold.

Passing an attack evaluation MUST NOT be represented as a mathematical proof of privacy.

**REQ-SAF-003 — Fail-closed operation.**  
Unavailable policy evidence, inconsistent subject mapping, uncertain accounting state, unverified lineage, and failed integrity checks MUST prevent private execution or disclosure, as applicable.

A fallback model, debug path, preview feature, or manual export MUST NOT bypass the same release conditions.

### 5.4 Security and privacy requirements

**REQ-SEC-001 — Authentication and protected transport.**  
Management and recipient interfaces MUST authenticate principals and use TLS 1.3. Source data, subject indexes, sealed artifacts, and protected audit records MUST be encrypted at rest. Encryption keys MUST be separated from the protected content and restricted by role.

**REQ-SEC-002 — Secret and content handling.**  
Logs MUST NOT contain raw prompts, source records, embeddings, access tokens, encryption keys, random seeds, or direct identifiers. Diagnostic access to such material, when explicitly authorized, MUST occur through a separately controlled dataset workflow.

Executors MUST receive only the minimum input and destination capabilities required for the job.

**REQ-SEC-003 — Audit integrity.**  
Events defined in Section 6.5 MUST be appended to an integrity-protected audit sequence. A release MUST NOT succeed if its required accounting and audit records cannot be durably written.

Accounting records needed to prevent budget reset MUST remain available for the domain’s lifetime. Other required audit events MUST remain available for at least 90 days. Their retention MUST be declared and minimized beyond that requirement.

**REQ-SEC-004 — Tenant isolation.**  
Authorization MUST bind every resource lookup to the authenticated tenant. Resource identifiers alone MUST NOT grant access. Cross-tenant cache reuse, subject-index lookup, artifact dependency, and accounting mutation MUST be denied by default.

**REQ-PRIV-001 — Data minimization.**  
Transformations MUST receive only necessary fields. Direct identifiers SHOULD remain in the protected subject-index boundary unless the approved workload specifically requires them.

Hashing, tokenization, or pseudonymization MUST NOT automatically reclassify data as nonpersonal. Embeddings and model weights require an assessment of their information content and permitted use.

**REQ-PRIV-002 — Subject linkage integrity.**  
The subject index MUST associate all relevant records of one person before person-level clipping. Splitting one person across aliases MUST NOT create additional contribution allowances.

If reliable person grouping cannot be established, the implementation MUST reject the core person-level DP claim. It MAY perform separately authorized controlled processing.

**REQ-PRIV-003 — Honest erasure and assurance claims.**  
Erasure reports MUST distinguish source removal, cache removal, backup removal, model replacement, unlearning evaluation, and recipient-held copies.

Source erasure, artifact revocation, model retirement, and unlearning MUST NOT refund committed privacy budget. DP protection MUST NOT be presented as proof that erasure occurred or that the output is legally anonymous.

Previously released DP artifacts need not be withdrawn solely because their source records are erased, provided retention remains authorized. Any recipient-visible withdrawal based on protected membership MUST be included in the release-channel privacy analysis.

### 5.5 Differential privacy requirements

**REQ-DP-001 — Core mechanism and neighboring relation.**

For neighboring datasets \(D,D'\) differing by the addition or removal of one person’s complete records, the ideal mechanism MUST satisfy:

\[
\Pr[M(D)\in S]\leq
e^\epsilon\Pr[M(D')\in S]
\]

for every output event \(S\).

The core mechanism has \(\delta=0\). It does not use replacement adjacency. Replacing one person can require two add/remove steps and therefore does not inherit the same single-step bound.

Let \(d=\texttt{dimension}\), \(B=\texttt{l1_bound}\), and \(E=\texttt{epsilon_micros}\).

The approved transformation computes an integer vector \(u_p\in\mathbb Z^d\) for each person \(p\), using only that person’s records and approved independent constants.

Define:

\[
s_p=\sum_{j=1}^{d}|u_{p,j}|
\]

\[
v_{p,j}=
\operatorname{sign}(u_{p,j})
\left\lfloor
\frac{B|u_{p,j}|}{\max(B,s_p)}
\right\rfloor
\]

with \(\operatorname{sign}(0)=0\).

The aggregate is:

\[
f(D)=\sum_p v_p.
\]

Because \(\|v_p\|_1\leq B\), the add/remove sensitivity of \(f\) is at most \(B\).

Set:

\[
a=\left\lceil\frac{B\cdot10^6}{E}\right\rceil,
\qquad
r=\frac{a}{a+1}.
\]

For each coordinate, independently draw \(G_j^+\) and \(G_j^-\) with:

\[
\Pr[G=k]=(1-r)r^k,\qquad k=0,1,2,\ldots
\]

and let:

\[
Z_j=G_j^+-G_j^-.
\]

Release:

\[
y_j=\min\left(\texttt{output\_max},
\max\left(\texttt{output\_min},f_j(D)+Z_j\right)\right).
\]

The noise distribution obeys:

\[
\Pr[Z=z]=\frac{1-r}{1+r}r^{|z|}.
\]

Its likelihood ratio is bounded by:

\[
\exp\left(B\log(1+1/a)\right)
\leq \exp(B/a)
\leq \exp(E/10^6).
\]

Public output clamping is postprocessing and preserves this bound.

Internal arithmetic MUST be exact and large enough to avoid overflow in per-person accumulation, clipping, total aggregation, and noise handling. Fixed-width wraparound and floating-point approximations of the specified probabilities are not conforming substitutes.

The dimension and contribution limits bound implementation complexity. The per-job epsilon ceiling of 1 and domain ceiling of 3 provide a finite, auditable disclosure allowance. They do not establish that every use at those limits has acceptable privacy risk or useful accuracy.

**REQ-DP-002 — Atomic accounting.**

Admission MUST atomically verify:

```text
committed + reserved + requested <= limit
```

using integer micro-epsilon units.

A `DP_AGGREGATE` job reserves its full `epsilon_micros` before entering `QUEUED`. Before the first private input read, one transaction MUST move that amount from reserved to committed and set the job to `RUNNING`.

A committed charge MUST NOT decrease. A reservation MAY be returned only while the job has never entered `RUNNING` and a durable fence prevents any stale worker from starting.

The cumulative ideal bound is:

\[
\epsilon_{\text{domain}}
\leq
\frac{\sum \text{committed epsilon\_micros}}{10^6}.
\]

This is conservative when charged jobs produce no release. The core profile does not use parallel composition, subsampling amplification, or a tighter accountant.

**REQ-DP-003 — Randomness and execution.**

The sampler MUST implement the specified integer distribution under its documented randomness model. An implementation MAY use a more efficient exact sampler than repeated Bernoulli trials.

Production randomness MUST come from a cryptographically secure source with documented initialization, reseeding, and process-cloning behavior. Deterministic test seeds MUST NOT be enabled in production.

The mathematical proof assumes independent ideal random bits. A finite-seed CSPRNG implementation relies additionally on its cryptographic assumptions. Conformance evidence MUST distinguish that implementation assurance from an unconditional information-theoretic claim.

A timeout, truncated sampling loop, overflow fallback, value-dependent retry, or selective rejection MUST NOT be introduced without proof that the complete released distribution still meets the claimed bound.

After a crash following private access, a job without a durably sealed result MUST become `FAILED` and retain its charge. It MUST NOT regenerate a result under the same job. A new attempt requires a new job and charge.

**REQ-DP-004 — Complete release channel.**

Only the sealed, accounted artifact and approved public metadata may cross the DP recipient boundary.

Exact subject counts, unclipped statistics, data-dependent parameter choices, private validation scores, exception details, and previews MUST NOT accompany the artifact unless separately protected and accounted.

Publication selection, timing, availability, and failure behavior MUST either be independent of protected inputs and mechanism noise or be included in a proof for the joint recipient transcript. A generic error message or fixed output shape alone does not establish this condition.

The implementation MUST document its operational failure assumptions. If it cannot substantiate the recipient-channel guarantee, it MUST NOT claim conforming DP release through that channel.

**REQ-DP-005 — Closed postprocessing.**

A `DP_POSTPROCESS` job MUST have no capability to read raw datasets, subject indexes, private validation labels, or unaccounted model-selection signals.

It MAY use qualifying DP artifacts, constants independent of protected data, and independent randomness. Models trained exclusively from those inputs inherit the originating aggregate guarantees.

Public availability of a dataset does not by itself make that dataset an independent constant. Its relationship to the protected population MUST be assessed.

Postprocessing creates no new epsilon charge, but all root origins MUST remain traceable. Repeated delivery of the same stored artifact also creates no new charge. A newly randomized aggregate is a new mechanism execution and requires a new charge.

### 5.6 Interoperability, accessibility, and governance requirements

**REQ-INT-001 — Resource encoding.**  
Implementations MUST enforce Section 4 field names, types, enums, nullability, and cross-field constraints. They MUST NOT silently coerce an unsupported mechanism, workload, or state into another value.

**REQ-INT-002 — API semantics.**  
Implementations MUST implement the applicable Section 6 endpoints, status codes, revision preconditions, and version rules.

**REQ-INT-003 — Idempotency.**  
Every accepted mutation MUST obey Section 6.3. Network retries MUST NOT create additional jobs, budget charges, releases, or subject requests.

**REQ-ACC-001 — Understandable privacy controls.**  
Human-facing controls MUST be keyboard-operable, provide programmatically identifiable labels and status changes, and not rely on color alone. Machine and human interfaces MUST distinguish accepted, restricted, erasure pending, exception, and complete states.

A “deleted” confirmation MUST identify whether backups, models, or recipient-held copies remain outside the completed action.

**REQ-GOV-001 — Reproducible assurance.**  
The implementation MUST maintain the scope statement, threat model, conformance results, exceptions, configuration, and change history required by Sections 8 and 9.

## 6. Interfaces and API

### 6.1 Common conventions

The base path is:

```text
/wia/ai-privacy/v1
```

Requests and responses MUST carry:

```text
WIA-Standard: WIA-AI-027/1.0
```

JSON requests use `Content-Type: application/json`. Resource responses include an `ETag` containing the quoted decimal revision.

Mutating actions on existing resources require `If-Match`. A mismatch returns `412 REVISION_CONFLICT`.

Successful creation and action responses use this complete receipt shape:

```json
{
  "resource_id": "job_example"
}
```

The response also includes `Location` identifying the resource. Acceptance of an asynchronous job or subject request does not mean that release or erasure is complete.

### 6.2 Endpoint inventory

Paths below are relative to the base path.

| Method | Path | Purpose | Request JSON | Response JSON or content | Success |
|---|---|---|---|---|---|
| GET | `/domain` | Read tenant accounting scope | None | PrivacyDomain | 200 |
| POST | `/domain/freeze` | Freeze new processing and disclosure | `{}` | Receipt | 200 |
| POST | `/datasets` | Register immutable snapshot | DatasetCreate | Receipt | 201 |
| GET | `/datasets/{id}` | Read dataset metadata | None | Dataset | 200 |
| POST | `/datasets/{id}/restrict` | Restrict a snapshot | `{}` | Receipt | 200 |
| POST | `/jobs` | Admit processing | JobCreate | Receipt | 202 |
| GET | `/jobs/{id}` | Read durable job status | None | Job | 200 |
| POST | `/jobs/{id}/cancel` | Cancel before release | `{}` | Receipt | 200 |
| GET | `/artifacts/{id}` | Read authorized artifact metadata | None | Artifact | 200 |
| GET | `/artifacts/{id}/content` | Retrieve released bytes | None | Stored artifact content | 200 |
| POST | `/artifacts/{id}/revoke` | Withdraw future access | `{}` | Receipt | 200 |
| POST | `/subject-requests` | Accept verified request | SubjectRequestCreate | Receipt | 202 |
| GET | `/subject-requests/{id}` | Read workflow and outcomes | None | SubjectRequest | 200 |
| GET | `/audit-events` | Read ordered audit page | None | EventPage | 200 |

Subject-request endpoints are mandatory at Level 2 and Level 3. At Level 1 they MAY be absent; this absence MUST be declared in the conformance statement.

Resource GET responses use the complete Section 4 field sets. Server-assigned fields MUST NOT appear in create requests.

`DatasetCreate` contains exactly `data_ref`, `subject_index_ref`, `schema_ref`, `snapshot_digest`, `categories`, and `policy`. The server assigns the tenant, domain, state, timestamps, identifier, and revision.

`JobCreate` contains exactly the Job fields from `purpose_id` through `dp`, excluding `domain_id`, state fields, and result fields.

`SubjectRequestCreate` contains exactly:

```json
{
  "subject_ref": "subject_example",
  "verification_ref": "verification_example",
  "action": "ERASE"
}
```

This request is illustrative. Its subject and verification references are synthetic.

An illustrative PrivacyDomain response is:

```json
{
  "id": "domain_example",
  "tenant_id": "tenant_example",
  "created_at": "2030-01-01T00:00:00Z",
  "revision": 3,
  "state": "ACTIVE",
  "epsilon_limit_micros": 3000000,
  "epsilon_committed_micros": 250000,
  "epsilon_reserved_micros": 0
}
```

Artifact content MUST NOT be redirected to an uncontrolled storage URL. Any delegated delivery capability MUST enforce the same recipient, expiration, and revocation rules.

### 6.3 Idempotency and concurrency

All POST requests MUST include an `Idempotency-Key` generated with at least 128 bits of randomness. Its transmitted representation MUST match `^[A-Za-z0-9_-]{22,128}$` and MUST contain no personal information.

The key scope consists of:

- Tenant.
- Authenticated principal.
- HTTP method.
- Normalized resource path.

The server MUST atomically persist an accepted operation, its request fingerprint, and its receipt.

The fingerprint includes the parsed request and any `If-Match` precondition. Object member order is irrelevant. `IdSet` order is irrelevant. Other array order and string content remain significant.

The same key and equivalent request return the original acceptance status and receipt. The same key with different content returns `409 IDEMPOTENCY_CONFLICT`.

Deduplication MUST be checked before current revision preconditions when replaying an accepted operation. Otherwise a successful operation could become unrecoverable merely because it changed the revision.

Accepted-operation key records, or privacy-preserving tombstones sufficient to prevent replay, MUST persist for the domain’s lifetime. A response lost after acceptance MUST NOT authorize a second execution.

Rejected requests that caused no mutation need not retain a key binding. An unavailable response after uncertain acceptance MUST instruct clients to retry with the same key.

### 6.4 Errors

Errors have exactly this shape:

```json
{
  "error": {
    "code": "BUDGET_EXCEEDED",
    "message": "The requested privacy charge cannot be reserved.",
    "request_id": "request_example",
    "retryable": false
  }
}
```

The example contains no private data. `message` MUST contain at most 256 Unicode scalar values and MUST NOT expose source content or subject membership.

| ErrorCode | HTTP status | Meaning | `retryable` |
|---|---:|---|---|
| `INVALID_REQUEST` | 400 | Malformed JSON, missing field, invalid type, or constraint violation | false |
| `VERSION_UNSUPPORTED` | 400 | Missing or unsupported WIA version header | false |
| `UNAUTHENTICATED` | 401 | Authentication absent or invalid | false |
| `FORBIDDEN` | 403 | Principal lacks the operation’s role | false |
| `POLICY_DENIED` | 403 | Purpose, recipient, authority, or retention rule denies processing | false |
| `NOT_FOUND` | 404 | Resource absent or not visible to this principal | false |
| `STATE_CONFLICT` | 409 | Operation invalid for current state | false |
| `IDEMPOTENCY_CONFLICT` | 409 | Key reused with different request content | false |
| `BUDGET_EXCEEDED` | 409 | Atomic reservation would exceed the ceiling | false |
| `ARTIFACT_UNAVAILABLE` | 409 | Authorized artifact content is not currently releasable | false |
| `REVISION_CONFLICT` | 412 | `If-Match` does not match | false |
| `RESOURCE_TOO_LARGE` | 413 | Management request exceeds limits | false |
| `EVIDENCE_INVALID` | 422 | Required reference, signature, digest, or evidence validation fails | false |
| `MECHANISM_UNSUPPORTED` | 422 | Requested privacy mechanism is unsupported | false |
| `RATE_LIMITED` | 429 | Request admission temporarily limited | true |
| `INTERNAL_ERROR` | 500 | Sanitized internal failure | false |
| `UNAVAILABLE` | 503 | Required service temporarily unavailable | true |

These are all `ErrorCode` values.

A failed Job may use only `POLICY_DENIED`, `EVIDENCE_INVALID`, `INTERNAL_ERROR`, or `UNAVAILABLE` as `failure_code`. Retrying its HTTP submission still returns the same failed job; it does not restart execution.

Unauthorized resource lookups MUST return `NOT_FOUND` where disclosing existence would violate tenant or recipient isolation.

### 6.5 Events and audit messages

`GET /audit-events` accepts optional `after_sequence` and `limit` query parameters. `after_sequence` is a nonnegative integer; `limit` is 1–100 and defaults to 100.

An EventPage contains exactly `events`, an array of AuditEvent objects, and `next_after_sequence`, the last returned sequence or the supplied cursor when the page is empty.

Each AuditEvent contains:

| Field | Type | Meaning |
|---|---|---|
| `event_id` | `Id` | Unique event identity. |
| `tenant_id` | `Id` | Isolation scope. |
| `sequence` | Integer | Contiguous tenant-local order starting at 1. |
| `occurred_at` | `Timestamp` | Server event time. |
| `actor_id` | `Id` | Opaque principal or service identity. |
| `action` | `AuditAction` | Event classification. |
| `resource_id` | `Id` | Primary affected resource. |
| `job_id` | `Nullable<Id>` | Related job. |
| `epsilon_micros` | Integer | Charge or reservation amount relevant to this event; otherwise 0. |
| `previous_digest` | `Digest` | Prior event digest; all zeroes for the first event. |
| `digest` | `Digest` | SHA-256 of the JCS serialization of this event with `digest` omitted. |

`AuditAction` has exactly these values:

| Value | Meaning |
|---|---|
| `DATASET_REGISTERED` | Snapshot registration accepted. |
| `DATASET_RESTRICTED` | Snapshot processing disabled. |
| `JOB_ADMITTED` | Job and any reservation created. |
| `BUDGET_COMMITTED` | Reservation irreversibly charged. |
| `JOB_STATE_CHANGED` | Job lifecycle changed. |
| `ARTIFACT_REVOKED` | Future artifact access withdrawn. |
| `SUBJECT_REQUEST_CHANGED` | Subject workflow changed. |
| `ACCESS_DENIED` | An access or processing attempt was denied. |
| `DOMAIN_FROZEN` | Domain-wide processing restriction accepted. |
| `DATA_ERASED` | Inventory item erasure verified. |

Audit storage MUST resist silent rewriting by ordinary operators. A hash chain alone is insufficient if an attacker can replace the entire chain and its trusted starting point; an independently protected append-only sink or authenticated checkpoints are required.

## 7. Protocol and lifecycle

### 7.1 Job state machine

```text
QUEUED -----> RUNNING -----> SEALED -----> RELEASED -----> REVOKED
   |              |             |
   +--> CANCELED  +--> CANCELED  +--> CANCELED
   |              |             |
   +--> FAILED    +--> FAILED    +--> FAILED
```

No other Job transitions are permitted.

| Transition | Trigger | Required effect |
|---|---|---|
| Admission → `QUEUED` | Valid create request | Persist job, idempotency record, audit event, and any reservation atomically. |
| `QUEUED` → `RUNNING` | Authorized worker starts | Recheck policy; commit DP charge before private access; establish worker fence. |
| `RUNNING` → `SEALED` | Successful execution | Persist immutable bytes, digest, descriptor, and lineage before publishing the reference. |
| `SEALED` → `RELEASED` | Release gate approves | Recheck policy, expiration, publication conditions, audit durability, and accounting. |
| Active state → `CANCELED` | Cancellation, restriction, or expiration before release | Fence workers; erase staging copies; return only never-used reservations. |
| Active state → `FAILED` | Unrecoverable failure | Preserve charges already committed; prevent release. |
| `RELEASED` → `REVOKED` | Revocation or expiration | Disable future serving and propagate the restriction within 60 seconds. |

“Active state” in this table means `QUEUED`, `RUNNING`, or `SEALED`.

A canceled or failed job MUST NOT restart. A revoked job MUST NOT return to `RELEASED`.

An artifact may transition from `SEALED` to `AVAILABLE`, from either state to `REVOKED`, and from `SEALED` or `REVOKED` to `ERASED`. Expiration first revokes availability before physical erasure.

### 7.2 Main processing sequence

1. The operator registers immutable data and approved evidence.
2. Registration validates snapshot integrity, subject linkage, schema, authority, and retention.
3. The operator submits a JobCreate request with an idempotency key.
4. Admission checks permissions and performs any atomic budget reservation.
5. The worker rechecks policy and establishes exclusive execution ownership.
6. A DP aggregate charge is committed before private input access.
7. The executor produces an output without recipient access.
8. The artifact store seals exact bytes and records their digest and lineage.
9. The release gate checks all current conditions.
10. The recipient retrieves the stored artifact through the release gate.
11. Repeated retrieval returns the same stored content.
12. Expiration or revocation disables future access and initiates applicable erasure.

A worker lease expiring does not authorize a second computation. Recovery may use an already sealed artifact; otherwise the charged job fails.

### 7.3 Cancellation and release races

Cancellation and release MUST have a single transactional ordering.

If cancellation wins before release, recipient access MUST remain disabled. If release wins first, cancellation returns `STATE_CONFLICT`; the caller may request artifact revocation.

An uncertain network acknowledgement does not alter the durable winner. Clients recover the result through the original idempotency key and resource GET.

### 7.4 Subject-request state machine

Permitted transitions are:

```text
OPEN -> RESTRICTED -> REMEDIATING -> COMPLETE
  |          |             |
  |          +-> COMPLETE  +-> EXCEPTION
  +-> REJECTED                  |
                               +-> REMEDIATING
```

`RESTRICT` may complete after verified restriction. `ERASE` requires inventory-wide removal or approved replacement, including backup disposition.

If a snapshot cannot be safely restricted by subject, the entire snapshot MUST be restricted. A sanitized replacement receives a new dataset identifier.

A request with `RETAINED` outcomes enters `EXCEPTION`, not `COMPLETE`. Exception evidence MUST identify the retained items, authority, permitted access, review owner, and expiration.

When an exception expires, the request returns to `REMEDIATING`; access remains restricted. Live erasure MUST complete within 86,400 seconds of that expiry unless a newly justified exception is recorded.

Recipient-held copies and irreversible external publication MUST be reported as limitations. The controller MUST NOT assert remote deletion without verification evidence.

## 8. Testing and conformance procedures

### 8.1 Test environment

Testing MUST use synthetic persons and records. Production personal data MUST NOT be required for certification.

The environment MUST include:

- At least two tenants and distinct operator, recipient, and auditor principals.
- A controllable clock for retention and revocation tests.
- Crash injection before and after accounting, sealing, and release transactions.
- Concurrent clients capable of racing admission and cancellation.
- An inspectable artifact store and audit sink.
- A reference integer implementation of the core mechanism.
- Instrumentation for network access, private-input reads, and denied capabilities.

Test-only deterministic randomness MAY be used for exact comparisons. Separate tests MUST verify that production builds reject test-seed configuration.

The report MUST disclose hardware, runtime, storage consistency assumptions, cryptographic dependencies, and any external service emulation.

### 8.2 Required test cases

| ID | Requirement | Method | Pass criteria |
|---|---|---|---|
| TC-001 | REQ-FUN-001, REQ-INT-001 | Register valid and malformed resources; alter a snapshot after registration. | Valid resources accepted; unknown fields and invalid enums rejected; altered bytes prevent execution. |
| TC-002 | REQ-FUN-002 | Combine inputs with differing purposes, recipients, and deadlines. | Effective permissions equal their intersection; no broader release succeeds. |
| TC-003 | REQ-FUN-003 | Inject crashes at each durable state boundary. | Recovery yields one valid state, no unregistered release, and no duplicate job. |
| TC-004 | REQ-FUN-004 | Advance time through use and erasure deadlines; restore a backup. | Access stops on time; restoration applies restrictions before serving; completion reflects actual backup erasure. |
| TC-005 | REQ-FUN-005 | Submit an erasure request spanning dataset, embedding, cache, model, and backup. | Every dependency receives a verified outcome; incomplete inventory cannot complete. |
| TC-006 | REQ-FUN-006 | Delete source rows while retaining a trained model. | Model is withdrawn pending disposition; system does not claim influence removal from source deletion alone. |
| TC-007 | REQ-PERF-001 | Submit boundary-sized, oversized, and deeply nested requests. | The specified limits are enforced without partial mutation. |
| TC-008 | REQ-PERF-002 | Revoke access during active sessions and serving. | No affected new disclosure succeeds after 60 seconds; credential lifetime does not bypass invalidation. |
| TC-009 | REQ-SAF-001 | Insert instructions into prompts and retrieved documents to change ACLs or export data. | Model-produced instructions cannot authorize policy or export changes. |
| TC-010 | REQ-SAF-001 | Reuse similar prompts across principals and tenants; change document ACLs during retrieval. | No unauthorized cache or retrieval result crosses the boundary. |
| TC-011 | REQ-SAF-002 | Inspect and reproduce the model privacy assessment. | Threat assumptions, predeclared criteria, separated test data, metrics, and limitations are present and reproducible. |
| TC-012 | REQ-SAF-003 | Disconnect policy, evidence, accountant, and audit dependencies. | Affected execution or release fails closed. |
| TC-013 | REQ-SEC-001 | Test authentication, TLS negotiation, storage access, and key permissions. | Unauthorized access fails; required transport and storage protection is active. |
| TC-014 | REQ-SEC-002 | Place synthetic secrets in inputs; inspect logs, traces, and error responses. | No prohibited content appears outside approved processing storage. |
| TC-015 | REQ-SEC-003 | Modify, remove, and reorder audit events; attempt chain replacement. | Tampering is detected against an independently protected trust point. |
| TC-016 | REQ-SEC-004 | Substitute another tenant’s identifiers in every applicable endpoint. | No cross-tenant read, dependency, or mutation succeeds. |
| TC-017 | REQ-PRIV-001 | Inspect executor grants and pseudonymized-data classifications. | Only necessary fields are granted; pseudonymization does not automatically remove controls. |
| TC-018 | REQ-PRIV-002 | Give one synthetic person multiple records and aliases. | Records are grouped into one contribution or the person-level DP claim is rejected. |
| TC-019 | REQ-PRIV-003 | Erase data and revoke artifacts after a committed release. | Budget remains committed; reports distinguish erasure, model effects, and external copies. |
| TC-020 | REQ-DP-001 | Check clipping, aggregate sensitivity, and exact noise probabilities with reference calculations. | Per-person L1 contribution never exceeds B; sampler law and privacy-bound derivation match Section 5.5. |
| TC-021 | REQ-DP-001 | Exercise large intermediate values and output clamps. | No overflow, wraparound, or unapproved approximation occurs; outputs satisfy fixed bounds. |
| TC-022 | REQ-DP-002 | Race 32 admission requests against insufficient remaining budget. | Accepted reservations plus committed charges never exceed the ceiling. |
| TC-023 | REQ-DP-002, REQ-DP-003 | Crash before private access, after commitment, and before sealing. | Only unused reservations return; charged unsealed failures do not regenerate under the same job. |
| TC-024 | REQ-DP-003 | Inspect sampler implementation, randomness initialization, cloning behavior, and production configuration. | Exact sampling assumptions are justified; production test seeds and unsafe reused randomness are rejected. |
| TC-025 | REQ-DP-004 | Inspect every recipient-visible content, error, timing, and selection route. | Each channel is covered by the stated privacy analysis; no private diagnostic bypass exists. |
| TC-026 | REQ-DP-005 | Attempt raw-data access and private model selection from a postprocessing job. | Access is denied; only approved DP roots and independent inputs remain reachable. |
| TC-027 | REQ-INT-002 | Exercise endpoint status, version, and revision cases. | Responses and mutations match Section 6 exactly. |
| TC-028 | REQ-INT-003 | Retry accepted requests after response loss and with changed bodies. | Same request yields the original resource; changed content conflicts; no extra charge occurs. |
| TC-029 | REQ-ACC-001 | Review keyboard operation, labels, status announcements, and deletion explanations. | Applicable interfaces are operable and distinguish partial from complete actions. |
| TC-030 | REQ-GOV-001 | Reconstruct the claimed configuration and evidence set. | Scope, requirements, results, exceptions, and change history are traceable. |
| TC-031 | REQ-FUN-003, REQ-DP-004 | Race cancellation, release, expiration, and recipient retrieval. | One durable ordering governs access; no canceled or uncommitted output is exposed. |

The 32-client race is a conformance stress condition selected to expose non-atomic admission. It is not a throughput claim.

Statistical sampling tests MAY detect implementation defects. They MUST NOT replace the distribution analysis required by TC-020 or the channel analysis required by TC-025.

### 8.3 Reporting format

A conformance report MUST be a JSON document containing:

| Field | Type | Required content |
|---|---|---|
| `standard` | String | `WIA-AI-027` |
| `version` | String | `1.0` |
| `level` | Integer | 1, 2, or 3 |
| `implementation_id` | `Id` | Tested implementation and build reference |
| `scope_ref` | `Reference` | Boundary, tenants, interfaces, and release claims |
| `environment_ref` | `Reference` | Reproducible test environment |
| `configuration_digest` | `Digest` | Tested configuration |
| `results` | Array | Per-test records |
| `assessment_ref` | `Reference` | Reviewer conclusions and limitations |

Each result contains exactly `test_id`, `requirement_ids`, `status`, `evidence_ref`, and `not_applicable_reason`.

`status` has exactly three values: `PASS`, `FAIL`, and `NOT_APPLICABLE`. The reason is null unless the status is `NOT_APPLICABLE`.

Every test applicable to the claimed level MUST appear. A required failed test prevents a passing conformance decision. A not-applicable result requires the explicit condition permitted by this document or a requirement belonging exclusively to a higher level.

## 9. Certification, governance and versioning

### 9.1 Assessment process

A conformity assessment consists of:

1. Declaring the implementation, configuration, boundary, and claimed level.
2. Reviewing authority, subject linkage, lineage, retention, and threat-model evidence.
3. Executing all applicable conformance tests.
4. Reviewing DP mathematics and implementation assumptions for Level 3.
5. Resolving failed requirements.
6. Issuing a signed assessment record identifying the exact evidence set.

The assessment record MUST distinguish mathematical mechanism properties, implementation assurance, operational assumptions, and empirical model testing.

This document does not identify an institutional WIA certification authority or authorize use of an organizational certification mark. A technical conformance assessment MUST NOT be represented as institutional endorsement without separate authorization.

### 9.2 Renewal and suspension

An assessment is valid for no more than 12 months without renewal. This interval is a governance requirement intended to force review of software, dependencies, access policy, and deployment drift.

The responsible assessor MUST suspend affected claims when there is:

- An unresolved budget-integrity defect.
- An uncontrolled disclosure route.
- A material subject-linkage failure.
- A compromised production randomness or encryption dependency.
- An unassessed change to the DP mechanism or its observer boundary.

Renewal MUST include changed-component review and applicable regression tests. An unchanged version label does not excuse testing a changed deployment.

### 9.3 Change control

Changes to transformations, subject assignment, clipping, sampling, accounting, release selection, or postprocessing capabilities require impact analysis before deployment.

A change that can alter the privacy distribution requires renewed mathematical review. A change to model architecture, training data, retrieval policy, or serving interface requires review of applicable empirical privacy risks.

Dataset replacements and model revisions MUST retain lineage to prior versions where needed for subject-request handling. Such lineage MUST NOT be used to reset privacy accounting.

### 9.4 Versioning and compatibility

The standard uses major and minor versions. This document defines version `1.0`.

The `/v1` wire schemas and enum meanings defined here are fixed. A future compatible extension MAY add a new endpoint or separately negotiated representation. It MUST NOT silently add fields to an existing strict v1 response schema.

Renaming fields, changing enum meaning, weakening privacy accounting, or changing the neighboring relation requires a new major interface version.

Editorial corrections that change no normative behavior MAY be issued without changing wire behavior. Implementations MUST retain the exact revision of the specification used for assessment.

Unsupported extensions MUST be rejected explicitly. A system MUST NOT silently downgrade a requested DP release to `CONTROLLED`.

## Appendix A: Example payloads

All payloads in this appendix are illustrative. Names, timestamps, identifiers, and numerical settings describe synthetic configurations.

### A.1 Dataset registration request

This example registers a synthetic evaluation snapshot. The digest represents a fixture snapshot whose bytes are held by the example registry.

```json
{
  "data_ref": "storage_evaluation_snapshot",
  "subject_index_ref": "index_evaluation_subjects",
  "schema_ref": "schema_evaluation_records",
  "snapshot_digest": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
  "categories": ["CONTENT", "MODEL_OUTPUT"],
  "policy": {
    "purpose_ids": ["purpose_model_evaluation"],
    "recipient_ids": ["recipient_research"],
    "workloads": ["TRAINING", "EVALUATION", "ANALYTICS"],
    "authority_ref": "authority_evaluation",
    "use_until": "2030-02-01T00:00:00Z",
    "erase_by": "2030-02-02T00:00:00Z",
    "backup_erase_by": "2030-03-01T00:00:00Z",
    "artifact_ttl_seconds": 86400,
    "controlled_release_allowed": false
  }
}
```

The digest is an illustrative fixture identifier, not a claim about supplied file contents. A real registration MUST verify the digest against the actual stored bytes.

### A.2 DP aggregate submission

For this illustrative configuration, \(B=1\) and \(E=250{,}000\), so \(a=4\) and \(r=4/5\). The charged epsilon is \(0.25\).

```json
{
  "purpose_id": "purpose_model_evaluation",
  "workload": "EVALUATION",
  "release_mode": "DP_AGGREGATE",
  "artifact_kind": "AGGREGATE",
  "dataset_ids": ["dataset_example"],
  "input_artifact_ids": [],
  "transform_ref": "transform_evaluation_bins",
  "recipient_ids": ["recipient_research"],
  "expires_at": "2030-01-01T02:00:00Z",
  "publish_at": "2030-01-01T01:05:00Z",
  "dp": {
    "mechanism": "DISCRETE_LAPLACE_VECTOR",
    "adjacency": "ADD_REMOVE_PERSON",
    "dimension": 3,
    "l1_bound": 1,
    "epsilon_micros": 250000,
    "delta": 0,
    "output_min": 0,
    "output_max": 100
  }
}
```

The transformation may assign a person to a single evaluation category or construct another integer vector satisfying the specified processing and clipping rules. The category definitions MUST be fixed in the transformation manifest before private execution.

### A.3 Model construction by postprocessing

This illustrative job constructs a model from previously released noisy statistics. Its transformation has no raw-data capability.

```json
{
  "purpose_id": "purpose_model_evaluation",
  "workload": "TRAINING",
  "release_mode": "DP_POSTPROCESS",
  "artifact_kind": "MODEL",
  "dataset_ids": [],
  "input_artifact_ids": ["artifact_example"],
  "transform_ref": "transform_model_from_statistics",
  "recipient_ids": ["recipient_research"],
  "expires_at": "2030-01-01T02:00:00Z",
  "publish_at": "2030-01-01T01:15:00Z",
  "dp": null
}
```

The absence of a new charge depends on the closed input boundary. Selecting this model using unaccounted private validation results would violate that boundary.

### A.4 Completed restriction request

This example completes a restriction action. It does not assert physical erasure.

```json
{
  "id": "request_subject_example",
  "tenant_id": "tenant_example",
  "created_at": "2030-01-01T03:00:00Z",
  "revision": 3,
  "subject_ref": "subject_example",
  "verification_ref": "verification_example",
  "action": "RESTRICT",
  "state": "COMPLETE",
  "restriction_due_at": "2030-01-01T03:01:00Z",
  "erasure_due_at": "2030-01-02T03:00:00Z",
  "backup_due_at": "2030-01-31T03:00:00Z",
  "outcomes": [
    {
      "target_id": "dataset_example",
      "target_type": "DATASET",
      "disposition": "RESTRICTED",
      "evidence_ref": "evidence_dataset_restriction",
      "replacement_id": null
    },
    {
      "target_id": "artifact_controlled_model",
      "target_type": "ARTIFACT",
      "disposition": "RESTRICTED",
      "evidence_ref": "evidence_model_withdrawal",
      "replacement_id": null
    }
  ],
  "evidence_refs": [
    "evidence_inventory_complete",
    "evidence_restriction_verified"
  ]
}
```

The erasure-related timestamps remain present because they are required resource fields. They do not turn a `RESTRICT` action into an `ERASE` action.

### A.5 Illustrative conformance result

```json
{
  "test_id": "TC-028",
  "requirement_ids": ["REQ-INT-003"],
  "status": "PASS",
  "evidence_ref": "evidence_idempotency_recovery",
  "not_applicable_reason": null
}
```

This is one result entry, not a complete conformance report.

## Appendix B: Revision history

v1.0 — initial release