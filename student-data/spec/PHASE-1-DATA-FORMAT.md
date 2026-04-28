# WIA-EDU-010 — Phase 1: Data Format

> Student-data canonical Phase 1: profile + academic-record + attendance envelopes.

# WIA-EDU-010 Student Data Standard v1.0

## Phase 1: Data Format & Structure

**Status:** ✅ Complete
**Version:** 1.0.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the foundational data format and structure requirements for WIA-compliant student data systems. Phase 1 establishes JSON as the primary format with comprehensive schemas for student profiles, academic records, enrollment data, and privacy controls.

## 2. Scope

Phase 1 covers:
- Student profile data structures
- Academic records format
- Enrollment and attendance schemas
- Privacy and consent models
- Data validation requirements
- Metadata standards

## 3. Core Data Model

### 3.1 Student Profile

The student profile is the central entity containing all personal and enrollment information.

```json
{
  "studentId": "string (required, unique)",
  "personalInfo": {
    "firstName": "string (required)",
    "lastName": "string (required)",
    "middleName": "string (optional)",
    "preferredName": "string (optional)",
    "dateOfBirth": "date (required, ISO 8601)",
    "email": "string (required, format: email)",
    "phone": "string (optional, E.164 format)",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "zipCode": "string",
      "country": "string (ISO 3166-1 alpha-2)"
    },
    "emergencyContact": {
      "name": "string (required)",
      "relationship": "string",
      "phone": "string (required)",
      "email": "string (optional)"
    }
  },
  "enrollment": {
    "status": "enum (active, leave_of_absence, withdrawn, graduated, suspended, expelled)",
    "enrollmentDate": "date (required, ISO 8601)",
    "expectedGraduation": "date (optional, ISO 8601)",
    "program": "string (required)",
    "major": "string (required)",
    "minor": "string (optional)",
    "academicLevel": "enum (undergraduate, graduate, doctoral, certificate)",
    "currentYear": "integer (1-8)",
    "fullTime": "boolean"
  },
  "privacy": {
    "directoryInformation": "boolean (default: true)",
    "parentAccess": "boolean (default: false if age >= 18)",
    "thirdPartySharing": "boolean (default: false)",
    "researchParticipation": "boolean (default: false)",
    "marketingCommunications": "boolean (default: false)"
  },
  "metadata": {
    "createdAt": "datetime (ISO 8601 with timezone)",
    "updatedAt": "datetime (ISO 8601 with timezone)",
    "createdBy": "string (user ID)",
    "updatedBy": "string (user ID)",
    "version": "string (semantic versioning)",
    "standard": "string (WIA-EDU-010)"
  }
}
```

### 3.2 Academic Records

Academic records track course enrollments, grades, and academic performance.

```json
{
  "recordId": "string (required, unique)",
  "studentId": "string (required, foreign key)",
  "semester": {
    "term": "enum (Fall, Spring, Summer, Winter)",
    "year": "integer (YYYY)",
    "startDate": "date (ISO 8601)",
    "endDate": "date (ISO 8601)"
  },
  "courses": [
    {
      "courseId": "string (required)",
      "courseCode": "string (required)",
      "courseName": "string (required)",
      "credits": "number (required)",
      "creditsAttempted": "number",
      "creditsEarned": "number",
      "gradeScale": "enum (letter, pass_fail, numerical, percentage)",
      "grade": "string",
      "gradePoints": "number",
      "status": "enum (in_progress, completed, withdrawn, incomplete, failed)",
      "instructor": "string",
      "department": "string"
    }
  ],
  "gpa": {
    "semester": "number (0.00-4.00 or custom scale)",
    "cumulative": "number (0.00-4.00 or custom scale)",
    "major": "number (optional)",
    "creditsAttempted": "integer",
    "creditsEarned": "integer",
    "qualityPoints": "number"
  },
  "academicStanding": "enum (good_standing, probation, suspension, honors, deans_list)"
}
```

### 3.3 Attendance Records

```json
{
  "attendanceId": "string (required, unique)",
  "studentId": "string (required, foreign key)",
  "courseId": "string (required, foreign key)",
  "attendanceRecords": [
    {
      "date": "date (ISO 8601)",
      "status": "enum (present, absent_excused, absent_unexcused, tardy, early_departure)",
      "arrivalTime": "time (HH:MM:SS)",
      "departureTime": "time (HH:MM:SS)",
      "reason": "string (optional)",
      "documentation": "string (optional, URI to supporting document)",
      "notes": "string (optional)"
    }
  ],
  "summary": {
    "totalClasses": "integer",
    "present": "integer",
    "excusedAbsences": "integer",
    "unexcusedAbsences": "integer",
    "tardies": "integer",
    "attendanceRate": "number (0.00-1.00)"
  }
}
```

## 4. Data Validation

### 4.1 JSON Schema

All data structures MUST be validated against JSON Schema (Draft 2020-12).

**Required Validations:**
- Type checking for all fields
- Format validation (email, date, phone)
- Required field enforcement
- Enumeration constraint checking
- Range validation for numbers
- Pattern matching for IDs

### 4.2 Business Rules

**Student ID:**
- Must be unique within institution
- Pattern: alphanumeric with optional hyphens
- Maximum length: 50 characters

**Email:**
- Must be valid RFC 5322 format
- Should be unique (institutional policy)

**Dates:**
- All dates in ISO 8601 format (YYYY-MM-DD)
- Enrollment date cannot be in the future
- Date of birth must be at least 3 years in the past

**GPA:**
- Must be between 0.00 and institutional maximum (typically 4.00 or 5.00)
- Rounded to 2 decimal places

## 5. Privacy and Consent

### 5.1 Privacy Settings

All student records MUST include privacy settings that control data access and disclosure.

**Default Privacy Levels:**
- Directory Information: Opt-out (default: visible)
- Parent Access: Age-dependent (default: false if student ≥ 18)
- Third-Party Sharing: Opt-in (default: false)
- Research Participation: Opt-in (default: false)

### 5.2 Consent Tracking

```json
{
  "consentId": "string (required, unique)",
  "studentId": "string (required)",
  "purpose": "string (required)",
  "granted": "boolean",
  "grantedAt": "datetime (ISO 8601 with timezone)",
  "expiresAt": "datetime (optional, ISO 8601 with timezone)",
  "withdrawnAt": "datetime (optional, ISO 8601 with timezone)",
  "legalBasis": "enum (consent, contract, legal_obligation, legitimate_interest)",
  "metadata": {
    "ipAddress": "string (for audit)",
    "userAgent": "string (for audit)",
    "version": "string (consent form version)"
  }
}
```

## 6. Metadata Standards

### 6.1 Required Metadata

All records MUST include:
- `createdAt`: Timestamp of record creation
- `updatedAt`: Timestamp of last modification
- `version`: Semantic version of the data structure
- `standard`: WIA standard identifier (WIA-EDU-010)

### 6.2 Audit Metadata

For compliance and security:
- `createdBy`: User ID who created the record
- `updatedBy`: User ID who last modified the record
- `changeLog`: Array of all modifications (optional but recommended)

## 7. Internationalization

### 7.1 Character Encoding

- UTF-8 encoding REQUIRED for all text fields
- Support for Unicode characters in names and addresses

### 7.2 Date and Time

- All dates in ISO 8601 format
- Times include timezone information
- Support for multiple calendars (Gregorian default)

### 7.3 Localization

- Country codes: ISO 3166-1 alpha-2
- Language codes: ISO 639-1
- Currency codes: ISO 4217

## 8. Compliance Requirements

### 8.1 FERPA Compliance

- Explicit tracking of directory information opt-out
- Parent access controls for students under 18
- Third-party disclosure requires consent

### 8.2 GDPR Compliance

- Right to access: Complete data export in JSON
- Right to rectification: Change tracking and versioning
- Right to erasure: Soft delete with retention policies
- Right to data portability: WIA-EDU-010 JSON format

## 9. Implementation Guidelines

### 9.1 Database Schema

Recommended database approach:
- Relational database for structured data
- Document store for flexible/extended attributes
- Separate tables for audit logs
- Encryption at rest for all PII

### 9.2 API Endpoints

Minimum required endpoints:
- `GET /students/{studentId}`: Retrieve student profile
- `POST /students`: Create new student
- `PUT /students/{studentId}`: Update student information
- `GET /students/{studentId}/records`: Retrieve academic records
- `POST /students/{studentId}/records`: Add academic record

---

## Appendix A: JSON Schema

Full JSON schemas available at:
- https://schemas.wia.org/edu-010/v1.0/student-profile.json
- https://schemas.wia.org/edu-010/v1.0/academic-record.json
- https://schemas.wia.org/edu-010/v1.0/attendance-record.json

## Appendix B: Example Data

See `examples/` directory for complete sample datasets.

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*WIA-EDU-010 empowers students with control over their educational data while enabling institutions to collaborate more effectively.*

---

## A.1 Student-profile envelope (canonical)

The canonical student-profile envelope carries personal information, enrollment record, privacy preferences, and metadata. The personal-information block carries name, contact, address, emergency contact; the enrollment block tracks status (`active`, `leave_of_absence`, `withdrawn`, `graduated`, `suspended`, `expelled`), program, major, and academic level. Privacy preferences default to FERPA-conservative settings (directory-information opt-in, parent-access disabled at age 18, third-party-sharing opt-in only).

## A.2 Academic-record envelope

The academic-record envelope groups course enrollments, grades, and per-semester GPA into a single signed payload. Each course entry carries code, name, credits attempted vs. earned, grade scale (letter / pass-fail / numerical / percentage), grade, status (`in_progress`, `completed`, `withdrawn`, `incomplete`, `failed`), instructor, and department. The cumulative-GPA field is recomputed on every course-status change so consumers can rely on one read for the latest figure.

## A.3 Attendance envelope

Attendance records carry per-class status (`present`, `absent_excused`, `absent_unexcused`, `tardy`, `early_departure`), arrival and departure timestamps, and optional supporting documentation. The summary block aggregates the totals at envelope-write time so consumers don't need to re-aggregate at read time.

## A.4 Privacy and consent

The consent record carries the consent identifier, purpose, timestamp, expiry, and legal basis (`consent`, `contract`, `legal_obligation`, `legitimate_interest`). Withdrawn consents preserve the audit trail; downstream processors observe the withdrawal within 24 hours and stop the corresponding processing within 72 hours.

## A.5 Validation and metadata

All envelopes are validated against JSON Schema draft 2020-12. Required validations: type checking, format validation (email per RFC 5322, date per ISO 8601, phone per E.164), required-field enforcement, enumeration constraint checking, range validation, and pattern matching for identifiers. Metadata includes `createdAt`, `updatedAt`, `createdBy`, `updatedBy`, `version`, and `standard`.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/student-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-student-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/student-data-host:1.0.0` ships every student-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/student-data.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Student-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
