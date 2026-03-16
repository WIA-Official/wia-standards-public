# WIA-SEC-010: Access Control - Glossary

**Version:** 1.0
**Standard ID:** WIA-SEC-010
**Last Updated:** 2025-12-25

---

## A

### ABAC (Attribute-Based Access Control)
An access control model where authorization decisions are based on attributes of subjects, resources, actions, and environment. Unlike RBAC which uses roles, ABAC evaluates arbitrary attributes and conditions.

**Example:** "Allow access if user.department=finance AND resource.classification=confidential AND time=business_hours"

### Access Control
The process of granting or denying specific requests for obtaining and using information and related information processing services.

### Access Control List (ACL)
A list of permissions attached to an object specifying which users or system processes can access that object and what operations they can perform.

### Access Control Matrix
A table that defines access rights between subjects and objects in a security system. Rows represent subjects, columns represent objects, and cells contain allowed operations.

### Access Token
A credential that can be used by an application to access an API or resource, typically containing information about the authorization granted.

### Action
An operation that a subject wants to perform on a resource (e.g., read, write, delete, execute).

### Advice
Optional information returned with an authorization decision providing recommendations but not mandatory requirements (unlike Obligations).

### Applicability
Determines whether a policy applies to a particular authorization request based on the target specification.

### Attribute
A characteristic or property of a subject, resource, action, or environment used in access control decisions.

**Types:**
- **Subject Attributes**: User properties (role, department, clearance)
- **Resource Attributes**: Object properties (classification, owner)
- **Action Attributes**: Operation properties (method, protocol)
- **Environment Attributes**: Context (time, location, threat level)

### Attribute Provider
A system component that supplies attributes for authorization decisions (also called Policy Information Point).

### Audit Log
A chronological record of system activities that provides documentary evidence of authorization decisions and access events.

### Authorization
The process of determining whether a subject is allowed to perform a specific action on a resource.

**Note:** Different from Authentication (verifying identity) and Accounting (tracking resource usage).

---

## B

### Bell-LaPadula Model
A mandatory access control model focused on data confidentiality using security clearances.

**Key Rules:**
- **Simple Security Property**: No read up (subject cannot read above clearance)
- **Star Property**: No write down (subject cannot write below clearance)

### Break-Glass
An emergency access mechanism that allows bypassing normal access controls in critical situations, with enhanced auditing.

### Business Hours
Predefined time periods when normal business operations occur, often used in temporal access control policies.

---

## C

### Cache
Temporary storage of authorization decisions or attributes to improve performance and reduce latency.

### Capability
A token that grants specific access rights to a resource, combining both identification and authorization.

### Clearance
A subject's authorization level in a mandatory access control system, determining which classified information they can access.

### Combining Algorithm
A method for reconciling multiple policy decisions into a single final decision.

**Common Algorithms:**
- **deny-overrides**: Any DENY wins
- **permit-overrides**: Any PERMIT wins
- **first-applicable**: First matching policy wins
- **deny-unless-permit**: DENY unless explicit PERMIT

### Compartment
A non-hierarchical category in mandatory access control used to segregate information by topic or program (e.g., NUCLEAR, CRYPTO).

### Condition
A Boolean expression evaluated as part of a policy rule to determine if the rule applies.

### Confidentiality
The property that information is not made available to unauthorized individuals, entities, or processes.

### Context
The circumstances or environment in which an access request occurs (time, location, device, network, etc.).

### Credential
Evidence or documentation that proves a subject's identity or attributes (e.g., password, certificate, token).

---

## D

### DAC (Discretionary Access Control)
An access control model where resource owners can decide who has access to their resources.

### Delegation
The transfer of access rights or permissions from one subject to another, either permanently or temporarily.

### DENY
An authorization decision that explicitly refuses access to a resource.

### Distributed PDP
Multiple Policy Decision Points deployed across different locations for improved performance and availability.

---

## E

### Effect
The intended result of a policy rule, either PERMIT or DENY.

### Enforcement
The act of applying authorization decisions by allowing or blocking access to resources (performed by PEP).

### Environment
The operational context surrounding an access request (time, location, network, threat level, etc.).

### Evaluation
The process of applying policies to an authorization request to produce a decision.

### Expiration
A time limit after which a permission, token, or credential is no longer valid.

---

## F

### Federation
A trust relationship between organizations allowing users from one organization to access resources in another.

### Fine-Grained Access Control
Access control policies that specify permissions at a detailed level (e.g., column-level database access, field-level API access).

---

## G

### Grant
To provide a subject with access rights to a resource.

### Group
A collection of subjects (typically users) that can be assigned permissions collectively.

---

## H

### Hierarchical RBAC
An RBAC model where roles can inherit permissions from other roles, creating parent-child relationships.

### HIPAA (Health Insurance Portability and Accountability Act)
U.S. legislation requiring privacy and security protections for healthcare data.

---

## I

### Identity
The unique representation of a subject (user, service, device).

### INDETERMINATE
An authorization decision indicating an error occurred during evaluation and a definitive decision could not be reached.

### Inheritance
The ability of a role to acquire permissions from one or more parent roles.

### Integrity
The property that data has not been altered in an unauthorized manner.

---

## J

### JWT (JSON Web Token)
A compact, URL-safe token format used for transmitting claims between parties, commonly used for authentication and authorization.

---

## L

### Least Privilege
Security principle stating that subjects should be granted only the minimum permissions necessary to perform their tasks.

---

## M

### MAC (Mandatory Access Control)
An access control model where the system enforces access policies based on security labels, and users cannot change these policies.

### MFA (Multi-Factor Authentication)
An authentication method requiring two or more verification factors to gain access.

### Multi-Tenancy
An architecture where a single system serves multiple independent organizations (tenants) with data isolation.

---

## N

### NOT_APPLICABLE
An authorization decision indicating that no policies apply to the given request.

---

## O

### OAuth 2.0
An authorization framework that enables applications to obtain limited access to user resources without exposing credentials.

### Obligation
A requirement that must be fulfilled in conjunction with an authorization decision (e.g., logging, notification, rate limiting).

### Owner
The subject responsible for a resource, often with special rights to control access to that resource.

---

## P

### PAP (Policy Administration Point)
The component responsible for creating, managing, and storing access control policies.

### PDP (Policy Decision Point)
The component that evaluates access requests against policies and returns authorization decisions.

### PEP (Policy Enforcement Point)
The component that intercepts access requests and enforces authorization decisions from the PDP.

### Permission
The authorization to perform a specific action on a resource.

### PERMIT
An authorization decision that allows access to a resource.

### PII (Personally Identifiable Information)
Information that can be used to identify an individual (name, SSN, email, etc.).

### PIP (Policy Information Point)
The component that provides attribute information to the PDP for policy evaluation.

### Policy
A set of rules that define under what conditions access should be granted or denied.

### Policy Set
A collection of related policies that are evaluated together using a combining algorithm.

### Principal
An entity (user, service, device) that can be authenticated and authorized to access resources. Synonym for Subject.

### Privilege
A right to perform an action or access a resource.

### Privilege Escalation
An attack where a subject gains unauthorized elevated access or permissions.

---

## Q

### Quorum
The minimum number of nodes in a distributed system required to agree on a decision for consensus.

---

## R

### RBAC (Role-Based Access Control)
An access control model where permissions are assigned to roles, and users are assigned to roles.

**Key Components:**
- Users
- Roles
- Permissions
- Sessions

### Resource
An object or asset that access control policies protect (files, APIs, data, services).

### Revocation
The act of removing previously granted access rights or permissions.

### Risk Score
A quantitative assessment of the security risk associated with granting an access request.

### Role
A named collection of permissions representing a job function or responsibility.

### Role Hierarchy
A structure where roles inherit permissions from other roles, creating parent-child relationships.

---

## S

### SAML (Security Assertion Markup Language)
An XML-based standard for exchanging authentication and authorization data between parties.

### Scope
The extent of access granted by a permission or token (e.g., read-only, full-access, specific resources).

### Separation of Duties (SoD)
A security principle requiring that critical functions be divided among multiple people to prevent fraud or error.

### Session
A period of time during which a subject's authentication and authorization remain valid.

### Subject
An entity (user, service, device) requesting access to a resource.

### Symmetric Encryption
Encryption where the same key is used for both encryption and decryption.

---

## T

### Target
The specification in a policy that defines which resources, subjects, or actions the policy applies to.

### Temporal Constraint
A restriction based on time (e.g., access only during business hours, token valid for 1 hour).

### Tenant
An independent organization or customer in a multi-tenant system.

### Threat Level
An assessment of current security risk in the environment (low, medium, high, critical).

### TLS (Transport Layer Security)
A cryptographic protocol providing secure communication over a network.

### Token
A piece of data that represents authorization or authentication (e.g., access token, session token).

### Trust Level
A measure of confidence in a subject, device, or network connection.

---

## U

### User
A human subject that interacts with a system.

### User Attribute
A property or characteristic of a user (department, job title, clearance level).

---

## V

### Verifiable Credential (VC)
A tamper-evident credential with cryptographic proof of authorship, following W3C standards.

### Version Vector
A data structure used in distributed systems to track the version or state of data across multiple nodes.

---

## W

### Watermarking
The process of embedding identifying information into data to track its distribution and use.

### Whitelist
A list of explicitly permitted subjects, resources, or actions (opposite of blacklist).

### WIA (World Certification Industry Association)
The organization that publishes and maintains WIA standards including WIA-SEC-010.

---

## X

### XACML (eXtensible Access Control Markup Language)
An OASIS standard for expressing access control policies in XML format, which influenced WIA-SEC-010.

---

## Z

### Zero Trust
A security model that assumes no implicit trust and requires verification for every access request regardless of location.

---

## Acronyms

| Acronym | Full Form |
|---------|-----------|
| ABAC | Attribute-Based Access Control |
| ACL | Access Control List |
| API | Application Programming Interface |
| DAC | Discretionary Access Control |
| GDPR | General Data Protection Regulation |
| HIPAA | Health Insurance Portability and Accountability Act |
| JWT | JSON Web Token |
| MAC | Mandatory Access Control |
| MFA | Multi-Factor Authentication |
| OAuth | Open Authorization |
| PAP | Policy Administration Point |
| PDP | Policy Decision Point |
| PEP | Policy Enforcement Point |
| PII | Personally Identifiable Information |
| PIP | Policy Information Point |
| RBAC | Role-Based Access Control |
| SAML | Security Assertion Markup Language |
| SoD | Separation of Duties |
| TLS | Transport Layer Security |
| VC | Verifiable Credential |
| VPN | Virtual Private Network |
| XACML | eXtensible Access Control Markup Language |

---

## Related Standards

### WIA Standards
- **WIA-SEC-001**: Encryption Standards
- **WIA-SEC-002**: Authentication Framework
- **WIA-SEC-003**: Digital Signatures
- **WIA-SEC-010**: Access Control (this standard)

### External Standards
- **NIST RBAC**: NIST Standard for Role-Based Access Control
- **XACML 3.0**: OASIS eXtensible Access Control Markup Language
- **OAuth 2.0**: RFC 6749 - The OAuth 2.0 Authorization Framework
- **SAML 2.0**: Security Assertion Markup Language 2.0
- **ISO/IEC 27001**: Information Security Management
- **SOC 2**: Service Organization Control 2

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-25 | Initial glossary release |

---

弘益人間 (홍익인간) · Benefit All Humanity

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
