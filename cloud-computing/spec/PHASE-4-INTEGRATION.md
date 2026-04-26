# WIA-COMM-012 PHASE 4 — Integration Specification

**Standard:** WIA-COMM-012
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 13. Cost Optimization (FinOps)

### 13.1 Resource Tagging

```json
{
  "tagStrategy": {
    "required": [
      {
        "key": "Environment",
        "values": ["production", "staging", "development"]
      },
      {
        "key": "CostCenter",
        "pattern": "^[A-Z]{2}-[0-9]{4}$"
      },
      {
        "key": "Owner",
        "type": "email"
      },
      {
        "key": "Application",
        "description": "Application name"
      }
    ],
    "optional": [
      {
        "key": "Project",
        "description": "Project identifier"
      }
    ]
  }
}
```

### 13.2 Cost Allocation

```json
{
  "costAllocation": {
    "groupBy": ["Environment", "CostCenter", "Application"],
    "period": "monthly",
    "currency": "USD",
    "budgets": [
      {
        "name": "production-monthly",
        "amount": 50000,
        "filters": {
          "tags": {
            "Environment": "production"
          }
        },
        "alerts": [
          {
            "threshold": 80,
            "type": "percentage",
            "recipients": ["finance@example.com"]
          },
          {
            "threshold": 100,
            "type": "percentage",
            "recipients": ["cto@example.com"]
          }
        ]
      }
    ]
  }
}
```

---

## 14. Disaster Recovery

### 14.1 Backup Strategy

```json
{
  "backup": {
    "resources": ["ebs-volumes", "rds-databases", "dynamodb-tables"],
    "schedule": {
      "frequency": "daily",
      "time": "02:00 UTC",
      "retention": {
        "daily": 7,
        "weekly": 4,
        "monthly": 12,
        "yearly": 7
      }
    },
    "crossRegion": {
      "enabled": true,
      "destinations": ["us-west-2", "eu-west-1"]
    },
    "encryption": true
  }
}
```

### 14.2 Disaster Recovery Tiers

| Tier | RTO | RPO | Strategy | Cost |
|------|-----|-----|----------|------|
| Tier 1 (Critical) | < 1 hour | < 15 min | Active-Active | Highest |
| Tier 2 (Important) | < 4 hours | < 1 hour | Warm Standby | High |
| Tier 3 (Normal) | < 24 hours | < 4 hours | Pilot Light | Medium |
| Tier 4 (Low Priority) | < 72 hours | < 24 hours | Backup & Restore | Lowest |

---

## 15. Compliance and Governance

### 15.1 Compliance Frameworks

**Supported Standards:**
- SOC 2 Type II
- ISO 27001, ISO 27017, ISO 27018
- HIPAA
- PCI DSS
- GDPR
- FedRAMP (US Government)

### 15.2 Audit Logging

```json
{
  "auditLogging": {
    "enabled": true,
    "services": ["iam", "s3", "ec2", "rds", "lambda"],
    "events": [
      "CreateUser",
      "DeleteUser",
      "PutBucketPolicy",
      "CreateInstance",
      "TerminateInstance"
    ],
    "destination": {
      "type": "CloudWatch",
      "logGroup": "/aws/cloudtrail",
      "encryption": true
    },
    "retention": 2555  // 7 years in days
  }
}
```

---

## 16. Monitoring and Observability

### 16.1 Metrics

```json
{
  "metrics": {
    "infrastructure": [
      "CPUUtilization",
      "MemoryUtilization",
      "DiskUtilization",
      "NetworkIn",
      "NetworkOut"
    ],
    "application": [
      "RequestCount",
      "ResponseTime",
      "ErrorRate",
      "ActiveConnections"
    ],
    "business": [
      "TransactionVolume",
      "RevenuePerMinute",
      "ConversionRate"
    ],
    "collection": {
      "interval": 60,
      "retention": 15  // days
    }
  }
}
```

### 16.2 Distributed Tracing

```yaml
tracing:
  enabled: true
  sampler:
    type: probabilistic
    rate: 0.1  # 10% of requests
  exporters:
    - type: jaeger
      endpoint: http://jaeger-collector:14268/api/traces
    - type: otlp
      endpoint: https://otel-collector:4318
  propagation:
    - tracecontext
    - baggage
```

---

## 17. Multi-Cloud and Hybrid Cloud

### 17.1 Multi-Cloud Strategy

```json
{
  "multiCloud": {
    "providers": [
      {
        "name": "aws",
        "region": "us-east-1",
        "role": "primary",
        "services": ["compute", "storage", "database"]
      },
      {
        "name": "azure",
        "region": "eastus",
        "role": "secondary",
        "services": ["compute", "storage"]
      },
      {
        "name": "gcp",
        "region": "us-central1",
        "role": "specialized",
        "services": ["ml", "analytics"]
      }
    ],
    "workloadPlacement": {
      "strategy": "best-fit",
      "criteria": ["cost", "performance", "compliance"]
    }
  }
}
```

---

## 18. Data Formats

### 18.1 Cloud Resource Definition

```json
{
  "$schema": "https://wiastandards.com/schemas/cloud-resource-v1.json",
  "resourceType": "VirtualMachine",
  "metadata": {
    "name": "web-server-01",
    "labels": {
      "environment": "production",
      "application": "web-api"
    },
    "annotations": {
      "description": "Production web server",
      "owner": "platform-team@example.com"
    }
  },
  "spec": {
    "provider": "aws",
    "region": "us-east-1",
    "instanceType": "t3.medium",
    "image": "ami-0c55b159cbfafe1f0",
    "network": {
      "vpc": "vpc-12345678",
      "subnet": "subnet-abc",
      "securityGroups": ["sg-web"]
    },
    "storage": [
      {
        "device": "/dev/sda1",
        "size": 30,
        "type": "gp3"
      }
    ]
  }
}
```

---

## 19. API Specifications

### 19.1 Cloud Management API

**Base URL:** `https://api.cloudprovider.com/v1`

**Authentication:** Bearer token (OAuth 2.0)

#### Endpoints

**Create Instance:**
```
POST /compute/instances
Content-Type: application/json
Authorization: Bearer {token}

{
  "name": "web-server-01",
  "instanceType": "t3.medium",
  "image": "ami-...",
  "network": {...},
  "tags": {...}
}

Response:
{
  "instanceId": "i-1234567890abcdef0",
  "state": "pending",
  "launchTime": "2025-12-26T10:00:00Z"
}
```

---

## 20. Implementation Guidelines

### 20.1 Infrastructure as Code

**Terraform Example:**
```hcl
resource "aws_instance" "web" {
  ami           = "ami-0c55b159cbfafe1f0"
  instance_type = "t3.medium"

  vpc_security_group_ids = [aws_security_group.web.id]
  subnet_id              = aws_subnet.public.id

  tags = {
    Name        = "web-server-01"
    Environment = "production"
  }
}
```

### 20.2 Well-Architected Framework

**Five Pillars:**
1. **Operational Excellence**: Automation, monitoring, continuous improvement
2. **Security**: Defense in depth, least privilege, encryption
3. **Reliability**: High availability, fault tolerance, disaster recovery
4. **Performance Efficiency**: Right-sizing, caching, CDN
5. **Cost Optimization**: Reserved instances, auto-scaling, monitoring

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
