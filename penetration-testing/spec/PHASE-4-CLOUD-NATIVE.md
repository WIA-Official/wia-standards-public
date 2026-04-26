# WIA-SEC-019: Penetration Testing — Phase 4 Specification

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-12-25
**Standard ID:** WIA-SEC-019
**Category:** Security (SEC)
**Color:** #8B5CF6

---

## Phase 4: Cloud-Native and Container Security Testing

### 4.1 Overview

Phase 4 addresses modern cloud architectures, containerized applications, and serverless computing. Coverage spans IAM, network controls, control plane configurations, and supply-chain security for OCI images.

### 4.2 Cloud Infrastructure Penetration Testing

```json
{
  "cloudPentest": {
    "providers": ["AWS", "Azure", "GCP", "Alibaba Cloud"],
    "scope": {
      "iam": "Identity and Access Management",
      "storage": "S3/Blob/Cloud Storage security",
      "compute": "EC2/VM/Compute Engine misconfigurations",
      "networking": "VPC/VNet security groups and firewall rules",
      "serverless": "Lambda/Functions security",
      "kubernetes": "Managed K8s cluster security"
    },
    "awsSpecific": {
      "tests": [
        "S3 bucket enumeration and public access",
        "IAM privilege escalation paths",
        "EC2 instance metadata service (IMDS) exploitation",
        "Security group misconfigurations",
        "Lambda function injection",
        "RDS snapshot access",
        "CloudTrail logging gaps"
      ]
    }
  }
}
```

**AWS Read-Only Probe Examples**:

```bash
# Enumerate S3 buckets (read-only)
aws s3 ls

# Inspect security groups for overly permissive rules
aws ec2 describe-security-groups --group-ids sg-xxxxx

# Verify IMDSv2 enforcement (NOT exploiting credentials in production)
aws ec2 describe-instances --query 'Reservations[].Instances[].MetadataOptions'
```

All probes that attempt actions requiring write access MUST be authorised in the engagement RoE and executed against test accounts only.

### 4.3 Container Security Testing

```json
{
  "containerSecurity": {
    "imageSecurity": {
      "tests": [
        "Vulnerability scanning (CVE detection)",
        "Secrets in image layers",
        "Privileged container detection",
        "Root user execution",
        "Excessive capabilities"
      ],
      "tools": ["Trivy", "Clair", "Anchore", "Docker Bench"]
    },
    "runtimeSecurity": {
      "tests": [
        "Container escape attempts",
        "Privileged escalation",
        "Host filesystem access",
        "Docker socket exposure",
        "Resource exhaustion"
      ]
    },
    "orchestration": {
      "kubernetes": {
        "tests": [
          "RBAC misconfigurations",
          "Pod security policies",
          "Network policy violations",
          "Secrets management",
          "API server authentication",
          "Kubelet security"
        ]
      }
    }
  }
}
```

**Kubernetes Read-Only Reconnaissance**:

```bash
# Enumerate Kubernetes resources (with proper auth)
kubectl get pods --all-namespaces
kubectl get serviceaccounts --all-namespaces

# Verify RBAC for the default service account
kubectl auth can-i --list --as=system:serviceaccount:default:default

# Check for privileged pods
kubectl get pods --all-namespaces \
  -o jsonpath='{range .items[?(@.spec.securityContext.privileged==true)]}{.metadata.name}{"\n"}{end}'
```

### 4.4 Container Escape Catalog

```json
{
  "containerEscape": {
    "techniques": [
      {"name": "Privileged Container Escape", "mitigation": "Avoid privileged containers, enforce baseline PSS"},
      {"name": "Docker Socket Exposure",       "mitigation": "Never mount docker.sock into containers"},
      {"name": "Kernel Exploit",               "mitigation": "Patch kernel, use container-optimized OS"},
      {"name": "Capabilities Abuse",           "mitigation": "Drop ALL capabilities, add only what is required"}
    ]
  }
}
```

### 4.5 Serverless Security Testing

```json
{
  "serverlessSecurity": {
    "lambda": {
      "tests": [
        "Function enumeration",
        "Environment variable extraction (secrets)",
        "IAM role privilege escalation",
        "Injection attacks (event data manipulation)",
        "Denial of wallet (resource exhaustion)",
        "Layer security (malicious dependencies)"
      ],
      "guardrails": [
        "Use AWS Secrets Manager or Parameter Store, never env-var secrets",
        "Apply per-function IAM with least privilege",
        "Set concurrent execution limits and budget alarms"
      ]
    }
  }
}
```

### 4.6 API Security Testing (OWASP API Top-10 mapping)

```json
{
  "apiSecurity": {
    "owaspAPITop10": [
      {"id": "API1:2023", "name": "Broken Object Level Authorization"},
      {"id": "API2:2023", "name": "Broken Authentication"},
      {"id": "API3:2023", "name": "Broken Object Property Level Authorization"},
      {"id": "API4:2023", "name": "Unrestricted Resource Consumption"},
      {"id": "API5:2023", "name": "Broken Function Level Authorization"},
      {"id": "API6:2023", "name": "Unrestricted Access to Sensitive Business Flows"},
      {"id": "API7:2023", "name": "Server Side Request Forgery"},
      {"id": "API8:2023", "name": "Security Misconfiguration"},
      {"id": "API9:2023", "name": "Improper Inventory Management"},
      {"id": "API10:2023","name": "Unsafe Consumption of APIs"}
    ]
  }
}
```

### 4.7 Infrastructure as Code (IaC) Security

```json
{
  "iacSecurity": {
    "tools": ["Checkov", "tfsec", "Terraform Sentinel"],
    "rules": [
      "No public S3 buckets",
      "Encrypt all data at rest",
      "Encrypt all data in transit",
      "No hardcoded credentials",
      "Minimum privilege IAM policies",
      "Enable logging and monitoring",
      "Restrict security group ingress",
      "Use private subnets for resources"
    ]
  }
}
```

### 4.8 Supply Chain Security

WIA-SEC-019 Phase 4 MUST verify the provenance of every container image and binary touched by the engagement scope:

- **SBOM** — generated as CycloneDX 1.5 or SPDX 2.3 for every artefact.
- **Signatures** — Cosign / Sigstore signatures over the image digest, verified at admission control.
- **SLSA Provenance** — Level ≥ 2 attestations stored alongside the artefact in OCI registry.
- **Image Pinning** — admission webhooks MUST refuse `:latest` tags; only digests are accepted in production.

### 4.9 Cloud Control Plane Hardening Checklist

| Control | Why | Verification |
|---------|-----|--------------|
| Multi-account / multi-project boundary | Blast-radius isolation | List of accounts with explicit owner |
| Centralised audit logging | Detection | CloudTrail / Cloud Audit Logs in dedicated account |
| Org-wide service control policies | Prevention | SCP / VPC-SC policies attached |
| Continuous configuration compliance | Detection | Config rules / Policy Insights enabled |
| Identity federation w/ MFA | Prevention | No long-lived access keys for humans |
| KMS key separation per environment | Containment | Distinct CMKs per env |
| Egress-controlled VPCs | Prevention | NAT Gateway + allow-list domains |

### 4.10 Reporting & Remediation

The Phase 4 deliverable MUST include the same Executive / Technical / Detection-gap structure as Phase 2 (§2.8) plus:

- **Cloud Posture Score** — quantitative readiness across IAM, Network, Storage, Compute, Logging.
- **K8s Cluster Health** — admission policies, runtime sensors (Falco), CIS Benchmark gap.
- **Cost-to-Remediate** — engineering days estimate per finding.
- **Compensating Controls** — what offsets a finding when fix is delayed.

### 4.11 Tenant Isolation Verification

For shared platforms (multi-tenant SaaS, K8s cluster federations) the test plan MUST include cross-tenant probes:

- Attempt to read another tenant's logs via shared sidecars
- Verify network policies block tenant-to-tenant DNS resolution
- Confirm KMS data keys are unique per tenant and inaccessible cross-tenant
- Check observability pipelines tag every event with the originating tenant
- Run a "noisy neighbour" test that saturates CPU/IO and confirms the affected tenant's SLA holds

Each probe MUST produce a pass/fail line in the report; one failure is enough to delay a tenancy claim of strong isolation.

### 4.11.1 Cloud Identity Federation Tests

Identity federation (SAML, OIDC, AWS IAM Identity Center, Azure AD, Workforce Identity Federation) is a frequent target. The standard MUST exercise:

- SAML response signature validation
- Audience and recipient binding
- Replay protection within the configured window
- Key rotation behaviour during a federation handshake
- Token-mismatch failure modes (audience mismatch, expired tokens, mismatched issuer)

### 4.12 Normative References

- ISO/IEC 27001:2022 — ISMS controls A.8 family
- ISO/IEC 27017:2015 — Cloud-specific information security controls
- ISO/IEC 27018:2019 — Protection of PII in public clouds
- NIST SP 800-190 — Application Container Security Guide
- NIST SP 800-204C — Implementation of DevSecOps for Microservices-based Apps
- CIS Kubernetes Benchmark v1.8
- Cloud Native Computing Foundation TAG-Security white paper
- SLSA Framework v1.0
- CycloneDX 1.5 — SBOM specification

---

## Phase Summary

### Phase 2: Advanced Red Team Operations
- Sophisticated adversary simulation
- Custom malware and TTPs
- Social engineering campaigns
- Purple team collaboration

### Phase 3: Automated Continuous Testing
- CI/CD integration
- Breach and attack simulation
- ML-assisted triage (advisory only)
- Continuous security validation

### Phase 4: Cloud-Native Security
- Cloud infrastructure testing (AWS, Azure, GCP)
- Container and Kubernetes security
- Serverless security assessment
- API and IaC security
- Supply-chain provenance

---

**弘益人間 (홍익인간) · Benefit All Humanity**

By implementing comprehensive security testing across all phases, organizations can proactively identify and remediate vulnerabilities, building robust defenses against modern cyber threats.

---

**Document Control**:
- Version: 1.0
- Last Updated: 2025-12-25
- Next Review: 2026-06-25
- Maintained by: WIA Security Standards Committee

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
