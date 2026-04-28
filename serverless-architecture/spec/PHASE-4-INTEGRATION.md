# WIA-COMP-008 — Phase 4: Integration

> Serverless canonical Phase 4: ecosystem integration (CloudEvents + Knative + OpenTelemetry + W3C Trace Context + edge + FinOps).

# WIA-COMP-008: Serverless Architecture Specification v1.0

> **Standard ID:** WIA-COMP-008  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active

---



---

## A.1 Standards cross-walk

| Concern                              | Standard                                     |
|--------------------------------------|----------------------------------------------|
| HTTP semantics                       | RFC 9110 + 9111 + 9112 + 9113 + 9114         |
| HTTP/3 over QUIC                     | RFC 9000 + 9001 + 9002 + 9114                 |
| TLS 1.3                              | RFC 8446                                      |
| OAuth 2.1 + OIDC                     | RFC 6749 + 8252 + 9700 + OpenID Connect Core  |
| OAuth 2.0 mTLS                       | RFC 8705                                      |
| JWT                                  | RFC 7519 + 7515 + 7516 + 7517 + 7518         |
| CloudEvents                          | CNCF CloudEvents 1.0.2                        |
| AsyncAPI                             | AsyncAPI 3.0 Specification                    |
| OpenAPI                              | OpenAPI 3.1.0 Specification                   |
| Knative Serving + Eventing           | CNCF Knative                                  |
| OCI runtime + image                  | OCI Runtime Spec + OCI Image Spec             |
| OpenTelemetry                        | OpenTelemetry Specification                   |
| W3C Trace Context                    | W3C Trace Context Level 2                     |
| Container security baseline          | NIST SP 800-190 + CIS Docker Benchmark        |
| Cloud-native security                | CNCF Cloud Native Security Whitepaper         |
| Per-platform reference               | AWS Lambda Developer Guide + Azure Functions  |
|                                      | + GCP Cloud Functions + Cloud Run + Knative   |
|                                      | + OpenFaaS + Apache OpenWhisk                 |

## A.2 Cloud-native ecosystem integration envelope

Cloud-native ecosystem integration covers: Kubernetes integration
per CNCF Knative for serverless on Kubernetes; Knative Serving for
HTTP-triggered scaled-to-zero workloads with the per-revision
traffic-splitting envelope; Knative Eventing for event-driven
workloads with the per-broker subscription envelope; KEDA per
CNCF KEDA for event-driven autoscaling on Kubernetes (per KEDA
ScaledObject + ScaledJob spec); per-platform ingress envelope
(Nginx Ingress + Istio + Envoy + Traefik); per-platform service-
mesh envelope per Istio + Linkerd + Cilium for inter-function
mTLS + per-call observability + per-call policy enforcement.

## A.3 Edge-and-distributed-execution integration envelope

Edge-and-distributed-execution integration covers: edge-FaaS
envelope (Cloudflare Workers per Cloudflare doc; AWS Lambda@Edge
per AWS CloudFront integration; Akamai EdgeWorkers per Akamai;
Fastly Compute@Edge per Fastly per WebAssembly + WASI; Vercel
Edge Functions per Vercel; Netlify Edge Functions per Netlify);
per-edge V8 isolate envelope per Cloudflare Workers V8 + Deno Deploy;
per-edge WebAssembly envelope per WASI 0.2 + WASI Preview 2 +
Component Model per Bytecode Alliance + W3C WebAssembly CG; per-
edge regional execution envelope (per-region cold-pool + per-
region warm-pool per the per-platform region map); per-edge
geo-affinity envelope per the operator's per-tenant data-residency
envelope.

## A.4 Observability-and-cost integration envelope

Observability-and-cost integration covers: per-platform billing
granularity envelope (AWS Lambda 1-ms billing per AWS pricing;
Azure Functions Consumption-plan + Premium-plan per Microsoft
pricing; GCP Cloud Functions per-100ms + 2nd gen per-1ms; per-
platform-equivalent); per-function cost-attribution envelope per
the operator's chargeback policy per FinOps Foundation Framework;
per-function carbon-attribution envelope per the operator's GHG
Protocol Scope 3 inventory + per-cloud carbon-data envelope (AWS
Customer Carbon Footprint Tool + Azure Sustainability + GCP Carbon
Footprint); per-function security-posture envelope (CSPM per
Wiz / Lacework / Prisma Cloud / native cloud security envelope);
per-function compliance envelope (SOC 2 + ISO 27001 + PCI-DSS +
HIPAA + GDPR per the operator's per-jurisdiction policy).

## A.5 Migration-and-portability integration envelope

Migration-and-portability integration covers: cross-platform function-
runtime portability envelope (handler-signature compatibility across
AWS Lambda + Azure Functions + GCP Cloud Functions per the
respective vendor SDK + per-vendor adapter); per-handler abstraction
layer per Serverless Framework + AWS SAM + Azure Functions Core
Tools + GCP Functions Framework + Knative Functions per CNCF
Knative; per-function infrastructure-as-code envelope (Terraform
per HashiCorp + AWS CDK + Pulumi + Azure Bicep + GCP Cloud
Deployment Manager); per-function CI/CD envelope (GitHub Actions
+ GitLab CI + Azure DevOps + AWS CodePipeline + Argo Workflows /
Argo CD per CNCF Argo); per-function immutable-deployment envelope
per the operator's per-environment promotion policy.

## A.6 References

- RFC 9110 / 9111 / 9112 / 9113 / 9114: HTTP semantics + HTTP/3
- RFC 9000 / 9001 / 9002: QUIC transport + TLS over QUIC
- RFC 8446: TLS 1.3
- RFC 6749 / 8252 / 9700: OAuth 2.0 + 2.1
- RFC 7519: JSON Web Token (JWT)
- RFC 8705: OAuth 2.0 Mutual-TLS Client Authentication
- CNCF CloudEvents 1.0.2: CloudEvents Specification
- AsyncAPI 3.0: Asynchronous API Specification
- OpenAPI 3.1.0: OpenAPI Specification
- CNCF Knative: Knative Serving + Eventing
- OCI Runtime Spec + OCI Image Spec: Open Container Initiative
- WASI Preview 2 + WebAssembly Component Model: Bytecode Alliance + W3C WebAssembly CG
- OpenTelemetry Specification: CNCF OpenTelemetry
- W3C Trace Context: Distributed Tracing Specification
- NIST SP 800-190: Application Container Security Guide
- NIST SP 800-53: Security and Privacy Controls
- CIS Docker Benchmark + CIS Kubernetes Benchmark
- AWS Lambda Developer Guide: Function model + execution
- Azure Functions Documentation: Hosting models + bindings
- GCP Cloud Functions + Cloud Run Documentation
- Cloudflare Workers + Akamai EdgeWorkers + Fastly Compute@Edge
- Apache OpenWhisk + OpenFaaS + Fission + Kubeless
- FinOps Foundation: FinOps Framework
- GHG Protocol: Corporate Standard + Scope 3 Standard


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/serverless-architecture/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-serverless-architecture-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/serverless-architecture-host:1.0.0` ships every serverless-architecture envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/serverless-architecture.sh` ships sample envelope generators with no
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
ecosystem. Serverless-architecture deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-serverless-architecture-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

## Z.9 Disaster recovery and continuity-of-operations envelope

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-
exit envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity. The DR
envelope composes with WIA Secure Enclave for sealed-backup
envelopes and with WIA-AIR-SHIELD for runtime trust-list re-
hydration on the failover instance.

## Z.10 Supply-chain and software-bill-of-materials envelope

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per the operator's chosen specification: SPDX 2.3 / 3.0 per
ISO/IEC 5962 + Linux Foundation SPDX, or CycloneDX 1.6 per OWASP
Foundation. The SBOM enumerates every direct + transitive dependency
with the per-component name + version + licence + supplier + per-
component hash + per-component PURL (Package URL per package-url
spec) + per-component CPE (Common Platform Enumeration per NIST).
The host MUST publish per-release SBOM updates and MUST flag
breaking dependency-version migrations so downstream consumers
can plan ahead. Supply-chain attestation follows in-toto per
CNCF in-toto + SLSA (Supply-chain Levels for Software Artifacts)
per OpenSSF SLSA Framework — typically targeting SLSA Level 3 for
hosted production deployments.

弘益人間 — Benefit All Humanity.
