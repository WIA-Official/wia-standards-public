# WIA-SEC-005: Zero Trust Architecture
## Phase 4 - Integration Specifications

**Standard ID:** WIA-SEC-005
**Category:** Security (SEC)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 1. Overview

This document defines integration patterns, APIs, and connectors for integrating Zero Trust Architecture with enterprise security systems including IAM, SIEM, EDR/XDR, firewalls, API gateways, and service mesh platforms.

---

## 2. Identity & Access Management (IAM) Integration

### 2.1 Supported IAM Providers

- **Okta**
- **Microsoft Azure AD / Entra ID**
- **Google Cloud Identity**
- **Auth0**
- **Keycloak**
- **Ping Identity**
- **ForgeRock**

### 2.2 IAM Integration Architecture

```
┌─────────────────────────────────────────────────────┐
│           Zero Trust Policy Engine                  │
│                                                     │
│   ┌──────────────────────────────────────────┐    │
│   │       IAM Integration Layer              │    │
│   │  ┌──────────┐  ┌──────────┐  ┌────────┐ │    │
│   │  │  SAML    │  │  OIDC    │  │ SCIM   │ │    │
│   │  │ Adapter  │  │ Adapter  │  │Adapter │ │    │
│   │  └──────────┘  └──────────┘  └────────┘ │    │
│   └──────────────────────────────────────────┘    │
└─────────────────┬───────────────────────────────────┘
                  │
        ┌─────────┼─────────┐
        │         │         │
┌───────▼───┐ ┌──▼─────┐ ┌─▼────────┐
│   Okta    │ │Azure AD│ │  Auth0   │
└───────────┘ └────────┘ └──────────┘
```

### 2.3 SAML Integration

#### SAML Request
```xml
<samlp:AuthnRequest
    xmlns:samlp="urn:oasis:names:tc:SAML:2.0:protocol"
    ID="_abc123"
    Version="2.0"
    IssueInstant="2025-12-25T10:30:00Z"
    Destination="https://idp.example.com/saml/sso"
    AssertionConsumerServiceURL="https://zt-engine.example.com/saml/acs">
    <saml:Issuer>https://zt-engine.example.com</saml:Issuer>
    <samlp:NameIDPolicy Format="urn:oasis:names:tc:SAML:1.1:nameid-format:emailAddress"/>
    <samlp:RequestedAuthnContext Comparison="exact">
        <saml:AuthnContextClassRef>
            urn:oasis:names:tc:SAML:2.0:ac:classes:PasswordProtectedTransport
        </saml:AuthnContextClassRef>
    </samlp:RequestedAuthnContext>
</samlp:AuthnRequest>
```

#### SAML Assertion Processing
```typescript
interface SAMLAssertion {
    nameID: string;
    attributes: {
        email: string;
        firstName: string;
        lastName: string;
        groups: string[];
        roles: string[];
        department: string;
        employeeId: string;
    };
    authenticationMethod: string;
    authenticationInstant: string;
    sessionNotOnOrAfter: string;
}

async function processSAMLAssertion(assertion: SAMLAssertion): Promise<TrustScore> {
    const identityScore = calculateIdentityScore({
        authMethod: assertion.authenticationMethod,
        attributes: assertion.attributes
    });

    return {
        identityScore,
        userId: assertion.nameID,
        groups: assertion.attributes.groups,
        roles: assertion.attributes.roles
    };
}
```

### 2.4 OpenID Connect (OIDC) Integration

#### Authorization Code Flow
```typescript
// 1. Authorization Request
const authUrl = new URL('https://idp.example.com/oauth/authorize');
authUrl.searchParams.append('response_type', 'code');
authUrl.searchParams.append('client_id', 'zt-engine-client');
authUrl.searchParams.append('redirect_uri', 'https://zt-engine.example.com/callback');
authUrl.searchParams.append('scope', 'openid profile email groups');
authUrl.searchParams.append('state', generateState());
authUrl.searchParams.append('nonce', generateNonce());
authUrl.searchParams.append('acr_values', 'urn:mace:incommon:iap:silver');

// 2. Token Exchange
const tokenResponse = await fetch('https://idp.example.com/oauth/token', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({
        grant_type: 'authorization_code',
        code: authorizationCode,
        redirect_uri: 'https://zt-engine.example.com/callback',
        client_id: 'zt-engine-client',
        client_secret: process.env.CLIENT_SECRET
    })
});

// 3. Validate ID Token
const idToken = await validateJWT(tokenResponse.id_token);
const userInfo = await fetchUserInfo(tokenResponse.access_token);

// 4. Calculate Trust Score
const trustScore = await calculateTrustScore({
    identity: idToken,
    userAttributes: userInfo,
    authMethod: idToken.amr
});
```

### 2.5 SCIM Provisioning

#### User Provisioning
```json
POST /scim/v2/Users
Content-Type: application/scim+json

{
  "schemas": ["urn:ietf:params:scim:schemas:core:2.0:User"],
  "userName": "john.doe@example.com",
  "name": {
    "givenName": "John",
    "familyName": "Doe"
  },
  "emails": [{
    "value": "john.doe@example.com",
    "primary": true
  }],
  "groups": [{
    "value": "finance-team",
    "display": "Finance Team"
  }],
  "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User": {
    "employeeNumber": "12345",
    "department": "Finance",
    "manager": {
      "value": "manager-123"
    }
  }
}
```

#### Group Sync
```typescript
async function syncGroupsFromIAM() {
    const groups = await iamClient.getGroups();

    for (const group of groups) {
        await policyEngine.updateGroup({
            groupId: group.id,
            groupName: group.displayName,
            members: group.members,
            policies: mapGroupToPolicies(group)
        });
    }
}
```

---

## 3. SIEM Integration

### 3.1 Supported SIEM Platforms

- **Splunk**
- **IBM QRadar**
- **Elastic Security (ELK Stack)**
- **Microsoft Sentinel**
- **LogRhythm**
- **Sumo Logic**
- **Chronicle (Google)**

### 3.2 Syslog Integration

#### RFC 5424 Format
```
<134>1 2025-12-25T10:30:00.123Z zt-engine-01.example.com zero-trust-engine 1234 access-decision [wia@32473 trustScore="91.5" decision="allow" userId="user-12345"] User john.doe@example.com granted access to res-finance-001
```

#### CEF (Common Event Format)
```
CEF:0|WIA|ZeroTrust|2.5.0|ACCESS_DECISION|Access Decision Made|5|
rt=Dec 25 2025 10:30:00
src=192.168.1.100
suser=john.doe@example.com
dst=res-finance-001
act=allow
cs1Label=trustScore
cs1=91.5
cs2Label=deviceId
cs2=device-67890
cs3Label=decision
cs3=allow
```

### 3.3 Splunk HTTP Event Collector

```typescript
async function sendToSplunk(event: ZeroTrustEvent) {
    const splunkEvent = {
        time: new Date(event.timestamp).getTime() / 1000,
        host: 'zt-engine-01',
        source: 'zero-trust-engine',
        sourcetype: 'wia:zero-trust:access',
        event: {
            eventType: event.eventType,
            userId: event.subject.userId,
            deviceId: event.subject.deviceId,
            resourceId: event.resource.resourceId,
            decision: event.details.decision,
            trustScore: event.details.trustScore,
            riskFactors: event.details.riskFactors
        }
    };

    await fetch('https://splunk.example.com:8088/services/collector/event', {
        method: 'POST',
        headers: {
            'Authorization': `Splunk ${SPLUNK_HEC_TOKEN}`,
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(splunkEvent)
    });
}
```

### 3.4 Elastic Common Schema (ECS)

```json
{
  "@timestamp": "2025-12-25T10:30:00.123Z",
  "event": {
    "kind": "event",
    "category": ["authentication", "iam"],
    "type": ["access", "allowed"],
    "outcome": "success",
    "action": "access-decision",
    "dataset": "wia.zero-trust.access"
  },
  "user": {
    "id": "user-12345",
    "name": "john.doe@example.com",
    "email": "john.doe@example.com",
    "roles": ["analyst"]
  },
  "source": {
    "ip": "192.168.1.100",
    "geo": {
      "country_name": "United States",
      "city_name": "San Francisco"
    }
  },
  "wia": {
    "zero_trust": {
      "trust_score": 91.5,
      "decision": "allow",
      "device_id": "device-67890",
      "resource_id": "res-finance-001",
      "policy_id": "pol-12345"
    }
  }
}
```

### 3.5 Real-Time Alerting

```typescript
class SIEMAlertManager {
    async createAlert(event: ZeroTrustEvent) {
        if (event.details.trustScore < CRITICAL_THRESHOLD) {
            await this.sendAlert({
                severity: 'critical',
                title: 'Critical: Low Trust Score Access Attempt',
                description: `User ${event.subject.userId} attempted access with trust score ${event.details.trustScore}`,
                indicators: {
                    userId: event.subject.userId,
                    deviceId: event.subject.deviceId,
                    sourceIP: event.subject.ipAddress,
                    trustScore: event.details.trustScore
                },
                recommendedActions: [
                    'Review user activity logs',
                    'Verify device compliance',
                    'Check for account compromise indicators'
                ]
            });
        }
    }
}
```

---

## 4. EDR/XDR Integration

### 4.1 Supported EDR/XDR Platforms

- **CrowdStrike Falcon**
- **Microsoft Defender for Endpoint**
- **SentinelOne**
- **Carbon Black**
- **Palo Alto Cortex XDR**
- **Cisco Secure Endpoint**

### 4.2 CrowdStrike Falcon Integration

```typescript
class CrowdStrikeIntegration {
    private apiClient: CrowdStrikeAPI;

    async getDeviceHealth(deviceId: string): Promise<DeviceHealth> {
        // Get device details from CrowdStrike
        const device = await this.apiClient.getDevice(deviceId);

        // Get detection events
        const detections = await this.apiClient.getDetections({
            filter: `device_id:'${deviceId}'+severity:['high','critical']`,
            limit: 10
        });

        // Calculate health score
        const healthScore = this.calculateHealthScore({
            osVersion: device.os_version,
            lastSeen: device.last_seen,
            agentVersion: device.agent_version,
            detections: detections.resources,
            preventionPolicyApplied: device.policies
        });

        return {
            deviceId,
            compliant: healthScore >= 90,
            healthScore,
            threats: {
                activeThreat: detections.resources.length > 0,
                threatCount: detections.resources.length,
                lastDetection: detections.resources[0]?.created_timestamp
            },
            compliance: {
                osVersion: this.validateOSVersion(device.os_version),
                agentUpToDate: this.validateAgentVersion(device.agent_version),
                preventionEnabled: device.policies.prevention_policy_applied
            }
        };
    }

    async enforceDevicePolicy(deviceId: string, policy: DevicePolicy) {
        await this.apiClient.updateDevicePolicy(deviceId, {
            device_control: policy.deviceControl,
            firewall: policy.firewallEnabled,
            prevention_policy_id: policy.preventionPolicyId
        });
    }
}
```

### 4.3 Microsoft Defender Integration

```typescript
class DefenderIntegration {
    async getDeviceRiskScore(deviceId: string): Promise<number> {
        const graphClient = this.getGraphClient();

        // Query Microsoft Graph Security API
        const device = await graphClient
            .api(`/security/devices/${deviceId}`)
            .get();

        const alerts = await graphClient
            .api('/security/alerts')
            .filter(`deviceId eq '${deviceId}' and severity eq 'high'`)
            .top(50)
            .get();

        // Calculate risk based on device exposure level
        const riskScore = this.calculateRiskFromExposure({
            exposureLevel: device.exposureLevel, // Low, Medium, High
            riskScore: device.riskScore, // Low, Medium, High
            alertCount: alerts.value.length,
            healthStatus: device.healthStatus
        });

        return riskScore;
    }
}
```

### 4.4 SentinelOne Integration

```typescript
class SentinelOneIntegration {
    async queryThreats(deviceId: string): Promise<ThreatInfo> {
        const threats = await this.apiClient.get('/web/api/v2.1/threats', {
            params: {
                endpointIds: deviceId,
                resolved: false
            }
        });

        return {
            activeThreat: threats.data.length > 0,
            threats: threats.data.map(threat => ({
                id: threat.id,
                classification: threat.classification,
                mitigationStatus: threat.mitigationStatus,
                threatName: threat.threatName,
                severity: threat.severity
            }))
        };
    }

    async isolateDevice(deviceId: string, reason: string) {
        await this.apiClient.post('/web/api/v2.1/agents/actions/disconnect', {
            filter: {
                ids: [deviceId]
            },
            data: {
                reason: reason
            }
        });
    }
}
```

---

## 5. Network Security Integration

### 5.1 Firewall Integration

#### Palo Alto Networks

```typescript
class PaloAltoIntegration {
    async createSecurityPolicy(policy: ZeroTrustPolicy) {
        const paPolicy = {
            '@name': `zt-${policy.policyId}`,
            from: { member: policy.conditions.network.allowedNetworks },
            to: { member: ['trust'] },
            source: { member: policy.conditions.network.allowedIPRanges },
            destination: { member: [policy.resource.resourceId] },
            application: { member: ['ssl', 'web-browsing'] },
            service: { member: ['application-default'] },
            action: policy.decision === 'allow' ? 'allow' : 'deny',
            'log-start': 'yes',
            'log-end': 'yes'
        };

        await this.panosAPI.createSecurityRule(paPolicy);
    }

    async getUserIdMapping(userId: string, ipAddress: string) {
        await this.panosAPI.userIdLogin({
            user: userId,
            ip: ipAddress,
            timeout: 3600
        });
    }
}
```

#### Cisco Firepower

```typescript
class FirepowerIntegration {
    async applyDynamicObjectMapping(trustScore: number, ipAddress: string) {
        const objectGroup = trustScore >= 80 ? 'high-trust-users' : 'low-trust-users';

        await this.fmcAPI.addToNetworkGroup({
            groupName: objectGroup,
            networks: [{
                type: 'Host',
                value: ipAddress
            }]
        });
    }
}
```

### 5.2 Network Access Control (NAC)

```typescript
class NACIntegration {
    async assignVLAN(deviceId: string, trustScore: number): Promise<number> {
        let vlan: number;

        if (trustScore >= 85) {
            vlan = 100; // Trusted VLAN
        } else if (trustScore >= 70) {
            vlan = 200; // Restricted VLAN
        } else {
            vlan = 300; // Quarantine VLAN
        }

        await this.nacController.assignPort({
            deviceId,
            vlan,
            acl: this.getACLForTrustLevel(trustScore)
        });

        return vlan;
    }
}
```

---

## 6. API Gateway Integration

### 6.1 Kong Gateway

```typescript
// Kong Plugin for Zero Trust
class ZeroTrustKongPlugin {
    async handler(config: PluginConfig) {
        // Extract authentication token
        const token = kong.request.get_header('Authorization');

        // Verify token with Zero Trust engine
        const verification = await this.verifyWithZeroTrust({
            token,
            resource: kong.request.get_path(),
            method: kong.request.get_method(),
            sourceIP: kong.client.get_forwarded_ip()
        });

        if (!verification.allowed) {
            return kong.response.exit(403, {
                message: 'Access denied',
                trustScore: verification.trustScore,
                reason: verification.reason
            });
        }

        // Add trust score to upstream headers
        kong.service.request.set_header('X-Trust-Score', verification.trustScore);
        kong.service.request.set_header('X-User-ID', verification.userId);
    }
}
```

### 6.2 NGINX Plus

```nginx
# NGINX Zero Trust Module
location /api/ {
    # Call Zero Trust auth subrequest
    auth_request /zero-trust-verify;
    auth_request_set $trust_score $upstream_http_x_trust_score;
    auth_request_set $user_id $upstream_http_x_user_id;

    # Add headers to upstream
    proxy_set_header X-Trust-Score $trust_score;
    proxy_set_header X-User-ID $user_id;

    proxy_pass http://backend;
}

location = /zero-trust-verify {
    internal;
    proxy_pass https://zt-engine.example.com/api/v1/verify;
    proxy_pass_request_body off;
    proxy_set_header Content-Length "";
    proxy_set_header X-Original-URI $request_uri;
    proxy_set_header X-Original-Method $request_method;
}
```

### 6.3 AWS API Gateway

```typescript
// Lambda Authorizer for Zero Trust
export const handler = async (event: APIGatewayAuthorizerEvent) => {
    const token = event.headers?.Authorization;

    // Verify with Zero Trust engine
    const decision = await zeroTrustClient.verify({
        token,
        resource: event.methodArn,
        sourceIP: event.requestContext.identity.sourceIp,
        userAgent: event.requestContext.identity.userAgent
    });

    if (!decision.allowed) {
        throw new Error('Unauthorized');
    }

    return {
        principalId: decision.userId,
        policyDocument: {
            Version: '2012-10-17',
            Statement: [{
                Action: 'execute-api:Invoke',
                Effect: 'Allow',
                Resource: event.methodArn
            }]
        },
        context: {
            trustScore: decision.trustScore,
            userId: decision.userId,
            deviceId: decision.deviceId
        }
    };
};
```

---

## 7. Service Mesh Integration

### 7.1 Istio Integration

```yaml
# EnvoyFilter for Zero Trust
apiVersion: networking.istio.io/v1alpha3
kind: EnvoyFilter
metadata:
  name: zero-trust-filter
  namespace: istio-system
spec:
  configPatches:
  - applyTo: HTTP_FILTER
    match:
      context: SIDECAR_INBOUND
    patch:
      operation: INSERT_BEFORE
      value:
        name: envoy.filters.http.ext_authz
        typed_config:
          "@type": type.googleapis.com/envoy.extensions.filters.http.ext_authz.v3.ExtAuthz
          grpc_service:
            envoy_grpc:
              cluster_name: zero-trust-authz
            timeout: 0.5s
          failure_mode_allow: false
---
apiVersion: v1
kind: Service
metadata:
  name: zero-trust-authz
  namespace: istio-system
spec:
  ports:
  - port: 9191
    name: grpc
  selector:
    app: zero-trust-authz
```

### 7.2 Linkerd Integration

```yaml
# Linkerd AuthorizationPolicy
apiVersion: policy.linkerd.io/v1beta1
kind: AuthorizationPolicy
metadata:
  name: zero-trust-policy
  namespace: default
spec:
  targetRef:
    kind: Service
    name: finance-dashboard
  requiredAuthenticationRefs:
  - name: zero-trust-auth
    kind: MeshTLSAuthentication
---
apiVersion: policy.linkerd.io/v1alpha1
kind: MeshTLSAuthentication
metadata:
  name: zero-trust-auth
  namespace: default
spec:
  identities:
  - "*.zero-trust.svc.cluster.local"
```

---

## 8. Cloud Platform Integration

### 8.1 AWS Integration

```typescript
class AWSIntegration {
    async integrateWithIAMRoles(trustScore: number, userId: string) {
        const sts = new AWS.STS();

        // Assume role based on trust score
        const roleArn = trustScore >= 85
            ? 'arn:aws:iam::123456789:role/HighTrustRole'
            : 'arn:aws:iam::123456789:role/LowTrustRole';

        const credentials = await sts.assumeRole({
            RoleArn: roleArn,
            RoleSessionName: userId,
            DurationSeconds: this.getDurationByTrustScore(trustScore),
            Tags: [{
                Key: 'TrustScore',
                Value: trustScore.toString()
            }]
        }).promise();

        return credentials.Credentials;
    }

    async publishToEventBridge(event: ZeroTrustEvent) {
        const eventBridge = new AWS.EventBridge();

        await eventBridge.putEvents({
            Entries: [{
                Source: 'wia.zero-trust',
                DetailType: 'AccessDecision',
                Detail: JSON.stringify(event),
                EventBusName: 'zero-trust-events'
            }]
        }).promise();
    }
}
```

### 8.2 Azure Integration

```typescript
class AzureIntegration {
    async integrateWithConditionalAccess(trustScore: number, userId: string) {
        const graphClient = this.getGraphClient();

        // Create Conditional Access policy based on trust score
        if (trustScore < 70) {
            await graphClient
                .api('/identity/conditionalAccess/policies')
                .post({
                    displayName: `Low Trust - ${userId}`,
                    state: 'enabled',
                    conditions: {
                        users: {
                            includeUsers: [userId]
                        },
                        applications: {
                            includeApplications: ['All']
                        }
                    },
                    grantControls: {
                        operator: 'AND',
                        builtInControls: ['mfa', 'compliantDevice']
                    }
                });
        }
    }

    async logToSentinel(event: ZeroTrustEvent) {
        const logAnalyticsClient = new LogAnalyticsClient();

        await logAnalyticsClient.sendLogs({
            workspaceId: process.env.SENTINEL_WORKSPACE_ID,
            logType: 'ZeroTrustAccessLog',
            records: [event]
        });
    }
}
```

### 8.3 Google Cloud Integration

```typescript
class GCPIntegration {
    async integrateWithIdentityAwareProxy(trustScore: number) {
        const iap = new IdentityAwareProxyClient();

        // Configure IAP policy based on trust score
        const policy = {
            bindings: [{
                role: 'roles/iap.httpsResourceAccessor',
                members: [`user:${userId}`],
                condition: {
                    title: 'Trust Score Check',
                    expression: `request.auth.claims.trust_score >= ${trustScore}`
                }
            }]
        };

        await iap.setIamPolicy({
            resource: 'projects/my-project/iap_web/my-backend',
            policy
        });
    }
}
```

---

## 9. WIA Standards Integration

### 9.1 WIA-SEC-001 (Multi-Factor Authentication)

```typescript
class MFAIntegration {
    async requireMFABasedOnTrustScore(trustScore: number, userId: string) {
        if (trustScore < 85) {
            // Require MFA per WIA-SEC-001
            const mfaChallenge = await wiaSecClient.createMFAChallenge({
                userId,
                methods: ['totp', 'biometric'],
                timeout: 300
            });

            return {
                stepUpRequired: true,
                challengeId: mfaChallenge.id,
                allowedMethods: mfaChallenge.methods
            };
        }
    }
}
```

### 9.2 WIA-SEC-002 (Biometric Authentication)

```typescript
class BiometricIntegration {
    async verifyBiometric(trustScore: number, biometricData: BiometricData) {
        // Use WIA-SEC-002 biometric verification
        const verification = await wiaBiometricClient.verify({
            userId: biometricData.userId,
            biometricType: biometricData.type,
            template: biometricData.template,
            liveness: true
        });

        if (verification.matched && verification.livenessConfirmed) {
            return {
                verified: true,
                identityScore: 100,
                method: 'biometric'
            };
        }
    }
}
```

### 9.3 WIA-SEC-003 (Identity Management)

```typescript
class IdentityIntegration {
    async syncWithWIAIdentity(userId: string) {
        // Sync with WIA-SEC-003 identity standard
        const identity = await wiaIdentityClient.getIdentity(userId);

        return {
            userId: identity.id,
            verifiedAttributes: identity.verifiedAttributes,
            trustLevel: identity.trustLevel,
            credentials: identity.credentials
        };
    }
}
```

### 9.4 WIA-SEC-004 (Blockchain Security)

```typescript
class BlockchainIntegration {
    async auditOnBlockchain(event: ZeroTrustEvent) {
        // Record audit trail on blockchain per WIA-SEC-004
        const transaction = await wiaBlockchainClient.recordAudit({
            eventId: event.eventId,
            timestamp: event.timestamp,
            userId: event.subject.userId,
            decision: event.details.decision,
            trustScore: event.details.trustScore,
            hash: this.hashEvent(event)
        });

        return transaction.blockHash;
    }
}
```

---

## 10. Monitoring & Observability

### 10.1 Prometheus Metrics

```typescript
// Prometheus metrics for Zero Trust
const trustScoreGauge = new Gauge({
    name: 'zero_trust_score',
    help: 'Current trust score for active sessions',
    labelNames: ['user_id', 'device_id']
});

const accessDecisionCounter = new Counter({
    name: 'zero_trust_access_decisions_total',
    help: 'Total number of access decisions',
    labelNames: ['decision', 'resource_type', 'policy_id']
});

const trustCalculationHistogram = new Histogram({
    name: 'zero_trust_calculation_duration_seconds',
    help: 'Duration of trust score calculations',
    buckets: [0.1, 0.5, 1, 2, 5]
});

// Record metrics
trustScoreGauge.set({ user_id: userId, device_id: deviceId }, trustScore);
accessDecisionCounter.inc({ decision: 'allow', resource_type: 'application', policy_id: 'pol-123' });
trustCalculationHistogram.observe(calculationDuration);
```

### 10.2 OpenTelemetry Tracing

```typescript
import { trace } from '@opentelemetry/api';

const tracer = trace.getTracer('zero-trust-engine');

async function processAccessRequest(request: AccessRequest) {
    const span = tracer.startSpan('process_access_request', {
        attributes: {
            'user.id': request.subject.userId,
            'device.id': request.device.deviceId,
            'resource.id': request.resource.resourceId
        }
    });

    try {
        // Calculate trust score
        const trustSpan = tracer.startSpan('calculate_trust_score', { parent: span });
        const trustScore = await calculateTrustScore(request);
        trustSpan.setAttribute('trust.score', trustScore);
        trustSpan.end();

        // Evaluate policy
        const policySpan = tracer.startSpan('evaluate_policy', { parent: span });
        const decision = await evaluatePolicy(request, trustScore);
        policySpan.setAttribute('decision', decision);
        policySpan.end();

        return decision;
    } finally {
        span.end();
    }
}
```

---

## 11. Webhook Integration

### 11.1 Webhook Configuration

```json
{
  "webhookId": "webhook-123",
  "url": "https://app.example.com/webhooks/zero-trust",
  "events": [
    "access.granted",
    "access.denied",
    "trust.score.changed",
    "session.revoked",
    "policy.violated"
  ],
  "headers": {
    "X-Webhook-Secret": "secret-value"
  },
  "retryPolicy": {
    "maxRetries": 3,
    "backoff": "exponential"
  }
}
```

### 11.2 Webhook Payload

```json
{
  "webhookId": "webhook-123",
  "eventId": "evt-abc123",
  "eventType": "access.denied",
  "timestamp": "2025-12-25T10:30:00.000Z",
  "data": {
    "userId": "user-12345",
    "deviceId": "device-67890",
    "resourceId": "res-finance-001",
    "decision": "deny",
    "trustScore": 65,
    "reason": "Insufficient trust score",
    "riskFactors": ["unusual_location", "device_non_compliant"]
  },
  "signature": "sha256=abc123..."
}
```

---

## 12. Reference Implementations

### 12.1 Complete Integration Example

See `/examples/integrations/` for complete reference implementations:

- **`aws-lambda-authorizer/`** - AWS API Gateway integration
- **`istio-authz-server/`** - Istio service mesh integration
- **`kong-plugin/`** - Kong API Gateway plugin
- **`splunk-app/`** - Splunk SIEM application
- **`crowdstrike-connector/`** - CrowdStrike EDR integration

---

**Document Status:** ✅ Complete

---

© 2025 SmileStory Inc. / WIA
**弘익人間 (홍익인간)** - Benefit All Humanity
