# Chapter 7: Security Framework

## 7.1 Building Cybersecurity Landscape

### 7.1.1 The Convergence of IT and OT

Modern building energy management systems represent a critical convergence point between Information Technology (IT) and Operational Technology (OT). This convergence creates unique security challenges that require specialized approaches.

**IT/OT Convergence in Buildings:**

```
Traditional IT Domain                 Traditional OT Domain
┌────────────────────┐               ┌────────────────────┐
│  Enterprise Apps   │               │   BAS Controllers  │
│  Cloud Services    │◄─────────────►│   HVAC Equipment   │
│  User Devices      │   WIA-BEMS    │   Sensors/Meters   │
│  Network Infra     │               │   Safety Systems   │
└────────────────────┘               └────────────────────┘
         │                                     │
         └─────────────────┬───────────────────┘
                           │
              ┌────────────┴────────────┐
              │   CONVERGED SECURITY    │
              │                         │
              │  • Unified Monitoring   │
              │  • Integrated IAM       │
              │  • Cross-Domain Threat  │
              │    Detection            │
              └─────────────────────────┘
```

### 7.1.2 Building-Specific Threats

**Threat Landscape:**

| Threat Category | Examples | Impact | Likelihood |
|----------------|----------|--------|------------|
| Ransomware | BAS lockout, data encryption | High | Medium-High |
| Unauthorized Access | Control manipulation | High | Medium |
| Data Breach | Occupancy data, energy patterns | Medium | Medium |
| Denial of Service | System unavailability | Medium | Low-Medium |
| Supply Chain | Compromised firmware/updates | High | Low |
| Insider Threat | Malicious operator actions | Medium | Low |
| Physical-Cyber | Sensor manipulation | Medium | Low |

**Attack Vectors:**

```typescript
interface BuildingAttackVectors {
  networkBased: {
    internetExposure: "Misconfigured firewalls exposing BAS";
    lateralMovement: "IT compromise spreading to OT";
    wirelessAttacks: "Rogue access points, WiFi attacks";
  };

  protocolBased: {
    bacnetExploits: "Unauthenticated BACnet commands";
    modbusManipulation: "Unencrypted Modbus traffic";
    legacyProtocols: "Vulnerable legacy systems";
  };

  applicationBased: {
    webVulnerabilities: "OWASP Top 10 in BEMS UI";
    apiAbuse: "Broken authentication, injection";
    mobileApps: "Insecure tenant apps";
  };

  humanBased: {
    phishing: "Credential theft targeting FM staff";
    socialEngineering: "Physical access manipulation";
    insiderThreat: "Disgruntled employees";
  };
}
```

## 7.2 Security Architecture

### 7.2.1 Defense in Depth

**Layered Security Model:**

```
┌─────────────────────────────────────────────────────────────────┐
│                     GOVERNANCE LAYER                             │
│  Policies │ Standards │ Compliance │ Risk Management             │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                    PERIMETER LAYER                               │
│  Firewalls │ WAF │ DDoS Protection │ VPN │ Zero Trust Access    │
└─────────────────────────────┼───────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                    NETWORK LAYER                                 │
│  Segmentation │ IDS/IPS │ Network Monitoring │ NAC              │
└─────────────────────────────┼───────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                   APPLICATION LAYER                              │
│  Authentication │ Authorization │ Input Validation │ Encryption │
└─────────────────────────────┼───────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                      DATA LAYER                                  │
│  Encryption at Rest │ Masking │ Access Logging │ Backup         │
└─────────────────────────────┼───────────────────────────────────┘
                              │
┌─────────────────────────────┼───────────────────────────────────┐
│                   ENDPOINT LAYER                                 │
│  Device Hardening │ Firmware Security │ Secure Boot             │
└─────────────────────────────────────────────────────────────────┘
```

### 7.2.2 Network Segmentation

**Recommended Network Architecture:**

```
                            Internet
                                │
                    ┌───────────┴───────────┐
                    │       Firewall        │
                    │    (Edge Security)    │
                    └───────────┬───────────┘
                                │
              ┌─────────────────┼─────────────────┐
              │                 │                 │
       ┌──────┴──────┐  ┌───────┴───────┐  ┌─────┴─────┐
       │   DMZ       │  │  Corporate    │  │   OT      │
       │   VLAN 10   │  │   VLAN 20     │  │  VLAN 30  │
       └──────┬──────┘  └───────┬───────┘  └─────┬─────┘
              │                 │                 │
        ┌─────┴─────┐    ┌──────┴──────┐   ┌─────┴─────┐
        │ Web Apps  │    │ Workstations│   │  BAS Net  │
        │ API GW    │    │ Servers     │   │  VLAN 31  │
        │ Reverse   │    │ AD/LDAP     │   └─────┬─────┘
        │ Proxy     │    └─────────────┘         │
        └───────────┘                      ┌─────┴─────┐
                                           │ Controllers│
                                           │ VLAN 32   │
                                           └─────┬─────┘
                                                 │
                                           ┌─────┴─────┐
                                           │ Field     │
                                           │ Devices   │
                                           │ VLAN 33   │
                                           └───────────┘

Firewall Rules Between Zones:
├── Internet → DMZ: HTTPS (443) only
├── DMZ → Corporate: API calls only (specific ports)
├── Corporate → OT: Read-only by default
├── OT → Internet: Blocked (use proxy if needed)
└── Inter-OT: Restricted by function
```

### 7.2.3 Zero Trust Architecture

**Zero Trust Principles for Buildings:**

```typescript
interface ZeroTrustArchitecture {
  principles: {
    neverTrust: "Verify every access request regardless of source";
    alwaysVerify: "Authenticate and authorize every transaction";
    leastPrivilege: "Minimal access rights for each identity";
    assumeBreach: "Design assuming attackers are already inside";
    explicitVerification: "Use multiple signals for access decisions";
  };

  implementation: {
    identity: {
      mfa: "Multi-factor for all users and devices";
      deviceTrust: "Device health verification before access";
      continuousAuth: "Session monitoring and re-verification";
    };

    network: {
      microsegmentation: "Application-level isolation";
      sdp: "Software-Defined Perimeter";
      encryptedTransit: "TLS everywhere";
    };

    application: {
      justInTime: "Temporary elevated access";
      contextAware: "Location, time, device-based policies";
      applicationProxy: "No direct application exposure";
    };
  };
}
```

## 7.3 Authentication and Identity Management

### 7.3.1 OAuth 2.0 Implementation

**WIA-BEMS OAuth 2.0 Configuration:**

```typescript
// OAuth 2.0 Authorization Server Configuration
interface OAuthServerConfig {
  issuer: string;
  authorizationEndpoint: string;
  tokenEndpoint: string;
  jwksUri: string;

  grantTypes: [
    "authorization_code",
    "client_credentials",
    "refresh_token",
    "urn:ietf:params:oauth:grant-type:device_code"
  ];

  tokenSettings: {
    accessTokenLifetime: 3600;      // 1 hour
    refreshTokenLifetime: 2592000;  // 30 days
    idTokenLifetime: 3600;
    authorizationCodeLifetime: 300; // 5 minutes
  };

  pkceRequired: true;
  issueRefreshToken: true;
  rotateRefreshTokens: true;

  scopeDefinitions: {
    "buildings:read": "Read building information";
    "buildings:write": "Create and update buildings";
    "energy:read": "Read energy data";
    "energy:export": "Export bulk energy data";
    "equipment:read": "Read equipment status";
    "equipment:control": "Send control commands";
    "admin:users": "Manage users";
    "admin:audit": "Access audit logs";
  };
}
```

**Token Validation:**

```python
import jwt
from cryptography.hazmat.primitives import serialization

class TokenValidator:
    """
    Validate OAuth 2.0 access tokens
    """

    def __init__(self, config: OAuthConfig):
        self.config = config
        self.jwks = self._fetch_jwks()

    def validate_token(self, token: str) -> TokenClaims:
        """
        Validate access token and extract claims
        """
        try:
            # Decode header to get key ID
            header = jwt.get_unverified_header(token)
            kid = header.get('kid')

            # Get public key from JWKS
            public_key = self._get_public_key(kid)

            # Verify and decode token
            claims = jwt.decode(
                token,
                public_key,
                algorithms=['RS256'],
                audience=self.config.audience,
                issuer=self.config.issuer,
                options={
                    'verify_exp': True,
                    'verify_iat': True,
                    'verify_nbf': True,
                    'require': ['exp', 'iat', 'sub', 'scope']
                }
            )

            return TokenClaims(
                subject=claims['sub'],
                scopes=claims['scope'].split(' '),
                expires_at=datetime.fromtimestamp(claims['exp']),
                issued_at=datetime.fromtimestamp(claims['iat']),
                client_id=claims.get('client_id'),
                tenant_id=claims.get('tenant_id')
            )

        except jwt.ExpiredSignatureError:
            raise AuthenticationError("Token has expired")
        except jwt.InvalidTokenError as e:
            raise AuthenticationError(f"Invalid token: {str(e)}")

    def _fetch_jwks(self) -> dict:
        """Fetch JSON Web Key Set from authorization server"""
        response = requests.get(
            self.config.jwks_uri,
            timeout=10
        )
        return response.json()

    def _get_public_key(self, kid: str):
        """Get public key for key ID"""
        for key in self.jwks['keys']:
            if key['kid'] == kid:
                return jwt.algorithms.RSAAlgorithm.from_jwk(key)
        raise AuthenticationError(f"Key {kid} not found in JWKS")
```

### 7.3.2 Multi-Factor Authentication

**MFA Implementation:**

```typescript
interface MFAConfiguration {
  factors: {
    password: {
      required: true;
      minLength: 12;
      complexity: "uppercase,lowercase,number,special";
      maxAge: 90;  // days
      history: 12;  // prevent reuse
    };

    totp: {
      algorithm: "SHA256";
      digits: 6;
      period: 30;
      issuer: "WIA-BEMS";
    };

    webAuthn: {
      rpId: "bems.example.com";
      attestation: "none";
      userVerification: "preferred";
      allowedAuthenticators: ["platform", "cross-platform"];
    };

    push: {
      provider: "custom";
      timeout: 60;  // seconds
      allowOffline: false;
    };
  };

  policies: {
    adminUsers: ["password", "webAuthn"];  // Strongest
    operators: ["password", "totp"];
    viewers: ["password"];  // Single factor for read-only
    apiClients: ["clientCertificate"];
    devices: ["clientCertificate", "deviceAttestation"];
  };
}

class MFAService {
  async initiateAuthentication(
    userId: string,
    firstFactor: Credential
  ): Promise<MFAChallenge> {
    // Verify first factor
    const user = await this.verifyFirstFactor(userId, firstFactor);

    // Determine required second factors
    const requiredFactors = this.getRequiredFactors(user.roles);

    if (requiredFactors.length === 0) {
      // Single factor sufficient
      return this.completeAuthentication(user);
    }

    // Generate challenges for available second factors
    const challenges = await Promise.all(
      requiredFactors.map(factor => this.generateChallenge(user, factor))
    );

    return {
      sessionId: this.generateSessionId(),
      challenges: challenges,
      expiresAt: new Date(Date.now() + 300000)  // 5 minutes
    };
  }

  async completeAuthentication(
    sessionId: string,
    secondFactor: SecondFactorResponse
  ): Promise<AuthResult> {
    const session = await this.getSession(sessionId);

    // Verify second factor
    const valid = await this.verifySecondFactor(
      session.userId,
      session.challengeType,
      secondFactor
    );

    if (!valid) {
      await this.recordFailedAttempt(session.userId);
      throw new AuthenticationError("Invalid second factor");
    }

    // Issue tokens
    return this.issueTokens(session.user);
  }
}
```

### 7.3.3 Device Authentication

**Device Identity and Attestation:**

```python
class DeviceAuthenticator:
    """
    Authenticate IoT devices and controllers
    """

    def __init__(self, config: DeviceAuthConfig):
        self.config = config
        self.ca_cert = self._load_ca_certificate()

    def authenticate_device(
        self,
        device_id: str,
        client_certificate: bytes,
        attestation: Optional[bytes] = None
    ) -> DeviceIdentity:
        """
        Authenticate device using X.509 certificate
        """
        # Parse and validate certificate
        cert = x509.load_pem_x509_certificate(client_certificate)

        # Verify certificate chain
        self._verify_certificate_chain(cert)

        # Check certificate validity
        now = datetime.utcnow()
        if now < cert.not_valid_before or now > cert.not_valid_after:
            raise AuthenticationError("Certificate expired or not yet valid")

        # Extract device ID from certificate
        cert_device_id = self._extract_device_id(cert)
        if cert_device_id != device_id:
            raise AuthenticationError("Device ID mismatch")

        # Verify device attestation (if provided)
        if attestation and self.config.require_attestation:
            self._verify_attestation(device_id, attestation)

        # Check device registration
        registered_device = self._lookup_device(device_id)
        if not registered_device:
            raise AuthenticationError("Device not registered")

        # Check device status
        if registered_device.status != 'active':
            raise AuthenticationError(f"Device status: {registered_device.status}")

        return DeviceIdentity(
            device_id=device_id,
            device_type=registered_device.type,
            building_id=registered_device.building_id,
            capabilities=registered_device.capabilities,
            certificate_fingerprint=cert.fingerprint(hashes.SHA256()).hex()
        )

    def _verify_certificate_chain(self, cert: x509.Certificate):
        """Verify certificate was issued by trusted CA"""
        store = crypto.X509Store()
        store.add_cert(self.ca_cert)

        store_ctx = crypto.X509StoreContext(
            store,
            crypto.X509.from_cryptography(cert)
        )

        try:
            store_ctx.verify_certificate()
        except crypto.X509StoreContextError as e:
            raise AuthenticationError(f"Certificate validation failed: {e}")

    def _verify_attestation(self, device_id: str, attestation: bytes):
        """Verify device attestation (TPM or secure enclave)"""
        # Parse attestation statement
        att_statement = cbor2.loads(attestation)

        # Verify signature
        # Implementation depends on attestation format (TPM, Android, iOS)
        pass
```

## 7.4 Encryption and Data Protection

### 7.4.1 Transport Layer Security

**TLS Configuration:**

```yaml
tls_configuration:
  minimum_version: "TLS 1.3"
  fallback_version: "TLS 1.2"  # For legacy devices

  cipher_suites_tls13:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
    - TLS_AES_128_GCM_SHA256

  cipher_suites_tls12:
    - TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384
    - TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256
    - TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305_SHA256

  certificate_requirements:
    key_type: "ECDSA P-384 or RSA 4096"
    signature_algorithm: "SHA384withECDSA or SHA384withRSA"
    validity_period: "1 year maximum"
    certificate_transparency: required

  ocsp_stapling: true
  hsts:
    enabled: true
    max_age: 31536000  # 1 year
    include_subdomains: true
    preload: true
```

### 7.4.2 Data Encryption

**Encryption at Rest:**

```python
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC

class DataEncryptionService:
    """
    Encrypt sensitive data at rest
    """

    def __init__(self, key_vault: KeyVaultClient):
        self.key_vault = key_vault
        self.cache = TTLCache(maxsize=100, ttl=3600)

    async def encrypt_field(
        self,
        data: bytes,
        context: EncryptionContext
    ) -> EncryptedData:
        """
        Encrypt data using envelope encryption
        """
        # Generate data encryption key (DEK)
        dek = os.urandom(32)

        # Encrypt data with DEK
        nonce = os.urandom(12)
        cipher = AESGCM(dek)
        ciphertext = cipher.encrypt(nonce, data, context.aad)

        # Encrypt DEK with key encryption key (KEK) from vault
        kek_id = context.key_id or self._get_default_key()
        encrypted_dek = await self.key_vault.encrypt(
            key_id=kek_id,
            plaintext=dek,
            algorithm='RSA-OAEP-256'
        )

        return EncryptedData(
            ciphertext=ciphertext,
            nonce=nonce,
            encrypted_dek=encrypted_dek,
            kek_id=kek_id,
            algorithm='AES-256-GCM'
        )

    async def decrypt_field(
        self,
        encrypted_data: EncryptedData,
        context: EncryptionContext
    ) -> bytes:
        """
        Decrypt envelope-encrypted data
        """
        # Decrypt DEK using KEK from vault
        dek = await self.key_vault.decrypt(
            key_id=encrypted_data.kek_id,
            ciphertext=encrypted_data.encrypted_dek,
            algorithm='RSA-OAEP-256'
        )

        # Decrypt data with DEK
        cipher = AESGCM(dek)
        plaintext = cipher.decrypt(
            encrypted_data.nonce,
            encrypted_data.ciphertext,
            context.aad
        )

        return plaintext


class SensitiveFieldEncryption:
    """
    Schema-aware field-level encryption
    """

    # Fields requiring encryption
    SENSITIVE_FIELDS = {
        'occupancy': ['count', 'detection_method'],
        'user': ['email', 'phone', 'badge_id'],
        'location': ['detailed_address'],
        'financial': ['cost', 'rate', 'account_number']
    }

    async def encrypt_document(
        self,
        document: dict,
        schema: str
    ) -> dict:
        """
        Encrypt sensitive fields in document
        """
        sensitive = self.SENSITIVE_FIELDS.get(schema, [])
        encrypted = document.copy()

        for field in sensitive:
            if field in document and document[field] is not None:
                value = json.dumps(document[field]).encode()
                encrypted[field] = await self.encryption_service.encrypt_field(
                    value,
                    EncryptionContext(schema=schema, field=field)
                )

        return encrypted
```

## 7.5 Access Control and Authorization

### 7.5.1 Attribute-Based Access Control (ABAC)

**ABAC Policy Engine:**

```python
class ABACPolicyEngine:
    """
    Attribute-Based Access Control for building resources
    """

    def evaluate(
        self,
        subject: Subject,
        action: str,
        resource: Resource,
        environment: Environment
    ) -> AccessDecision:
        """
        Evaluate access request against policies
        """
        # Collect all applicable policies
        policies = self.policy_store.find_policies(
            resource_type=resource.type,
            action=action
        )

        decisions = []

        for policy in policies:
            decision = self._evaluate_policy(
                policy, subject, action, resource, environment
            )
            decisions.append(decision)

        # Combine decisions (deny overrides)
        return self._combine_decisions(decisions)

    def _evaluate_policy(
        self,
        policy: Policy,
        subject: Subject,
        action: str,
        resource: Resource,
        environment: Environment
    ) -> PolicyDecision:
        """
        Evaluate single policy
        """
        # Check all conditions
        conditions_met = all(
            self._evaluate_condition(cond, subject, resource, environment)
            for cond in policy.conditions
        )

        if not conditions_met:
            return PolicyDecision(effect='not_applicable')

        return PolicyDecision(
            effect=policy.effect,
            obligations=policy.obligations
        )

    def _evaluate_condition(
        self,
        condition: Condition,
        subject: Subject,
        resource: Resource,
        environment: Environment
    ) -> bool:
        """
        Evaluate single condition
        """
        # Get attribute value
        if condition.attribute_source == 'subject':
            value = getattr(subject, condition.attribute)
        elif condition.attribute_source == 'resource':
            value = getattr(resource, condition.attribute)
        elif condition.attribute_source == 'environment':
            value = getattr(environment, condition.attribute)
        else:
            return False

        # Apply operator
        return condition.operator.evaluate(value, condition.value)


# Example policies
POLICIES = [
    Policy(
        id="building-access",
        effect="permit",
        conditions=[
            # Subject must have building in their assigned buildings
            Condition(
                attribute_source="subject",
                attribute="assigned_buildings",
                operator=Contains(),
                value=ResourceAttribute("building_id")
            ),
            # Resource must be active
            Condition(
                attribute_source="resource",
                attribute="status",
                operator=Equals(),
                value="active"
            )
        ]
    ),

    Policy(
        id="control-during-business-hours",
        effect="permit",
        conditions=[
            # Action is control
            Condition(
                attribute_source="action",
                attribute="type",
                operator=Equals(),
                value="control"
            ),
            # Current time within business hours
            Condition(
                attribute_source="environment",
                attribute="current_hour",
                operator=Between(),
                value=(6, 22)
            ),
            # Or subject has after-hours permission
            Condition(
                attribute_source="subject",
                attribute="permissions",
                operator=Contains(),
                value="after_hours_control"
            )
        ],
        operator="any"  # OR between condition groups
    ),

    Policy(
        id="deny-critical-without-approval",
        effect="deny",
        conditions=[
            Condition(
                attribute_source="resource",
                attribute="criticality",
                operator=Equals(),
                value="critical"
            ),
            Condition(
                attribute_source="subject",
                attribute="has_approval",
                operator=Equals(),
                value=False
            )
        ]
    )
]
```

### 7.5.2 Command Authorization

**Control Command Authorization:**

```typescript
interface CommandAuthorizationService {
  authorizeCommand(
    command: ControlCommand,
    user: AuthenticatedUser,
    context: ExecutionContext
  ): Promise<AuthorizationResult>;
}

class ControlCommandAuthorizer implements CommandAuthorizationService {
  private policyEngine: ABACPolicyEngine;
  private auditLog: AuditLogger;

  async authorizeCommand(
    command: ControlCommand,
    user: AuthenticatedUser,
    context: ExecutionContext
  ): Promise<AuthorizationResult> {
    // 1. Check basic permissions
    if (!user.scopes.includes('equipment:control')) {
      return this.deny('Missing equipment:control scope');
    }

    // 2. Check equipment assignment
    const equipment = await this.equipmentService.get(command.equipmentId);
    if (!user.assignedBuildings.includes(equipment.buildingId)) {
      return this.deny('Equipment not in assigned buildings');
    }

    // 3. Check command-specific rules
    const commandRules = this.getCommandRules(command.type);

    // Priority check
    if (command.priority < commandRules.minPriority) {
      if (!user.roles.includes('engineer')) {
        return this.deny(`Priority ${command.priority} requires engineer role`);
      }
    }

    // Value range check
    if (commandRules.valueRange) {
      if (!this.inRange(command.value, commandRules.valueRange)) {
        return this.deny('Command value out of allowed range');
      }
    }

    // Time-based restrictions
    if (commandRules.scheduleRestriction) {
      if (!this.withinSchedule(context.timestamp, commandRules.scheduleRestriction)) {
        return this.deny('Command not allowed at this time');
      }
    }

    // 4. Check for conflicting commands
    const conflicts = await this.checkConflicts(command, equipment);
    if (conflicts.length > 0) {
      return this.deny('Conflicts with existing commands', conflicts);
    }

    // 5. Run ABAC policy evaluation
    const abacResult = await this.policyEngine.evaluate(
      user, command.type, equipment, context
    );

    if (abacResult.effect === 'deny') {
      return this.deny('Policy denied', abacResult.obligations);
    }

    // 6. Log authorization
    await this.auditLog.log({
      eventType: 'command_authorized',
      user: user.id,
      command: command,
      equipment: equipment.id,
      context: context
    });

    return {
      authorized: true,
      obligations: abacResult.obligations,
      expiresAt: this.calculateExpiry(command)
    };
  }

  private getCommandRules(commandType: string): CommandRules {
    return COMMAND_RULES[commandType] || DEFAULT_COMMAND_RULES;
  }
}

const COMMAND_RULES: Record<string, CommandRules> = {
  'set_temperature': {
    minPriority: 8,
    valueRange: { min: 15, max: 30 },
    scheduleRestriction: null  // Allowed anytime
  },
  'override_schedule': {
    minPriority: 5,
    valueRange: null,
    scheduleRestriction: { requiresApproval: true }
  },
  'emergency_stop': {
    minPriority: 1,
    valueRange: null,
    scheduleRestriction: null,
    requiresRole: 'operator'
  }
};
```

## 7.6 Security Monitoring and Incident Response

### 7.6.1 Security Information and Event Management

**SIEM Integration:**

```python
class SecurityMonitor:
    """
    Security monitoring and alerting for WIA-BEMS
    """

    def __init__(self, config: SecurityConfig):
        self.siem_client = SIEMClient(config.siem_endpoint)
        self.alert_rules = self._load_alert_rules()
        self.anomaly_detector = AnomalyDetector()

    async def process_event(self, event: SecurityEvent):
        """
        Process security event and generate alerts
        """
        # Normalize event
        normalized = self._normalize_event(event)

        # Send to SIEM
        await self.siem_client.ingest(normalized)

        # Check alert rules
        alerts = self._evaluate_rules(normalized)

        # Check for anomalies
        if self.anomaly_detector.is_anomaly(normalized):
            alerts.append(Alert(
                severity='medium',
                type='anomaly',
                description=f"Anomalous event detected: {event.type}"
            ))

        # Process alerts
        for alert in alerts:
            await self._process_alert(alert, normalized)

    def _evaluate_rules(self, event: NormalizedEvent) -> List[Alert]:
        """
        Evaluate event against alert rules
        """
        alerts = []

        for rule in self.alert_rules:
            if rule.matches(event):
                alerts.append(Alert(
                    rule_id=rule.id,
                    severity=rule.severity,
                    type=rule.alert_type,
                    description=rule.format_description(event),
                    event=event
                ))

        return alerts

    async def _process_alert(self, alert: Alert, event: NormalizedEvent):
        """
        Process and escalate alert
        """
        # Store alert
        await self.alert_store.save(alert)

        # Notify based on severity
        if alert.severity == 'critical':
            await self.notify_on_call(alert)
            await self.create_incident(alert)

        elif alert.severity == 'high':
            await self.notify_security_team(alert)

        elif alert.severity == 'medium':
            await self.send_email_notification(alert)

        # Log to audit trail
        await self.audit_log.log_alert(alert, event)


# Alert rules
ALERT_RULES = [
    AlertRule(
        id='AUTH-001',
        name='Multiple Failed Logins',
        condition=lambda e: (
            e.type == 'authentication_failed' and
            e.count_in_window('authentication_failed', user=e.user, minutes=5) >= 5
        ),
        severity='high',
        alert_type='brute_force'
    ),

    AlertRule(
        id='AUTH-002',
        name='Login from New Location',
        condition=lambda e: (
            e.type == 'authentication_success' and
            not e.user.known_locations.contains(e.location)
        ),
        severity='medium',
        alert_type='suspicious_login'
    ),

    AlertRule(
        id='CTRL-001',
        name='Unusual Control Command',
        condition=lambda e: (
            e.type == 'control_command' and
            e.time.hour not in range(6, 22) and
            e.user.role != 'operator'
        ),
        severity='medium',
        alert_type='unusual_activity'
    ),

    AlertRule(
        id='CTRL-002',
        name='Safety Override Attempt',
        condition=lambda e: (
            e.type == 'control_command' and
            e.command.target in SAFETY_CRITICAL_POINTS
        ),
        severity='critical',
        alert_type='safety_override'
    ),

    AlertRule(
        id='DATA-001',
        name='Bulk Data Export',
        condition=lambda e: (
            e.type == 'data_export' and
            e.record_count > 100000
        ),
        severity='medium',
        alert_type='data_exfiltration'
    )
]
```

### 7.6.2 Incident Response

**Incident Response Playbook:**

```markdown
## WIA-BEMS Incident Response Playbook

### 1. Detection and Analysis

**Initial Triage:**
- [ ] Confirm alert is not false positive
- [ ] Determine scope (single device, zone, building, portfolio)
- [ ] Assess potential impact on operations
- [ ] Classify severity (Critical, High, Medium, Low)

**Classification Criteria:**
| Severity | Criteria | Response Time |
|----------|----------|---------------|
| Critical | Safety risk, data breach, ransomware | 15 minutes |
| High | System compromise, control manipulation | 1 hour |
| Medium | Suspicious activity, policy violation | 4 hours |
| Low | Minor policy violation | 24 hours |

### 2. Containment

**Immediate Actions:**
- [ ] Isolate affected systems (network isolation)
- [ ] Preserve evidence (logs, memory dumps)
- [ ] Block attacking IPs/accounts
- [ ] Enable enhanced logging

**OT-Specific Containment:**
- [ ] Switch affected systems to manual control if needed
- [ ] Verify critical safety systems unaffected
- [ ] Notify facility management

### 3. Eradication

**Malware/Compromise:**
- [ ] Identify all affected systems
- [ ] Remove malicious software/access
- [ ] Patch vulnerabilities exploited
- [ ] Reset compromised credentials

**Data Breach:**
- [ ] Identify data accessed/exfiltrated
- [ ] Revoke compromised access tokens
- [ ] Notify affected parties (legal review)

### 4. Recovery

**System Restoration:**
- [ ] Restore from known-good backups
- [ ] Verify system integrity
- [ ] Re-enable monitoring
- [ ] Gradual return to normal operations

**Validation:**
- [ ] Test control sequences
- [ ] Verify data integrity
- [ ] Confirm security controls active

### 5. Post-Incident

**Documentation:**
- [ ] Complete incident report
- [ ] Root cause analysis
- [ ] Lessons learned

**Improvements:**
- [ ] Update security controls
- [ ] Revise detection rules
- [ ] Improve response procedures
- [ ] Conduct training if needed
```

## 7.7 Compliance and Audit

### 7.7.1 Compliance Frameworks

**Applicable Standards:**

| Framework | Scope | Key Requirements |
|-----------|-------|------------------|
| SOC 2 Type II | Cloud BEMS | Security, availability, confidentiality |
| ISO 27001 | Information Security | ISMS implementation |
| NIST CSF | Cybersecurity | Identify, Protect, Detect, Respond, Recover |
| IEC 62443 | Industrial Security | OT-specific security levels |
| GDPR | EU Data Privacy | Personal data processing |
| CCPA | CA Data Privacy | Consumer data rights |

### 7.7.2 Audit Logging

**Comprehensive Audit Trail:**

```typescript
interface AuditEvent {
  eventId: string;          // UUID
  timestamp: string;        // ISO 8601 with timezone
  eventType: string;        // Categorized event type
  actor: {
    type: 'user' | 'service' | 'device';
    id: string;
    name: string;
    ip: string;
    userAgent?: string;
  };
  action: string;           // Specific action taken
  resource: {
    type: string;
    id: string;
    name: string;
  };
  outcome: 'success' | 'failure';
  details: Record<string, any>;
  risk: 'low' | 'medium' | 'high';
}

class AuditLogger {
  async log(event: Partial<AuditEvent>): Promise<void> {
    const fullEvent: AuditEvent = {
      eventId: uuid(),
      timestamp: new Date().toISOString(),
      ...event,
      risk: this.calculateRisk(event)
    };

    // Write to immutable store
    await this.auditStore.write(fullEvent);

    // Forward to SIEM
    await this.siem.forward(fullEvent);

    // Check for alertable conditions
    await this.securityMonitor.checkEvent(fullEvent);
  }

  private calculateRisk(event: Partial<AuditEvent>): RiskLevel {
    // Risk calculation based on action and context
    if (event.action?.includes('delete') && event.resource?.type === 'building') {
      return 'high';
    }
    if (event.action === 'control_command' && event.details?.priority < 5) {
      return 'high';
    }
    if (event.outcome === 'failure' && event.action?.includes('auth')) {
      return 'medium';
    }
    return 'low';
  }
}
```

---

**Chapter Summary:**

This chapter covered the WIA-BEMS Security Framework:

- IT/OT convergence security challenges
- Defense-in-depth and Zero Trust architectures
- OAuth 2.0, MFA, and device authentication
- Encryption for data in transit and at rest
- Attribute-based access control for fine-grained authorization
- Security monitoring and incident response
- Compliance and audit logging requirements

In the next chapter, we will provide a comprehensive Implementation Guide for deploying WIA-BEMS.

---

© 2025 World Certification Industry Association (WIA)
弘益人間 (Hongik Ingan) - Benefit All Humanity
