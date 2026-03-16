# WIA-SEC-007: Biometric Authentication - Phase 4: Integration & Implementation

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## 1. Overview

This specification defines integration patterns for biometric authentication with modern web standards (FIDO2/WebAuthn), mobile platforms (iOS/Android), and enterprise systems.

---

## 2. FIDO2 / WebAuthn Integration

### 2.1 WebAuthn Registration

```javascript
// Client-side: Register biometric authenticator
async function registerWebAuthn(username) {
    // 1. Get registration options from server
    const response = await fetch('/api/webauthn/register/begin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username })
    });

    const options = await response.json();

    // 2. Convert challenge from base64
    options.challenge = Uint8Array.from(
        atob(options.challenge), c => c.charCodeAt(0)
    );
    options.user.id = Uint8Array.from(
        atob(options.user.id), c => c.charCodeAt(0)
    );

    // 3. Call WebAuthn API
    const credential = await navigator.credentials.create({
        publicKey: {
            ...options,
            authenticatorSelection: {
                authenticatorAttachment: 'platform',  // Built-in biometric
                userVerification: 'required',         // Biometric required
                residentKey: 'preferred'              // Passwordless
            },
            attestation: 'direct'  // Get authenticator certificate
        }
    });

    // 4. Send credential to server
    const attestationResponse = {
        id: credential.id,
        rawId: btoa(String.fromCharCode(...new Uint8Array(credential.rawId))),
        response: {
            clientDataJSON: btoa(String.fromCharCode(...new Uint8Array(credential.response.clientDataJSON))),
            attestationObject: btoa(String.fromCharCode(...new Uint8Array(credential.response.attestationObject)))
        },
        type: credential.type
    };

    await fetch('/api/webauthn/register/complete', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(attestationResponse)
    });

    return { success: true };
}
```

### 2.2 WebAuthn Authentication

```javascript
// Client-side: Authenticate with biometric
async function authenticateWebAuthn(username) {
    // 1. Get authentication options from server
    const response = await fetch('/api/webauthn/authenticate/begin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username })
    });

    const options = await response.json();

    // 2. Convert challenge from base64
    options.challenge = Uint8Array.from(
        atob(options.challenge), c => c.charCodeAt(0)
    );

    if (options.allowCredentials) {
        options.allowCredentials = options.allowCredentials.map(cred => ({
            ...cred,
            id: Uint8Array.from(atob(cred.id), c => c.charCodeAt(0))
        }));
    }

    // 3. Call WebAuthn API - triggers biometric prompt
    const assertion = await navigator.credentials.get({
        publicKey: {
            ...options,
            userVerification: 'required'  // Biometric required
        }
    });

    // 4. Send assertion to server for verification
    const assertionResponse = {
        id: assertion.id,
        rawId: btoa(String.fromCharCode(...new Uint8Array(assertion.rawId))),
        response: {
            clientDataJSON: btoa(String.fromCharCode(...new Uint8Array(assertion.response.clientDataJSON))),
            authenticatorData: btoa(String.fromCharCode(...new Uint8Array(assertion.response.authenticatorData))),
            signature: btoa(String.fromCharCode(...new Uint8Array(assertion.response.signature))),
            userHandle: assertion.response.userHandle ?
                btoa(String.fromCharCode(...new Uint8Array(assertion.response.userHandle))) : null
        },
        type: assertion.type
    };

    const result = await fetch('/api/webauthn/authenticate/complete', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(assertionResponse)
    });

    return await result.json();
}
```

### 2.3 Server-Side WebAuthn Verification (Python/FastAPI)

```python
from fastapi import FastAPI, HTTPException
from webauthn import (
    generate_registration_options,
    verify_registration_response,
    generate_authentication_options,
    verify_authentication_response
)
import secrets

app = FastAPI()

# In-memory storage (use database in production)
users_db = {}
credentials_db = {}

@app.post('/api/webauthn/register/begin')
async def register_begin(request: dict):
    username = request['username']

    # Generate challenge
    challenge = secrets.token_bytes(32)

    # Generate registration options
    options = generate_registration_options(
        rp_id='wiastandards.com',
        rp_name='WIA Standards',
        user_id=username.encode('utf-8'),
        user_name=username,
        user_display_name=username,
        challenge=challenge,
        authenticator_selection={
            'authenticatorAttachment': 'platform',
            'userVerification': 'required'
        }
    )

    # Store challenge for verification
    users_db[username] = {
        'challenge': challenge,
        'options': options
    }

    return options

@app.post('/api/webauthn/register/complete')
async def register_complete(request: dict):
    username = request.get('username')
    user_data = users_db.get(username)

    if not user_data:
        raise HTTPException(status_code=400, detail='Invalid registration')

    # Verify attestation
    verification = verify_registration_response(
        credential=request,
        expected_challenge=user_data['challenge'],
        expected_origin='https://wiastandards.com',
        expected_rp_id='wiastandards.com'
    )

    # Store credential
    credentials_db[verification.credential_id] = {
        'username': username,
        'public_key': verification.credential_public_key,
        'sign_count': verification.sign_count
    }

    return {'verified': True}

@app.post('/api/webauthn/authenticate/begin')
async def authenticate_begin(request: dict):
    username = request['username']

    # Find user's credentials
    user_credentials = [
        {'id': cred_id, 'type': 'public-key'}
        for cred_id, cred in credentials_db.items()
        if cred['username'] == username
    ]

    if not user_credentials:
        raise HTTPException(status_code=404, detail='User not found')

    # Generate challenge
    challenge = secrets.token_bytes(32)

    options = generate_authentication_options(
        rp_id='wiastandards.com',
        challenge=challenge,
        allow_credentials=user_credentials,
        user_verification='required'
    )

    users_db[username] = {'challenge': challenge}

    return options

@app.post('/api/webauthn/authenticate/complete')
async def authenticate_complete(request: dict):
    credential_id = request['id']
    credential = credentials_db.get(credential_id)

    if not credential:
        raise HTTPException(status_code=404, detail='Credential not found')

    username = credential['username']
    user_data = users_db.get(username)

    # Verify assertion
    verification = verify_authentication_response(
        credential=request,
        expected_challenge=user_data['challenge'],
        expected_origin='https://wiastandards.com',
        expected_rp_id='wiastandards.com',
        credential_public_key=credential['public_key'],
        credential_current_sign_count=credential['sign_count']
    )

    # Update sign count (prevents replay attacks)
    credentials_db[credential_id]['sign_count'] = verification.new_sign_count

    return {
        'verified': True,
        'username': username,
        'user_verified': verification.user_verified
    }
```

---

## 3. Mobile Platform Integration

### 3.1 iOS - Touch ID / Face ID

```swift
import LocalAuthentication

class BiometricAuthManager {
    func authenticateWithBiometrics(completion: @escaping (Bool, Error?) -> Void) {
        let context = LAContext()
        var error: NSError?

        // Check if biometrics are available
        guard context.canEvaluatePolicy(.deviceOwnerAuthenticationWithBiometrics, error: &error) else {
            completion(false, error)
            return
        }

        // Determine biometric type
        let biometricType: String
        if #available(iOS 11.0, *) {
            switch context.biometryType {
            case .faceID:
                biometricType = "Face ID"
            case .touchID:
                biometricType = "Touch ID"
            default:
                biometricType = "Biometrics"
            }
        } else {
            biometricType = "Touch ID"
        }

        // Authenticate
        context.evaluatePolicy(
            .deviceOwnerAuthenticationWithBiometrics,
            localizedReason: "Authenticate with \(biometricType)"
        ) { success, error in
            DispatchQueue.main.async {
                completion(success, error)
            }
        }
    }

    func authenticateAndGetCredential(completion: @escaping (Data?, Error?) -> Void) {
        let context = LAContext()

        // Authenticate with biometrics
        context.evaluatePolicy(
            .deviceOwnerAuthenticationWithBiometrics,
            localizedReason: "Access your secure credential"
        ) { success, error in
            if success {
                // Retrieve encrypted credential from Keychain
                let query: [String: Any] = [
                    kSecClass as String: kSecClassGenericPassword,
                    kSecAttrAccount as String: "biometric_credential",
                    kSecReturnData as String: true,
                    kSecUseAuthenticationContext as String: context
                ]

                var item: CFTypeRef?
                let status = SecItemCopyMatching(query as CFDictionary, &item)

                if status == errSecSuccess,
                   let data = item as? Data {
                    completion(data, nil)
                } else {
                    completion(nil, NSError(domain: "KeychainError", code: Int(status)))
                }
            } else {
                completion(nil, error)
            }
        }
    }

    func storeCredentialInKeychain(_ credential: Data, completion: @escaping (Bool) -> Void) {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrAccount as String: "biometric_credential",
            kSecValueData as String: credential,
            kSecAttrAccessControl as String: createAccessControl()
        ]

        SecItemDelete(query as CFDictionary)  // Delete existing
        let status = SecItemAdd(query as CFDictionary, nil)

        completion(status == errSecSuccess)
    }

    private func createAccessControl() -> SecAccessControl {
        return SecAccessControlCreateWithFlags(
            nil,
            kSecAttrAccessibleWhenUnlockedThisDeviceOnly,
            .biometryCurrentSet,  // Invalidate if biometry changes
            nil
        )!
    }
}
```

### 3.2 Android - BiometricPrompt

```kotlin
import androidx.biometric.BiometricManager
import androidx.biometric.BiometricPrompt
import androidx.core.content.ContextCompat
import androidx.fragment.app.FragmentActivity

class BiometricAuthManager(private val activity: FragmentActivity) {

    fun authenticateWithBiometrics(
        onSuccess: (BiometricPrompt.AuthenticationResult) -> Unit,
        onError: (Int, CharSequence) -> Unit
    ) {
        // Check biometric availability
        val biometricManager = BiometricManager.from(activity)
        when (biometricManager.canAuthenticate(BiometricManager.Authenticators.BIOMETRIC_STRONG)) {
            BiometricManager.BIOMETRIC_SUCCESS -> {
                // Biometrics available
            }
            BiometricManager.BIOMETRIC_ERROR_NO_HARDWARE -> {
                onError(-1, "No biometric hardware")
                return
            }
            BiometricManager.BIOMETRIC_ERROR_HW_UNAVAILABLE -> {
                onError(-1, "Biometric hardware unavailable")
                return
            }
            BiometricManager.BIOMETRIC_ERROR_NONE_ENROLLED -> {
                onError(-1, "No biometrics enrolled")
                return
            }
        }

        // Create BiometricPrompt
        val executor = ContextCompat.getMainExecutor(activity)
        val biometricPrompt = BiometricPrompt(
            activity,
            executor,
            object : BiometricPrompt.AuthenticationCallback() {
                override fun onAuthenticationSucceeded(result: BiometricPrompt.AuthenticationResult) {
                    onSuccess(result)
                }

                override fun onAuthenticationError(errorCode: Int, errString: CharSequence) {
                    onError(errorCode, errString)
                }

                override fun onAuthenticationFailed() {
                    // User's biometric doesn't match
                }
            }
        )

        // Build prompt info
        val promptInfo = BiometricPrompt.PromptInfo.Builder()
            .setTitle("Biometric Authentication")
            .setSubtitle("Authenticate to continue")
            .setNegativeButtonText("Cancel")
            .setAllowedAuthenticators(BiometricManager.Authenticators.BIOMETRIC_STRONG)
            .build()

        // Show prompt
        biometricPrompt.authenticate(promptInfo)
    }

    fun authenticateWithCrypto(
        cryptoObject: BiometricPrompt.CryptoObject,
        onSuccess: (BiometricPrompt.AuthenticationResult) -> Unit,
        onError: (Int, CharSequence) -> Unit
    ) {
        val executor = ContextCompat.getMainExecutor(activity)
        val biometricPrompt = BiometricPrompt(
            activity,
            executor,
            object : BiometricPrompt.AuthenticationCallback() {
                override fun onAuthenticationSucceeded(result: BiometricPrompt.AuthenticationResult) {
                    // Use result.cryptoObject?.cipher to encrypt/decrypt
                    onSuccess(result)
                }

                override fun onAuthenticationError(errorCode: Int, errString: CharSequence) {
                    onError(errorCode, errString)
                }
            }
        )

        val promptInfo = BiometricPrompt.PromptInfo.Builder()
            .setTitle("Biometric Authentication")
            .setSubtitle("Authenticate to access secure data")
            .setNegativeButtonText("Cancel")
            .setAllowedAuthenticators(BiometricManager.Authenticators.BIOMETRIC_STRONG)
            .build()

        biometricPrompt.authenticate(promptInfo, cryptoObject)
    }
}
```

---

## 4. REST API Specification

### 4.1 Enrollment API

```yaml
openapi: 3.0.0
info:
  title: WIA-SEC-007 Biometric Authentication API
  version: 1.0.0
paths:
  /api/v1/biometric/enroll:
    post:
      summary: Enroll new biometric template
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                user_id:
                  type: string
                modality:
                  type: string
                  enum: [fingerprint, iris, face]
                template:
                  type: object
                  description: Encrypted biometric template
                quality:
                  type: number
                  minimum: 0
                  maximum: 1
      responses:
        '200':
          description: Enrollment successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  template_id:
                    type: string
                  enrolled_at:
                    type: string
                    format: date-time
        '400':
          description: Invalid template or poor quality
        '409':
          description: Template already exists
```

### 4.2 Authentication API

```yaml
  /api/v1/biometric/authenticate:
    post:
      summary: Authenticate user with biometric
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                user_id:
                  type: string
                modality:
                  type: string
                  enum: [fingerprint, iris, face]
                template:
                  type: object
                  description: Captured biometric template
                challenge_response:
                  type: string
                  description: Optional challenge signature
      responses:
        '200':
          description: Authentication successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  success:
                    type: boolean
                  score:
                    type: number
                  jwt_token:
                    type: string
                    description: Session token
        '401':
          description: Authentication failed
        '403':
          description: Account locked
```

---

## 5. Enterprise Integration

### 5.1 Active Directory Integration

```csharp
using System.DirectoryServices.AccountManagement;

public class BiometricADIntegration
{
    public bool AuthenticateWithAD(string username, byte[] biometricTemplate)
    {
        // 1. Authenticate biometrically
        bool biometricMatch = BiometricMatcher.Match(username, biometricTemplate);

        if (!biometricMatch)
        {
            return false;
        }

        // 2. Validate AD account
        using (PrincipalContext context = new PrincipalContext(ContextType.Domain, "COMPANY.LOCAL"))
        {
            UserPrincipal user = UserPrincipal.FindByIdentity(context, username);

            if (user == null || !user.Enabled)
            {
                return false;  // User not found or disabled
            }

            // Check account policies
            if (user.IsAccountLockedOut())
            {
                return false;  // Account locked
            }

            // 3. Log authentication event in AD
            LogADEvent(username, "Biometric authentication successful");

            return true;
        }
    }

    private void LogADEvent(string username, string message)
    {
        // Log to Windows Event Log
        EventLog.WriteEntry(
            "BiometricAuth",
            $"User: {username}, Event: {message}",
            EventLogEntryType.Information
        );
    }
}
```

### 5.2 LDAP Integration

```python
import ldap

def authenticate_ldap_biometric(username, biometric_template, ldap_server='ldap://company.local'):
    """
    Authenticate user with biometric + LDAP
    """
    # 1. Verify biometric
    if not verify_biometric(username, biometric_template):
        return False

    # 2. Bind to LDAP
    try:
        conn = ldap.initialize(ldap_server)
        conn.set_option(ldap.OPT_REFERRALS, 0)

        # Search for user
        base_dn = 'dc=company,dc=local'
        search_filter = f'(sAMAccountName={username})'
        result = conn.search_s(base_dn, ldap.SCOPE_SUBTREE, search_filter)

        if not result:
            return False  # User not found

        user_dn = result[0][0]
        user_attrs = result[0][1]

        # Check if account is enabled
        user_account_control = int(user_attrs.get('userAccountControl', [b'0'])[0])
        if user_account_control & 0x0002:  # ACCOUNTDISABLE flag
            return False  # Account disabled

        # 3. Log event in LDAP
        conn.modify_s(user_dn, [(ldap.MOD_ADD, 'description', [b'Biometric auth success'])])

        conn.unbind_s()
        return True

    except ldap.LDAPError as e:
        print(f"LDAP Error: {e}")
        return False
```

---

## 6. Compliance & Certification

### 6.1 ISO/IEC 19795 Testing

```python
def iso_19795_performance_test(genuine_scores, impostor_scores):
    """
    Conduct ISO/IEC 19795 compliant biometric performance testing
    """
    # Part 1: Test subjects
    num_genuine = len(genuine_scores)
    num_impostor = len(impostor_scores)

    print(f"Genuine attempts: {num_genuine}")
    print(f"Impostor attempts: {num_impostor}")

    # Part 2: Performance metrics
    thresholds = np.linspace(
        min(impostor_scores.min(), genuine_scores.min()),
        max(impostor_scores.max(), genuine_scores.max()),
        1000
    )

    far_list = []
    frr_list = []

    for t in thresholds:
        far = np.sum(impostor_scores >= t) / num_impostor
        frr = np.sum(genuine_scores < t) / num_genuine
        far_list.append(far)
        frr_list.append(frr)

    # Find EER
    eer_idx = np.argmin(np.abs(np.array(far_list) - np.array(frr_list)))
    eer = (far_list[eer_idx] + frr_list[eer_idx]) / 2

    # Part 3: Report
    report = {
        "eer": eer,
        "far_at_001_frr": far_list[np.argmin(np.abs(np.array(frr_list) - 0.001))],
        "frr_at_001_far": frr_list[np.argmin(np.abs(np.array(far_list) - 0.001))],
        "num_genuine_attempts": num_genuine,
        "num_impostor_attempts": num_impostor
    }

    return report
```

---

## 7. Example: Complete System Integration

```python
from fastapi import FastAPI, HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt

app = FastAPI()
security = HTTPBearer()

# Complete biometric authentication system
@app.post('/api/v1/auth/biometric/login')
async def biometric_login(request: dict):
    """
    Complete biometric login flow
    """
    username = request['username']
    template = request['template']
    modality = request['modality']

    # 1. Validate input
    if not username or not template:
        raise HTTPException(status_code=400, detail='Missing credentials')

    # 2. Quality check
    quality = assess_quality(template, modality)
    if quality < 0.6:
        raise HTTPException(status_code=400, detail='Poor quality biometric sample')

    # 3. Liveness detection
    if not detect_liveness(template, modality):
        raise HTTPException(status_code=403, detail='Failed liveness check')

    # 4. Retrieve enrolled template
    enrolled = await retrieve_template_from_db(username, modality)
    if not enrolled:
        raise HTTPException(status_code=404, detail='User not enrolled')

    # 5. Match templates
    score, is_match = match_templates(template, enrolled, modality)

    if not is_match:
        # Log failed attempt
        await log_failed_attempt(username, modality, score)
        raise HTTPException(status_code=401, detail='Biometric match failed')

    # 6. Check account status (enterprise integration)
    if not await check_ldap_account(username):
        raise HTTPException(status_code=403, detail='Account disabled')

    # 7. Generate JWT token
    token = jwt.encode(
        {
            'username': username,
            'modality': modality,
            'exp': datetime.utcnow() + timedelta(hours=1)
        },
        secret_key='your-secret-key',
        algorithm='HS256'
    )

    # 8. Log successful authentication
    await log_successful_auth(username, modality, score)

    return {
        'success': True,
        'token': token,
        'score': score,
        'modality': modality
    }

@app.get('/api/v1/protected')
async def protected_route(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Protected route requiring biometric authentication
    """
    try:
        payload = jwt.decode(credentials.credentials, 'your-secret-key', algorithms=['HS256'])
        return {
            'message': 'Access granted',
            'user': payload['username'],
            'modality': payload['modality']
        }
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail='Token expired')
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail='Invalid token')
```

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
Licensed under MIT License
