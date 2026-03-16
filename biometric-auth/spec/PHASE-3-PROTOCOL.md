# WIA-SEC-007: Biometric Authentication - Phase 3: Protocols & Security

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## 1. Overview

This specification defines secure enrollment and authentication protocols for biometric systems, including template protection, secure communication, and anti-spoofing measures.

---

## 2. Enrollment Protocol

### 2.1 Enrollment Flow

```
User → Capture Device → Quality Check → Feature Extraction → Template Protection → Secure Storage
```

#### 2.1.1 Step-by-Step Enrollment

```python
def enroll_user(user_id, modality='fingerprint', num_samples=3):
    """
    Complete biometric enrollment protocol
    """
    samples = []

    # Step 1: Capture multiple samples
    for i in range(num_samples):
        print(f"Capture sample {i+1}/{num_samples}")
        sample = capture_biometric(modality)

        # Step 2: Quality check
        quality = assess_quality(sample, modality)
        if quality < 0.6:
            print("Low quality. Please retry.")
            continue

        # Step 3: Liveness detection
        if not detect_liveness(sample, modality):
            print("Failed liveness check. Please retry.")
            continue

        samples.append(sample)

    if len(samples) < num_samples:
        return {"success": False, "reason": "Insufficient quality samples"}

    # Step 4: Feature extraction
    features_list = [extract_features(s, modality) for s in samples]

    # Step 5: Template generation (combine multiple samples)
    template = generate_template(features_list, modality)

    # Step 6: Template protection (cancelable biometrics)
    protected_template = protect_template(template, user_id)

    # Step 7: Encryption
    encrypted_template = encrypt_template(protected_template, algorithm='AES-256-GCM')

    # Step 8: Secure storage
    template_id = store_template(user_id, encrypted_template, modality)

    return {
        "success": True,
        "template_id": template_id,
        "modality": modality,
        "enrolled_at": datetime.utcnow().isoformat()
    }
```

### 2.2 Multi-Sample Enrollment

Combining multiple samples improves robustness:

```python
def combine_fingerprint_samples(samples):
    """
    Combine multiple fingerprint samples into single template
    """
    all_minutiae = []

    # Collect minutiae from all samples
    for sample in samples:
        minutiae = sample['minutiae']
        all_minutiae.extend(minutiae)

    # Cluster minutiae to remove duplicates
    clustered = cluster_minutiae(all_minutiae, distance_threshold=2.0)

    # Select most reliable minutiae (highest quality, most frequent)
    final_minutiae = select_reliable_minutiae(clustered, top_n=40)

    return {
        'minutiae': final_minutiae,
        'num_samples': len(samples),
        'quality': np.mean([s['quality'] for s in samples])
    }
```

---

## 3. Authentication Protocol

### 3.1 Authentication Flow

```
User → Capture → Quality/Liveness Check → Feature Extract → Template Match → Decision
```

#### 3.1.1 Challenge-Response Authentication

```python
def authenticate_user(user_id, modality='fingerprint'):
    """
    Secure authentication with challenge-response
    """
    # Step 1: Server sends challenge (random nonce)
    challenge = generate_random_nonce(32)  # 32 bytes
    send_to_client(challenge)

    # Step 2: Client captures biometric
    sample = capture_biometric(modality)

    # Quality check
    if assess_quality(sample, modality) < 0.6:
        return {"success": False, "reason": "Poor quality sample"}

    # Liveness check
    if not detect_liveness(sample, modality):
        return {"success": False, "reason": "Failed liveness detection"}

    # Step 3: Extract features
    captured_template = extract_features(sample, modality)

    # Step 4: Retrieve enrolled template from secure storage
    enrolled_template = retrieve_template(user_id, modality)
    if not enrolled_template:
        return {"success": False, "reason": "User not enrolled"}

    # Decrypt template
    decrypted_template = decrypt_template(enrolled_template)

    # Step 5: Match templates
    score, is_match = match_templates(captured_template, decrypted_template, modality)

    if not is_match:
        return {"success": False, "reason": "Biometric match failed", "score": score}

    # Step 6: Sign challenge with private key (optional for strong auth)
    signature = sign_with_private_key(challenge, user_private_key)

    # Step 7: Log authentication
    log_authentication(user_id, success=True, modality=modality, score=score)

    return {
        "success": True,
        "user_id": user_id,
        "score": score,
        "signature": signature,
        "timestamp": datetime.utcnow().isoformat()
    }
```

### 3.2 Continuous Authentication

For high-security applications, continuously verify user identity:

```python
def continuous_authentication(user_id, modality='face', interval_seconds=60):
    """
    Periodic re-authentication in background
    """
    while True:
        time.sleep(interval_seconds)

        # Capture biometric without user interaction (e.g., face from webcam)
        sample = capture_biometric_silent(modality)

        if sample is None:
            continue  # User not in frame

        # Authenticate
        result = authenticate_user(user_id, modality)

        if not result['success']:
            # Lock session
            lock_user_session(user_id)
            notify_user("Session locked. Please re-authenticate.")
            break
```

---

## 4. Template Protection

### 4.1 Cancelable Biometrics (BioHashing)

```python
import hashlib
import numpy as np

def bio_hash(template, user_token):
    """
    Generate cancelable biometric template using BioHashing

    Args:
        template: Original biometric feature vector
        user_token: User-specific secret (e.g., password, PIN, or random key)

    Returns:
        Cancelable template (can be revoked by changing user_token)
    """
    # Generate pseudo-random matrix from user token
    seed = int(hashlib.sha256(user_token.encode()).hexdigest(), 16) % (2**32)
    np.random.seed(seed)
    random_matrix = np.random.randn(len(template), len(template))

    # Project template onto random subspace
    projected = np.dot(random_matrix, template)

    # Binarize (sign function)
    bio_hash_code = (projected > 0).astype(np.uint8)

    return bio_hash_code
```

### 4.2 Fuzzy Vault

```python
def create_fuzzy_vault(template, secret_key, num_chaff=200):
    """
    Encode secret key using fuzzy vault scheme

    Args:
        template: Genuine biometric features (minutiae, iris code, etc.)
        secret_key: Secret to protect
        num_chaff: Number of chaff points to add

    Returns:
        Vault containing genuine + chaff points
    """
    # Encode secret as polynomial coefficients
    poly_coeffs = encode_secret(secret_key)

    # Evaluate polynomial at genuine feature points
    vault = []
    for feature in template:
        x = feature['x']
        y = evaluate_polynomial(poly_coeffs, x)
        vault.append({'x': x, 'y': y, 'genuine': True})

    # Add chaff points (random, not on polynomial)
    for _ in range(num_chaff):
        x = np.random.uniform(0, 100)
        y = np.random.uniform(0, 100)
        vault.append({'x': x, 'y': y, 'genuine': False})

    # Shuffle vault
    np.random.shuffle(vault)

    return vault

def unlock_fuzzy_vault(vault, query_template, threshold=8):
    """
    Unlock vault using query template

    Returns secret if enough genuine points match
    """
    matched_points = []

    for feature in query_template:
        for vault_point in vault:
            if distance(feature, vault_point) < 5.0:  # Close enough
                matched_points.append(vault_point)

    if len(matched_points) < threshold:
        return None  # Failed to unlock

    # Reconstruct polynomial using matched points
    poly_coeffs = reconstruct_polynomial(matched_points)

    # Decode secret
    secret = decode_secret(poly_coeffs)

    return secret
```

### 4.3 Homomorphic Encryption

```python
from phe import paillier  # Partially homomorphic encryption

def encrypt_template_homomorphic(template):
    """
    Encrypt template with Paillier homomorphic encryption
    Allows matching in encrypted domain
    """
    # Generate keypair
    public_key, private_key = paillier.generate_paillier_keypair()

    # Encrypt each feature value
    encrypted_template = [public_key.encrypt(float(x)) for x in template]

    return encrypted_template, public_key, private_key

def match_encrypted_templates(enc_template1, enc_template2, public_key):
    """
    Compute similarity between encrypted templates without decryption
    """
    # Compute encrypted distance
    encrypted_distance = sum([
        (e1 - e2) ** 2
        for e1, e2 in zip(enc_template1, enc_template2)
    ])

    # (Distance must be decrypted by private key holder to make decision)
    return encrypted_distance
```

---

## 5. Secure Communication

### 5.1 TLS/SSL for Template Transmission

```python
import ssl
import socket

def send_template_secure(template, server_address, port=8443):
    """
    Send biometric template over TLS 1.3
    """
    # Create SSL context
    context = ssl.create_default_context()
    context.minimum_version = ssl.TLSVersion.TLSv1_3

    # Serialize template
    template_json = json.dumps(template)

    # Connect with TLS
    with socket.create_connection((server_address, port)) as sock:
        with context.wrap_socket(sock, server_hostname=server_address) as ssock:
            # Send encrypted template
            ssock.sendall(template_json.encode('utf-8'))

            # Receive acknowledgment
            response = ssock.recv(1024).decode('utf-8')
            return json.loads(response)
```

### 5.2 Mutual TLS (mTLS) for Device Authentication

```python
def mutual_tls_connection(client_cert, client_key, server_cert, server_address):
    """
    Establish mTLS connection for device authentication
    """
    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.load_cert_chain(certfile=client_cert, keyfile=client_key)
    context.load_verify_locations(cafile=server_cert)
    context.check_hostname = True

    with socket.create_connection((server_address, 8443)) as sock:
        with context.wrap_socket(sock, server_hostname=server_address) as ssock:
            # Both client and server authenticated
            print(f"Connected with {ssock.version()}")
            print(f"Cipher: {ssock.cipher()}")
            return ssock
```

---

## 6. Anti-Spoofing Protocols

### 6.1 Challenge-Response Liveness

```python
def challenge_response_liveness(modality='face'):
    """
    Request user to perform specific action to prove liveness
    """
    if modality == 'face':
        actions = ['blink', 'smile', 'turn_left', 'turn_right', 'open_mouth']
        challenge_action = np.random.choice(actions)

        print(f"Please {challenge_action}")

        # Capture video
        video = capture_video(duration=3)

        # Verify action performed
        if detect_action(video, challenge_action):
            return True
        else:
            return False

    elif modality == 'fingerprint':
        # Request user to move finger
        print("Please slide your finger")
        samples = capture_fingerprint_sequence(num_frames=5)

        # Verify motion consistency
        if verify_finger_motion(samples):
            return True
        else:
            return False
```

### 6.2 Multi-Spectral Imaging

```python
def multispectral_liveness_check(fingerprint_image):
    """
    Use multiple wavelengths to detect live skin vs fake
    """
    # Capture at multiple wavelengths
    visible_image = capture_wavelength(wavelength=550)  # Green light
    nir_image = capture_wavelength(wavelength=850)       # Near-infrared

    # Compute spectral features
    visible_features = extract_spectral_features(visible_image)
    nir_features = extract_spectral_features(nir_image)

    # Classify as live or fake using ML model
    features = np.concatenate([visible_features, nir_features])
    liveness_score = liveness_classifier.predict_proba(features)[0][1]

    return liveness_score > 0.8  # Threshold
```

---

## 7. Privacy-Preserving Protocols

### 7.1 Zero-Knowledge Proof

```python
def zkp_biometric_authentication(user_id):
    """
    Prove identity without revealing biometric data
    Using zero-knowledge proof
    """
    # Step 1: Prover (client) commits to biometric template
    commitment = commit_to_template(biometric_template)
    send_to_server(commitment)

    # Step 2: Verifier (server) sends challenge
    challenge = receive_from_server()

    # Step 3: Prover generates proof
    proof = generate_zkp_proof(biometric_template, challenge)
    send_to_server(proof)

    # Step 4: Verifier checks proof
    is_valid = server_verify_zkp(commitment, challenge, proof)

    return is_valid
```

### 7.2 Secure Multi-Party Computation (SMPC)

```python
def smpc_biometric_matching(client_template, server_template):
    """
    Match templates using SMPC without revealing either template
    """
    # Split templates into shares
    client_shares = secret_share(client_template, num_parties=2)
    server_shares = secret_share(server_template, num_parties=2)

    # Each party computes on their shares
    party1_result = compute_similarity_share(client_shares[0], server_shares[0])
    party2_result = compute_similarity_share(client_shares[1], server_shares[1])

    # Combine results
    similarity = reconstruct_secret(party1_result, party2_result)

    return similarity
```

---

## 8. Audit & Logging

### 8.1 Authentication Event Logging

```python
def log_authentication_event(user_id, success, modality, score, reason=None):
    """
    Log all authentication attempts for audit
    """
    event = {
        "timestamp": datetime.utcnow().isoformat(),
        "user_id": hash_user_id(user_id),  # Hash for privacy
        "modality": modality,
        "success": success,
        "score": score,
        "reason": reason,
        "ip_address": get_client_ip(),
        "device_id": get_device_id()
    }

    # Store in secure audit log
    audit_log.append(event)

    # Alert on suspicious activity
    if is_suspicious(event):
        alert_security_team(event)
```

### 8.2 Tamper-Evident Logs

```python
import hashlib

class TamperEvidentLog:
    def __init__(self):
        self.log = []
        self.previous_hash = "0" * 64  # Genesis hash

    def append(self, event):
        """
        Append event with hash chain
        """
        event_hash = hashlib.sha256(
            (json.dumps(event) + self.previous_hash).encode()
        ).hexdigest()

        log_entry = {
            "event": event,
            "previous_hash": self.previous_hash,
            "hash": event_hash
        }

        self.log.append(log_entry)
        self.previous_hash = event_hash

    def verify_integrity(self):
        """
        Verify log has not been tampered with
        """
        prev_hash = "0" * 64

        for entry in self.log:
            computed_hash = hashlib.sha256(
                (json.dumps(entry['event']) + prev_hash).encode()
            ).hexdigest()

            if computed_hash != entry['hash']:
                return False  # Tampered!

            prev_hash = entry['hash']

        return True  # Intact
```

---

## 9. Example: Complete Secure Protocol

```python
def secure_biometric_system(user_id, mode='enroll'):
    """
    Complete secure biometric authentication system
    """
    if mode == 'enroll':
        # Enrollment
        result = enroll_user(user_id, modality='fingerprint', num_samples=3)
        log_authentication_event(user_id, result['success'], 'fingerprint', None)
        return result

    elif mode == 'authenticate':
        # Authentication with all security measures
        sample = capture_biometric('fingerprint')

        # 1. Quality check
        if assess_quality(sample, 'fingerprint') < 0.6:
            log_authentication_event(user_id, False, 'fingerprint', 0, 'Poor quality')
            return {"success": False, "reason": "Poor quality"}

        # 2. Liveness detection
        if not challenge_response_liveness('fingerprint'):
            log_authentication_event(user_id, False, 'fingerprint', 0, 'Failed liveness')
            return {"success": False, "reason": "Failed liveness detection"}

        # 3. Feature extraction
        features = extract_features(sample, 'fingerprint')

        # 4. Template protection (BioHashing)
        user_token = retrieve_user_token(user_id)
        protected_features = bio_hash(features, user_token)

        # 5. Retrieve enrolled template
        enrolled_template = retrieve_template(user_id, 'fingerprint')
        decrypted = decrypt_template(enrolled_template)

        # 6. Match
        score, is_match = match_templates(protected_features, decrypted, 'fingerprint')

        # 7. Log
        log_authentication_event(user_id, is_match, 'fingerprint', score)

        return {
            "success": is_match,
            "score": score,
            "timestamp": datetime.utcnow().isoformat()
        }
```

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
Licensed under MIT License
