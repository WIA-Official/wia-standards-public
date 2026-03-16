# WIA-SEC-007: Biometric Authentication - Phase 2: Data Processing & Algorithms

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01
**Primary Color:** #8B5CF6 (Purple - Security)

---

## 1. Overview

This specification defines data processing algorithms for biometric authentication, including feature extraction, matching algorithms, and performance metrics (FAR/FRR).

---

## 2. Feature Extraction

### 2.1 Fingerprint Feature Extraction

#### 2.1.1 Image Enhancement

```python
def enhance_fingerprint(image):
    """
    Enhance fingerprint image quality
    Steps:
    1. Normalization (mean=0, variance=1)
    2. Orientation field estimation
    3. Frequency domain filtering (Gabor filters)
    4. Binarization and thinning
    """
    # Normalize image
    normalized = (image - np.mean(image)) / np.std(image)

    # Estimate ridge orientation
    orientation = estimate_orientation(normalized)

    # Apply Gabor filters
    enhanced = gabor_filter(normalized, orientation)

    # Binarize and thin
    binary = binarize(enhanced)
    thinned = thin_ridges(binary)

    return thinned
```

#### 2.1.2 Minutiae Extraction

```python
def extract_minutiae(thinned_image):
    """
    Extract ridge endings and bifurcations
    """
    minutiae = []

    for y in range(1, height-1):
        for x in range(1, width-1):
            if thinned_image[y, x] == 1:
                # Count neighbors using crossing number
                cn = crossing_number(thinned_image, x, y)

                if cn == 1:  # Ridge ending
                    angle = estimate_ridge_direction(thinned_image, x, y)
                    minutiae.append({
                        'x': x, 'y': y,
                        'type': 'ending',
                        'angle': angle
                    })
                elif cn == 3:  # Ridge bifurcation
                    angle = estimate_ridge_direction(thinned_image, x, y)
                    minutiae.append({
                        'x': x, 'y': y,
                        'type': 'bifurcation',
                        'angle': angle
                    })

    return minutiae
```

### 2.2 Iris Feature Extraction (Daugman's Algorithm)

```python
def extract_iris_code(iris_image):
    """
    Extract 2048-bit IrisCode using Daugman's algorithm
    """
    # Normalize iris to rectangular coordinates
    normalized = normalize_iris(iris_image, pupil_center, iris_center)

    # Apply 2D Gabor wavelets
    filters = create_gabor_filters()
    iris_code = np.zeros((256, 8), dtype=np.uint8)  # 2048 bits

    for i, (filter_real, filter_imag) in enumerate(filters):
        # Convolve with Gabor filter
        response_real = convolve(normalized, filter_real)
        response_imag = convolve(normalized, filter_imag)

        # Quantize phase
        phase = np.arctan2(response_imag, response_real)
        iris_code[i] = (phase > 0).astype(np.uint8)

    # Generate mask for eyelids/eyelashes
    mask = generate_mask(iris_image)

    return iris_code, mask
```

### 2.3 Face Feature Extraction (Deep Learning)

```python
import torch
import torchvision

def extract_face_embedding(face_image):
    """
    Extract 512-D face embedding using ArcFace or FaceNet
    """
    # Load pre-trained model
    model = load_arcface_model()
    model.eval()

    # Preprocess face image
    face_aligned = align_face(face_image)  # Align to canonical pose
    face_tensor = preprocess_face(face_aligned)

    # Extract embedding
    with torch.no_grad():
        embedding = model(face_tensor)
        # L2 normalize
        embedding = embedding / torch.norm(embedding, p=2)

    return embedding.cpu().numpy()  # 512-D vector
```

---

## 3. Matching Algorithms

### 3.1 Fingerprint Matching

```python
def match_fingerprints(template1, template2, threshold=40):
    """
    Match two fingerprint templates using minutiae correspondence

    Returns:
        match_score (int): Number of matching minutiae
        is_match (bool): True if score >= threshold
    """
    minutiae1 = template1['minutiae']
    minutiae2 = template2['minutiae']

    matches = 0
    max_distance = 5.0  # mm
    max_angle_diff = 15  # degrees

    for m1 in minutiae1:
        for m2 in minutiae2:
            # Check spatial distance
            dist = np.sqrt((m1['x'] - m2['x'])**2 + (m1['y'] - m2['y'])**2)

            # Check angle difference
            angle_diff = abs(m1['angle'] - m2['angle'])
            if angle_diff > 180:
                angle_diff = 360 - angle_diff

            if dist < max_distance and angle_diff < max_angle_diff:
                matches += 1
                break

    is_match = matches >= threshold
    return matches, is_match
```

### 3.2 Iris Matching (Hamming Distance)

```python
def match_iris(template1, template2, threshold=0.32):
    """
    Match two iris codes using normalized Hamming distance

    Returns:
        hamming_distance (float): Normalized HD (0-1)
        is_match (bool): True if HD <= threshold
    """
    code1 = template1['irisCode']
    code2 = template2['irisCode']
    mask1 = template1['maskCode']
    mask2 = template2['maskCode']

    # XOR to find differing bits
    xor = np.bitwise_xor(code1, code2)

    # AND masks to find valid bits
    valid_mask = np.bitwise_and(mask1, mask2)

    # Count mismatches in valid regions
    mismatches = np.sum(np.bitwise_and(xor, valid_mask))
    valid_bits = np.sum(valid_mask)

    # Normalized Hamming distance
    hamming_distance = mismatches / valid_bits if valid_bits > 0 else 1.0

    is_match = hamming_distance <= threshold
    return hamming_distance, is_match
```

### 3.3 Face Matching (Cosine Similarity)

```python
def match_faces(template1, template2, threshold=0.6):
    """
    Match two face embeddings using cosine similarity

    Returns:
        similarity (float): Cosine similarity (0-1)
        is_match (bool): True if similarity >= threshold
    """
    emb1 = np.array(template1['embedding'])
    emb2 = np.array(template2['embedding'])

    # Cosine similarity (already L2-normalized)
    similarity = np.dot(emb1, emb2)

    is_match = similarity >= threshold
    return similarity, is_match
```

---

## 4. Performance Metrics

### 4.1 False Accept Rate (FAR)

**Definition**: Probability that the system incorrectly accepts an impostor.

```
FAR = (Number of false accepts) / (Number of impostor attempts)
```

**Typical values**:
- Low security: FAR = 0.1% (1 in 1,000)
- Medium security: FAR = 0.01% (1 in 10,000)
- High security: FAR = 0.001% (1 in 100,000)

### 4.2 False Reject Rate (FRR)

**Definition**: Probability that the system incorrectly rejects a genuine user.

```
FRR = (Number of false rejects) / (Number of genuine attempts)
```

**Typical values**:
- Convenient: FRR = 1-5%
- Balanced: FRR = 0.1-1%
- Strict: FRR = 0.01-0.1%

### 4.3 Equal Error Rate (EER)

**Definition**: The point where FAR = FRR. Lower EER indicates better overall performance.

**Typical EER values**:
- Fingerprint: 0.01% - 1%
- Iris: 0.001% - 0.1%
- Face (2D): 1% - 5%
- Face (3D): 0.1% - 1%

### 4.4 ROC Curve (Receiver Operating Characteristic)

```python
import matplotlib.pyplot as plt
from sklearn.metrics import roc_curve, auc

def plot_roc_curve(genuine_scores, impostor_scores):
    """
    Plot FAR vs FRR curve
    """
    # Combine scores with labels
    scores = np.concatenate([genuine_scores, impostor_scores])
    labels = np.concatenate([
        np.ones(len(genuine_scores)),    # Genuine = 1
        np.zeros(len(impostor_scores))   # Impostor = 0
    ])

    # Compute ROC curve
    fpr, tpr, thresholds = roc_curve(labels, scores)
    roc_auc = auc(fpr, tpr)

    # Plot
    plt.figure()
    plt.plot(fpr, tpr, label=f'ROC curve (AUC = {roc_auc:.4f})')
    plt.plot([0, 1], [0, 1], 'k--', label='Random')
    plt.xlabel('False Accept Rate (FAR)')
    plt.ylabel('True Accept Rate (1 - FRR)')
    plt.title('ROC Curve')
    plt.legend()
    plt.grid()
    plt.show()
```

### 4.5 DET Curve (Detection Error Tradeoff)

```python
def plot_det_curve(genuine_scores, impostor_scores):
    """
    Plot FAR vs FRR on log-log scale
    """
    from scipy.stats import norm

    # Compute FAR and FRR at different thresholds
    thresholds = np.linspace(min(impostor_scores), max(genuine_scores), 1000)
    far_list = []
    frr_list = []

    for t in thresholds:
        far = np.sum(impostor_scores >= t) / len(impostor_scores)
        frr = np.sum(genuine_scores < t) / len(genuine_scores)
        far_list.append(far)
        frr_list.append(frr)

    # Plot on normal deviate scale
    plt.figure()
    plt.plot(norm.ppf(far_list), norm.ppf(frr_list))
    plt.xlabel('False Accept Rate (%)')
    plt.ylabel('False Reject Rate (%)')
    plt.title('DET Curve')
    plt.grid()
    plt.show()
```

---

## 5. Threshold Selection

### 5.1 Security-Centric Threshold

Minimize FAR (prioritize security):

```python
def select_security_threshold(genuine_scores, impostor_scores, target_far=0.001):
    """
    Select threshold to achieve target FAR
    """
    impostor_scores_sorted = np.sort(impostor_scores)[::-1]
    idx = int(len(impostor_scores_sorted) * target_far)
    threshold = impostor_scores_sorted[idx]

    frr = np.sum(genuine_scores < threshold) / len(genuine_scores)

    print(f"Threshold: {threshold}")
    print(f"FAR: {target_far*100:.4f}%")
    print(f"FRR: {frr*100:.4f}%")

    return threshold
```

### 5.2 Convenience-Centric Threshold

Minimize FRR (prioritize convenience):

```python
def select_convenience_threshold(genuine_scores, impostor_scores, target_frr=0.01):
    """
    Select threshold to achieve target FRR
    """
    genuine_scores_sorted = np.sort(genuine_scores)
    idx = int(len(genuine_scores_sorted) * target_frr)
    threshold = genuine_scores_sorted[idx]

    far = np.sum(impostor_scores >= threshold) / len(impostor_scores)

    print(f"Threshold: {threshold}")
    print(f"FAR: {far*100:.4f}%")
    print(f"FRR: {target_frr*100:.4f}%")

    return threshold
```

### 5.3 Balanced Threshold (EER)

Minimize EER (balance security and convenience):

```python
def select_eer_threshold(genuine_scores, impostor_scores):
    """
    Select threshold at Equal Error Rate (EER)
    """
    thresholds = np.linspace(
        min(impostor_scores.min(), genuine_scores.min()),
        max(impostor_scores.max(), genuine_scores.max()),
        1000
    )

    best_threshold = None
    min_eer = float('inf')

    for t in thresholds:
        far = np.sum(impostor_scores >= t) / len(impostor_scores)
        frr = np.sum(genuine_scores < t) / len(genuine_scores)
        eer = abs(far - frr)

        if eer < min_eer:
            min_eer = eer
            best_threshold = t

    final_far = np.sum(impostor_scores >= best_threshold) / len(impostor_scores)
    final_frr = np.sum(genuine_scores < best_threshold) / len(genuine_scores)

    print(f"EER Threshold: {best_threshold}")
    print(f"FAR: {final_far*100:.4f}%")
    print(f"FRR: {final_frr*100:.4f}%")
    print(f"EER: {(final_far + final_frr) / 2 * 100:.4f}%")

    return best_threshold
```

---

## 6. Multi-Modal Fusion

Combining multiple biometric modalities improves accuracy:

### 6.1 Score-Level Fusion

```python
def fuse_scores(fingerprint_score, face_score, weights=[0.6, 0.4]):
    """
    Weighted fusion of fingerprint and face scores
    """
    # Normalize scores to [0, 1]
    fp_norm = normalize_score(fingerprint_score, method='min-max')
    face_norm = normalize_score(face_score, method='min-max')

    # Weighted sum
    fused_score = weights[0] * fp_norm + weights[1] * face_norm

    return fused_score

def normalize_score(score, method='min-max', score_range=(0, 100)):
    """
    Normalize score to [0, 1]
    """
    if method == 'min-max':
        return (score - score_range[0]) / (score_range[1] - score_range[0])
    elif method == 'z-score':
        return (score - mean) / std
    else:
        raise ValueError(f"Unknown normalization method: {method}")
```

### 6.2 Decision-Level Fusion

```python
def fuse_decisions(fp_match, iris_match, face_match, rule='majority'):
    """
    Fuse binary decisions from multiple modalities
    """
    decisions = [fp_match, iris_match, face_match]

    if rule == 'AND':
        return all(decisions)
    elif rule == 'OR':
        return any(decisions)
    elif rule == 'majority':
        return sum(decisions) >= len(decisions) / 2
    else:
        raise ValueError(f"Unknown fusion rule: {rule}")
```

---

## 7. Example: Complete Matching Pipeline

```python
def authenticate_user(captured_template, enrolled_template, modality='fingerprint'):
    """
    Complete authentication pipeline
    """
    # 1. Quality check
    quality = check_quality(captured_template)
    if quality < 0.6:
        return {
            'success': False,
            'reason': 'Poor quality sample',
            'quality': quality
        }

    # 2. Feature extraction
    features = extract_features(captured_template, modality)

    # 3. Liveness detection
    liveness_score = detect_liveness(captured_template, modality)
    if liveness_score < 0.8:
        return {
            'success': False,
            'reason': 'Failed liveness detection',
            'liveness_score': liveness_score
        }

    # 4. Matching
    if modality == 'fingerprint':
        score, is_match = match_fingerprints(features, enrolled_template)
    elif modality == 'iris':
        score, is_match = match_iris(features, enrolled_template)
    elif modality == 'face':
        score, is_match = match_faces(features, enrolled_template)

    # 5. Return result
    return {
        'success': is_match,
        'score': score,
        'quality': quality,
        'liveness_score': liveness_score,
        'timestamp': datetime.utcnow().isoformat()
    }
```

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association (WIA)
Licensed under MIT License
