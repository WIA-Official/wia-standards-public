# PHASE 2: Algorithm Specification

## WIA-PET-EMOTION Detection Algorithms

> "Algorithms that bridge the gap between species through emotional understanding."

---

## 1. Multi-Modal Emotion Detection

### 1.1 Fusion Architecture

```python
from dataclasses import dataclass
from typing import List, Optional, Dict
from enum import Enum
import numpy as np

class EmotionType(Enum):
    HAPPY = "HAPPY"
    SAD = "SAD"
    ANXIOUS = "ANXIOUS"
    CALM = "CALM"
    EXCITED = "EXCITED"
    FEARFUL = "FEARFUL"
    AFFECTIONATE = "AFFECTIONATE"
    FRUSTRATED = "FRUSTRATED"
    PLAYFUL = "PLAYFUL"
    CONTENT = "CONTENT"
    CURIOUS = "CURIOUS"
    AGGRESSIVE = "AGGRESSIVE"
    BORED = "BORED"
    TIRED = "TIRED"
    ALERT = "ALERT"

@dataclass
class ModalityScore:
    modality: str
    emotion_scores: Dict[EmotionType, float]
    confidence: float
    weight: float

class MultiModalEmotionFusion:
    """
    Fuses emotion predictions from multiple data modalities
    using weighted late fusion with attention mechanism.
    """

    def __init__(self):
        self.modality_weights = {
            "visual_body": 0.25,
            "visual_face": 0.20,
            "audio": 0.15,
            "physiological": 0.25,
            "behavioral": 0.15
        }
        self.attention_enabled = True

    def fuse_modalities(
        self,
        modality_scores: List[ModalityScore]
    ) -> Dict[EmotionType, float]:
        """
        Combine predictions from multiple modalities.

        Returns emotion probability distribution.
        """
        if not modality_scores:
            raise ValueError("At least one modality required")

        # Initialize combined scores
        combined = {e: 0.0 for e in EmotionType}
        total_weight = 0.0

        for modality in modality_scores:
            # Apply attention weighting based on confidence
            if self.attention_enabled:
                attention_weight = self._calculate_attention(modality)
            else:
                attention_weight = 1.0

            effective_weight = modality.weight * modality.confidence * attention_weight

            for emotion, score in modality.emotion_scores.items():
                combined[emotion] += score * effective_weight

            total_weight += effective_weight

        # Normalize
        if total_weight > 0:
            for emotion in combined:
                combined[emotion] /= total_weight

        return combined

    def _calculate_attention(self, modality: ModalityScore) -> float:
        """
        Calculate attention weight based on modality reliability.
        """
        # Higher confidence gets more attention
        base_attention = modality.confidence ** 2

        # Penalize if modality has low variance (uninformative)
        scores = list(modality.emotion_scores.values())
        variance = np.var(scores)
        informativeness = min(1.0, variance * 10)

        return base_attention * (0.5 + 0.5 * informativeness)

    def determine_primary_emotion(
        self,
        emotion_distribution: Dict[EmotionType, float],
        threshold: float = 0.3
    ) -> tuple:
        """
        Determine primary emotion from probability distribution.

        Returns (emotion, confidence, is_ambiguous)
        """
        sorted_emotions = sorted(
            emotion_distribution.items(),
            key=lambda x: x[1],
            reverse=True
        )

        primary = sorted_emotions[0]
        secondary = sorted_emotions[1] if len(sorted_emotions) > 1 else None

        # Check for ambiguity
        is_ambiguous = False
        if secondary and (primary[1] - secondary[1]) < 0.15:
            is_ambiguous = True

        # Check confidence threshold
        if primary[1] < threshold:
            return (EmotionType.UNKNOWN, primary[1], True)

        return (primary[0], primary[1], is_ambiguous)
```

---

## 2. Visual Analysis

### 2.1 Body Posture Detection

```python
@dataclass
class PostureFeatures:
    spine_angle: float          # Degrees from horizontal
    leg_extension: float        # 0-1, crouched to extended
    body_tension: float         # 0-1, relaxed to tense
    weight_distribution: str    # forward, balanced, backward
    overall_height: float       # Relative to baseline

class BodyPostureAnalyzer:
    """
    Analyzes body posture from pose estimation keypoints.
    """

    # Posture-emotion mappings
    POSTURE_EMOTIONS = {
        "relaxed_low": {EmotionType.CALM: 0.7, EmotionType.TIRED: 0.3},
        "relaxed_normal": {EmotionType.CONTENT: 0.6, EmotionType.CALM: 0.4},
        "alert_high": {EmotionType.ALERT: 0.5, EmotionType.CURIOUS: 0.3, EmotionType.EXCITED: 0.2},
        "tense_forward": {EmotionType.AGGRESSIVE: 0.4, EmotionType.ALERT: 0.3, EmotionType.ANXIOUS: 0.3},
        "crouched_back": {EmotionType.FEARFUL: 0.6, EmotionType.ANXIOUS: 0.4},
        "play_bow": {EmotionType.PLAYFUL: 0.9, EmotionType.HAPPY: 0.1},
        "submissive": {EmotionType.FEARFUL: 0.4, EmotionType.ANXIOUS: 0.3, EmotionType.SAD: 0.3}
    }

    def __init__(self, species: str):
        self.species = species
        self.keypoint_model = self._load_species_model(species)

    def analyze_posture(
        self,
        keypoints: np.ndarray,
        baseline_height: float
    ) -> tuple:
        """
        Analyze body posture from detected keypoints.

        Returns (posture_features, emotion_scores, confidence)
        """
        features = self._extract_features(keypoints, baseline_height)
        posture_class = self._classify_posture(features)
        emotion_scores = self.POSTURE_EMOTIONS.get(
            posture_class,
            {EmotionType.CALM: 1.0}
        )

        confidence = self._calculate_confidence(keypoints)

        return (features, emotion_scores, confidence)

    def _extract_features(
        self,
        keypoints: np.ndarray,
        baseline_height: float
    ) -> PostureFeatures:
        """Extract posture features from keypoints."""
        # Calculate spine angle
        spine_vector = keypoints["hip"] - keypoints["shoulder"]
        spine_angle = np.degrees(np.arctan2(spine_vector[1], spine_vector[0]))

        # Calculate leg extension
        leg_length = np.linalg.norm(keypoints["hip"] - keypoints["paw"])
        max_leg = baseline_height * 0.4
        leg_extension = min(1.0, leg_length / max_leg)

        # Estimate body tension from keypoint stability
        body_tension = self._estimate_tension(keypoints)

        # Weight distribution
        weight_dist = self._analyze_weight_distribution(keypoints)

        # Overall height relative to baseline
        current_height = keypoints["shoulder"][1]
        relative_height = current_height / baseline_height

        return PostureFeatures(
            spine_angle=spine_angle,
            leg_extension=leg_extension,
            body_tension=body_tension,
            weight_distribution=weight_dist,
            overall_height=relative_height
        )

    def _classify_posture(self, features: PostureFeatures) -> str:
        """Classify posture into predefined categories."""
        if features.leg_extension < 0.5 and features.body_tension < 0.3:
            return "relaxed_low"
        elif features.body_tension < 0.3:
            return "relaxed_normal"
        elif features.overall_height > 1.1 and features.body_tension > 0.5:
            return "alert_high"
        elif features.weight_distribution == "forward" and features.body_tension > 0.6:
            return "tense_forward"
        elif features.weight_distribution == "backward" and features.leg_extension < 0.6:
            return "crouched_back"
        elif features.spine_angle > 20 and features.weight_distribution == "forward":
            return "play_bow"
        elif features.overall_height < 0.7:
            return "submissive"
        else:
            return "relaxed_normal"

    def _load_species_model(self, species: str):
        """Load species-specific pose estimation model."""
        pass

    def _estimate_tension(self, keypoints: np.ndarray) -> float:
        """Estimate body tension from keypoint positions."""
        pass

    def _analyze_weight_distribution(self, keypoints: np.ndarray) -> str:
        """Analyze weight distribution."""
        pass

    def _calculate_confidence(self, keypoints: np.ndarray) -> float:
        """Calculate detection confidence."""
        pass
```

### 2.2 Facial Expression Analysis

```python
class FacialExpressionAnalyzer:
    """
    Analyzes pet facial expressions for emotion detection.
    """

    # Facial feature emotion mappings for dogs
    CANINE_FACIAL_EMOTIONS = {
        "relaxed_mouth_soft_eyes": {EmotionType.HAPPY: 0.7, EmotionType.CONTENT: 0.3},
        "whale_eye": {EmotionType.ANXIOUS: 0.6, EmotionType.FEARFUL: 0.4},
        "hard_stare": {EmotionType.AGGRESSIVE: 0.5, EmotionType.ALERT: 0.5},
        "squinting": {EmotionType.CONTENT: 0.5, EmotionType.AFFECTIONATE: 0.5},
        "panting_relaxed": {EmotionType.HAPPY: 0.4, EmotionType.EXCITED: 0.6},
        "lip_licking": {EmotionType.ANXIOUS: 0.7, EmotionType.SAD: 0.3},
        "yawning": {EmotionType.ANXIOUS: 0.4, EmotionType.TIRED: 0.6}
    }

    # Facial feature emotion mappings for cats
    FELINE_FACIAL_EMOTIONS = {
        "slow_blink": {EmotionType.AFFECTIONATE: 0.8, EmotionType.CONTENT: 0.2},
        "dilated_pupils": {EmotionType.EXCITED: 0.4, EmotionType.FEARFUL: 0.4, EmotionType.PLAYFUL: 0.2},
        "constricted_pupils": {EmotionType.AGGRESSIVE: 0.5, EmotionType.ALERT: 0.5},
        "flattened_ears": {EmotionType.FEARFUL: 0.5, EmotionType.AGGRESSIVE: 0.5},
        "forward_ears": {EmotionType.CURIOUS: 0.6, EmotionType.ALERT: 0.4},
        "whiskers_forward": {EmotionType.CURIOUS: 0.5, EmotionType.EXCITED: 0.5},
        "whiskers_back": {EmotionType.FEARFUL: 0.6, EmotionType.ANXIOUS: 0.4}
    }

    def __init__(self, species: str):
        self.species = species
        self.emotion_mappings = (
            self.CANINE_FACIAL_EMOTIONS if species == "CANINE"
            else self.FELINE_FACIAL_EMOTIONS
        )

    def analyze_face(
        self,
        facial_landmarks: np.ndarray,
        image: Optional[np.ndarray] = None
    ) -> tuple:
        """
        Analyze facial expression.

        Returns (facial_features, emotion_scores, confidence)
        """
        features = self._extract_facial_features(facial_landmarks, image)
        expression_class = self._classify_expression(features)
        emotion_scores = self.emotion_mappings.get(
            expression_class,
            {EmotionType.CALM: 1.0}
        )

        confidence = self._calculate_confidence(facial_landmarks)

        return (features, emotion_scores, confidence)

    def _extract_facial_features(
        self,
        landmarks: np.ndarray,
        image: Optional[np.ndarray]
    ) -> dict:
        """Extract facial features from landmarks."""
        features = {}

        # Eye analysis
        features["eye_openness"] = self._calculate_eye_openness(landmarks)
        features["eye_direction"] = self._calculate_eye_direction(landmarks)

        # Mouth analysis
        features["mouth_openness"] = self._calculate_mouth_openness(landmarks)
        features["lip_position"] = self._analyze_lips(landmarks)

        # Ear analysis (if visible)
        features["ear_position"] = self._analyze_ears(landmarks)

        # Species-specific features
        if self.species == "FELINE":
            features["pupil_dilation"] = self._analyze_pupils(landmarks, image)
            features["whisker_position"] = self._analyze_whiskers(landmarks)

        return features

    def _classify_expression(self, features: dict) -> str:
        """Classify facial expression."""
        pass

    def _calculate_eye_openness(self, landmarks: np.ndarray) -> float:
        """Calculate eye openness ratio."""
        pass

    def _calculate_eye_direction(self, landmarks: np.ndarray) -> str:
        """Determine where eyes are looking."""
        pass

    def _calculate_mouth_openness(self, landmarks: np.ndarray) -> float:
        """Calculate mouth openness."""
        pass

    def _analyze_lips(self, landmarks: np.ndarray) -> str:
        """Analyze lip position."""
        pass

    def _analyze_ears(self, landmarks: np.ndarray) -> str:
        """Analyze ear position."""
        pass

    def _analyze_pupils(self, landmarks: np.ndarray, image: np.ndarray) -> str:
        """Analyze pupil dilation (cats)."""
        pass

    def _analyze_whiskers(self, landmarks: np.ndarray) -> str:
        """Analyze whisker position (cats)."""
        pass

    def _calculate_confidence(self, landmarks: np.ndarray) -> float:
        """Calculate detection confidence."""
        pass
```

---

## 3. Audio Analysis

### 3.1 Vocalization Classifier

```python
class VocalizationAnalyzer:
    """
    Analyzes pet vocalizations to detect emotional state.
    """

    # Vocalization-emotion mappings
    CANINE_VOCALIZATION_EMOTIONS = {
        "playful_bark": {EmotionType.PLAYFUL: 0.7, EmotionType.EXCITED: 0.3},
        "alert_bark": {EmotionType.ALERT: 0.6, EmotionType.ANXIOUS: 0.4},
        "aggressive_bark": {EmotionType.AGGRESSIVE: 0.7, EmotionType.FEARFUL: 0.3},
        "whine": {EmotionType.ANXIOUS: 0.5, EmotionType.SAD: 0.3, EmotionType.FRUSTRATED: 0.2},
        "growl": {EmotionType.AGGRESSIVE: 0.5, EmotionType.FEARFUL: 0.3, EmotionType.FRUSTRATED: 0.2},
        "howl": {EmotionType.SAD: 0.4, EmotionType.ANXIOUS: 0.4, EmotionType.ALERT: 0.2},
        "yelp": {EmotionType.FEARFUL: 0.6, EmotionType.SAD: 0.4}
    }

    FELINE_VOCALIZATION_EMOTIONS = {
        "meow_greeting": {EmotionType.HAPPY: 0.5, EmotionType.AFFECTIONATE: 0.5},
        "meow_demand": {EmotionType.FRUSTRATED: 0.6, EmotionType.ALERT: 0.4},
        "purr": {EmotionType.CONTENT: 0.6, EmotionType.CALM: 0.2, EmotionType.ANXIOUS: 0.2},
        "hiss": {EmotionType.FEARFUL: 0.5, EmotionType.AGGRESSIVE: 0.5},
        "growl": {EmotionType.AGGRESSIVE: 0.6, EmotionType.FEARFUL: 0.4},
        "chirp": {EmotionType.EXCITED: 0.5, EmotionType.CURIOUS: 0.5},
        "yowl": {EmotionType.FRUSTRATED: 0.4, EmotionType.ANXIOUS: 0.4, EmotionType.SAD: 0.2}
    }

    def __init__(self, species: str, sample_rate: int = 16000):
        self.species = species
        self.sample_rate = sample_rate
        self.emotion_mappings = (
            self.CANINE_VOCALIZATION_EMOTIONS if species == "CANINE"
            else self.FELINE_VOCALIZATION_EMOTIONS
        )

    def analyze_audio(
        self,
        audio_segment: np.ndarray,
        duration: float
    ) -> tuple:
        """
        Analyze audio segment for vocalization.

        Returns (vocalization_type, features, emotion_scores, confidence)
        """
        # Extract audio features
        features = self._extract_audio_features(audio_segment)

        # Classify vocalization type
        vocalization_type = self._classify_vocalization(features)

        # Get emotion scores
        emotion_key = f"{vocalization_type.lower()}"
        emotion_scores = self.emotion_mappings.get(
            emotion_key,
            {EmotionType.ALERT: 1.0}
        )

        confidence = self._calculate_confidence(features)

        return (vocalization_type, features, emotion_scores, confidence)

    def _extract_audio_features(self, audio: np.ndarray) -> dict:
        """Extract acoustic features."""
        features = {}

        # Fundamental frequency (pitch)
        features["f0_mean"] = self._calculate_f0(audio)
        features["f0_variance"] = self._calculate_f0_variance(audio)

        # Energy features
        features["energy_mean"] = np.mean(audio ** 2)
        features["energy_variance"] = np.var(audio ** 2)

        # Spectral features
        features["spectral_centroid"] = self._spectral_centroid(audio)
        features["spectral_bandwidth"] = self._spectral_bandwidth(audio)

        # Temporal features
        features["duration"] = len(audio) / self.sample_rate
        features["zero_crossing_rate"] = self._zero_crossing_rate(audio)

        # MFCC features
        features["mfcc"] = self._extract_mfcc(audio)

        return features

    def _classify_vocalization(self, features: dict) -> str:
        """Classify type of vocalization."""
        # Use trained classifier or rule-based system
        if self.species == "CANINE":
            return self._classify_canine_vocalization(features)
        else:
            return self._classify_feline_vocalization(features)

    def _classify_canine_vocalization(self, features: dict) -> str:
        """Classify dog vocalization."""
        f0 = features["f0_mean"]
        energy = features["energy_mean"]
        duration = features["duration"]

        if features["energy_variance"] > 0.5 and duration < 0.3:
            if f0 > 400:
                return "playful_bark"
            elif f0 > 200:
                return "alert_bark"
            else:
                return "aggressive_bark"
        elif duration > 1.0 and f0 < 300:
            return "howl"
        elif f0 > 500 and energy < 0.3:
            return "whine"
        elif f0 < 200 and energy > 0.3:
            return "growl"
        else:
            return "alert_bark"

    def _classify_feline_vocalization(self, features: dict) -> str:
        """Classify cat vocalization."""
        f0 = features["f0_mean"]
        energy = features["energy_mean"]
        zcr = features["zero_crossing_rate"]

        # Purr has distinctive low frequency rumble
        if f0 < 100 and zcr < 0.1:
            return "purr"
        # Hiss has high frequency noise
        elif zcr > 0.5 and energy > 0.3:
            return "hiss"
        elif f0 > 400:
            return "chirp"
        elif features["duration"] > 1.0:
            return "yowl"
        else:
            if energy > 0.4:
                return "meow_demand"
            else:
                return "meow_greeting"

    def _calculate_f0(self, audio: np.ndarray) -> float:
        """Calculate fundamental frequency."""
        pass

    def _calculate_f0_variance(self, audio: np.ndarray) -> float:
        """Calculate pitch variance."""
        pass

    def _spectral_centroid(self, audio: np.ndarray) -> float:
        """Calculate spectral centroid."""
        pass

    def _spectral_bandwidth(self, audio: np.ndarray) -> float:
        """Calculate spectral bandwidth."""
        pass

    def _zero_crossing_rate(self, audio: np.ndarray) -> float:
        """Calculate zero crossing rate."""
        crossings = np.sum(np.abs(np.diff(np.sign(audio)))) / 2
        return crossings / len(audio)

    def _extract_mfcc(self, audio: np.ndarray) -> np.ndarray:
        """Extract MFCC features."""
        pass

    def _calculate_confidence(self, features: dict) -> float:
        """Calculate classification confidence."""
        pass
```

---

## 4. Physiological Analysis

### 4.1 Heart Rate Emotion Mapping

```python
class PhysiologicalEmotionAnalyzer:
    """
    Analyzes physiological signals for emotional state detection.
    """

    def __init__(self, baseline: dict):
        """
        Initialize with pet's baseline physiological values.

        baseline should contain:
        - resting_hr: Resting heart rate in BPM
        - resting_hrv: Resting HRV in ms
        - normal_temp: Normal body temperature
        """
        self.baseline = baseline

    def analyze_physiology(
        self,
        heart_rate: float,
        hrv: float,
        temperature: Optional[float] = None,
        respiration_rate: Optional[float] = None
    ) -> tuple:
        """
        Analyze physiological data for emotional indicators.

        Returns (features, emotion_scores, confidence)
        """
        features = {}

        # Heart rate deviation
        hr_deviation = (heart_rate - self.baseline["resting_hr"]) / self.baseline["resting_hr"]
        features["hr_deviation"] = hr_deviation

        # HRV analysis (low HRV = stress)
        hrv_deviation = (hrv - self.baseline["resting_hrv"]) / self.baseline["resting_hrv"]
        features["hrv_deviation"] = hrv_deviation

        # Temperature deviation
        if temperature:
            temp_deviation = temperature - self.baseline["normal_temp"]
            features["temp_deviation"] = temp_deviation

        # Calculate arousal level
        arousal = self._calculate_arousal(hr_deviation, hrv_deviation)
        features["arousal"] = arousal

        # Calculate stress level
        stress = self._calculate_stress(hr_deviation, hrv_deviation)
        features["stress"] = stress

        # Map to emotions
        emotion_scores = self._map_to_emotions(arousal, stress, hr_deviation)

        confidence = self._calculate_confidence(features)

        return (features, emotion_scores, confidence)

    def _calculate_arousal(
        self,
        hr_deviation: float,
        hrv_deviation: float
    ) -> float:
        """
        Calculate arousal level (0-1).
        High HR and low HRV indicate high arousal.
        """
        hr_arousal = max(0, min(1, (hr_deviation + 0.5) / 1.0))
        hrv_arousal = max(0, min(1, (-hrv_deviation + 0.5) / 1.0))

        return 0.6 * hr_arousal + 0.4 * hrv_arousal

    def _calculate_stress(
        self,
        hr_deviation: float,
        hrv_deviation: float
    ) -> float:
        """
        Calculate stress level (0-1).
        Elevated HR with decreased HRV indicates stress.
        """
        if hr_deviation > 0.1 and hrv_deviation < -0.1:
            stress_indicator = hr_deviation - hrv_deviation
            return max(0, min(1, stress_indicator))
        return max(0, min(1, hr_deviation * 0.5 - hrv_deviation * 0.5))

    def _map_to_emotions(
        self,
        arousal: float,
        stress: float,
        hr_deviation: float
    ) -> Dict[EmotionType, float]:
        """
        Map physiological state to emotion probabilities.
        """
        emotions = {}

        # High arousal, low stress = excited/happy
        if arousal > 0.6 and stress < 0.4:
            emotions[EmotionType.EXCITED] = arousal * 0.6
            emotions[EmotionType.HAPPY] = arousal * 0.4

        # High arousal, high stress = anxious/fearful
        elif arousal > 0.6 and stress > 0.6:
            emotions[EmotionType.ANXIOUS] = stress * 0.5
            emotions[EmotionType.FEARFUL] = stress * 0.5

        # Low arousal, low stress = calm/content
        elif arousal < 0.4 and stress < 0.4:
            emotions[EmotionType.CALM] = (1 - arousal) * 0.6
            emotions[EmotionType.CONTENT] = (1 - arousal) * 0.4

        # Low arousal, high stress = sad
        elif arousal < 0.4 and stress > 0.5:
            emotions[EmotionType.SAD] = stress * 0.7
            emotions[EmotionType.ANXIOUS] = stress * 0.3

        # Medium states
        else:
            emotions[EmotionType.ALERT] = 0.5
            emotions[EmotionType.CALM] = 0.5

        # Normalize
        total = sum(emotions.values())
        if total > 0:
            emotions = {k: v / total for k, v in emotions.items()}

        return emotions

    def _calculate_confidence(self, features: dict) -> float:
        """Calculate confidence based on data quality."""
        pass
```

---

## 5. Emotion Intensity Estimation

```python
class IntensityEstimator:
    """
    Estimates emotional intensity on a 1-5 scale.
    """

    def estimate_intensity(
        self,
        emotion_type: EmotionType,
        behavioral_indicators: dict,
        physiological_indicators: Optional[dict] = None
    ) -> int:
        """
        Estimate emotion intensity level (1-5).
        """
        intensity_score = 0.0

        # Behavioral intensity factors
        if "activity_level" in behavioral_indicators:
            activity = behavioral_indicators["activity_level"]
            intensity_score += activity * 0.3

        if "vocalization_intensity" in behavioral_indicators:
            vocal = behavioral_indicators["vocalization_intensity"]
            intensity_score += vocal * 0.2

        if "body_tension" in behavioral_indicators:
            tension = behavioral_indicators["body_tension"]
            intensity_score += tension * 0.2

        # Physiological intensity factors
        if physiological_indicators:
            if "hr_deviation" in physiological_indicators:
                hr_dev = abs(physiological_indicators["hr_deviation"])
                intensity_score += min(1.0, hr_dev * 2) * 0.3

        # Map to 1-5 scale
        if intensity_score < 0.2:
            return 1  # Minimal
        elif intensity_score < 0.4:
            return 2  # Mild
        elif intensity_score < 0.6:
            return 3  # Moderate
        elif intensity_score < 0.8:
            return 4  # Strong
        else:
            return 5  # Intense
```

---

## 6. Temporal Pattern Analysis

```python
class TemporalPatternAnalyzer:
    """
    Analyzes emotional patterns over time.
    """

    def __init__(self, window_size: int = 60):
        """
        Initialize with analysis window in seconds.
        """
        self.window_size = window_size
        self.history = []

    def add_reading(self, emotion_reading: dict):
        """Add new emotion reading to history."""
        self.history.append(emotion_reading)
        self._cleanup_old_readings()

    def analyze_trend(self) -> dict:
        """
        Analyze emotional trend over recent window.

        Returns trend analysis with stability metrics.
        """
        if len(self.history) < 3:
            return {"status": "insufficient_data"}

        emotions_over_time = [r["primary_emotion"] for r in self.history]
        intensities = [r["intensity"] for r in self.history]

        # Calculate stability
        stability = self._calculate_stability(emotions_over_time)

        # Detect trend direction
        intensity_trend = self._calculate_intensity_trend(intensities)

        # Detect patterns
        patterns = self._detect_patterns(emotions_over_time)

        return {
            "stability": stability,
            "intensity_trend": intensity_trend,
            "patterns": patterns,
            "dominant_emotion": self._get_dominant_emotion(emotions_over_time)
        }

    def _calculate_stability(self, emotions: list) -> float:
        """Calculate emotional stability (0-1)."""
        if len(emotions) < 2:
            return 1.0

        changes = sum(1 for i in range(1, len(emotions))
                     if emotions[i] != emotions[i-1])
        return 1.0 - (changes / (len(emotions) - 1))

    def _calculate_intensity_trend(self, intensities: list) -> str:
        """Determine if intensity is increasing, stable, or decreasing."""
        if len(intensities) < 3:
            return "stable"

        first_half = np.mean(intensities[:len(intensities)//2])
        second_half = np.mean(intensities[len(intensities)//2:])

        diff = second_half - first_half
        if diff > 0.5:
            return "increasing"
        elif diff < -0.5:
            return "decreasing"
        else:
            return "stable"

    def _detect_patterns(self, emotions: list) -> list:
        """Detect recurring emotional patterns."""
        patterns = []

        # Check for oscillation
        if self._is_oscillating(emotions):
            patterns.append("oscillating")

        # Check for escalation
        if self._is_escalating(emotions):
            patterns.append("escalating")

        return patterns

    def _is_oscillating(self, emotions: list) -> bool:
        """Check if emotions are oscillating between states."""
        if len(emotions) < 4:
            return False

        changes = [emotions[i] != emotions[i-1] for i in range(1, len(emotions))]
        change_rate = sum(changes) / len(changes)
        return change_rate > 0.6

    def _is_escalating(self, emotions: list) -> bool:
        """Check if negative emotions are escalating."""
        negative_emotions = {EmotionType.ANXIOUS, EmotionType.FEARFUL,
                          EmotionType.AGGRESSIVE, EmotionType.SAD}
        recent = emotions[-5:] if len(emotions) >= 5 else emotions
        return sum(1 for e in recent if e in negative_emotions) > len(recent) * 0.6

    def _get_dominant_emotion(self, emotions: list) -> EmotionType:
        """Get most frequent emotion in window."""
        from collections import Counter
        return Counter(emotions).most_common(1)[0][0]

    def _cleanup_old_readings(self):
        """Remove readings older than window."""
        pass
```

---

## Document Information

**WIA-PET-EMOTION Phase 2: Algorithms**
**Version**: 1.0.0
**Hong-ik Ingan**
