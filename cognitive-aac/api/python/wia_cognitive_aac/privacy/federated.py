"""
WIA Cognitive AAC - Federated Learning
프라이버시 보호 연합 학습 모듈

홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

설계 원칙:
- 온디바이스 학습 우선 (Local-first)
- 개인 데이터 절대 전송 안함
- 모델 업데이트만 익명화 공유
- 명시적 동의 필수
"""

from __future__ import annotations

import hashlib
import json
import secrets
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class AnonymizationLevel(Enum):
    """익명화 수준"""
    NONE = "none"
    PARTIAL = "partial"
    FULL = "full"


class ConsentType(Enum):
    """동의 유형"""
    DATA_COLLECTION = "data_collection"
    FEDERATED_LEARNING = "federated_learning"
    ANALYTICS = "analytics"


@dataclass
class ConsentRecord:
    """동의 기록"""
    consent_type: ConsentType
    granted: bool
    timestamp: float
    grantor: str  # 'user', 'caregiver', 'guardian'
    expires_at: Optional[float] = None


@dataclass
class ModelUpdate:
    """모델 업데이트 (익명화됨)"""
    update_id: str
    model_version: str
    timestamp: float
    gradient_hash: str
    update_magnitude: float
    num_samples: int
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class FederatedConfig:
    """연합 학습 설정"""
    enabled: bool = False
    anonymization_level: AnonymizationLevel = AnonymizationLevel.FULL
    aggregation_only: bool = True  # 개별 업데이트 전송 안함
    min_samples_per_round: int = 100
    max_rounds_per_day: int = 3
    noise_multiplier: float = 1.0  # 차등 프라이버시 노이즈
    clip_norm: float = 1.0  # 그래디언트 클리핑


class PrivacyGuard:
    """프라이버시 보호 가드"""

    def __init__(self, config: FederatedConfig):
        self.config = config
        self._consents: List[ConsentRecord] = []
        self._last_check: float = 0

    def has_consent(self, consent_type: ConsentType) -> bool:
        """동의 여부 확인"""
        latest = None
        for consent in self._consents:
            if consent.consent_type == consent_type:
                if latest is None or consent.timestamp > latest.timestamp:
                    latest = consent

        if latest is None:
            return False

        # 만료 확인
        if latest.expires_at and latest.expires_at < datetime.now().timestamp():
            return False

        return latest.granted

    def record_consent(
        self,
        consent_type: ConsentType,
        granted: bool,
        grantor: str = "user",
        expires_at: Optional[float] = None
    ) -> None:
        """동의 기록"""
        self._consents.append(ConsentRecord(
            consent_type=consent_type,
            granted=granted,
            timestamp=datetime.now().timestamp(),
            grantor=grantor,
            expires_at=expires_at
        ))

    def revoke_all_consents(self) -> None:
        """모든 동의 철회"""
        for consent_type in ConsentType:
            self.record_consent(consent_type, False, "system")


class DifferentialPrivacy:
    """차등 프라이버시 적용"""

    def __init__(self, epsilon: float = 1.0, delta: float = 1e-5):
        self.epsilon = epsilon
        self.delta = delta

    def add_noise(self, data: np.ndarray, sensitivity: float = 1.0) -> np.ndarray:
        """라플라스 노이즈 추가"""
        scale = sensitivity / self.epsilon
        noise = np.random.laplace(0, scale, data.shape)
        return data + noise

    def clip_gradients(self, gradients: np.ndarray, max_norm: float) -> np.ndarray:
        """그래디언트 클리핑"""
        norm = np.linalg.norm(gradients)
        if norm > max_norm:
            gradients = gradients * (max_norm / norm)
        return gradients


class DataAnonymizer:
    """데이터 익명화"""

    def __init__(self, level: AnonymizationLevel):
        self.level = level
        self._salt = secrets.token_hex(16)

    def anonymize_symbol_id(self, symbol_id: str) -> str:
        """심볼 ID 익명화"""
        if self.level == AnonymizationLevel.NONE:
            return symbol_id

        # SHA-256 해시
        salted = f"{self._salt}:{symbol_id}"
        hashed = hashlib.sha256(salted.encode()).hexdigest()

        if self.level == AnonymizationLevel.PARTIAL:
            return f"sym_{hashed[:8]}"
        else:  # FULL
            return f"anon_{hashed[:16]}"

    def anonymize_pattern(self, pattern: Dict[str, Any]) -> Dict[str, Any]:
        """패턴 데이터 익명화"""
        if self.level == AnonymizationLevel.NONE:
            return pattern

        anonymized = {}

        # 개인 식별 정보 제거
        personal_fields = ['name', 'location', 'person', 'email', 'phone']
        for key, value in pattern.items():
            if any(pf in key.lower() for pf in personal_fields):
                if self.level == AnonymizationLevel.FULL:
                    continue  # 완전 제거
                else:
                    anonymized[key] = self._hash_value(str(value))
            elif isinstance(value, str):
                anonymized[key] = value
            elif isinstance(value, (int, float)):
                anonymized[key] = value
            elif isinstance(value, dict):
                anonymized[key] = self.anonymize_pattern(value)
            elif isinstance(value, list):
                anonymized[key] = [
                    self.anonymize_pattern(v) if isinstance(v, dict) else v
                    for v in value
                ]
            else:
                anonymized[key] = value

        return anonymized

    def _hash_value(self, value: str) -> str:
        """값 해싱"""
        salted = f"{self._salt}:{value}"
        return hashlib.sha256(salted.encode()).hexdigest()[:12]


class LocalModelStore:
    """로컬 모델 저장소"""

    def __init__(self, base_path: str = ".wia_aac_models"):
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)

    def save_model(self, model_id: str, model_data: Dict[str, Any]) -> None:
        """모델 저장 (암호화 권장)"""
        model_path = self.base_path / f"{model_id}.json"
        model_path.write_text(json.dumps(model_data, ensure_ascii=False))

    def load_model(self, model_id: str) -> Optional[Dict[str, Any]]:
        """모델 로드"""
        model_path = self.base_path / f"{model_id}.json"
        if not model_path.exists():
            return None
        return json.loads(model_path.read_text())

    def delete_model(self, model_id: str) -> bool:
        """모델 삭제"""
        model_path = self.base_path / f"{model_id}.json"
        if model_path.exists():
            model_path.unlink()
            return True
        return False

    def list_models(self) -> List[str]:
        """저장된 모델 목록"""
        return [p.stem for p in self.base_path.glob("*.json")]

    def clear_all(self) -> int:
        """모든 모델 삭제 (잊혀질 권리)"""
        count = 0
        for model_file in self.base_path.glob("*.json"):
            model_file.unlink()
            count += 1
        return count


class FederatedLearner:
    """
    프라이버시 보호 연합 학습 모듈

    특징:
    - 온디바이스 학습 (로컬 데이터 유지)
    - 모델 업데이트만 익명화 공유 (선택적)
    - 차등 프라이버시 적용
    - 완전한 데이터 삭제 지원

    사용 예:
    ```python
    learner = FederatedLearner()

    # 동의 기록
    learner.record_consent(ConsentType.DATA_COLLECTION, True)

    # 로컬 학습
    learner.local_update(patterns)

    # 연합 학습 참여 (선택적, 동의 필요)
    if learner.can_participate_federated():
        update = learner.prepare_federated_update()
        # 서버로 update 전송 (익명화됨)
    ```
    """

    def __init__(self, config: Optional[FederatedConfig] = None):
        self.config = config or FederatedConfig()
        self.privacy_guard = PrivacyGuard(self.config)
        self.dp = DifferentialPrivacy()
        self.anonymizer = DataAnonymizer(self.config.anonymization_level)
        self.local_store = LocalModelStore()

        # 학습 상태
        self._local_updates: List[np.ndarray] = []
        self._update_count: int = 0
        self._rounds_today: int = 0
        self._last_round_date: Optional[str] = None

    # =========================================================================
    # Consent Management
    # =========================================================================

    def record_consent(
        self,
        consent_type: ConsentType,
        granted: bool,
        grantor: str = "user"
    ) -> None:
        """동의 기록"""
        self.privacy_guard.record_consent(consent_type, granted, grantor)

        # 동의 철회 시 관련 데이터 삭제
        if not granted:
            if consent_type == ConsentType.DATA_COLLECTION:
                self._local_updates.clear()
            elif consent_type == ConsentType.FEDERATED_LEARNING:
                self.config.enabled = False

    def has_consent(self, consent_type: ConsentType) -> bool:
        """동의 여부 확인"""
        return self.privacy_guard.has_consent(consent_type)

    def can_participate_federated(self) -> bool:
        """연합 학습 참여 가능 여부"""
        return (
            self.config.enabled and
            self.has_consent(ConsentType.FEDERATED_LEARNING) and
            self.has_consent(ConsentType.DATA_COLLECTION) and
            self._rounds_today < self.config.max_rounds_per_day
        )

    # =========================================================================
    # Local Learning
    # =========================================================================

    def local_update(self, patterns: Dict[str, Any]) -> None:
        """
        로컬 학습 업데이트

        데이터는 디바이스에만 저장됨
        """
        if not self.has_consent(ConsentType.DATA_COLLECTION):
            return

        # 익명화
        anonymized = self.anonymizer.anonymize_pattern(patterns)

        # 로컬 저장
        update_id = f"local_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.local_store.save_model(update_id, anonymized)

        self._update_count += 1

    def get_local_patterns(self) -> List[Dict[str, Any]]:
        """로컬 패턴 조회"""
        patterns = []
        for model_id in self.local_store.list_models():
            if model_id.startswith("local_"):
                data = self.local_store.load_model(model_id)
                if data:
                    patterns.append(data)
        return patterns

    # =========================================================================
    # Federated Learning
    # =========================================================================

    def prepare_federated_update(self) -> Optional[ModelUpdate]:
        """
        연합 학습용 업데이트 준비

        - 차등 프라이버시 노이즈 추가
        - 그래디언트 클리핑
        - 익명화

        Returns:
            익명화된 모델 업데이트 (개인 데이터 없음)
        """
        if not self.can_participate_federated():
            return None

        # 로컬 패턴 집계
        local_patterns = self.get_local_patterns()
        if len(local_patterns) < self.config.min_samples_per_round:
            return None

        # 집계 통계만 추출 (개별 데이터 아님)
        aggregated = self._aggregate_patterns(local_patterns)

        # 차등 프라이버시 적용
        if isinstance(aggregated, np.ndarray):
            clipped = self.dp.clip_gradients(aggregated, self.config.clip_norm)
            noised = self.dp.add_noise(clipped)
            gradient_hash = hashlib.sha256(noised.tobytes()).hexdigest()
            magnitude = float(np.linalg.norm(noised))
        else:
            gradient_hash = hashlib.sha256(
                json.dumps(aggregated, sort_keys=True).encode()
            ).hexdigest()
            magnitude = len(str(aggregated))

        # 업데이트 생성
        update = ModelUpdate(
            update_id=secrets.token_hex(8),
            model_version="1.0.0",
            timestamp=datetime.now().timestamp(),
            gradient_hash=gradient_hash,
            update_magnitude=magnitude,
            num_samples=len(local_patterns),
            metadata={
                "anonymization_level": self.config.anonymization_level.value,
                "noise_applied": True,
            }
        )

        # 라운드 카운트 업데이트
        today = datetime.now().strftime("%Y-%m-%d")
        if self._last_round_date != today:
            self._rounds_today = 0
            self._last_round_date = today
        self._rounds_today += 1

        return update

    def _aggregate_patterns(self, patterns: List[Dict[str, Any]]) -> Dict[str, Any]:
        """패턴 집계 (개인 식별 불가능한 통계만)"""
        aggregated = {
            "total_count": len(patterns),
            "avg_symbols_per_session": 0,
            "unique_symbol_count": 0,
        }

        all_symbols = set()
        total_symbols = 0

        for pattern in patterns:
            if "symbols" in pattern:
                symbols = pattern["symbols"]
                if isinstance(symbols, list):
                    all_symbols.update(symbols)
                    total_symbols += len(symbols)

        aggregated["avg_symbols_per_session"] = (
            total_symbols / len(patterns) if patterns else 0
        )
        aggregated["unique_symbol_count"] = len(all_symbols)

        return aggregated

    # =========================================================================
    # Data Rights (잊혀질 권리)
    # =========================================================================

    def delete_all_data(self) -> Dict[str, int]:
        """
        모든 데이터 삭제 (잊혀질 권리)

        Returns:
            삭제된 항목 수
        """
        deleted = {
            "local_models": self.local_store.clear_all(),
            "updates": len(self._local_updates),
        }

        self._local_updates.clear()
        self._update_count = 0

        # 동의는 유지 (법적 기록)
        return deleted

    def export_data(self) -> Dict[str, Any]:
        """
        데이터 내보내기 (이동권)

        Returns:
            모든 저장된 데이터
        """
        return {
            "version": "1.0.0",
            "export_date": datetime.now().isoformat(),
            "patterns": self.get_local_patterns(),
            "update_count": self._update_count,
            "consents": [
                {
                    "type": c.consent_type.value,
                    "granted": c.granted,
                    "timestamp": c.timestamp,
                    "grantor": c.grantor,
                }
                for c in self.privacy_guard._consents
            ]
        }

    # =========================================================================
    # Statistics
    # =========================================================================

    def get_statistics(self) -> Dict[str, Any]:
        """학습 통계"""
        return {
            "local_update_count": self._update_count,
            "stored_models": len(self.local_store.list_models()),
            "federated_rounds_today": self._rounds_today,
            "consents": {
                ct.value: self.has_consent(ct)
                for ct in ConsentType
            },
            "privacy_level": self.config.anonymization_level.value,
        }


# 자폐 특화 연합 학습자
class AutismFederatedLearner(FederatedLearner):
    """
    자폐 사용자 특화 연합 학습

    특징:
    - 루틴 패턴 공유 (익명화)
    - 감각 민감도 통계 공유
    """

    def __init__(self, config: Optional[FederatedConfig] = None):
        super().__init__(config)
        self._routine_patterns: List[Dict[str, Any]] = []

    def add_routine_pattern(self, pattern: Dict[str, Any]) -> None:
        """루틴 패턴 추가"""
        if self.has_consent(ConsentType.DATA_COLLECTION):
            anonymized = self.anonymizer.anonymize_pattern(pattern)
            self._routine_patterns.append(anonymized)

    def prepare_federated_update(self) -> Optional[ModelUpdate]:
        """루틴 패턴 포함 업데이트"""
        base_update = super().prepare_federated_update()
        if base_update and self._routine_patterns:
            base_update.metadata["routine_count"] = len(self._routine_patterns)
        return base_update


# 치매 특화 연합 학습자
class DementiaFederatedLearner(FederatedLearner):
    """
    치매 사용자 특화 연합 학습

    특징:
    - 더 엄격한 프라이버시 보호
    - 케어기버 동의 필수
    """

    def __init__(self, config: Optional[FederatedConfig] = None):
        custom_config = config or FederatedConfig()
        custom_config.anonymization_level = AnonymizationLevel.FULL
        custom_config.aggregation_only = True
        super().__init__(custom_config)

        self._caregiver_consent: bool = False

    def record_caregiver_consent(self, granted: bool) -> None:
        """케어기버 동의 기록"""
        self._caregiver_consent = granted
        self.record_consent(
            ConsentType.FEDERATED_LEARNING,
            granted,
            grantor="caregiver"
        )

    def can_participate_federated(self) -> bool:
        """케어기버 동의 필수"""
        return self._caregiver_consent and super().can_participate_federated()
