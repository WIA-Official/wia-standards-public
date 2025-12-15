"""
WIA Cognitive AAC - Context Model
상황 인식 및 컨텍스트 기반 예측 모델

홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

설계 원칙:
- 다중 컨텍스트 신호 통합
- 시간/위치/활동 기반 예측
- 자폐/치매 특화 컨텍스트 처리
"""

from __future__ import annotations

import json
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np


@dataclass
class ContextSignal:
    """컨텍스트 신호"""
    signal_type: str  # 'time', 'location', 'activity', 'person', 'mood'
    value: str
    confidence: float
    timestamp: float


@dataclass
class ContextState:
    """현재 컨텍스트 상태"""
    time_of_day: str  # 'morning', 'afternoon', 'evening', 'night'
    day_type: str  # 'weekday', 'weekend'
    hour: int
    location: Optional[str] = None
    activity: Optional[str] = None
    conversation_partner: Optional[str] = None
    mood: Optional[str] = None
    signals: List[ContextSignal] = field(default_factory=list)


@dataclass
class ContextPrediction:
    """컨텍스트 기반 예측"""
    symbol_id: str
    probability: float
    context_type: str
    explanation: str
    rank: int


@dataclass
class ContextModelConfig:
    """컨텍스트 모델 설정"""
    enable_time_prediction: bool = True
    enable_location_prediction: bool = True
    enable_activity_prediction: bool = True
    enable_person_prediction: bool = True
    context_weight: float = 0.3
    recency_weight: float = 0.4
    min_confidence: float = 0.1
    decay_factor: float = 0.95
    max_history_size: int = 1000


class ContextModel:
    """
    상황 인식 기반 예측 모델

    특징:
    - 시간대별 심볼 사용 패턴
    - 위치 기반 추천
    - 활동/대화 상대 컨텍스트
    - 기분/환경 신호 통합
    """

    def __init__(self, config: Optional[ContextModelConfig] = None):
        self.config = config or ContextModelConfig()

        # 시간대별 빈도: {hour: {symbol_id: count}}
        self.hourly_frequency: Dict[int, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 요일별 빈도: {day_type: {symbol_id: count}}
        self.daily_frequency: Dict[str, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 위치별 빈도: {location: {symbol_id: count}}
        self.location_frequency: Dict[str, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 활동별 빈도: {activity: {symbol_id: count}}
        self.activity_frequency: Dict[str, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 대화 상대별 빈도: {person: {symbol_id: count}}
        self.person_frequency: Dict[str, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 기분별 빈도: {mood: {symbol_id: count}}
        self.mood_frequency: Dict[str, Dict[str, int]] = defaultdict(
            lambda: defaultdict(int)
        )

        # 컨텍스트 히스토리
        self._context_history: List[Tuple[ContextState, str]] = []
        self._current_context: Optional[ContextState] = None

    def update(self, symbol_id: str, context: ContextState) -> None:
        """
        심볼 선택과 컨텍스트 연결 학습

        Args:
            symbol_id: 선택된 심볼
            context: 선택 시점의 컨텍스트
        """
        # 시간대별
        self.hourly_frequency[context.hour][symbol_id] += 1

        # 요일별
        self.daily_frequency[context.day_type][symbol_id] += 1

        # 위치별
        if context.location:
            self.location_frequency[context.location][symbol_id] += 1

        # 활동별
        if context.activity:
            self.activity_frequency[context.activity][symbol_id] += 1

        # 대화 상대별
        if context.conversation_partner:
            self.person_frequency[context.conversation_partner][symbol_id] += 1

        # 기분별
        if context.mood:
            self.mood_frequency[context.mood][symbol_id] += 1

        # 히스토리 저장
        self._context_history.append((context, symbol_id))
        if len(self._context_history) > self.config.max_history_size:
            self._context_history = self._context_history[-self.config.max_history_size:]

        self._current_context = context

    def predict(self, context: ContextState, top_k: int = 8) -> List[ContextPrediction]:
        """
        컨텍스트 기반 심볼 예측

        Args:
            context: 현재 컨텍스트
            top_k: 반환할 예측 수

        Returns:
            예측 결과 리스트
        """
        predictions: Dict[str, ContextPrediction] = {}

        # 시간대 예측
        if self.config.enable_time_prediction:
            time_preds = self._predict_by_time(context)
            self._merge_predictions(predictions, time_preds)

        # 위치 예측
        if self.config.enable_location_prediction and context.location:
            location_preds = self._predict_by_location(context)
            self._merge_predictions(predictions, location_preds)

        # 활동 예측
        if self.config.enable_activity_prediction and context.activity:
            activity_preds = self._predict_by_activity(context)
            self._merge_predictions(predictions, activity_preds)

        # 대화 상대 예측
        if self.config.enable_person_prediction and context.conversation_partner:
            person_preds = self._predict_by_person(context)
            self._merge_predictions(predictions, person_preds)

        # 정렬 및 순위 부여
        sorted_preds = sorted(
            predictions.values(),
            key=lambda x: -x.probability
        )

        for i, pred in enumerate(sorted_preds):
            pred.rank = i + 1

        return sorted_preds[:top_k]

    def _predict_by_time(self, context: ContextState) -> List[ContextPrediction]:
        """시간대 기반 예측"""
        hour_counts = self.hourly_frequency.get(context.hour, {})
        if not hour_counts:
            return []

        total = sum(hour_counts.values())
        predictions = []

        for symbol_id, count in hour_counts.items():
            prob = count / total
            if prob >= self.config.min_confidence:
                predictions.append(ContextPrediction(
                    symbol_id=symbol_id,
                    probability=prob * self.config.context_weight,
                    context_type='time',
                    explanation=f'{context.hour}시에 자주 사용해요',
                    rank=0
                ))

        return predictions

    def _predict_by_location(self, context: ContextState) -> List[ContextPrediction]:
        """위치 기반 예측"""
        if not context.location:
            return []

        location_counts = self.location_frequency.get(context.location, {})
        if not location_counts:
            return []

        total = sum(location_counts.values())
        predictions = []

        for symbol_id, count in location_counts.items():
            prob = count / total
            if prob >= self.config.min_confidence:
                predictions.append(ContextPrediction(
                    symbol_id=symbol_id,
                    probability=prob * self.config.context_weight,
                    context_type='location',
                    explanation=f'{context.location}에서 자주 사용해요',
                    rank=0
                ))

        return predictions

    def _predict_by_activity(self, context: ContextState) -> List[ContextPrediction]:
        """활동 기반 예측"""
        if not context.activity:
            return []

        activity_counts = self.activity_frequency.get(context.activity, {})
        if not activity_counts:
            return []

        total = sum(activity_counts.values())
        predictions = []

        for symbol_id, count in activity_counts.items():
            prob = count / total
            if prob >= self.config.min_confidence:
                predictions.append(ContextPrediction(
                    symbol_id=symbol_id,
                    probability=prob * self.config.context_weight,
                    context_type='activity',
                    explanation=f'{context.activity} 중에 자주 사용해요',
                    rank=0
                ))

        return predictions

    def _predict_by_person(self, context: ContextState) -> List[ContextPrediction]:
        """대화 상대 기반 예측"""
        if not context.conversation_partner:
            return []

        person_counts = self.person_frequency.get(context.conversation_partner, {})
        if not person_counts:
            return []

        total = sum(person_counts.values())
        predictions = []

        for symbol_id, count in person_counts.items():
            prob = count / total
            if prob >= self.config.min_confidence:
                predictions.append(ContextPrediction(
                    symbol_id=symbol_id,
                    probability=prob * self.config.context_weight,
                    context_type='person',
                    explanation=f'{context.conversation_partner}님과 대화할 때 자주 사용해요',
                    rank=0
                ))

        return predictions

    def _merge_predictions(
        self,
        target: Dict[str, ContextPrediction],
        source: List[ContextPrediction]
    ) -> None:
        """예측 결과 병합"""
        for pred in source:
            if pred.symbol_id in target:
                # 확률 합산
                existing = target[pred.symbol_id]
                existing.probability = min(1.0, existing.probability + pred.probability * 0.5)
                # 더 높은 확률의 설명 사용
                if pred.probability > existing.probability:
                    existing.context_type = pred.context_type
                    existing.explanation = pred.explanation
            else:
                target[pred.symbol_id] = pred

    def detect_context(
        self,
        timestamp: Optional[datetime] = None,
        signals: Optional[List[ContextSignal]] = None
    ) -> ContextState:
        """
        현재 컨텍스트 감지

        Args:
            timestamp: 기준 시간 (기본: 현재)
            signals: 외부 컨텍스트 신호

        Returns:
            감지된 컨텍스트 상태
        """
        now = timestamp or datetime.now()
        hour = now.hour

        # 시간대 결정
        if 5 <= hour < 12:
            time_of_day = 'morning'
        elif 12 <= hour < 17:
            time_of_day = 'afternoon'
        elif 17 <= hour < 21:
            time_of_day = 'evening'
        else:
            time_of_day = 'night'

        # 요일 유형
        day_type = 'weekend' if now.weekday() >= 5 else 'weekday'

        context = ContextState(
            time_of_day=time_of_day,
            day_type=day_type,
            hour=hour,
            signals=signals or []
        )

        # 외부 신호 처리
        if signals:
            for signal in signals:
                if signal.signal_type == 'location':
                    context.location = signal.value
                elif signal.signal_type == 'activity':
                    context.activity = signal.value
                elif signal.signal_type == 'person':
                    context.conversation_partner = signal.value
                elif signal.signal_type == 'mood':
                    context.mood = signal.value

        return context

    def get_meal_time_context(self) -> Optional[str]:
        """현재 식사 시간 컨텍스트 반환"""
        hour = datetime.now().hour

        if 7 <= hour <= 9:
            return 'breakfast'
        elif 11 <= hour <= 13:
            return 'lunch'
        elif 17 <= hour <= 19:
            return 'dinner'

        return None

    def apply_decay(self) -> None:
        """시간 기반 감쇠 적용"""
        self._apply_decay_to_dict(self.hourly_frequency)
        self._apply_decay_to_dict(self.daily_frequency)
        self._apply_decay_to_dict(self.location_frequency)
        self._apply_decay_to_dict(self.activity_frequency)
        self._apply_decay_to_dict(self.person_frequency)
        self._apply_decay_to_dict(self.mood_frequency)

    def _apply_decay_to_dict(self, freq_dict: Dict[str, Dict[str, int]]) -> None:
        """딕셔너리에 감쇠 적용"""
        for key in list(freq_dict.keys()):
            for symbol in list(freq_dict[key].keys()):
                freq_dict[key][symbol] = int(
                    freq_dict[key][symbol] * self.config.decay_factor
                )
                if freq_dict[key][symbol] == 0:
                    del freq_dict[key][symbol]

    def save(self, path: str) -> None:
        """모델 저장"""
        data = {
            'hourly_frequency': dict(self.hourly_frequency),
            'daily_frequency': dict(self.daily_frequency),
            'location_frequency': dict(self.location_frequency),
            'activity_frequency': dict(self.activity_frequency),
            'person_frequency': dict(self.person_frequency),
            'mood_frequency': dict(self.mood_frequency),
        }

        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2))

    def load(self, path: str) -> None:
        """모델 로드"""
        data = json.loads(Path(path).read_text())

        self.hourly_frequency = defaultdict(
            lambda: defaultdict(int),
            {int(k): defaultdict(int, v) for k, v in data.get('hourly_frequency', {}).items()}
        )
        self.daily_frequency = defaultdict(
            lambda: defaultdict(int),
            {k: defaultdict(int, v) for k, v in data.get('daily_frequency', {}).items()}
        )
        self.location_frequency = defaultdict(
            lambda: defaultdict(int),
            {k: defaultdict(int, v) for k, v in data.get('location_frequency', {}).items()}
        )
        self.activity_frequency = defaultdict(
            lambda: defaultdict(int),
            {k: defaultdict(int, v) for k, v in data.get('activity_frequency', {}).items()}
        )
        self.person_frequency = defaultdict(
            lambda: defaultdict(int),
            {k: defaultdict(int, v) for k, v in data.get('person_frequency', {}).items()}
        )
        self.mood_frequency = defaultdict(
            lambda: defaultdict(int),
            {k: defaultdict(int, v) for k, v in data.get('mood_frequency', {}).items()}
        )

    def get_statistics(self) -> Dict[str, any]:
        """모델 통계"""
        return {
            'unique_locations': len(self.location_frequency),
            'unique_activities': len(self.activity_frequency),
            'unique_persons': len(self.person_frequency),
            'history_size': len(self._context_history),
        }


# 자폐 특화 컨텍스트 모델
class AutismContextModel(ContextModel):
    """
    자폐 사용자 특화 컨텍스트 모델

    특징:
    - 루틴 시간 강조
    - 전환 시간 예측
    - 예측 가능성 극대화
    """

    def __init__(self, config: Optional[ContextModelConfig] = None):
        super().__init__(config)
        self.routine_schedule: Dict[int, str] = {}  # {hour: activity}
        self.transition_warnings: bool = True

    def set_routine(self, hour: int, activity: str) -> None:
        """루틴 시간표 설정"""
        self.routine_schedule[hour] = activity

    def predict(self, context: ContextState, top_k: int = 8) -> List[ContextPrediction]:
        """루틴 강화 예측"""
        predictions = super().predict(context, top_k * 2)

        # 루틴 활동 확인
        current_routine = self.routine_schedule.get(context.hour)
        if current_routine and not context.activity:
            context.activity = current_routine

        # 다시 활동 기반 예측 추가
        if context.activity:
            activity_preds = self._predict_by_activity(context)
            for pred in activity_preds:
                pred.probability *= 1.5  # 루틴 부스트

            pred_dict = {p.symbol_id: p for p in predictions}
            self._merge_predictions(pred_dict, activity_preds)
            predictions = list(pred_dict.values())

        # 재정렬
        predictions.sort(key=lambda x: -x.probability)
        for i, pred in enumerate(predictions):
            pred.rank = i + 1

        return predictions[:top_k]

    def get_upcoming_transition(self) -> Optional[Tuple[int, str]]:
        """다음 전환 시간 반환"""
        current_hour = datetime.now().hour

        for hour in range(current_hour + 1, 24):
            if hour in self.routine_schedule:
                return (hour, self.routine_schedule[hour])

        return None


# 치매 특화 컨텍스트 모델
class DementiaContextModel(ContextModel):
    """
    치매 사용자 특화 컨텍스트 모델

    특징:
    - 시간 지남력 지원
    - 익숙한 사람/장소 강조
    - 반복 허용
    """

    def __init__(self, config: Optional[ContextModelConfig] = None):
        custom_config = config or ContextModelConfig()
        custom_config.decay_factor = 0.99  # 느린 감쇠
        super().__init__(custom_config)

        self.familiar_people: Set[str] = set()
        self.familiar_locations: Set[str] = set()

    def add_familiar_person(self, name: str) -> None:
        """익숙한 사람 등록"""
        self.familiar_people.add(name)

    def add_familiar_location(self, location: str) -> None:
        """익숙한 장소 등록"""
        self.familiar_locations.add(location)

    def predict(self, context: ContextState, top_k: int = 8) -> List[ContextPrediction]:
        """익숙함 기반 예측"""
        predictions = super().predict(context, top_k * 2)

        # 익숙한 컨텍스트 부스트
        for pred in predictions:
            boost = 1.0

            if context.conversation_partner in self.familiar_people:
                boost *= 1.3

            if context.location in self.familiar_locations:
                boost *= 1.2

            pred.probability *= boost

        # 재정렬
        predictions.sort(key=lambda x: -x.probability)
        for i, pred in enumerate(predictions):
            pred.rank = i + 1

        return predictions[:top_k]

    def get_orientation_info(self) -> Dict[str, str]:
        """시간 지남력 정보"""
        now = datetime.now()
        days_kr = ['월요일', '화요일', '수요일', '목요일', '금요일', '토요일', '일요일']

        return {
            'date': now.strftime('%Y년 %m월 %d일'),
            'day': days_kr[now.weekday()],
            'time': now.strftime('%H시 %M분'),
            'time_of_day': self._get_time_of_day_korean(now.hour),
        }

    def _get_time_of_day_korean(self, hour: int) -> str:
        """시간대 한국어 표현"""
        if 5 <= hour < 12:
            return '아침'
        elif 12 <= hour < 17:
            return '오후'
        elif 17 <= hour < 21:
            return '저녁'
        else:
            return '밤'
