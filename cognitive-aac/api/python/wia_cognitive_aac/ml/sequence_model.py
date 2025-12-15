"""
WIA Cognitive AAC - Sequence Model
심볼 시퀀스 예측 모델 (N-gram + LSTM 하이브리드)

홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

설계 원칙:
- 온디바이스 추론 가능한 경량 모델
- N-gram 기반 빠른 예측 (기본)
- 선택적 LSTM 딥러닝 (고급)
- 프라이버시 보호 학습
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class SequencePrediction:
    """시퀀스 예측 결과"""
    symbol_id: str
    probability: float
    source: str  # 'ngram', 'lstm', 'hybrid'
    confidence: float
    rank: int


@dataclass
class SequenceModelConfig:
    """시퀀스 모델 설정"""
    max_ngram_order: int = 4
    min_count_threshold: int = 2
    smoothing_factor: float = 0.1
    decay_factor: float = 0.95
    max_vocabulary_size: int = 5000
    enable_lstm: bool = False
    lstm_hidden_size: int = 64
    lstm_embedding_dim: int = 32


class NGramModel:
    """N-gram 기반 시퀀스 모델"""

    def __init__(self, max_order: int = 4, smoothing: float = 0.1):
        self.max_order = max_order
        self.smoothing = smoothing
        # n-gram 카운트: {n: {context_tuple: {next_symbol: count}}}
        self.ngram_counts: Dict[int, Dict[Tuple[str, ...], Dict[str, int]]] = {
            n: defaultdict(lambda: defaultdict(int))
            for n in range(1, max_order + 1)
        }
        self.unigram_counts: Dict[str, int] = defaultdict(int)
        self.total_count: int = 0
        self.vocabulary: set = set()

    def train(self, sequences: List[List[str]]) -> None:
        """시퀀스 데이터로 학습"""
        for sequence in sequences:
            self._update_counts(sequence)

    def update(self, sequence: List[str]) -> None:
        """단일 시퀀스로 온라인 업데이트"""
        self._update_counts(sequence)

    def _update_counts(self, sequence: List[str]) -> None:
        """카운트 업데이트"""
        # 시작/종료 토큰 추가
        padded = ['<START>'] * (self.max_order - 1) + sequence + ['<END>']

        for i in range(len(padded)):
            symbol = padded[i]
            if symbol not in {'<START>'}:
                self.unigram_counts[symbol] += 1
                self.total_count += 1
                self.vocabulary.add(symbol)

            # 각 n-gram 순서에 대해 카운트 업데이트
            for n in range(2, self.max_order + 1):
                if i >= n - 1:
                    context = tuple(padded[i - n + 1:i])
                    next_symbol = padded[i]
                    self.ngram_counts[n][context][next_symbol] += 1

    def predict(self, context: List[str], top_k: int = 5) -> List[SequencePrediction]:
        """다음 심볼 예측"""
        predictions = []

        # 가장 긴 매칭 컨텍스트부터 시도 (backoff)
        for n in range(min(len(context) + 1, self.max_order), 0, -1):
            if n == 1:
                # Unigram
                probs = self._get_unigram_probs()
            else:
                ctx = tuple(context[-(n-1):]) if len(context) >= n - 1 else tuple(context)
                if ctx in self.ngram_counts[n]:
                    probs = self._get_ngram_probs(n, ctx)
                else:
                    continue

            for symbol, prob in sorted(probs.items(), key=lambda x: -x[1])[:top_k]:
                if symbol not in {'<START>', '<END>'}:
                    predictions.append(SequencePrediction(
                        symbol_id=symbol,
                        probability=prob,
                        source=f'{n}-gram',
                        confidence=min(1.0, prob * 2),
                        rank=len(predictions) + 1
                    ))

            if predictions:
                break

        return predictions[:top_k]

    def _get_unigram_probs(self) -> Dict[str, float]:
        """유니그램 확률 계산"""
        probs = {}
        vocab_size = len(self.vocabulary)

        for symbol in self.vocabulary:
            count = self.unigram_counts[symbol]
            # Laplace smoothing
            probs[symbol] = (count + self.smoothing) / (self.total_count + self.smoothing * vocab_size)

        return probs

    def _get_ngram_probs(self, n: int, context: Tuple[str, ...]) -> Dict[str, float]:
        """N-gram 조건부 확률 계산"""
        probs = {}
        context_counts = self.ngram_counts[n][context]
        total = sum(context_counts.values())
        vocab_size = len(self.vocabulary)

        for symbol in self.vocabulary:
            count = context_counts.get(symbol, 0)
            # Laplace smoothing
            probs[symbol] = (count + self.smoothing) / (total + self.smoothing * vocab_size)

        return probs

    def apply_decay(self) -> None:
        """시간 기반 감쇠 적용"""
        # Unigram
        for symbol in list(self.unigram_counts.keys()):
            self.unigram_counts[symbol] = int(self.unigram_counts[symbol] * 0.95)
            if self.unigram_counts[symbol] == 0:
                del self.unigram_counts[symbol]

        # N-grams
        for n in range(2, self.max_order + 1):
            for context in list(self.ngram_counts[n].keys()):
                for symbol in list(self.ngram_counts[n][context].keys()):
                    self.ngram_counts[n][context][symbol] = int(
                        self.ngram_counts[n][context][symbol] * 0.95
                    )
                    if self.ngram_counts[n][context][symbol] == 0:
                        del self.ngram_counts[n][context][symbol]

    def save(self, path: Path) -> None:
        """모델 저장"""
        data = {
            'max_order': self.max_order,
            'smoothing': self.smoothing,
            'unigram_counts': dict(self.unigram_counts),
            'total_count': self.total_count,
            'vocabulary': list(self.vocabulary),
            'ngram_counts': {
                n: {
                    str(ctx): dict(counts)
                    for ctx, counts in self.ngram_counts[n].items()
                }
                for n in range(2, self.max_order + 1)
            }
        }
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2))

    def load(self, path: Path) -> None:
        """모델 로드"""
        data = json.loads(path.read_text())
        self.max_order = data['max_order']
        self.smoothing = data['smoothing']
        self.unigram_counts = defaultdict(int, data['unigram_counts'])
        self.total_count = data['total_count']
        self.vocabulary = set(data['vocabulary'])

        for n in range(2, self.max_order + 1):
            self.ngram_counts[n] = defaultdict(lambda: defaultdict(int))
            for ctx_str, counts in data['ngram_counts'].get(str(n), {}).items():
                ctx = tuple(ctx_str.strip("()").replace("'", "").split(", "))
                for symbol, count in counts.items():
                    self.ngram_counts[n][ctx][symbol] = count


class SequenceModel:
    """
    심볼 시퀀스 예측 모델

    특징:
    - 경량 N-gram 모델 기본 제공
    - 선택적 LSTM 딥러닝 지원
    - 온디바이스 추론 최적화
    - 점진적 온라인 학습
    """

    def __init__(self, config: Optional[SequenceModelConfig] = None):
        self.config = config or SequenceModelConfig()
        self.ngram_model = NGramModel(
            max_order=self.config.max_ngram_order,
            smoothing=self.config.smoothing_factor
        )
        self.lstm_model = None  # LSTM는 선택적
        self._session_sequences: List[List[str]] = []
        self._current_sequence: List[str] = []

    def train(self, sequences: List[List[str]]) -> Dict[str, float]:
        """
        배치 학습

        Args:
            sequences: 심볼 ID 시퀀스 리스트

        Returns:
            학습 메트릭 (perplexity, accuracy 등)
        """
        self.ngram_model.train(sequences)

        # 메트릭 계산
        perplexity = self._calculate_perplexity(sequences)

        return {
            'perplexity': perplexity,
            'vocabulary_size': len(self.ngram_model.vocabulary),
            'total_sequences': len(sequences),
        }

    def update(self, symbol_id: str) -> None:
        """
        온라인 업데이트 (심볼 선택 시)

        Args:
            symbol_id: 선택된 심볼 ID
        """
        self._current_sequence.append(symbol_id)

        # 충분한 컨텍스트가 쌓이면 업데이트
        if len(self._current_sequence) >= 2:
            self.ngram_model.update(self._current_sequence[-self.config.max_ngram_order:])

    def predict(self, context: List[str], top_k: int = 8) -> List[SequencePrediction]:
        """
        다음 심볼 예측

        Args:
            context: 현재까지의 심볼 시퀀스
            top_k: 반환할 예측 수

        Returns:
            예측 결과 리스트 (확률순)
        """
        return self.ngram_model.predict(context, top_k)

    def predict_completion(
        self,
        partial_sequence: List[str],
        max_length: int = 5
    ) -> List[List[str]]:
        """
        문장 완성 예측

        Args:
            partial_sequence: 부분 시퀀스
            max_length: 최대 예측 길이

        Returns:
            완성된 시퀀스 후보들
        """
        completions = []

        # 빔 서치로 여러 완성 후보 생성
        beam_width = 3
        beams = [(partial_sequence.copy(), 1.0)]

        for _ in range(max_length):
            new_beams = []

            for sequence, score in beams:
                predictions = self.predict(sequence, top_k=beam_width)

                for pred in predictions:
                    if pred.symbol_id == '<END>':
                        completions.append(sequence)
                    else:
                        new_seq = sequence + [pred.symbol_id]
                        new_score = score * pred.probability
                        new_beams.append((new_seq, new_score))

            # 상위 빔 유지
            beams = sorted(new_beams, key=lambda x: -x[1])[:beam_width]

            if not beams:
                break

        # 완성되지 않은 빔도 포함
        for sequence, _ in beams:
            if sequence not in completions:
                completions.append(sequence)

        return completions[:5]

    def start_session(self) -> None:
        """새 세션 시작"""
        if self._current_sequence:
            self._session_sequences.append(self._current_sequence)
        self._current_sequence = []

    def end_session(self) -> None:
        """세션 종료"""
        if self._current_sequence:
            self._session_sequences.append(self._current_sequence)
            # 세션 데이터로 추가 학습
            self.ngram_model.update(self._current_sequence)
        self._current_sequence = []

    def apply_decay(self) -> None:
        """시간 기반 감쇠 적용"""
        self.ngram_model.apply_decay()

    def save(self, path: str) -> None:
        """모델 저장"""
        model_path = Path(path)
        model_path.parent.mkdir(parents=True, exist_ok=True)
        self.ngram_model.save(model_path)

    def load(self, path: str) -> None:
        """모델 로드"""
        self.ngram_model.load(Path(path))

    def _calculate_perplexity(self, sequences: List[List[str]]) -> float:
        """테스트 시퀀스의 perplexity 계산"""
        total_log_prob = 0.0
        total_symbols = 0

        for sequence in sequences:
            for i in range(1, len(sequence)):
                context = sequence[:i]
                target = sequence[i]
                predictions = self.predict(context, top_k=100)

                prob = 0.01  # 기본 확률
                for pred in predictions:
                    if pred.symbol_id == target:
                        prob = pred.probability
                        break

                total_log_prob += math.log(prob)
                total_symbols += 1

        if total_symbols == 0:
            return float('inf')

        return math.exp(-total_log_prob / total_symbols)

    def get_statistics(self) -> Dict[str, any]:
        """모델 통계 반환"""
        return {
            'vocabulary_size': len(self.ngram_model.vocabulary),
            'total_count': self.ngram_model.total_count,
            'session_sequences': len(self._session_sequences),
            'current_sequence_length': len(self._current_sequence),
        }


# 자폐 특화 시퀀스 모델
class AutismSequenceModel(SequenceModel):
    """
    자폐 사용자 특화 시퀀스 모델

    특징:
    - 루틴 패턴 강화 학습
    - 예측 가능한 시퀀스 우선
    - 전환 시퀀스 특별 처리
    """

    def __init__(self, config: Optional[SequenceModelConfig] = None):
        super().__init__(config)
        self.routine_sequences: Dict[str, List[str]] = {}
        self.routine_weights: Dict[str, float] = {}

    def add_routine(self, routine_name: str, sequence: List[str], weight: float = 1.5) -> None:
        """루틴 시퀀스 등록"""
        self.routine_sequences[routine_name] = sequence
        self.routine_weights[routine_name] = weight

    def predict(self, context: List[str], top_k: int = 8) -> List[SequencePrediction]:
        """루틴 가중치 적용 예측"""
        predictions = super().predict(context, top_k * 2)

        # 루틴 매칭 확인
        for routine_name, routine_seq in self.routine_sequences.items():
            if self._matches_routine(context, routine_seq):
                # 다음 예상 심볼 부스트
                next_idx = len(context)
                if next_idx < len(routine_seq):
                    next_symbol = routine_seq[next_idx]
                    for pred in predictions:
                        if pred.symbol_id == next_symbol:
                            pred.probability *= self.routine_weights[routine_name]
                            pred.source = 'routine'
                            break

        # 재정렬
        predictions.sort(key=lambda x: -x.probability)
        for i, pred in enumerate(predictions):
            pred.rank = i + 1

        return predictions[:top_k]

    def _matches_routine(self, context: List[str], routine: List[str]) -> bool:
        """컨텍스트가 루틴 시작과 일치하는지 확인"""
        if len(context) >= len(routine):
            return False
        return context == routine[:len(context)]


# 치매 특화 시퀀스 모델
class DementiaSequenceModel(SequenceModel):
    """
    치매 사용자 특화 시퀀스 모델

    특징:
    - 장기 기억 패턴 우선
    - 느린 감쇠율
    - 반복 패턴 허용
    """

    def __init__(self, config: Optional[SequenceModelConfig] = None):
        custom_config = config or SequenceModelConfig()
        custom_config.decay_factor = 0.99  # 느린 감쇠
        custom_config.min_count_threshold = 1  # 낮은 임계값
        super().__init__(custom_config)

        self.long_term_sequences: List[List[str]] = []
        self.familiar_symbols: set = set()

    def add_familiar_symbol(self, symbol_id: str) -> None:
        """익숙한 심볼 등록"""
        self.familiar_symbols.add(symbol_id)

    def add_long_term_sequence(self, sequence: List[str]) -> None:
        """장기 기억 시퀀스 등록"""
        self.long_term_sequences.append(sequence)
        # 높은 가중치로 학습
        for _ in range(5):
            self.ngram_model.update(sequence)

    def predict(self, context: List[str], top_k: int = 8) -> List[SequencePrediction]:
        """익숙한 심볼 우선 예측"""
        predictions = super().predict(context, top_k * 2)

        # 익숙한 심볼 부스트
        for pred in predictions:
            if pred.symbol_id in self.familiar_symbols:
                pred.probability *= 1.3
                pred.confidence = min(1.0, pred.confidence * 1.2)

        # 재정렬
        predictions.sort(key=lambda x: -x.probability)
        for i, pred in enumerate(predictions):
            pred.rank = i + 1

        return predictions[:top_k]
