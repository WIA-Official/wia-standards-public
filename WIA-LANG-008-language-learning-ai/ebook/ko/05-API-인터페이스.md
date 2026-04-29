# 제5장: API 인터페이스

## RESTful API 설계

WIA-LANG-008 API는 RESTful 원칙을 따르는 HTTP 기반 인터페이스를 제공하여, 언어 학습 AI 시스템의 모든 기능에 접근할 수 있게 합니다.

### API 기본 구조

```
Base URL: https://api.wia.org/lang-008/v1/

인증: Bearer Token (OAuth 2.0 / API Key)
Content-Type: application/json
Character Encoding: UTF-8
```

### HTTP 메서드 규칙

| 메서드 | 용도 | 멱등성 | 예시 |
|-------|------|-------|------|
| GET | 리소스 조회 | ✓ | 학습 콘텐츠 가져오기 |
| POST | 리소스 생성 | ✗ | 학습 이벤트 기록 |
| PUT | 리소스 전체 업데이트 | ✓ | 사용자 프로필 갱신 |
| PATCH | 리소스 부분 업데이트 | ✗ | 스킬 점수만 업데이트 |
| DELETE | 리소스 삭제 | ✓ | 학습 세션 삭제 |

## 인증 및 권한 관리

### OAuth 2.0 인증 플로우

```python
import requests

class WIALangAuth:
    """WIA-LANG-008 API 인증"""

    def __init__(self, client_id, client_secret):
        self.client_id = client_id
        self.client_secret = client_secret
        self.token_url = "https://auth.wia.org/oauth/token"
        self.access_token = None

    def get_access_token(self):
        """액세스 토큰 획득"""
        data = {
            "grant_type": "client_credentials",
            "client_id": self.client_id,
            "client_secret": self.client_secret,
            "scope": "lang-008:read lang-008:write lang-008:assess"
        }

        response = requests.post(self.token_url, data=data)
        response.raise_for_status()

        token_data = response.json()
        self.access_token = token_data["access_token"]
        self.expires_in = token_data["expires_in"]

        return self.access_token

    def refresh_token(self, refresh_token):
        """토큰 갱신"""
        data = {
            "grant_type": "refresh_token",
            "refresh_token": refresh_token,
            "client_id": self.client_id,
            "client_secret": self.client_secret
        }

        response = requests.post(self.token_url, data=data)
        response.raise_for_status()

        return response.json()

    def get_headers(self):
        """API 요청 헤더 생성"""
        if not self.access_token:
            self.get_access_token()

        return {
            "Authorization": f"Bearer {self.access_token}",
            "Content-Type": "application/json",
            "Accept": "application/json"
        }
```

### 권한 스코프

```yaml
scopes:
  lang-008:read:
    description: "학습 콘텐츠 및 프로필 읽기"
    permissions:
      - get_content
      - get_learner_profile
      - get_progress

  lang-008:write:
    description: "학습 데이터 기록"
    permissions:
      - log_learning_events
      - update_progress
      - submit_exercises

  lang-008:assess:
    description: "평가 수행 및 결과 조회"
    permissions:
      - administer_assessments
      - get_assessment_results
      - generate_reports

  lang-008:admin:
    description: "관리자 권한"
    permissions:
      - manage_users
      - manage_content
      - view_analytics
```

## 주요 API 엔드포인트

### 1. 콘텐츠 배송 (Content Delivery)

#### 학습 콘텐츠 조회

```http
GET /content/{content_id}
```

**요청 예시**:

```bash
curl -X GET "https://api.wia.org/lang-008/v1/content/ko-grammar-particles-001" \
  -H "Authorization: Bearer {access_token}"
```

**응답 예시**:

```json
{
  "status": "success",
  "data": {
    "content_id": "ko-grammar-particles-001",
    "title": "Korean Particles: 은/는, 이/가",
    "type": "lesson",
    "difficulty": 2.5,
    "cefr_level": "A1",
    "estimated_time_minutes": 15,

    "content": {
      "text": { },
      "audio": { },
      "video": { },
      "exercises": [ ]
    }
  }
}
```

#### 개인화된 콘텐츠 추천

```http
POST /recommendations/next-content
```

**요청 본문**:

```json
{
  "user_id": "usr_7x9k2m4p",
  "count": 5,
  "preferences": {
    "content_types": ["lesson", "exercise", "video"],
    "max_difficulty": 4.0,
    "topics": ["daily_life", "food"]
  }
}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "recommendations": [
      {
        "content_id": "ko-conversation-restaurant-001",
        "title": "Ordering at a Restaurant",
        "relevance_score": 0.92,
        "reasoning": "Matches your interest in food and current level",
        "difficulty": 3.2,
        "type": "conversation"
      },
      {
        "content_id": "ko-vocabulary-food-002",
        "title": "Korean Food Vocabulary",
        "relevance_score": 0.88,
        "reasoning": "Supports your restaurant conversation learning",
        "difficulty": 2.8,
        "type": "vocabulary"
      }
    ],
    "learning_path": {
      "path_id": "path_food_culture_a2",
      "name": "Korean Food & Culture",
      "total_lessons": 12,
      "your_progress": 3
    }
  }
}
```

### 2. 진도 추적 (Progress Tracking)

#### 학습 이벤트 기록

```http
POST /progress/events
```

**요청 본문**:

```json
{
  "user_id": "usr_7x9k2m4p",
  "event_type": "exercise_completed",
  "content_id": "ko-grammar-particles-ex-015",
  "timestamp": "2025-03-15T14:30:25Z",

  "performance": {
    "score": 0.85,
    "time_spent_seconds": 125,
    "attempts": 1,
    "hints_used": 1
  },

  "detailed_results": {
    "questions": [
      {
        "question_id": "q1",
        "user_answer": "는",
        "correct_answer": "는",
        "is_correct": true
      }
    ]
  }
}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "event_id": "evt_xk4p9m2n",
    "recorded_at": "2025-03-15T14:30:26Z",

    "skill_updates": {
      "grammar": {
        "previous_mastery": 0.72,
        "new_mastery": 0.75,
        "change": 0.03
      }
    },

    "achievements_unlocked": [],

    "next_recommendation": {
      "content_id": "ko-grammar-particles-ex-016",
      "reason": "Continue practicing particles"
    }
  }
}
```

#### 진도 조회

```http
GET /progress/users/{user_id}?period=30days
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "user_id": "usr_7x9k2m4p",
    "period": "30days",

    "summary": {
      "total_study_time_hours": 12.5,
      "lessons_completed": 18,
      "exercises_completed": 142,
      "streak_days": 15,
      "current_level": "A2"
    },

    "skill_breakdown": {
      "listening": {
        "cefr": "A2",
        "score": 3.5,
        "progress_trend": "improving",
        "change_last_30_days": 0.3
      },
      "speaking": {
        "cefr": "A2",
        "score": 2.8,
        "progress_trend": "stable",
        "change_last_30_days": 0.1
      }
    },

    "learning_patterns": {
      "most_active_time": "08:00-09:00",
      "average_session_duration_minutes": 22,
      "weekly_active_days": 5.2
    }
  }
}
```

### 3. 음성 평가 (Speech Assessment)

#### 발음 평가

```http
POST /assess/pronunciation
```

**요청** (multipart/form-data):

```bash
curl -X POST "https://api.wia.org/lang-008/v1/assess/pronunciation" \
  -H "Authorization: Bearer {access_token}" \
  -F "audio=@recording.wav" \
  -F "reference_text=안녕하세요" \
  -F "language=ko" \
  -F "user_id=usr_7x9k2m4p"
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "assessment_id": "pron_assess_20250315_001",

    "overall_scores": {
      "overall": 78,
      "accuracy": 82,
      "fluency": 71,
      "completeness": 100,
      "prosody": 65
    },

    "phoneme_analysis": [
      {
        "phoneme": "ㅓ",
        "score": 62,
        "feedback": "Tongue position slightly too central",
        "reference_audio_url": "https://cdn.wia.org/phonemes/ko-eo.mp3"
      }
    ],

    "actionable_feedback": [
      {
        "priority": "high",
        "issue": "ㅓ vowel pronunciation",
        "practice_words": ["어머니", "서울", "거리"],
        "tutorial_video": "https://cdn.wia.org/tutorials/ko-vowel-eo.mp4"
      }
    ]
  }
}
```

### 4. 문법 교정 (Grammar Correction)

#### 작문 평가 및 교정

```http
POST /assess/writing
```

**요청**:

```json
{
  "user_id": "usr_7x9k2m4p",
  "text": "어제 나는 친구랑 쇼핑을 갔어요. 우리는 새 옷을 샀어요.",
  "language": "ko",
  "proficiency_level": "A2",
  "feedback_detail": "comprehensive"
}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "original_text": "어제 나는 친구랑 쇼핑을 갔어요. 우리는 새 옷을 샀어요.",

    "analysis": {
      "grammar_errors": [],
      "style_suggestions": [
        {
          "position": {"start": 3, "end": 5},
          "original": "나는",
          "suggestion": "저는",
          "type": "formality",
          "explanation": "In polite conversation (-어요 ending), use 저는 instead of 나는",
          "severity": "medium"
        }
      ],
      "vocabulary_level": "appropriate",
      "sentence_structure": "good"
    },

    "corrected_versions": {
      "minimal": "어제 저는 친구랑 쇼핑을 갔어요. 우리는 새 옷을 샀어요.",
      "improved": "어제 저는 친구와 함께 쇼핑을 갔어요. 우리는 새 옷을 샀어요.",
      "native_like": "어제 친구와 함께 쇼핑을 갔어요. 새 옷도 샀고요."
    },

    "learning_points": [
      {
        "grammar_point": "formality_consistency",
        "explanation": "When using polite endings like -어요, use formal pronouns like 저",
        "related_lesson": "ko-grammar-formality-001"
      }
    ],

    "scores": {
      "grammar": 85,
      "vocabulary": 80,
      "coherence": 90,
      "task_achievement": 95
    }
  }
}
```

### 5. 어휘 학습 (Vocabulary Learning)

#### 단어 추천

```http
GET /vocabulary/recommendations?user_id={user_id}&count=10&context=food
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "recommendations": [
      {
        "word": "김치",
        "pronunciation": "[kimchi]",
        "meaning": "kimchi (fermented vegetables)",
        "difficulty": 1.5,
        "frequency_rank": 450,
        "relevance_score": 0.95,
        "reason": "Essential food vocabulary",
        "example_sentence": "김치가 매워요.",
        "example_translation": "Kimchi is spicy.",
        "image_url": "https://cdn.wia.org/images/kimchi.jpg",
        "audio_url": "https://cdn.wia.org/audio/ko-kimchi.mp3"
      }
    ],

    "learning_context": {
      "topic": "food",
      "related_words_already_known": ["밥", "먹다", "맛있다"],
      "next_words_in_sequence": ["반찬", "국", "찌개"]
    }
  }
}
```

#### 간격 반복 복습 조회

```http
GET /vocabulary/review-due?user_id={user_id}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "due_count": 15,
    "overdue_count": 3,

    "cards": [
      {
        "card_id": "srs_ko_word_12450",
        "word": "환경",
        "meaning": "environment",
        "next_review": "2025-03-15",
        "status": "due",
        "interval_days": 14,
        "ease_factor": 2.6
      }
    ],

    "review_session": {
      "estimated_time_minutes": 8,
      "recommended_now": true,
      "optimal_review_time": "08:00-09:00"
    }
  }
}
```

#### 복습 결과 제출

```http
POST /vocabulary/review
```

**요청**:

```json
{
  "user_id": "usr_7x9k2m4p",
  "card_id": "srs_ko_word_12450",
  "recall_quality": 4,
  "response_time_seconds": 5
}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "card_id": "srs_ko_word_12450",

    "updated_schedule": {
      "previous_interval_days": 14,
      "new_interval_days": 35,
      "next_review_date": "2025-04-19",
      "ease_factor": 2.7
    },

    "progress": {
      "repetitions": 6,
      "mastery_level": "advanced",
      "retention_probability": 0.85
    }
  }
}
```

### 6. 숙련도 평가 (Proficiency Assessment)

#### 평가 시작

```http
POST /assessments/start
```

**요청**:

```json
{
  "user_id": "usr_7x9k2m4p",
  "assessment_type": "proficiency_test",
  "language": "ko",
  "skills": ["listening", "reading", "writing", "speaking"]
}
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "assessment_id": "assess_2025_03_15_001",
    "started_at": "2025-03-15T10:00:00Z",
    "estimated_duration_minutes": 90,

    "instructions": {
      "general": "This assessment will test your Korean proficiency across 4 skills",
      "requirements": [
        "Quiet environment",
        "Microphone for speaking section",
        "Stable internet connection"
      ]
    },

    "first_question": {
      "question_id": "q_listening_001",
      "type": "listening_comprehension",
      "audio_url": "https://cdn.wia.org/assessments/q_listening_001.mp3",
      "question_text": "What is the speaker talking about?",
      "options": ["Weather", "Food", "Travel", "Work"]
    }
  }
}
```

#### 평가 결과 조회

```http
GET /assessments/{assessment_id}/results
```

**응답**:

```json
{
  "status": "success",
  "data": {
    "assessment_id": "assess_2025_03_15_001",
    "completed_at": "2025-03-15T11:32:00Z",

    "overall_result": {
      "cefr_level": "A2",
      "confidence": 0.87,
      "numeric_score": 450,
      "percentile": 62
    },

    "skill_scores": {
      "listening": {"cefr": "B1", "score": 18, "max": 25},
      "reading": {"cefr": "A2", "score": 15, "max": 25},
      "writing": {"cefr": "A2", "score": 14, "max": 25},
      "speaking": {"cefr": "A2", "score": 13, "max": 25}
    },

    "recommendations": {
      "focus_areas": ["speaking_fluency", "writing_organization"],
      "estimated_time_to_b1": "24 weeks at 5 hours/week",
      "suggested_courses": ["korean_conversation_a2_b1"]
    },

    "certificate_url": "https://certificates.wia.org/lang-008/assess_2025_03_15_001.pdf"
  }
}
```

## SDK 구현 예시

### Python SDK

```python
from wia_lang008 import WIALanguageAPI

class WIALanguageAPI:
    """WIA-LANG-008 Python SDK"""

    def __init__(self, api_key, base_url="https://api.wia.org/lang-008/v1"):
        self.api_key = api_key
        self.base_url = base_url
        self.session = requests.Session()
        self.session.headers.update({
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        })

    # 콘텐츠 API
    def get_content(self, content_id):
        """학습 콘텐츠 조회"""
        response = self.session.get(f"{self.base_url}/content/{content_id}")
        response.raise_for_status()
        return response.json()["data"]

    def get_recommendations(self, user_id, count=5, preferences=None):
        """개인화 콘텐츠 추천"""
        payload = {
            "user_id": user_id,
            "count": count,
            "preferences": preferences or {}
        }
        response = self.session.post(
            f"{self.base_url}/recommendations/next-content",
            json=payload
        )
        response.raise_for_status()
        return response.json()["data"]["recommendations"]

    # 진도 API
    def log_event(self, user_id, event_type, content_id, performance):
        """학습 이벤트 기록"""
        payload = {
            "user_id": user_id,
            "event_type": event_type,
            "content_id": content_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "performance": performance
        }
        response = self.session.post(
            f"{self.base_url}/progress/events",
            json=payload
        )
        response.raise_for_status()
        return response.json()["data"]

    def get_progress(self, user_id, period="30days"):
        """진도 조회"""
        response = self.session.get(
            f"{self.base_url}/progress/users/{user_id}",
            params={"period": period}
        )
        response.raise_for_status()
        return response.json()["data"]

    # 평가 API
    def assess_pronunciation(self, audio_file, reference_text, language, user_id):
        """발음 평가"""
        files = {"audio": open(audio_file, "rb")}
        data = {
            "reference_text": reference_text,
            "language": language,
            "user_id": user_id
        }
        response = self.session.post(
            f"{self.base_url}/assess/pronunciation",
            files=files,
            data=data
        )
        response.raise_for_status()
        return response.json()["data"]

    def correct_writing(self, user_id, text, language, level="A2"):
        """작문 교정"""
        payload = {
            "user_id": user_id,
            "text": text,
            "language": language,
            "proficiency_level": level,
            "feedback_detail": "comprehensive"
        }
        response = self.session.post(
            f"{self.base_url}/assess/writing",
            json=payload
        )
        response.raise_for_status()
        return response.json()["data"]

    # 어휘 API
    def get_vocabulary_recommendations(self, user_id, count=10, context=None):
        """어휘 추천"""
        params = {"user_id": user_id, "count": count}
        if context:
            params["context"] = context
        response = self.session.get(
            f"{self.base_url}/vocabulary/recommendations",
            params=params
        )
        response.raise_for_status()
        return response.json()["data"]["recommendations"]

    def get_review_due(self, user_id):
        """복습 예정 단어 조회"""
        response = self.session.get(
            f"{self.base_url}/vocabulary/review-due",
            params={"user_id": user_id}
        )
        response.raise_for_status()
        return response.json()["data"]

    def submit_review(self, user_id, card_id, recall_quality, response_time):
        """복습 결과 제출"""
        payload = {
            "user_id": user_id,
            "card_id": card_id,
            "recall_quality": recall_quality,
            "response_time_seconds": response_time
        }
        response = self.session.post(
            f"{self.base_url}/vocabulary/review",
            json=payload
        )
        response.raise_for_status()
        return response.json()["data"]
```

### 사용 예시

```python
# SDK 초기화
api = WIALanguageAPI(api_key="your_api_key_here")

# 1. 학습 콘텐츠 추천 받기
recommendations = api.get_recommendations(
    user_id="usr_7x9k2m4p",
    count=5,
    preferences={"topics": ["food", "travel"]}
)

for rec in recommendations:
    print(f"추천: {rec['title']} (관련도: {rec['relevance_score']:.2f})")

# 2. 학습 진행
content = api.get_content(recommendations[0]["content_id"])
print(f"학습 중: {content['title']}")

# 3. 진도 기록
api.log_event(
    user_id="usr_7x9k2m4p",
    event_type="lesson_completed",
    content_id=content["content_id"],
    performance={"score": 0.90, "time_spent_seconds": 720}
)

# 4. 발음 평가
pronunciation_result = api.assess_pronunciation(
    audio_file="my_recording.wav",
    reference_text="안녕하세요",
    language="ko",
    user_id="usr_7x9k2m4p"
)
print(f"발음 점수: {pronunciation_result['overall_scores']['overall']}/100")

# 5. 어휘 복습
due_cards = api.get_review_due("usr_7x9k2m4p")
print(f"복습할 단어: {due_cards['due_count']}개")

for card in due_cards["cards"]:
    recall_quality = do_flashcard_review(card)  # 사용자 복습
    api.submit_review(
        user_id="usr_7x9k2m4p",
        card_id=card["card_id"],
        recall_quality=recall_quality,
        response_time=5
    )
```

### JavaScript/TypeScript SDK

```typescript
class WIALanguageAPI {
  private apiKey: string;
  private baseURL: string;

  constructor(apiKey: string, baseURL = "https://api.wia.org/lang-008/v1") {
    this.apiKey = apiKey;
    this.baseURL = baseURL;
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const url = `${this.baseURL}${endpoint}`;
    const headers = {
      "Authorization": `Bearer ${this.apiKey}`,
      "Content-Type": "application/json",
      ...options.headers
    };

    const response = await fetch(url, { ...options, headers });

    if (!response.ok) {
      throw new Error(`API error: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return data.data as T;
  }

  // 콘텐츠 API
  async getContent(contentId: string): Promise<LearningContent> {
    return this.request<LearningContent>(`/content/${contentId}`);
  }

  async getRecommendations(
    userId: string,
    count = 5,
    preferences?: ContentPreferences
  ): Promise<ContentRecommendation[]> {
    return this.request<ContentRecommendation[]>("/recommendations/next-content", {
      method: "POST",
      body: JSON.stringify({ user_id: userId, count, preferences })
    });
  }

  // 진도 API
  async logEvent(event: LearningEvent): Promise<EventResponse> {
    return this.request<EventResponse>("/progress/events", {
      method: "POST",
      body: JSON.stringify(event)
    });
  }

  async getProgress(userId: string, period = "30days"): Promise<ProgressData> {
    return this.request<ProgressData>(`/progress/users/${userId}?period=${period}`);
  }

  // 평가 API
  async assessPronunciation(
    audioBlob: Blob,
    referenceText: string,
    language: string,
    userId: string
  ): Promise<PronunciationAssessment> {
    const formData = new FormData();
    formData.append("audio", audioBlob, "recording.wav");
    formData.append("reference_text", referenceText);
    formData.append("language", language);
    formData.append("user_id", userId);

    const url = `${this.baseURL}/assess/pronunciation`;
    const response = await fetch(url, {
      method: "POST",
      headers: { "Authorization": `Bearer ${this.apiKey}` },
      body: formData
    });

    const data = await response.json();
    return data.data as PronunciationAssessment;
  }

  async correctWriting(
    userId: string,
    text: string,
    language: string,
    level = "A2"
  ): Promise<WritingCorrection> {
    return this.request<WritingCorrection>("/assess/writing", {
      method: "POST",
      body: JSON.stringify({
        user_id: userId,
        text,
        language,
        proficiency_level: level,
        feedback_detail: "comprehensive"
      })
    });
  }

  // 어휘 API
  async getVocabularyRecommendations(
    userId: string,
    count = 10,
    context?: string
  ): Promise<VocabularyWord[]> {
    const params = new URLSearchParams({ user_id: userId, count: count.toString() });
    if (context) params.append("context", context);

    return this.request<VocabularyWord[]>(`/vocabulary/recommendations?${params}`);
  }

  async getReviewDue(userId: string): Promise<ReviewSession> {
    return this.request<ReviewSession>(`/vocabulary/review-due?user_id=${userId}`);
  }

  async submitReview(
    userId: string,
    cardId: string,
    recallQuality: number,
    responseTime: number
  ): Promise<ReviewResult> {
    return this.request<ReviewResult>("/vocabulary/review", {
      method: "POST",
      body: JSON.stringify({
        user_id: userId,
        card_id: cardId,
        recall_quality: recallQuality,
        response_time_seconds: responseTime
      })
    });
  }
}

// 사용 예시
const api = new WIALanguageAPI("your_api_key");

// React 컴포넌트에서 사용
async function LearningDashboard({ userId }) {
  const [progress, setProgress] = useState(null);
  const [recommendations, setRecommendations] = useState([]);

  useEffect(() => {
    async function loadData() {
      const progressData = await api.getProgress(userId);
      setProgress(progressData);

      const recs = await api.getRecommendations(userId, 5);
      setRecommendations(recs);
    }
    loadData();
  }, [userId]);

  return (
    <div>
      <h2>Your Progress</h2>
      <ProgressChart data={progress} />

      <h2>Recommended for You</h2>
      <ContentList items={recommendations} />
    </div>
  );
}
```

## 에러 처리

### 표준 에러 응답

```json
{
  "status": "error",
  "error": {
    "code": "INVALID_AUDIO_FORMAT",
    "message": "Audio file must be WAV format with 16kHz sample rate",
    "details": {
      "received_format": "mp3",
      "required_format": "wav",
      "required_sample_rate": 16000
    }
  },
  "request_id": "req_xm9k4p2n",
  "timestamp": "2025-03-15T14:35:22Z"
}
```

### 에러 코드 목록

| 코드 | HTTP 상태 | 설명 |
|-----|---------|------|
| UNAUTHORIZED | 401 | 인증 실패 |
| FORBIDDEN | 403 | 권한 부족 |
| NOT_FOUND | 404 | 리소스 없음 |
| INVALID_REQUEST | 400 | 잘못된 요청 |
| RATE_LIMIT_EXCEEDED | 429 | 요청 한도 초과 |
| INTERNAL_ERROR | 500 | 서버 오류 |

---

## 핵심 요약

**주요 내용:**

1. **RESTful 설계**: HTTP 메서드를 적절히 사용하는 직관적인 API 구조.

2. **OAuth 2.0 인증**: 표준 인증 프로토콜로 보안성과 유연성 보장.

3. **6대 핵심 엔드포인트**: 콘텐츠 배송, 진도 추적, 음성 평가, 문법 교정, 어휘 학습, 숙련도 평가.

4. **SDK 제공**: Python, JavaScript/TypeScript SDK로 쉬운 통합.

5. **표준 에러 처리**: 일관된 에러 응답 형식으로 디버깅 용이.

6. **실전 예시**: 실제 사용 케이스를 보여주는 풍부한 코드 예제.

---

## 복습 문제

1. WIA-LANG-008 API의 OAuth 2.0 인증 플로우를 설명하고, 액세스 토큰의 역할과 갱신 방법을 논하시오.

2. 개인화 콘텐츠 추천 API의 요청/응답 구조를 설명하고, relevance_score가 어떻게 계산되는지 추론하시오.

3. 발음 평가 API의 4가지 평가 지표(accuracy, fluency, completeness, prosody)를 설명하고, API 응답에서 어떻게 표현되는지 분석하시오.

4. 간격 반복 시스템의 API 플로우(review-due 조회 → 복습 수행 → 결과 제출 → 일정 업데이트)를 설명하고, SM-2 알고리즘이 어떻게 적용되는지 논하시오.

5. Python SDK와 JavaScript SDK의 구현 차이를 비교하고, 각 언어의 특성을 고려한 설계 선택을 분석하시오.

6. API 에러 처리 메커니즘을 설명하고, 클라이언트가 다양한 에러 상황(인증 실패, 요청 한도 초과, 서버 오류)을 어떻게 처리해야 하는지 논하시오.

---

## 다음 장 미리보기

제6장에서는 WIA-LANG-008 프로토콜을 상세히 다룹니다. 학습 프로토콜 정의, 실시간 통신(WebSocket), 데이터 동기화, 오프라인 모드 지원, 그리고 보안 프로토콜을 살펴봅니다.

---

**제5장 완료** | 예상 페이지: 22

[이전: 제4장 - 데이터 형식](./04-데이터-형식.md) | [다음: 제6장 - 프로토콜](./06-프로토콜.md)

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
