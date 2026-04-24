# WIA AI Standard - Phase 4: Ecosystem Integration

**Version**: 1.0.0
**Status**: Draft
**Date**: 2024

---

## 1. 개요 (Overview)

### 1.1 목적

WIA AI Standard Phase 4는 AI 시스템과 WIA 생태계의 양방향 연동을 정의합니다. 이를 통해:

- AAC, BCI, Voice 등 WIA 입력 표준에서 AI 처리 활용
- AI 출력을 TTS, ISP(수어), Braille 등으로 변환
- 모든 WIA 표준 간 상호운용성 확보

### 1.2 범위

```
WIA 생태계
├── 입력 표준 (Input)
│   ├── WIA AAC (보완대체의사소통)
│   ├── WIA BCI (뇌-컴퓨터 인터페이스)
│   ├── WIA Voice (음성 인터페이스)
│   └── 기타 센서 입력
│
├── AI 처리 계층 (Processing)
│   ├── WIA AI Standard (본 문서)
│   └── 에이전트, 모델, 도구
│
└── 출력 표준 (Output)
    ├── TTS (텍스트-음성)
    ├── ISP (국제수어프로토콜)
    ├── WIA Braille (점자)
    └── 기타 출력
```

### 1.3 설계 원칙

1. **추상화**: 커넥터 인터페이스로 WIA 표준 추상화
2. **확장성**: 새로운 WIA 표준 쉽게 추가 가능
3. **접근성**: WCAG 2.1 Level AA 준수
4. **비동기**: 모든 I/O 작업 async 처리
5. **테스트 용이**: Mock 구현 제공

---

## 2. 아키텍처 (Architecture)

### 2.1 계층 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│              (AAC App, BCI App, Voice Assistant)             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      WIA-AI Hub                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                 Connector Registry                   │    │
│  │   AAC | BCI | Voice | TTS | ISP | Braille | ...     │    │
│  └─────────────────────────────────────────────────────┘    │
│                              │                               │
│  ┌──────────────┐    ┌──────────────┐                       │
│  │Input Adapters│    │Output Adapters│                       │
│  │ - Text       │    │ - Speech     │                       │
│  │ - Audio      │    │ - SignLang   │                       │
│  │ - Signal     │    │ - Braille    │                       │
│  │ - Image      │    │ - Text       │                       │
│  └──────┬───────┘    └──────┬───────┘                       │
│         │                   │                                │
│         ▼                   ▼                                │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              AI Processing Pipeline                  │    │
│  │  Agent Registry → Model → Tools → Response          │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    WIA Standards Layer                       │
│   AAC Spec | BCI Spec | Voice Spec | Braille Spec | ...     │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 데이터 흐름

```
입력 흐름:
WIA Device → WIA Message → Input Adapter → AI Input → Agent → AI Output

출력 흐름:
AI Output → Output Adapter → WIA Message → WIA Device → User
```

---

## 3. 핵심 인터페이스 (Core Interfaces)

### 3.1 WIA 표준 타입

```rust
/// WIA 표준 타입 열거형
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WiaStandardType {
    /// 보완대체의사소통 (Augmentative and Alternative Communication)
    Aac,
    /// 뇌-컴퓨터 인터페이스 (Brain-Computer Interface)
    Bci,
    /// 음성 인터페이스 (Voice Interface)
    Voice,
    /// 텍스트-음성 변환 (Text-to-Speech)
    Tts,
    /// 국제수어프로토콜 (International Sign Protocol)
    Isp,
    /// 점자 (Braille)
    Braille,
    /// XR (확장현실)
    Xr,
    /// 로봇
    Robot,
    /// 스마트홈
    SmartHome,
    /// 사용자 정의
    Custom(String),
}
```

### 3.2 WIA 메시지

```rust
/// WIA 표준 간 교환 메시지
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaMessage {
    /// 메시지 ID
    pub id: String,
    /// 발신 표준
    pub source: WiaStandardType,
    /// 수신 표준
    pub target: WiaStandardType,
    /// 메시지 타입
    pub message_type: WiaMessageType,
    /// 페이로드
    pub payload: WiaPayload,
    /// 메타데이터
    pub metadata: HashMap<String, Value>,
    /// 타임스탬프
    pub timestamp: DateTime<Utc>,
}

/// 메시지 타입
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WiaMessageType {
    /// 텍스트 메시지
    Text,
    /// 오디오 데이터
    Audio,
    /// 신호 데이터 (센서, BCI 등)
    Signal,
    /// 이미지 데이터
    Image,
    /// 명령
    Command,
    /// 응답
    Response,
    /// 이벤트
    Event,
    /// 에러
    Error,
}

/// 페이로드
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum WiaPayload {
    Text(String),
    Binary(Vec<u8>),
    Json(Value),
    Signal(SignalData),
}

/// 신호 데이터
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalData {
    pub channels: Vec<String>,
    pub samples: Vec<Vec<f64>>,
    pub sample_rate: f64,
    pub unit: String,
}
```

### 3.3 WIA 커넥터 트레이트

```rust
/// WIA 표준 연동 커넥터
#[async_trait]
pub trait WiaConnector: Send + Sync {
    /// 커넥터 식별자
    fn id(&self) -> &str;

    /// 연결하는 WIA 표준 타입
    fn standard_type(&self) -> WiaStandardType;

    /// 연결 상태 확인
    fn is_connected(&self) -> bool;

    /// 연결 수립
    async fn connect(&mut self) -> Result<(), WiaAiError>;

    /// 연결 해제
    async fn disconnect(&mut self) -> Result<(), WiaAiError>;

    /// 메시지 수신 (비동기)
    async fn receive(&self) -> Result<WiaMessage, WiaAiError>;

    /// 메시지 송신
    async fn send(&self, message: WiaMessage) -> Result<(), WiaAiError>;

    /// 메시지 스트림 수신
    fn receive_stream(&self) -> Pin<Box<dyn Stream<Item = Result<WiaMessage, WiaAiError>> + Send>>;
}
```

---

## 4. 입력 어댑터 (Input Adapters)

### 4.1 입력 어댑터 트레이트

```rust
/// AI 입력 타입
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AiInputType {
    Text,
    Audio,
    Image,
    Signal,
    Multimodal,
}

/// AI 입력 데이터
#[derive(Debug, Clone)]
pub struct AiInput {
    /// 입력 타입
    pub input_type: AiInputType,
    /// 텍스트 내용 (있는 경우)
    pub text: Option<String>,
    /// 바이너리 데이터 (오디오, 이미지 등)
    pub data: Option<Vec<u8>>,
    /// 원본 WIA 메시지
    pub source_message: Option<WiaMessage>,
    /// 컨텍스트
    pub context: HashMap<String, Value>,
}

/// WIA → AI 입력 변환 어댑터
#[async_trait]
pub trait AiInputAdapter: Send + Sync {
    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 지원하는 입력 타입
    fn input_type(&self) -> AiInputType;

    /// 지원하는 WIA 표준 타입들
    fn supported_standards(&self) -> Vec<WiaStandardType>;

    /// WIA 메시지를 AI 입력으로 변환
    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError>;

    /// 이 메시지를 처리할 수 있는지 확인
    fn can_handle(&self, message: &WiaMessage) -> bool;
}
```

### 4.2 AAC 입력 어댑터

```rust
/// AAC 센서 신호 → AI 입력 변환
pub struct AacInputAdapter {
    /// 지원하는 센서 타입
    supported_sensors: Vec<String>,
    /// 신호 처리기
    signal_processor: Box<dyn SignalProcessor>,
}

impl AacInputAdapter {
    pub fn new() -> Self {
        Self {
            supported_sensors: vec![
                "eye_tracker".into(),
                "emg".into(),
                "switch".into(),
                "breath".into(),
                "eeg".into(),
            ],
            signal_processor: Box::new(DefaultSignalProcessor),
        }
    }
}

#[async_trait]
impl AiInputAdapter for AacInputAdapter {
    fn name(&self) -> &str {
        "aac_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Signal
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Aac]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        // AAC 신호를 텍스트 의도로 변환
        let intent = self.signal_processor.process(&message.payload)?;

        Ok(AiInput {
            input_type: AiInputType::Text,
            text: Some(intent),
            data: None,
            source_message: Some(message),
            context: HashMap::new(),
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == WiaStandardType::Aac
    }
}
```

### 4.3 BCI 입력 어댑터

```rust
/// BCI 뇌파 신호 → AI 입력 변환
pub struct BciInputAdapter {
    /// EEG 채널 설정
    channels: Vec<String>,
    /// 분류 모델
    classifier: Box<dyn BciClassifier>,
}

#[async_trait]
impl AiInputAdapter for BciInputAdapter {
    fn name(&self) -> &str {
        "bci_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Signal
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Bci]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        // BCI 신호 분류
        let classification = self.classifier.classify(&message.payload)?;

        Ok(AiInput {
            input_type: AiInputType::Text,
            text: Some(classification.intent),
            data: None,
            source_message: Some(message),
            context: [
                ("confidence".into(), json!(classification.confidence)),
                ("mental_state".into(), json!(classification.state)),
            ].into(),
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == WiaStandardType::Bci
    }
}
```

### 4.4 Voice 입력 어댑터

```rust
/// 음성 → AI 입력 변환
pub struct VoiceInputAdapter {
    /// ASR (음성인식) 엔진
    asr_engine: Box<dyn AsrEngine>,
}

#[async_trait]
impl AiInputAdapter for VoiceInputAdapter {
    fn name(&self) -> &str {
        "voice_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Audio
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Voice]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        // 음성을 텍스트로 변환
        let transcript = self.asr_engine.transcribe(&message.payload).await?;

        Ok(AiInput {
            input_type: AiInputType::Text,
            text: Some(transcript.text),
            data: None,
            source_message: Some(message),
            context: [
                ("language".into(), json!(transcript.language)),
                ("confidence".into(), json!(transcript.confidence)),
            ].into(),
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == WiaStandardType::Voice
    }
}
```

---

## 5. 출력 어댑터 (Output Adapters)

### 5.1 출력 어댑터 트레이트

```rust
/// AI 출력 타입
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AiOutputType {
    Text,
    Speech,
    SignLanguage,
    Braille,
    Multimodal,
}

/// AI 출력 데이터
#[derive(Debug, Clone)]
pub struct AiOutput {
    /// 출력 타입
    pub output_type: AiOutputType,
    /// 텍스트 내용
    pub text: String,
    /// 추가 데이터
    pub data: Option<Vec<u8>>,
    /// 메타데이터
    pub metadata: HashMap<String, Value>,
}

/// AI → WIA 출력 변환 어댑터
#[async_trait]
pub trait AiOutputAdapter: Send + Sync {
    /// 어댑터 이름
    fn name(&self) -> &str;

    /// 출력 타입
    fn output_type(&self) -> AiOutputType;

    /// 대상 WIA 표준
    fn target_standard(&self) -> WiaStandardType;

    /// AI 출력을 WIA 메시지로 변환
    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError>;

    /// 스트리밍 출력 지원 여부
    fn supports_streaming(&self) -> bool {
        false
    }

    /// 스트리밍 출력 처리
    async fn stream_output(
        &self,
        output: Pin<Box<dyn Stream<Item = AiOutput> + Send>>
    ) -> Result<(), WiaAiError> {
        Err(WiaAiError::NotSupported("streaming".into()))
    }
}
```

### 5.2 TTS 출력 어댑터

```rust
/// AI 텍스트 → 음성 출력
pub struct TtsOutputAdapter {
    /// TTS 엔진
    tts_engine: Box<dyn TtsEngine>,
    /// 기본 음성 설정
    default_voice: VoiceConfig,
}

#[derive(Debug, Clone)]
pub struct VoiceConfig {
    pub voice_id: String,
    pub language: String,
    pub speed: f32,
    pub pitch: f32,
    pub volume: f32,
}

#[async_trait]
impl AiOutputAdapter for TtsOutputAdapter {
    fn name(&self) -> &str {
        "tts_output"
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::Speech
    }

    fn target_standard(&self) -> WiaStandardType {
        WiaStandardType::Tts
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        // 텍스트를 음성으로 합성
        let audio = self.tts_engine.synthesize(&output.text, &self.default_voice).await?;

        Ok(WiaMessage {
            id: Uuid::new_v4().to_string(),
            source: WiaStandardType::Custom("ai".into()),
            target: WiaStandardType::Tts,
            message_type: WiaMessageType::Audio,
            payload: WiaPayload::Binary(audio),
            metadata: [
                ("format".into(), json!("wav")),
                ("sample_rate".into(), json!(22050)),
            ].into(),
            timestamp: Utc::now(),
        })
    }

    fn supports_streaming(&self) -> bool {
        true
    }
}
```

### 5.3 수어 (ISP) 출력 어댑터

```rust
/// AI 텍스트 → 수어 코드 출력
pub struct IspOutputAdapter {
    /// 텍스트 → ISP 변환기
    text_to_isp: Box<dyn TextToIspConverter>,
}

#[async_trait]
impl AiOutputAdapter for IspOutputAdapter {
    fn name(&self) -> &str {
        "isp_output"
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::SignLanguage
    }

    fn target_standard(&self) -> WiaStandardType {
        WiaStandardType::Isp
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        // 텍스트를 ISP 코드로 변환
        let isp_codes = self.text_to_isp.convert(&output.text).await?;

        Ok(WiaMessage {
            id: Uuid::new_v4().to_string(),
            source: WiaStandardType::Custom("ai".into()),
            target: WiaStandardType::Isp,
            message_type: WiaMessageType::Command,
            payload: WiaPayload::Json(json!({
                "gestures": isp_codes,
                "original_text": output.text,
            })),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        })
    }
}
```

### 5.4 점자 출력 어댑터

```rust
/// AI 텍스트 → 점자 출력
pub struct BrailleOutputAdapter {
    /// 텍스트 → 점자 변환기
    text_to_braille: Box<dyn TextToBrailleConverter>,
    /// 기본 점자 등급
    default_grade: BrailleGrade,
}

#[derive(Debug, Clone)]
pub enum BrailleGrade {
    Grade1,  // 축약 없음
    Grade2,  // 축약 점자
}

#[async_trait]
impl AiOutputAdapter for BrailleOutputAdapter {
    fn name(&self) -> &str {
        "braille_output"
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::Braille
    }

    fn target_standard(&self) -> WiaStandardType {
        WiaStandardType::Braille
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        // 텍스트를 점자로 변환
        let braille = self.text_to_braille.convert(&output.text, &self.default_grade)?;

        Ok(WiaMessage {
            id: Uuid::new_v4().to_string(),
            source: WiaStandardType::Custom("ai".into()),
            target: WiaStandardType::Braille,
            message_type: WiaMessageType::Text,
            payload: WiaPayload::Json(json!({
                "braille": braille.unicode,
                "original_text": output.text,
                "grade": format!("{:?}", self.default_grade),
            })),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        })
    }
}
```

---

## 6. WIA-AI 통합 허브 (Integration Hub)

### 6.1 허브 구조

```rust
/// WIA-AI 통합 허브
pub struct WiaAiHub {
    /// 커넥터 레지스트리
    connectors: RwLock<HashMap<String, Arc<dyn WiaConnector>>>,
    /// 입력 어댑터 레지스트리
    input_adapters: RwLock<Vec<Arc<dyn AiInputAdapter>>>,
    /// 출력 어댑터 레지스트리
    output_adapters: RwLock<HashMap<WiaStandardType, Arc<dyn AiOutputAdapter>>>,
    /// AI 에이전트 레지스트리
    agent_registry: Arc<AgentRegistry>,
    /// 이벤트 버스
    event_bus: broadcast::Sender<HubEvent>,
}

impl WiaAiHub {
    /// 새 허브 생성
    pub fn new(agent_registry: Arc<AgentRegistry>) -> Self {
        let (event_bus, _) = broadcast::channel(1024);
        Self {
            connectors: RwLock::new(HashMap::new()),
            input_adapters: RwLock::new(Vec::new()),
            output_adapters: RwLock::new(HashMap::new()),
            agent_registry,
            event_bus,
        }
    }

    /// 커넥터 등록
    pub async fn register_connector(&self, connector: Arc<dyn WiaConnector>) {
        let mut connectors = self.connectors.write().await;
        connectors.insert(connector.id().to_string(), connector);
    }

    /// 입력 어댑터 등록
    pub async fn register_input_adapter(&self, adapter: Arc<dyn AiInputAdapter>) {
        let mut adapters = self.input_adapters.write().await;
        adapters.push(adapter);
    }

    /// 출력 어댑터 등록
    pub async fn register_output_adapter(&self, adapter: Arc<dyn AiOutputAdapter>) {
        let mut adapters = self.output_adapters.write().await;
        adapters.insert(adapter.target_standard(), adapter);
    }

    /// 입력 처리 파이프라인
    pub async fn process_input(
        &self,
        message: WiaMessage
    ) -> Result<AiOutput, WiaAiError> {
        // 1. 적절한 입력 어댑터 찾기
        let adapters = self.input_adapters.read().await;
        let adapter = adapters
            .iter()
            .find(|a| a.can_handle(&message))
            .ok_or_else(|| WiaAiError::NotSupported(
                format!("No adapter for {:?}", message.source)
            ))?;

        // 2. AI 입력으로 변환
        let ai_input = adapter.to_ai_input(message).await?;

        // 3. 에이전트 처리
        let agent = self.agent_registry
            .get_default_agent()
            .await
            .ok_or_else(|| WiaAiError::AgentNotFound("default".into()))?;

        let response = agent.process(ai_input).await?;

        Ok(response)
    }

    /// 출력 라우팅
    pub async fn route_output(
        &self,
        output: AiOutput,
        targets: Vec<WiaStandardType>,
    ) -> Result<Vec<WiaMessage>, WiaAiError> {
        let adapters = self.output_adapters.read().await;
        let mut messages = Vec::new();

        for target in targets {
            if let Some(adapter) = adapters.get(&target) {
                let message = adapter.from_ai_output(output.clone()).await?;

                // 커넥터로 전송
                if let Some(connector) = self.get_connector_for_standard(&target).await {
                    connector.send(message.clone()).await?;
                }

                messages.push(message);
            }
        }

        Ok(messages)
    }

    /// 양방향 대화 처리
    pub async fn conversation(
        &self,
        input: WiaMessage,
        output_targets: Vec<WiaStandardType>,
    ) -> Result<Vec<WiaMessage>, WiaAiError> {
        // 입력 처리
        let ai_output = self.process_input(input).await?;

        // 출력 라우팅
        let messages = self.route_output(ai_output, output_targets).await?;

        Ok(messages)
    }

    /// 이벤트 구독
    pub fn subscribe(&self) -> broadcast::Receiver<HubEvent> {
        self.event_bus.subscribe()
    }

    // 내부 헬퍼
    async fn get_connector_for_standard(
        &self,
        standard: &WiaStandardType
    ) -> Option<Arc<dyn WiaConnector>> {
        let connectors = self.connectors.read().await;
        connectors
            .values()
            .find(|c| &c.standard_type() == standard)
            .cloned()
    }
}
```

### 6.2 허브 이벤트

```rust
/// 허브 이벤트
#[derive(Debug, Clone)]
pub enum HubEvent {
    /// 커넥터 연결됨
    ConnectorConnected { id: String, standard: WiaStandardType },
    /// 커넥터 연결 해제됨
    ConnectorDisconnected { id: String },
    /// 메시지 수신됨
    MessageReceived { message: WiaMessage },
    /// 메시지 전송됨
    MessageSent { message: WiaMessage },
    /// 에러 발생
    Error { source: String, error: String },
}
```

---

## 7. 에러 처리 (Error Handling)

### 7.1 연동 관련 에러

```rust
/// WIA 연동 에러
#[derive(Debug, thiserror::Error)]
pub enum WiaIntegrationError {
    #[error("Connector not found: {0}")]
    ConnectorNotFound(String),

    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Adapter not found for {0:?}")]
    AdapterNotFound(WiaStandardType),

    #[error("Conversion failed: {0}")]
    ConversionFailed(String),

    #[error("Message send failed: {0}")]
    SendFailed(String),

    #[error("Message receive failed: {0}")]
    ReceiveFailed(String),

    #[error("Not supported: {0}")]
    NotSupported(String),

    #[error("Timeout after {0}ms")]
    Timeout(u64),
}
```

---

## 8. 사용 예제 (Examples)

### 8.1 AAC AI 어시스턴트

```rust
use wia_ai::integration::*;

#[tokio::main]
async fn main() -> Result<(), WiaAiError> {
    // 1. 허브 생성
    let agent_registry = Arc::new(AgentRegistry::new());
    let hub = WiaAiHub::new(agent_registry);

    // 2. AAC 커넥터 등록
    let aac_connector = MockWiaConnector::new(WiaStandardType::Aac);
    hub.register_connector(Arc::new(aac_connector)).await;

    // 3. 어댑터 등록
    hub.register_input_adapter(Arc::new(AacInputAdapter::new())).await;
    hub.register_output_adapter(Arc::new(TtsOutputAdapter::new())).await;

    // 4. AAC 입력 수신 및 처리
    let aac_message = WiaMessage {
        id: "msg-001".into(),
        source: WiaStandardType::Aac,
        target: WiaStandardType::Custom("ai".into()),
        message_type: WiaMessageType::Signal,
        payload: WiaPayload::Text("도움이 필요해요".into()),
        metadata: HashMap::new(),
        timestamp: Utc::now(),
    };

    // 5. 대화 처리 (입력 → AI → 출력)
    let responses = hub.conversation(
        aac_message,
        vec![WiaStandardType::Tts],
    ).await?;

    println!("Generated {} responses", responses.len());

    Ok(())
}
```

### 8.2 멀티모달 출력

```rust
// 여러 출력 형식으로 동시 전송
let ai_output = AiOutput {
    output_type: AiOutputType::Text,
    text: "안녕하세요, 무엇을 도와드릴까요?".into(),
    data: None,
    metadata: HashMap::new(),
};

// TTS + 수어 + 점자로 동시 출력
let messages = hub.route_output(
    ai_output,
    vec![
        WiaStandardType::Tts,
        WiaStandardType::Isp,
        WiaStandardType::Braille,
    ],
).await?;

for msg in messages {
    println!("Sent to {:?}", msg.target);
}
```

---

## 9. 접근성 요구사항 (Accessibility Requirements)

### 9.1 WCAG 2.1 준수

| 원칙 | 요구사항 | 구현 |
|------|---------|------|
| Perceivable | 다중 출력 형식 | TTS, ISP, Braille 어댑터 |
| Operable | 다양한 입력 방식 | AAC, BCI, Voice 어댑터 |
| Understandable | 명확한 피드백 | 상태 이벤트 시스템 |
| Robust | 보조기술 호환 | 표준 WIA 메시지 형식 |

### 9.2 응답 시간 요구사항

| 상호작용 | 최대 지연 | 비고 |
|---------|----------|------|
| 입력 → AI 처리 | 500ms | 사용자 인지 한계 |
| AI → TTS 출력 | 200ms | 실시간 대화 |
| AI → 점자 출력 | 100ms | 촉각 피드백 |

---

## 10. 참고 문헌 (References)

- WIA AAC Standard v1.0
- WIA BCI Standard v1.0
- WIA Voice Standard v1.0
- WIA Braille Standard v1.0
- WCAG 2.1 Guidelines
- ISO/IEC 40500:2012 (WCAG 2.0)
