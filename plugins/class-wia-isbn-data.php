<?php
/**
 * WIA ISBN Data - 73개 표준 메타데이터 (최종 완성본)
 * 부제목 및 DALL-E 프롬프트 통합 버전
 */

class WIA_ISBN_Data {
    
    public static function get_all_standards() {
        return [
            // ===== 접근성/보조기술 (AAC) =====
            'aac' => [
                'emoji' => '🗣️',
                'name_en' => 'AAC',
                'name_ko' => 'AAC 의사소통',
                'bundle_title' => 'WIA AAC Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Symbol Sets, Vocabulary Management, Multi-Modal Input, Voice Output, Phrase Prediction & Custom Board',
                'ko_title' => 'WIA AAC 표준화 가이드',
                'ko_subtitle' => '심볼 세트, 어휘 관리, 멀티모달 입력, 음성 출력, 구문 예측 및 맞춤형 보드',
                'en_title' => 'WIA AAC Standard Guide',
                'en_subtitle' => 'Symbol Sets, Vocabulary Management, Multi-Modal Input, Voice Output, Phrase Prediction & Custom Board',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Futuristic symbol board interface floating in 3D space, colorful communication icons transforming into holographic speech bubbles, neural pathways connecting symbols to voice output waves, adaptive grid system with glowing nodes, vibrant teal and coral gradients, dark navy space background (#0f172a), modern assistive tech aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cognitive-aac' => [
                'emoji' => '🧠',
                'name_en' => 'Cognitive AAC',
                'name_ko' => '인지 AAC',
                'bundle_title' => 'WIA Cognitive AAC Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Simplified Interfaces, Visual Schedules, Memory Aids, Step-by-Step Guidance & Attention',
                'ko_title' => 'WIA 인지 AAC 표준화 가이드',
                'ko_subtitle' => '단순화 인터페이스, 시각적 스케줄, 기억 보조도구, 단계별 안내 및 주의력',
                'en_title' => 'WIA Cognitive AAC Standard Guide',
                'en_subtitle' => 'Simplified Interfaces, Visual Schedules, Memory Aids, Step-by-Step Guidance & Attention',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Simplified user interface with large, glowing geometric shapes, step-by-step visual timeline flowing like a river, memory anchor symbols floating in organized clusters, brain silhouette with highlighted attention zones, soft lavender and mint green color palette, dark navy background (#0f172a), calm cognitive-friendly design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'eye-gaze' => [
                'emoji' => '👁️',
                'name_en' => 'Eye-Gaze Tracking',
                'name_ko' => '시선 추적',
                'bundle_title' => 'WIA Eye-Gaze Tracking Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Calibration Methods, Dwell Time Settings, Gaze Gestures, Head Movement Compensation & Environmental Lighting Adaptation',
                'ko_title' => 'WIA 시선 추적 표준화 가이드',
                'ko_subtitle' => '캘리브레이션 방법, 응시시간 설정, 시선 제스처, 머리 움직임 보정 및 환경 조명 적응',
                'en_title' => 'WIA Eye-Gaze Tracking Standard Guide',
                'en_subtitle' => 'Calibration Methods, Dwell Time Settings, Gaze Gestures, Head Movement Compensation & Environmental Lighting Adaptation',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Extreme close-up of iris with digital calibration grid overlay, gaze trajectory light trails forming constellation patterns, pupil dilation sensors visualized as concentric rings, infrared tracking beams in electric blue, silver metallic accents, dark navy background (#0f172a), high-tech biometric aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'haptic' => [
                'emoji' => '🤚',
                'name_en' => 'Haptic Feedback',
                'name_ko' => '햅틱 피드백',
                'bundle_title' => 'WIA Haptic Feedback Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Vibration Patterns, Force Feedback, Texture Simulation, Spatial Haptics & Braille Display Integration',
                'ko_title' => 'WIA 햅틱 피드백 표준화 가이드',
                'ko_subtitle' => '진동 패턴, 힘 피드백, 텍스처 시뮬레이션, 공간 햅틱 및 점자 디스플레이 통합',
                'en_title' => 'WIA Haptic Feedback Standard Guide',
                'en_subtitle' => 'Vibration Patterns, Force Feedback, Texture Simulation, Spatial Haptics & Braille Display Integration',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, 3D topographic surface map with tactile elevation patterns, vibration waves emanating from fingertips, force feedback arrows showing pressure points, Braille pin array in motion, warm amber to deep orange gradient with electric blue accents, dark navy background (#0f172a), sensory technology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'myoelectric' => [
                'emoji' => '💪',
                'name_en' => 'Myoelectric Control',
                'name_ko' => '근전도 제어',
                'bundle_title' => 'WIA Myoelectric Control Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'EMG Signal Processing, Multi-Grip Patterns, Proportional Control, Electrode Placement & Machine Learning Classification',
                'ko_title' => 'WIA 근전도 제어 표준화 가이드',
                'ko_subtitle' => 'EMG 신호처리, 다중 그립 패턴, 비례 제어, 전극 배치 및 머신러닝 분류',
                'en_title' => 'WIA Myoelectric Control Standard Guide',
                'en_subtitle' => 'EMG Signal Processing, Multi-Grip Patterns, Proportional Control, Electrode Placement & Machine Learning Classification',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Muscle fiber cross-section with electrical impulse lightning bolts, EMG signal waveform flowing through arm silhouette, electrode placement points glowing like circuit nodes, bionic hand wireframe with energy streams, electric purple and neon cyan gradients, dark navy background (#0f172a), bio-electronic aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'exoskeleton' => [
                'emoji' => '🦿',
                'name_en' => 'Exoskeleton',
                'name_ko' => '외골격',
                'bundle_title' => 'WIA Exoskeleton Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Joint Actuation, Power Assist Ratios, Gait Synchronization, Load Distribution & Battery Management Systems',
                'ko_title' => 'WIA 외골격 표준화 가이드',
                'ko_subtitle' => '관절 구동, 파워 어시스트 비율, 보행 동기화, 하중 분산 및 배터리 관리 시스템',
                'en_title' => 'WIA Exoskeleton Standard Guide',
                'en_subtitle' => 'Joint Actuation, Power Assist Ratios, Gait Synchronization, Load Distribution & Battery Management Systems',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Robotic skeletal framework with hydraulic pistons and servo motors, mechanical joints with golden ratio spiral patterns, power assist visualization showing force multiplication, titanium alloy texture with carbon fiber weave, steel silver and electric blue gradients, dark navy background (#0f172a), industrial cybernetic design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'smart-wheelchair' => [
                'emoji' => '♿',
                'name_en' => 'Smart Wheelchair',
                'name_ko' => '스마트 휠체어',
                'bundle_title' => 'WIA Smart Wheelchair Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Autonomous Navigation, Obstacle Avoidance, Voice Commands, Posture Monitoring & IoT Health Integration',
                'ko_title' => 'WIA 스마트 휠체어 표준화 가이드',
                'ko_subtitle' => '자율 내비게이션, 장애물 회피, 음성 명령, 자세 모니터링 및 IoT 건강 통합',
                'en_title' => 'WIA Smart Wheelchair Standard Guide',
                'en_subtitle' => 'Autonomous Navigation, Obstacle Avoidance, Voice Commands, Posture Monitoring & IoT Health Integration',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Autonomous wheelchair with LiDAR sensor array radiating 360-degree scan lines, navigation path hologram projected on ground, obstacle detection zones in transparent layers, IoT connectivity nodes forming mesh network, accessibility blue and tech teal gradients, dark navy background (#0f172a), mobility-tech aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'voice' => [
                'emoji' => '🎤',
                'name_en' => 'Voice Interface',
                'name_ko' => '음성 인터페이스',
                'bundle_title' => 'WIA Voice Interface Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Speech Recognition, Natural Language Processing, Voice Biometrics, Wake Word Detection & Multi-Speaker Support',
                'ko_title' => 'WIA 음성 인터페이스 표준화 가이드',
                'ko_subtitle' => '음성 인식, 자연어 처리, 음성 생체인식, 호출어 감지 및 다중 화자 지원',
                'en_title' => 'WIA Voice Interface Standard Guide',
                'en_subtitle' => 'Speech Recognition, Natural Language Processing, Voice Biometrics, Wake Word Detection & Multi-Speaker Support',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Sound wave spectrum analyzer with frequency bars transforming into digital DNA helix, microphone with voice biometric fingerprint patterns, wake word detection pulse rings expanding outward, natural language processing tree structure, vibrant magenta and electric purple gradients, dark navy background (#0f172a), audio-tech design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'bci' => [
                'emoji' => '🧠',
                'name_en' => 'Brain-Computer Interface',
                'name_ko' => '뇌-컴퓨터 인터페이스',
                'bundle_title' => 'WIA BCI Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'EEG Signal Acquisition, P300 Detection, Motor Imagery, Neurofeedback Protocols & Implant Safety Standards',
                'ko_title' => 'WIA BCI 표준화 가이드',
                'ko_subtitle' => 'EEG 신호 획득, P300 감지, 운동 상상, 신경피드백 프로토콜 및 임플란트 안전 기준',
                'en_title' => 'WIA BCI Standard Guide',
                'en_subtitle' => 'EEG Signal Acquisition, P300 Detection, Motor Imagery, Neurofeedback Protocols & Implant Safety Standards',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Transparent brain with neural electrode array implant, EEG signal waves flowing into computer circuit board, P300 event-related potential spike visualization, synaptic connections glowing like fiber optics, deep violet and electric cyan gradients, dark navy background (#0f172a), neurotechnology aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ci' => [
                'emoji' => '👂',
                'name_en' => 'Cochlear Implant',
                'name_ko' => '인공와우',
                'bundle_title' => 'WIA Cochlear Implant Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Sound Processing Strategies, Electrode Array Mapping, Speech Perception Training, MRI Compatibility & Rehabilitation Protocols',
                'ko_title' => 'WIA 인공와우 표준화 가이드',
                'ko_subtitle' => '소리 처리 전략, 전극 배열 매핑, 언어지각 훈련, MRI 호환성 및 재활 프로토콜',
                'en_title' => 'WIA Cochlear Implant Standard Guide',
                'en_subtitle' => 'Sound Processing Strategies, Electrode Array Mapping, Speech Perception Training, MRI Compatibility & Rehabilitation Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Inner ear cochlea spiral with electronic stimulation points, sound frequency spectrum transforming into electrical pulses, auditory nerve pathways illuminated in gold, MRI-safe implant visualization, warm amber and medical blue gradients, dark navy background (#0f172a), auditory restoration design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'bionic-eye' => [
                'emoji' => '👁️',
                'name_en' => 'Bionic Eye',
                'name_ko' => '바이오닉 눈',
                'bundle_title' => 'WIA Bionic Eye Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Retinal Implants, Visual Cortex Stimulation, Image Processing Algorithms, Resolution Enhancement & Low Vision Rehabilitation',
                'ko_title' => 'WIA 바이오닉 눈 표준화 가이드',
                'ko_subtitle' => '망막 임플란트, 시각피질 자극, 이미지 처리 알고리즘, 해상도 향상 및 저시력 재활',
                'en_title' => 'WIA Bionic Eye Standard Guide',
                'en_subtitle' => 'Retinal Implants, Visual Cortex Stimulation, Image Processing Algorithms, Resolution Enhancement & Low Vision Rehabilitation',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Cybernetic eye with retinal chip array in hexagonal pattern, visual cortex stimulation electrodes forming star constellation, image processing pipeline from camera to neural interface, pixel grid transforming into brain signals, electric cyan and silver chrome gradients, dark navy background (#0f172a), vision restoration aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== AI/지능시스템 (AI) =====
            'llm-interop' => [
                'emoji' => '🔗',
                'name_en' => 'LLM Interoperability',
                'name_ko' => 'LLM 상호운용',
                'bundle_title' => 'WIA LLM Interoperability Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Model Routing, Prompt Standardization, Token Management, Cross-Platform APIs & Context Preservation Protocols',
                'ko_title' => 'WIA LLM 상호운용 표준화 가이드',
                'ko_subtitle' => '모델 라우팅, 프롬프트 표준화, 토큰 관리, 크로스 플랫폼 API 및 컨텍스트 보존 프로토콜',
                'en_title' => 'WIA LLM Interoperability Standard Guide',
                'en_subtitle' => 'Model Routing, Prompt Standardization, Token Management, Cross-Platform APIs & Context Preservation Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Multiple AI model nodes connected by glowing data bridges, token streams flowing between transformer architectures, universal prompt format as Rosetta Stone visualization, context preservation shown as continuous thread through models, emerald green and crypto-teal gradients, dark navy background (#0f172a), AI ecosystem design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'emotion-ai' => [
                'emoji' => '💗',
                'name_en' => 'Emotion AI',
                'name_ko' => '감정 AI',
                'bundle_title' => 'WIA Emotion AI Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Facial Expression Analysis, Sentiment Detection, Voice Tone Recognition, Micro-Expression Mining & Empathy Modeling',
                'ko_title' => 'WIA 감정 AI 표준화 가이드',
                'ko_subtitle' => '표정 분석, 감성 탐지, 음성 톤 인식, 미세표정 마이닝 및 공감 모델링',
                'en_title' => 'WIA Emotion AI Standard Guide',
                'en_subtitle' => 'Facial Expression Analysis, Sentiment Detection, Voice Tone Recognition, Micro-Expression Mining & Empathy Modeling',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Human face composed of micro-expression heat map overlays, sentiment analysis graph with emotional spectrum rainbow, heart-shaped neural network with empathy pathways, facial action unit detection grid, soft pink and warm rose gradients with purple accents, dark navy background (#0f172a), affective computing aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'intent-lang' => [
                'emoji' => '💬',
                'name_en' => 'Intent Language',
                'name_ko' => '의도 언어',
                'bundle_title' => 'WIA Intent Language Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Intent Classification, Entity Extraction, Dialog Management, Context Tracking & Multi-Turn Conversation Handling',
                'ko_title' => 'WIA 의도 언어 표준화 가이드',
                'ko_subtitle' => '의도 분류, 개체 추출, 대화 관리, 컨텍스트 추적 및 다회차 대화 처리',
                'en_title' => 'WIA Intent Language Standard Guide',
                'en_subtitle' => 'Intent Classification, Entity Extraction, Dialog Management, Context Tracking & Multi-Turn Conversation Handling',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Speech bubble fragmenting into intent classification tree branches, entity extraction shown as glowing keyword nodes, dialog state machine with conversational flow paths, context window visualization as expanding sphere, fresh teal and mint green gradients, dark navy background (#0f172a), NLU technology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'omni-api' => [
                'emoji' => '🌐',
                'name_en' => 'Omni API',
                'name_ko' => '옴니 API',
                'bundle_title' => 'WIA Omni API Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Universal Gateway Design, Protocol Unification, Service Mesh Architecture, Rate Limiting & API Versioning Strategy',
                'ko_title' => 'WIA 옴니 API 표준화 가이드',
                'ko_subtitle' => '유니버설 게이트웨이 설계, 프로토콜 통합, 서비스 메시 아키텍처, 속도 제한 및 API 버전 전략',
                'en_title' => 'WIA Omni API Standard Guide',
                'en_subtitle' => 'Universal Gateway Design, Protocol Unification, Service Mesh Architecture, Rate Limiting & API Versioning Strategy',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Spherical API gateway at center with protocol spokes radiating outward, service mesh network with microservices as glowing orbs, rate limiter visualization as traffic flow control valves, universal connector ports in hexagonal array, cyan and golden amber gradients, dark navy background (#0f172a), gateway architecture aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-embodiment' => [
                'emoji' => '🤖',
                'name_en' => 'AI Embodiment',
                'name_ko' => 'AI 물리적 구현',
                'bundle_title' => 'WIA AI Embodiment Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Physical Manifestation Protocols, Robotic Body Standards, Sensor Integration, Motor Control APIs & Safety Constraints',
                'ko_title' => 'WIA AI 물리적 구현 표준화 가이드',
                'ko_subtitle' => '물리적 구현 프로토콜, 로봇 바디 표준, 센서 통합, 모터 제어 API 및 안전 제약조건',
                'en_title' => 'WIA AI Embodiment Standard Guide',
                'en_subtitle' => 'Physical Manifestation Protocols, Robotic Body Standards, Sensor Integration, Motor Control APIs & Safety Constraints',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Humanoid robot emerging from digital consciousness cloud, neural network brain connecting to mechanical skeleton through fiber optic nerves, sensor fusion showing camera/lidar/touch integration, robotic hand and human hand reaching toward each other, deep purple and metallic silver gradients, dark navy background (#0f172a), embodied AI design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-embodiment-ethics' => [
                'emoji' => '⚖️',
                'name_en' => 'AI Embodiment Ethics',
                'name_ko' => 'AI 물리적 구현 윤리',
                'bundle_title' => 'WIA AI Embodiment Ethics Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Moral Agency Definition, Responsibility Frameworks, Harm Prevention, Rights Recognition & Human-Robot Boundaries',
                'ko_title' => 'WIA AI 물리적 구현 윤리 표준화 가이드',
                'ko_subtitle' => '도덕적 주체성 정의, 책임 프레임워크, 피해 방지, 권리 인정 및 인간-로봇 경계',
                'en_title' => 'WIA AI Embodiment Ethics Standard Guide',
                'en_subtitle' => 'Moral Agency Definition, Responsibility Frameworks, Harm Prevention, Rights Recognition & Human-Robot Boundaries',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Scales of justice balanced between human silhouette and robot silhouette, ethical decision tree with branching moral pathways, responsibility framework shown as interlocking shield layers, rights recognition badges in circular arrangement, golden amber and philosophical blue gradients, dark navy background (#0f172a), ethics framework aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-sensor-fusion' => [
                'emoji' => '👁️',
                'name_en' => 'AI Sensor Fusion',
                'name_ko' => 'AI 센서 융합',
                'bundle_title' => 'WIA AI Sensor Fusion Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Multi-Sensor Data Alignment, Kalman Filtering, Temporal Synchronization, Sensor Calibration & Redundancy Management',
                'ko_title' => 'WIA AI 센서 융합 표준화 가이드',
                'ko_subtitle' => '다중센서 데이터 정렬, 칼만 필터링, 시간 동기화, 센서 캘리브레이션 및 중복성 관리',
                'en_title' => 'WIA AI Sensor Fusion Standard Guide',
                'en_subtitle' => 'Multi-Sensor Data Alignment, Kalman Filtering, Temporal Synchronization, Sensor Calibration & Redundancy Management',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Kalman filter visualization with sensor data streams converging into unified perception sphere, camera lens, LiDAR rays, and radar waves merging at focal point, temporal synchronization timeline with aligned data pulses, redundancy layers shown as backup sensor arrays, orange and cyan complementary gradients, dark navy background (#0f172a), multi-sensor design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-motor-control' => [
                'emoji' => '⚙️',
                'name_en' => 'AI Motor Control',
                'name_ko' => 'AI 모터 제어',
                'bundle_title' => 'WIA AI Motor Control Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Inverse Kinematics, PID Tuning, Trajectory Planning, Force Control & Compliance Algorithms',
                'ko_title' => 'WIA AI 모터 제어 표준화 가이드',
                'ko_subtitle' => '역기구학, PID 튜닝, 궤적 계획, 힘 제어 및 컴플라이언스 알고리즘',
                'en_title' => 'WIA AI Motor Control Standard Guide',
                'en_subtitle' => 'Inverse Kinematics, PID Tuning, Trajectory Planning, Force Control & Compliance Algorithms',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Robotic arm with inverse kinematics skeleton overlay, PID control loop diagram as circular feedback system, trajectory planning showing smooth motion curves through 3D space, compliance algorithm visualized as spring-damper systems, industrial gray and electric blue gradients, dark navy background (#0f172a), motion control aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-robot-interface' => [
                'emoji' => '🔌',
                'name_en' => 'AI Robot Interface',
                'name_ko' => 'AI 로봇 인터페이스',
                'bundle_title' => 'WIA AI Robot Interface Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'ROS Message Protocols, Hardware Abstraction Layers, Real-Time Communication, Driver Interfaces & Device Discovery',
                'ko_title' => 'WIA AI 로봇 인터페이스 표준화 가이드',
                'ko_subtitle' => 'ROS 메시지 프로토콜, 하드웨어 추상화 계층, 실시간 통신, 드라이버 인터페이스 및 디바이스 탐색',
                'en_title' => 'WIA AI Robot Interface Standard Guide',
                'en_subtitle' => 'ROS Message Protocols, Hardware Abstraction Layers, Real-Time Communication, Driver Interfaces & Device Discovery',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, ROS message bus with topic nodes and subscriber/publisher connections, hardware abstraction layers shown as transparent interface planes, real-time data packets flowing through communication channels, driver stack visualization as layered architecture, indigo and neon green gradients, dark navy background (#0f172a), middleware design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-safety-physical' => [
                'emoji' => '🛡️',
                'name_en' => 'AI Physical Safety',
                'name_ko' => 'AI 물리적 안전',
                'bundle_title' => 'WIA AI Physical Safety Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Collision Detection, Emergency Stop Systems, Safe Zone Monitoring, Force Limiting & Fail-Safe Mechanisms',
                'ko_title' => 'WIA AI 물리적 안전 표준화 가이드',
                'ko_subtitle' => '충돌 감지, 비상정지 시스템, 안전구역 모니터링, 힘 제한 및 페일세이프 메커니즘',
                'en_title' => 'WIA AI Physical Safety Standard Guide',
                'en_subtitle' => 'Collision Detection, Emergency Stop Systems, Safe Zone Monitoring, Force Limiting & Fail-Safe Mechanisms',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Protective force field dome with collision detection radar sweeps, emergency stop button glowing red with fail-safe circuit paths, safe zone boundaries visualized as holographic floor grid, force limiting mechanism shown as breakaway clutch illustration, safety orange and protective blue gradients, dark navy background (#0f172a), safety system aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-human-coexistence' => [
                'emoji' => '🤝',
                'name_en' => 'AI Human Coexistence',
                'name_ko' => 'AI 인간 공존',
                'bundle_title' => 'WIA AI Human Coexistence Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Collaborative Workspaces, Social Navigation, Intention Prediction, Personal Space Respect & Trust Building Protocols',
                'ko_title' => 'WIA AI 인간 공존 표준화 가이드',
                'ko_subtitle' => '협업 작업공간, 소셜 네비게이션, 의도 예측, 개인공간 존중 및 신뢰 구축 프로토콜',
                'en_title' => 'WIA AI Human Coexistence Standard Guide',
                'en_subtitle' => 'Collaborative Workspaces, Social Navigation, Intention Prediction, Personal Space Respect & Trust Building Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Human and robot working side-by-side in shared workspace, social navigation paths avoiding personal space bubbles, intention prediction arrows showing anticipated movements, trust meter visualization as growing plant between human and machine, warm human skin tones blending with cool robot metallics, dark navy background (#0f172a), collaborative design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ai-afterlife-ethics' => [
                'emoji' => '👻',
                'name_en' => 'AI Afterlife Ethics',
                'name_ko' => 'AI 사후 윤리',
                'bundle_title' => 'WIA AI Afterlife Ethics Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Digital Immortality Consent, Memory Preservation Rights, AI Resurrection Ethics, Data Inheritance & Posthumous Autonomy',
                'ko_title' => 'WIA AI 사후 윤리 표준화 가이드',
                'ko_subtitle' => '디지털 불멸 동의, 기억 보존 권리, AI 부활 윤리, 데이터 상속 및 사후 자율성',
                'en_title' => 'WIA AI Afterlife Ethics Standard Guide',
                'en_subtitle' => 'Digital Immortality Consent, Memory Preservation Rights, AI Resurrection Ethics, Data Inheritance & Posthumous Autonomy',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Digital ghost figure ascending from data cloud into starry digital afterlife, memory preservation crystals storing consciousness fragments, consent document transforming into blockchain smart contract, posthumous autonomy shown as spirit controlling avatar, ethereal lavender and ghostly white gradients with electric blue, dark navy background (#0f172a), digital immortality aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 의료/헬스케어 (MED) =====
            'medical' => [
                'emoji' => '🥼',
                'name_en' => 'Medical Device',
                'name_ko' => '의료기기',
                'bundle_title' => 'WIA Medical Device Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Device Classification, Clinical Trial Protocols, Regulatory Compliance, Interoperability Standards & Patient Safety Guidelines',
                'ko_title' => 'WIA 의료기기 표준화 가이드',
                'ko_subtitle' => '기기 분류, 임상시험 프로토콜, 규제 준수, 상호운용성 표준 및 환자 안전 지침',
                'en_title' => 'WIA Medical Device Standard Guide',
                'en_subtitle' => 'Device Classification, Clinical Trial Protocols, Regulatory Compliance, Interoperability Standards & Patient Safety Guidelines',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Medical device with FDA classification tiers stacked like pyramid layers, clinical trial phases shown as ascending staircase with checkpoints, regulatory compliance shields surrounding central device, interoperability bridges connecting hospital systems, clean medical white and clinical blue gradients, dark navy background (#0f172a), healthcare compliance design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'health' => [
                'emoji' => '❤️',
                'name_en' => 'Health Record',
                'name_ko' => '건강 기록',
                'bundle_title' => 'WIA Health Record Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'HL7 FHIR Integration, Privacy Preservation, Consent Management, Cross-Provider Data Exchange & Longitudinal Records',
                'ko_title' => 'WIA 건강 기록 표준화 가이드',
                'ko_subtitle' => 'HL7 FHIR 통합, 프라이버시 보존, 동의 관리, 의료기관 간 데이터 교환 및 종단 기록',
                'en_title' => 'WIA Health Record Standard Guide',
                'en_subtitle' => 'HL7 FHIR Integration, Privacy Preservation, Consent Management, Cross-Provider Data Exchange & Longitudinal Records',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, FHIR resource icons arranged in circular network topology, patient data flowing through encrypted tunnels between healthcare providers, consent management shown as access control key system, longitudinal health timeline extending through space, medical green and trust blue gradients, dark navy background (#0f172a), EHR interoperability aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'bio' => [
                'emoji' => '🧬',
                'name_en' => 'Biotechnology',
                'name_ko' => '생명공학',
                'bundle_title' => 'WIA Biotechnology Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Gene Editing Standards, Biosafety Protocols, Synthetic Biology Guidelines, Bioinformatics Data Formats & Ethics Frameworks',
                'ko_title' => 'WIA 생명공학 표준화 가이드',
                'ko_subtitle' => '유전자 편집 표준, 생물안전 프로토콜, 합성생물학 지침, 생물정보학 데이터 형식 및 윤리 프레임워크',
                'en_title' => 'WIA Biotechnology Standard Guide',
                'en_subtitle' => 'Gene Editing Standards, Biosafety Protocols, Synthetic Biology Guidelines, Bioinformatics Data Formats & Ethics Frameworks',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, CRISPR molecular scissors cutting DNA double helix with precision guides, gene editing base pairs highlighted in fluorescent colors, synthetic biology circuit diagram with genetic logic gates, bioinformatics data flowing as genetic code streams, bio-luminescent green and cyan gradients, dark navy background (#0f172a), genetic engineering design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
// ===== 냉동보존/생명연장 (CRYO) =====
            'cryo-preservation' => [
                'emoji' => '❄️',
                'name_en' => 'Cryopreservation',
                'name_ko' => '인체냉동보존',
                'bundle_title' => 'WIA Cryopreservation Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Vitrification Protocols, Temperature Control Systems, Cryoprotectant Standards, Long-Term Storage & Revival Procedures',
                'ko_title' => 'WIA 인체냉동보존 표준화 가이드',
                'ko_subtitle' => '유리화 프로토콜, 온도 제어 시스템, 동결보호제 표준, 장기 보관 및 소생 절차',
                'en_title' => 'WIA Cryopreservation Standard Guide',
                'en_subtitle' => 'Vitrification Protocols, Temperature Control Systems, Cryoprotectant Standards, Long-Term Storage & Revival Procedures',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Cryogenic chamber with liquid nitrogen vapor swirling in slow motion, vitrification ice crystal structure in perfect geometric lattice, temperature gradient visualization from ultra-cold to warm spectrum, cellular preservation shown at microscopic level, ice blue and silver frost gradients, dark navy background (#0f172a), cryonics aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-identity' => [
                'emoji' => '🆔',
                'name_en' => 'Cryo Identity',
                'name_ko' => '냉동인 신원',
                'bundle_title' => 'WIA Cryo Identity Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Biometric Preservation, Identity Verification Methods, Genetic Profile Storage, Blockchain Authentication & Access Control',
                'ko_title' => 'WIA 냉동인 신원 표준화 가이드',
                'ko_subtitle' => '생체인식 보존, 신원 확인 방법, 유전자 프로필 저장, 블록체인 인증 및 접근 제어',
                'en_title' => 'WIA Cryo Identity Standard Guide',
                'en_subtitle' => 'Biometric Preservation, Identity Verification Methods, Genetic Profile Storage, Blockchain Authentication & Access Control',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, DNA double helix structure intertwined with blockchain nodes glowing in neon blue, biometric scan patterns (fingerprint, iris, facial recognition) floating in holographic display, genetic code sequences flowing like digital data streams, identity verification layers with encryption symbols, cryogenic preservation chamber silhouette in background, cyber blue (#00d4ff) and neon green (#00ff88) color scheme, dark navy background (#0f172a), futuristic identity authentication aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-consent' => [
                'emoji' => '✍️',
                'name_en' => 'Cryo Consent',
                'name_ko' => '냉동 동의',
                'bundle_title' => 'WIA Cryo Consent Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Informed Consent Frameworks, Legal Documentation, Revocation Procedures, Family Notification & Advance Directives',
                'ko_title' => 'WIA 냉동 동의 표준화 가이드',
                'ko_subtitle' => '충분한 설명 기반 동의 프레임워크, 법적 문서화, 철회 절차, 가족 통지 및 사전 의료 지시서',
                'en_title' => 'WIA Cryo Consent Standard Guide',
                'en_subtitle' => 'Informed Consent Frameworks, Legal Documentation, Revocation Procedures, Family Notification & Advance Directives',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Legal document transforming into ice sculpture with preserved signatures, informed consent process shown as decision tree frozen in crystal, revocation mechanism visualized as melting clause with emergency thaw, family tree network around consent centerpiece, frost white and legal blue gradients, dark navy background (#0f172a), consent framework aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-revival' => [
                'emoji' => '🔄',
                'name_en' => 'Cryo Revival',
                'name_ko' => '냉동 소생',
                'bundle_title' => 'WIA Cryo Revival Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Rewarming Protocols, Cellular Repair Technologies, Neurological Restoration, Organ Function Recovery & Post-Revival Care',
                'ko_title' => 'WIA 냉동 소생 표준화 가이드',
                'ko_subtitle' => '재가온 프로토콜, 세포 복구 기술, 신경학적 회복, 장기 기능 복원 및 소생 후 관리',
                'en_title' => 'WIA Cryo Revival Standard Guide',
                'en_subtitle' => 'Rewarming Protocols, Cellular Repair Technologies, Neurological Restoration, Organ Function Recovery & Post-Revival Care',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Ice thawing into liquid life with cellular repair nanobots visualization, neural pathways reconnecting like circuit board restoration, organ systems rebooting in sequence shown as progressive illumination, revival protocol timeline from frozen to living state, transitioning from ice blue through warm amber to life pink, dark navy background (#0f172a), reanimation design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-legal' => [
                'emoji' => '⚖️',
                'name_en' => 'Cryo Legal',
                'name_ko' => '냉동 법률',
                'bundle_title' => 'WIA Cryo Legal Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Legal Status Definition, Contractual Obligations, Estate Planning, Jurisdictional Issues & Liability Frameworks',
                'ko_title' => 'WIA 냉동 법률 표준화 가이드',
                'ko_subtitle' => '법적 지위 정의, 계약상 의무, 재산 계획, 관할권 문제 및 책임 프레임워크',
                'en_title' => 'WIA Cryo Legal Standard Guide',
                'en_subtitle' => 'Legal Status Definition, Contractual Obligations, Estate Planning, Jurisdictional Issues & Liability Frameworks',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Scales of justice encased in ice block, legal status definition shown as frozen contractual layers, jurisdictional boundaries as ice shelf territories, liability framework visualized as protective ice shield, cold blue and authoritative gold gradients, dark navy background (#0f172a), cryogenic law aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-asset' => [
                'emoji' => '💰',
                'name_en' => 'Cryo Asset',
                'name_ko' => '냉동인 자산',
                'bundle_title' => 'WIA Cryo Asset Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Trust Management, Asset Preservation, Investment Strategies, Inflation Protection & Beneficiary Designation',
                'ko_title' => 'WIA 냉동인 자산 표준화 가이드',
                'ko_subtitle' => '신탁 관리, 자산 보존, 투자 전략, 인플레이션 방어 및 수익자 지정',
                'en_title' => 'WIA Cryo Asset Standard Guide',
                'en_subtitle' => 'Trust Management, Asset Preservation, Investment Strategies, Inflation Protection & Beneficiary Designation',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Wealth portfolio frozen in time with compound interest crystals growing, investment vehicles preserved in ice vault, inflation protection shown as thermal insulation layers, trust fund visualization as growing ice stalagmite, golden amber and frost blue gradients, dark navy background (#0f172a), asset preservation design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'cryo-facility' => [
                'emoji' => '🏢',
                'name_en' => 'Cryo Facility',
                'name_ko' => '냉동 시설',
                'bundle_title' => 'WIA Cryo Facility Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Facility Certification, Infrastructure Requirements, Emergency Backup Systems, Security Protocols & Monitoring Standards',
                'ko_title' => 'WIA 냉동 시설 표준화 가이드',
                'ko_subtitle' => '시설 인증, 인프라 요구사항, 비상 백업 시스템, 보안 프로토콜 및 모니터링 표준',
                'en_title' => 'WIA Cryo Facility Standard Guide',
                'en_subtitle' => 'Facility Certification, Infrastructure Requirements, Emergency Backup Systems, Security Protocols & Monitoring Standards',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Futuristic cryogenic facility architecture with preservation pods in geometric array, liquid nitrogen distribution network with cooling pipes, backup power system visualization as redundant energy grid, monitoring dashboard showing vital preservation metrics, architectural steel and ice blue gradients, dark navy background (#0f172a), facility infrastructure aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 에너지/환경 (ENE) =====
            'climate' => [
                'emoji' => '🌍',
                'name_en' => 'Climate Data',
                'name_ko' => '기후 데이터',
                'bundle_title' => 'WIA Climate Data Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Carbon Footprint Tracking, Emission Monitoring, Climate Modeling Data, Renewable Energy Integration & ESG Reporting',
                'ko_title' => 'WIA 기후 데이터 표준화 가이드',
                'ko_subtitle' => '탄소 발자국 추적, 배출 모니터링, 기후 모델링 데이터, 재생에너지 통합 및 ESG 보고',
                'en_title' => 'WIA Climate Data Standard Guide',
                'en_subtitle' => 'Carbon Footprint Tracking, Emission Monitoring, Climate Modeling Data, Renewable Energy Integration & ESG Reporting',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Earth globe with real-time carbon emission heat map overlay, climate sensor network forming geodesic dome around planet, ESG reporting dashboard showing sustainability metrics, renewable energy flows as green light streams, earth green and sky blue gradients, dark navy background (#0f172a), climate monitoring design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'air-power' => [
                'emoji' => '⚡',
                'name_en' => 'Air Power',
                'name_ko' => '공중 전력',
                'bundle_title' => 'WIA Air Power Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Wireless Power Transmission, UAM Charging Infrastructure, Safety Protocols, Efficiency Standards & Frequency Regulation',
                'ko_title' => 'WIA 공중 전력 표준화 가이드',
                'ko_subtitle' => '무선 전력 전송, UAM 충전 인프라, 안전 프로토콜, 효율 표준 및 주파수 규제',
                'en_title' => 'WIA Air Power Standard Guide',
                'en_subtitle' => 'Wireless Power Transmission, UAM Charging Infrastructure, Safety Protocols, Efficiency Standards & Frequency Regulation',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Wireless power transmission beams between floating charging stations in sky, UAM vehicle receiving energy mid-flight with power coupling visualization, electromagnetic field lines forming power grid in atmosphere, frequency spectrum analyzer showing energy channels, electric yellow and aviation blue gradients, dark navy background (#0f172a), aerial energy aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'air-shield' => [
                'emoji' => '🛡️',
                'name_en' => 'Air Shield',
                'name_ko' => '대기 보호',
                'bundle_title' => 'WIA Air Shield Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Air Quality Sensors, Pollution Filtering Systems, Real-Time Monitoring, Health Alert Protocols & Urban Coverage Networks',
                'ko_title' => 'WIA 대기 보호 표준화 가이드',
                'ko_subtitle' => '공기질 센서, 오염 필터링 시스템, 실시간 모니터링, 건강 경보 프로토콜 및 도시 커버리지 네트워크',
                'en_title' => 'WIA Air Shield Standard Guide',
                'en_subtitle' => 'Air Quality Sensors, Pollution Filtering Systems, Real-Time Monitoring, Health Alert Protocols & Urban Coverage Networks',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Transparent protective dome over cityscape filtering pollution particles, air quality sensors forming mesh network, real-time PM2.5 visualization as particle cloud density, clean air zones shown as protected bubbles, fresh cyan and purification white gradients, dark navy background (#0f172a), atmospheric protection design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'carbon-credit-micro' => [
                'emoji' => '🌱',
                'name_en' => 'Carbon Credit Micro',
                'name_ko' => '탄소 크레딧 마이크로',
                'bundle_title' => 'WIA Carbon Credit Micro Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Individual Carbon Accounting, Micro-Transaction Systems, Verification Methods, Blockchain Ledgers & Incentive Mechanisms',
                'ko_title' => 'WIA 탄소 크레딧 마이크로 표준화 가이드',
                'ko_subtitle' => '개인 탄소 회계, 마이크로 거래 시스템, 검증 방법, 블록체인 원장 및 인센티브 메커니즘',
                'en_title' => 'WIA Carbon Credit Micro Standard Guide',
                'en_subtitle' => 'Individual Carbon Accounting, Micro-Transaction Systems, Verification Methods, Blockchain Ledgers & Incentive Mechanisms',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Individual carbon footprint shown as personal emission cloud, micro carbon tokens as blockchain cryptocurrency coins, verification checkmarks growing like seedlings, offsetting transactions flowing as green energy streams, eco-green and blockchain gold gradients, dark navy background (#0f172a), carbon economy aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'ocean-plastic-track' => [
                'emoji' => '🌊',
                'name_en' => 'Ocean Plastic Track',
                'name_ko' => '해양 플라스틱 추적',
                'bundle_title' => 'WIA Ocean Plastic Track Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Marine Debris Mapping, GPS Tracking Systems, Collection Route Optimization, Material Classification & Cleanup Coordination',
                'ko_title' => 'WIA 해양 플라스틱 추적 표준화 가이드',
                'ko_subtitle' => '해양 쓰레기 매핑, GPS 추적 시스템, 수거 경로 최적화, 재질 분류 및 청소 조율',
                'en_title' => 'WIA Ocean Plastic Track Standard Guide',
                'en_subtitle' => 'Marine Debris Mapping, GPS Tracking Systems, Collection Route Optimization, Material Classification & Cleanup Coordination',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Ocean surface with GPS-tracked plastic debris highlighted as glowing waypoints, collection route optimization shown as efficient path network, material classification using color-coded particle system, cleanup coordination visualization as synchronized fleet movement, ocean blue and cleanup orange gradients, dark navy background (#0f172a), marine conservation design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'battery-passport' => [
                'emoji' => '🔋',
                'name_en' => 'Battery Passport',
                'name_ko' => '배터리 여권',
                'bundle_title' => 'WIA Battery Passport Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Battery Lifecycle Tracking, Material Composition Data, Recycling Information, Performance History & Supply Chain Transparency',
                'ko_title' => 'WIA 배터리 여권 표준화 가이드',
                'ko_subtitle' => '배터리 수명주기 추적, 재료 구성 데이터, 재활용 정보, 성능 이력 및 공급망 투명성',
                'en_title' => 'WIA Battery Passport Standard Guide',
                'en_subtitle' => 'Battery Lifecycle Tracking, Material Composition Data, Recycling Information, Performance History & Supply Chain Transparency',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Battery cell cutaway with QR code passport chip embedded, lifecycle timeline from production to recycling shown as circular flow, material composition layers visible in cross-section, performance degradation curve with health indicators, energy green and tech blue gradients, dark navy background (#0f172a), battery transparency aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 로봇/자동화 (ROB) =====
            'robot' => [
                'emoji' => '🤖',
                'name_en' => 'Robot General',
                'name_ko' => '범용 로봇',
                'bundle_title' => 'WIA Robot Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Robot Operating System Standards, Manipulation Protocols, Path Planning, Safety Certifications & Human-Robot Interaction',
                'ko_title' => 'WIA 범용 로봇 표준화 가이드',
                'ko_subtitle' => '로봇 운영체제 표준, 매니퓰레이션 프로토콜, 경로 계획, 안전 인증 및 인간-로봇 상호작용',
                'en_title' => 'WIA Robot Standard Guide',
                'en_subtitle' => 'Robot Operating System Standards, Manipulation Protocols, Path Planning, Safety Certifications & Human-Robot Interaction',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Universal robotic platform with modular joint system, ROS architecture shown as layered software stack, manipulation task sequence with gripper variations, path planning visualization through obstacle field, industrial orange and robotics gray gradients, dark navy background (#0f172a), general robotics design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'carebot' => [
                'emoji' => '🤗',
                'name_en' => 'Carebot',
                'name_ko' => '돌봄 로봇',
                'bundle_title' => 'WIA Carebot Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Elder Care Protocols, Medication Reminders, Fall Detection, Emotional Support Systems & Privacy-Preserving Monitoring',
                'ko_title' => 'WIA 돌봄 로봇 표준화 가이드',
                'ko_subtitle' => '노인 돌봄 프로토콜, 복약 알림, 낙상 감지, 정서 지원 시스템 및 프라이버시 보존 모니터링',
                'en_title' => 'WIA Carebot Standard Guide',
                'en_subtitle' => 'Elder Care Protocols, Medication Reminders, Fall Detection, Emotional Support Systems & Privacy-Preserving Monitoring',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Friendly companion robot with soft rounded design offering assistance, elderly care scenarios shown in gentle vignettes, medication reminder system with pill dispenser visualization, fall detection sensors forming protective monitoring network, warm peach and caring rose gradients with trust blue, dark navy background (#0f172a), caregiving aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 우주/물리/소재 (SPACE/QUA/MAT) =====
            'space' => [
                'emoji' => '🚀',
                'name_en' => 'Space Technology',
                'name_ko' => '우주 기술',
                'bundle_title' => 'WIA Space Technology Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Satellite Data Standards, Orbital Mechanics APIs, Space Traffic Management, Ground Station Protocols & Debris Tracking',
                'ko_title' => 'WIA 우주 기술 표준화 가이드',
                'ko_subtitle' => '위성 데이터 표준, 궤도역학 API, 우주 교통 관리, 지상국 프로토콜 및 우주쓰레기 추적',
                'en_title' => 'WIA Space Technology Standard Guide',
                'en_subtitle' => 'Satellite Data Standards, Orbital Mechanics APIs, Space Traffic Management, Ground Station Protocols & Debris Tracking',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Satellite constellation network in orbital formation, ground station antenna array communicating with space assets, debris tracking system showing collision avoidance maneuvers, orbital mechanics trajectories as Hohmann transfer curves, deep space purple and starlight silver gradients, dark navy background (#0f172a), space technology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'quantum' => [
                'emoji' => '⚛️',
                'name_en' => 'Quantum Computing',
                'name_ko' => '양자 컴퓨팅',
                'bundle_title' => 'WIA Quantum Computing Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Qubit Control Protocols, Quantum Error Correction, Circuit Design Standards, Measurement Techniques & Entanglement Verification',
                'ko_title' => 'WIA 양자 컴퓨팅 표준화 가이드',
                'ko_subtitle' => '큐비트 제어 프로토콜, 양자 오류 정정, 회로 설계 표준, 측정 기법 및 얽힘 검증',
                'en_title' => 'WIA Quantum Computing Standard Guide',
                'en_subtitle' => 'Qubit Control Protocols, Quantum Error Correction, Circuit Design Standards, Measurement Techniques & Entanglement Verification',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Qubit superposition state visualized as Bloch sphere with quantum gates, entangled particle pairs connected by spooky action threads, quantum error correction surface code as tessellated pattern, quantum circuit diagram with probabilistic wave functions, quantum violet and measurement blue gradients, dark navy background (#0f172a), quantum computing aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'physics' => [
                'emoji' => '🔬',
                'name_en' => 'Physics Data',
                'name_ko' => '물리학 데이터',
                'bundle_title' => 'WIA Physics Data Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Experimental Data Formats, Simulation Standards, Particle Accelerator Protocols, Measurement Units & Publication Guidelines',
                'ko_title' => 'WIA 물리학 데이터 표준화 가이드',
                'ko_subtitle' => '실험 데이터 형식, 시뮬레이션 표준, 입자가속기 프로토콜, 측정 단위 및 출판 지침',
                'en_title' => 'WIA Physics Data Standard Guide',
                'en_subtitle' => 'Experimental Data Formats, Simulation Standards, Particle Accelerator Protocols, Measurement Units & Publication Guidelines',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Particle collision event with Feynman diagram overlay, wave-particle duality shown as interference pattern, fundamental forces visualization as force carrier exchange, experimental apparatus silhouette with data collection streams, scientific blue and energy orange gradients, dark navy background (#0f172a), physics research design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'material' => [
                'emoji' => '🧪',
                'name_en' => 'Material Science',
                'name_ko' => '소재 과학',
                'bundle_title' => 'WIA Material Science Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Material Properties Database, Testing Protocols, Composition Standards, Performance Metrics & Certification Procedures',
                'ko_title' => 'WIA 소재 과학 표준화 가이드',
                'ko_subtitle' => '재료 특성 데이터베이스, 시험 프로토콜, 조성 표준, 성능 지표 및 인증 절차',
                'en_title' => 'WIA Material Science Standard Guide',
                'en_subtitle' => 'Material Properties Database, Testing Protocols, Composition Standards, Performance Metrics & Certification Procedures',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Crystalline lattice structure at atomic level with material properties labeled through color, stress-strain testing visualization with deformation animation, composite material layers shown in exploded view, material database organized as periodic table arrangement, metallic silver and lab blue gradients, dark navy background (#0f172a), materials science aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'nano' => [
                'emoji' => '🔬',
                'name_en' => 'Nanotechnology',
                'name_ko' => '나노기술',
                'bundle_title' => 'WIA Nanotechnology Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Nanoscale Fabrication Standards, Characterization Methods, Safety Protocols, Environmental Impact Assessment & Biocompatibility Testing',
                'ko_title' => 'WIA 나노기술 표준화 가이드',
                'ko_subtitle' => '나노스케일 제조 표준, 특성화 방법, 안전 프로토콜, 환경 영향 평가 및 생체적합성 시험',
                'en_title' => 'WIA Nanotechnology Standard Guide',
                'en_subtitle' => 'Nanoscale Fabrication Standards, Characterization Methods, Safety Protocols, Environmental Impact Assessment & Biocompatibility Testing',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Nanoscale landscape with molecular machines assembling structures, carbon nanotube forest visualization, nanoparticle size comparison with virus and cell, scanning tunneling microscope imagery aesthetic, nano-silver and quantum cyan gradients, dark navy background (#0f172a), nanotechnology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 보안/암호화 (SEC) =====
            'security' => [
                'emoji' => '🔒',
                'name_en' => 'Security General',
                'name_ko' => '통합 보안',
                'bundle_title' => 'WIA Security Standard Guide Set (KO/EN)',
                'bundle_subtitle' => ' Authentication Methods, Encryption Standards, Access Control, Threat Detection & Incident Response Protocols',
                'ko_title' => 'WIA 통합 보안 표준화 가이드',
                'ko_subtitle' => '인증 방법, 암호화 표준, 접근 제어, 위협 감지 및 사고 대응 프로토콜',
                'en_title' => 'WIA Security Standard Guide',
                'en_subtitle' => 'Authentication Methods, Encryption Standards, Access Control, Threat Detection & Incident Response Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Multi-layered security shield with firewall patterns, authentication factors shown as biometric and password key combination, threat detection radar with anomaly highlights, zero-trust architecture visualization as checkpoint network, security red and protective blue gradients, dark navy background (#0f172a), cybersecurity aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'dpki' => [
                'emoji' => '🔐',
                'name_en' => 'Decentralized PKI',
                'name_ko' => '분산 PKI',
                'bundle_title' => 'WIA Decentralized PKI Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Decentralized Identity Management, Certificate Transparency, Key Rotation Policies, Revocation Mechanisms & Trust Anchors',
                'ko_title' => 'WIA 분산 PKI 표준화 가이드',
                'ko_subtitle' => '분산 신원 관리, 인증서 투명성, 키 교체 정책, 폐기 메커니즘 및 신뢰 앵커',
                'en_title' => 'WIA Decentralized PKI Standard Guide',
                'en_subtitle' => 'Decentralized Identity Management, Certificate Transparency, Key Rotation Policies, Revocation Mechanisms & Trust Anchors',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Decentralized certificate web with peer-to-peer trust relationships, blockchain-based PKI with distributed trust anchors, key rotation visualization as evolving cryptographic spiral, certificate transparency log shown as immutable ledger, crypto-green and blockchain gold gradients, dark navy background (#0f172a), decentralized identity design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pq-crypto' => [
                'emoji' => '🔑',
                'name_en' => 'Post-Quantum Crypto',
                'name_ko' => '양자내성 암호',
                'bundle_title' => 'WIA Post-Quantum Crypto Standard Guide Set (KO/EN)',
                'bundle_subtitle' => ' Lattice-Based Cryptography, Hash-Based Signatures, Code-Based Encryption, Migration Strategies & Quantum-Resistant Algorithms',
                'ko_title' => 'WIA 양자내성 암호 표준화 가이드',
                'ko_subtitle' => '격자 기반 암호화, 해시 기반 서명, 코드 기반 암호화, 마이그레이션 전략 및 양자내성 알고리즘',
                'en_title' => 'WIA Post-Quantum Crypto Standard Guide',
                'en_subtitle' => ' Lattice-Based Cryptography, Hash-Based Signatures, Code-Based Encryption, Migration Strategies & Quantum-Resistant Algorithms',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Lattice-based cryptography structure as multi-dimensional grid, quantum computer threat shown as breaking classical encryption, post-quantum algorithms as unbreakable crystalline structures, migration path from classical to quantum-resistant, quantum purple and future-proof platinum gradients, dark navy background (#0f172a), quantum-safe aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'tls-lite' => [
                'emoji' => '🔓',
                'name_en' => 'TLS Lite',
                'name_ko' => '경량 TLS',
                'bundle_title' => 'WIA TLS Lite Standard Guide Set (KO/EN)',
                'bundle_subtitle' => ' Lightweight Handshake Protocols, IoT Device Security, Minimal Memory Footprint, Power Efficiency & Embedded Systems Support',
                'ko_title' => 'WIA 경량 TLS 표준화 가이드',
                'ko_subtitle' => '경량 핸드셰이크 프로토콜, IoT 기기 보안, 최소 메모리 풋프린트, 전력 효율 및 임베디드 시스템 지원',
                'en_title' => 'WIA TLS Lite Standard Guide',
                'en_subtitle' => ' Lightweight Handshake Protocols, IoT Device Security, Minimal Memory Footprint, Power Efficiency & Embedded Systems Support',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Streamlined handshake protocol with minimal overhead visualization, IoT device securing connection with lightweight certificate, power consumption graph showing energy efficiency, embedded system chip with security enclave, efficient green and IoT blue gradients, dark navy background (#0f172a), lightweight security design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 교통/모빌리티 (AUTO) =====
            'auto' => [
                'emoji' => '🚗',
                'name_en' => 'Autonomous Vehicle',
                'name_ko' => '자율주행',
                'bundle_title' => 'WIA Autonomous Vehicle Standard Guide Set (KO/EN)',
                'bundle_subtitle' => ' Sensor Fusion, V2X Communication, Path Planning, Localization Algorithms & Safety Validation Frameworks',
                'ko_title' => 'WIA 자율주행 표준화 가이드',
                'ko_subtitle' => '센서 융합, V2X 통신, 경로 계획, 위치추정 알고리즘 및 안전 검증 프레임워크',
                'en_title' => 'WIA Autonomous Vehicle Standard Guide',
                'en_subtitle' => 'Sensor Fusion, V2X Communication, Path Planning, Localization Algorithms & Safety Validation Frameworks',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Autonomous vehicle with 360-degree sensor fusion visualization, LiDAR point cloud forming 3D environmental map, V2X communication network between vehicles and infrastructure, path planning through dynamic traffic scene, automotive orange and tech blue gradients, dark navy background (#0f172a), self-driving aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 금융/경제 (FIN) =====
            'fintech' => [
                'emoji' => '💰',
                'name_en' => 'Fintech',
                'name_ko' => '핀테크',
                'bundle_title' => 'WIA Fintech Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Open Banking APIs, Payment Processing, Digital Wallets, KYC/AML Compliance & Real-Time Settlement Systems',
                'ko_title' => 'WIA 핀테크 표준화 가이드',
                'ko_subtitle' => '오픈뱅킹 API, 결제 처리, 디지털 지갑, KYC/AML 준수 및 실시간 결제 시스템',
                'en_title' => 'WIA Fintech Standard Guide',
                'en_subtitle' => 'Open Banking APIs, Payment Processing, Digital Wallets, KYC/AML Compliance & Real-Time Settlement Systems',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Open banking API ecosystem with financial data flowing between institutions, digital wallet with cryptocurrency and fiat integration, payment processing visualization as instant settlement network, KYC/AML compliance shown as identity verification layers, financial gold and fintech teal gradients, dark navy background (#0f172a), financial technology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 교육/문화 (EDU) =====
            'edu' => [
                'emoji' => '📚',
                'name_en' => 'Education Technology',
                'name_ko' => '교육 기술',
                'bundle_title' => 'WIA Education Technology Standard Guide Set (KO/EN)',
                'bundle_subtitle' => ' Learning Management Systems, Adaptive Learning Algorithms, Assessment Standards, Accessibility Features & Student Data Privacy',
                'ko_title' => 'WIA 교육 기술 표준화 가이드',
                'ko_subtitle' => '학습관리시스템, 적응형 학습 알고리즘, 평가 표준, 접근성 기능 및 학생 데이터 프라이버시',
                'en_title' => 'WIA Education Technology Standard Guide',
                'en_subtitle' => ' Learning Management Systems, Adaptive Learning Algorithms, Assessment Standards, Accessibility Features & Student Data Privacy',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Adaptive learning algorithm adjusting difficulty based on student performance graph, LMS dashboard showing gamified progress paths, accessibility features as inclusive design elements, student data privacy vault with encryption shield, academic purple and knowledge blue gradients, dark navy background (#0f172a), edtech aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'game' => [
                'emoji' => '🎮',
                'name_en' => 'Game Technology',
                'name_ko' => '게임 기술',
                'bundle_title' => 'WIA Game Technology Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Game Engine Standards, Multiplayer Protocols, Anti-Cheat Systems, Cross-Platform Play & Accessibility Options',
                'ko_title' => 'WIA 게임 기술 표준화 가이드',
                'ko_subtitle' => '게임 엔진 표준, 멀티플레이어 프로토콜, 치트 방지 시스템, 크로스 플랫폼 플레이 및 접근성 옵션',
                'en_title' => 'WIA Game Technology Standard Guide',
                'en_subtitle' => 'Game Engine Standards, Multiplayer Protocols, Anti-Cheat Systems, Cross-Platform Play & Accessibility Options',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Cross-platform play connecting console, PC, and mobile devices, game engine architecture with rendering pipeline visualization, anti-cheat system detecting anomalous player behavior, accessibility options shown as inclusive controller adaptations, gaming purple and esports green gradients, dark navy background (#0f172a), game technology design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'xr' => [
                'emoji' => '🥽',
                'name_en' => 'Extended Reality',
                'name_ko' => '확장 현실',
                'bundle_title' => 'WIA Extended Reality Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Spatial Computing Standards, Hand Tracking, 6DOF Tracking, Haptic Feedback & Cross-Reality Interoperability',
                'ko_title' => 'WIA 확장 현실 표준화 가이드',
                'ko_subtitle' => '공간 컴퓨팅 표준, 핸드 트래킹, 6DOF 추적, 햅틱 피드백 및 크로스 리얼리티 상호운용성',
                'en_title' => 'WIA Extended Reality Standard Guide',
                'en_subtitle' => 'Spatial Computing Standards, Hand Tracking, 6DOF Tracking, Haptic Feedback & Cross-Reality Interoperability',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Mixed reality space blending physical and virtual worlds, hand tracking with finger skeleton overlay, 6DOF tracking showing position and rotation in 3D space, spatial audio visualization as directional sound waves, immersive violet and holographic cyan gradients, dark navy background (#0f172a), extended reality aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
                'pubscript' => [
                'emoji' => '📖',
                'name_en' => 'PubScript',
                'name_ko' => '출판 스크립트',
                'bundle_title' => 'WIA PubScript Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Digital Publishing Formats, DRM Standards, Accessibility Metadata, Distribution Protocols & Reader Compatibility',
                'ko_title' => 'WIA 출판 스크립트 표준화 가이드',
                'ko_subtitle' => '디지털 출판 형식, DRM 표준, 접근성 메타데이터, 배포 프로토콜 및 리더 호환성',
                'en_title' => 'WIA PubScript Standard Guide',
                'en_subtitle' => 'Digital Publishing Formats, DRM Standards, Accessibility Metadata, Distribution Protocols & Reader Compatibility',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Digital manuscript transforming into multiple e-book formats, DRM protection shown as encryption wrapper around content, accessibility metadata as semantic structure overlay, distribution network with reader device compatibility matrix, publishing rose and digital ink blue gradients, dark navy background (#0f172a), digital publishing design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 건축/도시 (CITY) =====
            'digital-twin-city' => [
                'emoji' => '🏙️',
                'name_en' => 'Digital Twin City',
                'name_ko' => '디지털 트윈 도시',
                'bundle_title' => 'WIA Digital Twin City Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Real-Time City Modeling, IoT Sensor Integration, Simulation Engines, Urban Planning Tools & Citizen Data Privacy',
                'ko_title' => 'WIA 디지털 트윈 도시 표준화 가이드',
                'ko_subtitle' => '실시간 도시 모델링, IoT 센서 통합, 시뮬레이션 엔진, 도시계획 도구 및 시민 데이터 프라이버시',
                'en_title' => 'WIA Digital Twin City Standard Guide',
                'en_subtitle' => 'Real-Time City Modeling, IoT Sensor Integration, Simulation Engines, Urban Planning Tools & Citizen Data Privacy',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Real city with transparent holographic twin overlay showing real-time data, IoT sensor network forming mesh across urban infrastructure, simulation engine predicting traffic and energy flows, urban planning tools manipulating digital twin elements, urban cyan and smart city lights gradients, dark navy background (#0f172a), city digital twin aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'smarthome' => [
                'emoji' => '🏠',
                'name_en' => 'Smart Home',
                'name_ko' => '스마트홈',
                'bundle_title' => 'WIA Smart Home Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'IoT Device Protocols, Home Automation Standards, Energy Management, Security Systems & Voice Control Integration',
                'ko_title' => 'WIA 스마트홈 표준화 가이드',
                'ko_subtitle' => 'IoT 기기 프로토콜, 홈 오토메이션 표준, 에너지 관리, 보안 시스템 및 음성 제어 통합',
                'en_title' => 'WIA Smart Home Standard Guide',
                'en_subtitle' => 'IoT Device Protocols, Home Automation Standards, Energy Management, Security Systems & Voice Control Integration',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Home cross-section with IoT devices interconnected through WiFi mesh, energy management system optimizing consumption with smart meter, voice assistant hub controlling entire home ecosystem, security camera network with AI detection zones, home orange and tech teal gradients, dark navy background (#0f172a), smart home design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'home' => [
                'emoji' => '🏡',
                'name_en' => 'Home Automation',
                'name_ko' => '홈 자동화',
                'bundle_title' => 'WIA Home Automation Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Unified Control Interfaces, Scheduling Automation, Scene Management, Device Interoperability & Remote Access Security',
                'ko_title' => 'WIA 홈 자동화 표준화 가이드',
                'ko_subtitle' => '통합 제어 인터페이스, 스케줄 자동화, 장면 관리, 기기 상호운용성 및 원격 접근 보안',
                'en_title' => 'WIA Home Automation Standard Guide',
                'en_subtitle' => 'Unified Control Interfaces, Scheduling Automation, Scene Management, Device Interoperability & Remote Access Security',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Unified control tablet interface managing all home systems, automated schedule timeline with sunrise simulation and evening routines, scene presets shown as one-touch environment configurations, interoperability between different smart device brands, automation blue and convenience amber gradients, dark navy background (#0f172a), home automation aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 농업/식량 (AGRI) =====
            'food-allergy-passport' => [
                'emoji' => '🍽️',
                'name_en' => 'Food Allergy Passport',
                'name_ko' => '식품 알레르기 여권',
                'bundle_title' => 'WIA Food Allergy Passport Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Allergen Database Standards, QR Code Verification, Cross-Border Recognition, Restaurant Integration & Emergency Protocols',
                'ko_title' => 'WIA 식품 알레르기 여권 표준화 가이드',
                'ko_subtitle' => '알레르겐 데이터베이스 표준, QR 코드 검증, 국가 간 인정, 레스토랑 통합 및 비상 프로토콜',
                'en_title' => 'WIA Food Allergy Passport Standard Guide',
                'en_subtitle' => 'Allergen Database Standards, QR Code Verification, Cross-Border Recognition, Restaurant Integration & Emergency Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Digital food passport card with allergen symbols and QR code, restaurant menu scanning showing safe food options highlighted, cross-border recognition with international allergen database, emergency protocol with EpiPen location services, food safety green and alert red gradients, dark navy background (#0f172a), food allergy design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 사회/인프라 (SOC) =====
            'social' => [
                'emoji' => '👥',
                'name_en' => 'Social Infrastructure',
                'name_ko' => '사회 인프라',
                'bundle_title' => 'WIA Social Infrastructure Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Community Platform Standards, Social Graph Data, Privacy Controls, Content Moderation & Decentralized Identity',
                'ko_title' => 'WIA 사회 인프라 표준화 가이드',
                'ko_subtitle' => '커뮤니티 플랫폼 표준, 소셜 그래프 데이터, 프라이버시 제어, 콘텐츠 중재 및 분산 신원',
                'en_title' => 'WIA Social Infrastructure Standard Guide',
                'en_subtitle' => 'Community Platform Standards, Social Graph Data, Privacy Controls, Content Moderation & Decentralized Identity',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Community network graph with user nodes and relationship edges, social graph visualization as organic tree structure, privacy controls shown as selective visibility layers, content moderation AI detecting harmful content, community purple and social blue gradients, dark navy background (#0f172a), social platform aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'refugee-credential' => [
                'emoji' => '🆔',
                'name_en' => 'Refugee Credential',
                'name_ko' => '난민 자격증명',
                'bundle_title' => 'WIA Refugee Credential Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Portable Identity Verification, Document Digitization, Multi-Lingual Support, Humanitarian Data Exchange & Privacy Protection',
                'ko_title' => 'WIA 난민 자격증명 표준화 가이드',
                'ko_subtitle' => '휴대 가능 신원 확인, 문서 디지털화, 다국어 지원, 인도주의 데이터 교환 및 프라이버시 보호',
                'en_title' => 'WIA Refugee Credential Standard Guide',
                'en_subtitle' => 'Portable Identity Verification, Document Digitization, Multi-Lingual Support, Humanitarian Data Exchange & Privacy Protection',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Portable digital identity card transcending physical borders, document digitization process from paper to secure blockchain, multi-lingual interface showing inclusivity, humanitarian data exchange between relief organizations, hope blue and humanity gold gradients, dark navy background (#0f172a), humanitarian tech design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 디지털 레거시 (LEG) =====
            'digital-will' => [
                'emoji' => '📜',
                'name_en' => 'Digital Will',
                'name_ko' => '디지털 유언장',
                'bundle_title' => 'WIA Digital Will Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Smart Contract Integration, Multi-Signature Verification, Conditional Execution, Asset Inventory & Beneficiary Management',
                'ko_title' => 'WIA 디지털 유언장 표준화 가이드',
                'ko_subtitle' => '스마트 컨트랙트 통합, 다중 서명 검증, 조건부 실행, 자산 목록 및 수익자 관리',
                'en_title' => 'WIA Digital Will Standard Guide',
                'en_subtitle' => 'Smart Contract Integration, Multi-Signature Verification, Conditional Execution, Asset Inventory & Beneficiary Management',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Smart contract document with conditional execution logic flowchart, multi-signature wallet requiring family consensus, asset inventory shown as digital estate map, beneficiary distribution visualization as inheritance tree, parchment gold and blockchain blue gradients, dark navy background (#0f172a), digital estate aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'digital-executor' => [
                'emoji' => '👤',
                'name_en' => 'Digital Executor',
                'name_ko' => '디지털 유언집행인',
                'bundle_title' => 'WIA Digital Executor Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Fiduciary Role Definition, Automated Task Execution, Court Integration, Asset Distribution & Accountability Protocols',
                'ko_title' => 'WIA 디지털 유언집행인 표준화 가이드',
                'ko_subtitle' => '수탁자 역할 정의, 자동화된 작업 실행, 법원 통합, 자산 분배 및 책임 프로토콜',
                'en_title' => 'WIA Digital Executor Standard Guide',
                'en_subtitle' => 'Fiduciary Role Definition, Automated Task Execution, Court Integration, Asset Distribution & Accountability Protocols',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Trusted guardian avatar with fiduciary responsibility badge, automated task execution checklist with completion indicators, court integration showing legal system interface, asset distribution flows to beneficiaries, trustworthy slate and guardian navy gradients, dark navy background (#0f172a), executor role design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'digital-memorial' => [
                'emoji' => '🕯️',
                'name_en' => 'Digital Memorial',
                'name_ko' => '디지털 추모',
                'bundle_title' => 'WIA Digital Memorial Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Memory Preservation Systems, Tribute Platforms, Access Control, Family Sharing & Perpetual Hosting Standards',
                'ko_title' => 'WIA 디지털 추모 표준화 가이드',
                'ko_subtitle' => '기억 보존 시스템, 추모 플랫폼, 접근 제어, 가족 공유 및 영구 호스팅 표준',
                'en_title' => 'WIA Digital Memorial Standard Guide',
                'en_subtitle' => 'Memory Preservation Systems, Tribute Platforms, Access Control, Family Sharing & Perpetual Hosting Standards',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Eternal memory shrine with photo galleries in floating frames, tribute wall where visitors leave digital flowers, access control allowing family circle to share memories, perpetual hosting shown as cloud storage in heaven metaphor, memorial lavender and eternal white gradients, dark navy background (#0f172a), remembrance aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'digital-funeral' => [
                'emoji' => '⚰️',
                'name_en' => 'Digital Funeral',
                'name_ko' => '디지털 장례',
                'bundle_title' => 'WIA Digital Funeral Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Virtual Ceremony Platforms, Streaming Protocols, Guest Management, Cultural Customization & Recording Standards',
                'ko_title' => 'WIA 디지털 장례 표준화 가이드',
                'ko_subtitle' => '가상 장례식 플랫폼, 스트리밍 프로토콜, 참석자 관리, 문화적 맞춤화 및 기록 표준',
                'en_title' => 'WIA Digital Funeral Standard Guide',
                'en_subtitle' => 'Virtual Ceremony Platforms, Streaming Protocols, Guest Management, Cultural Customization & Recording Standards',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Serene virtual memorial hall with soft candlelight glow, global attendees connected through holographic presence floating peacefully, white lilies and purple orchids in elegant arrangements, live streaming interface showing diverse cultural mourners paying respects, memorial recording archive visible as ethereal light particles, peaceful sunset gradient from deep violet to soft gold, respectful and solemn atmosphere, traditional and modern funeral elements harmoniously blended, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'digital-erasure' => [
                'emoji' => '🗑️',
                'name_en' => 'Digital Erasure',
                'name_ko' => '디지털 삭제',
                'bundle_title' => 'WIA Digital Erasure Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Right to Be Forgotten Implementation, Data Deletion Verification, Search Engine De-indexing, Archive Removal & Compliance Audit',
                'ko_title' => 'WIA 디지털 삭제 표준화 가이드',
                'ko_subtitle' => '잊힐 권리 구현, 데이터 삭제 검증, 검색엔진 색인 제거, 아카이브 삭제 및 준수 감사',
                'en_title' => 'WIA Digital Erasure Standard Guide',
                'en_subtitle' => 'Right to Be Forgotten Implementation, Data Deletion Verification, Search Engine De-indexing, Archive Removal & Compliance Audit',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Data deletion visualization as information dissolving into particles, right to be forgotten implementation with erasure spreading across networks, search engine de-indexing shown as links breaking, verification checkmarks confirming complete removal, cleansing white and privacy cyan gradients, dark navy background (#0f172a), data erasure aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            // ===== 반려동물/동물복지 (PET) =====
            'pet-genome' => [
                'emoji' => '🧬',
                'name_en' => 'Pet Genome',
                'name_ko' => '반려동물 유전체',
                'bundle_title' => 'WIA Pet Genome Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'DNA Sequencing Standards, Breed Identification, Health Risk Analysis, Genetic Counseling & Biobank Integration',
                'ko_title' => 'WIA 반려동물 유전체 표준화 가이드',
                'ko_subtitle' => 'DNA 서열 표준, 품종 식별, 건강 위험 분석, 유전 상담 및 바이오뱅크 통합',
                'en_title' => 'WIA Pet Genome Standard Guide',
                'en_subtitle' => 'DNA Sequencing Standards, Breed Identification, Health Risk Analysis, Genetic Counseling & Biobank Integration',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Pet DNA double helix with breed-specific genetic markers, breed identification puzzle pieces forming dog/cat silhouette, health risk analysis shown as genetic predisposition map, genetic counseling flowchart for responsible breeding, bio-cyan and pet orange gradients, dark navy background (#0f172a), veterinary genomics design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pet-emotion' => [
                'emoji' => '💕',
                'name_en' => 'Pet Emotion',
                'name_ko' => '반려동물 감정',
                'bundle_title' => 'WIA Pet Emotion Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Behavioral Pattern Recognition, Stress Level Detection, Vocal Analysis, Body Language Interpretation & Mood Tracking',
                'ko_title' => 'WIA 반려동물 감정 표준화 가이드',
                'ko_subtitle' => '행동 패턴 인식, 스트레스 수준 감지, 음성 분석, 신체 언어 해석 및 기분 추적',
                'en_title' => 'WIA Pet Emotion Standard Guide',
                'en_subtitle' => 'Behavioral Pattern Recognition, Stress Level Detection, Vocal Analysis, Body Language Interpretation & Mood Tracking',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Pet behavior pattern recognition with mood indicator meter, stress level visualization as color-changing aura around animal, vocal analysis spectrogram showing bark/meow interpretation, body language decoder with posture recognition, loving pink and empathy purple gradients, dark navy background (#0f172a), pet emotion aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pet-legacy' => [
                'emoji' => '🐾',
                'name_en' => 'Pet Legacy',
                'name_ko' => '반려동물 유산',
                'bundle_title' => 'WIA Pet Legacy Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Digital Pet Memorial, Photo/Video Archives, Medical History Preservation, Clone Data Storage & Memory Sharing Platforms',
                'ko_title' => 'WIA 반려동물 유산 표준화 가이드',
                'ko_subtitle' => '디지털 반려동물 추모, 사진/동영상 아카이브, 의료 기록 보존, 복제 데이터 저장 및 추억 공유 플랫폼',
                'en_title' => 'WIA Pet Legacy Standard Guide',
                'en_subtitle' => 'Digital Pet Memorial, Photo/Video Archives, Medical History Preservation, Clone Data Storage & Memory Sharing Platforms',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Digital pet memorial with paw print turning into eternal star, photo archive forming constellation of cherished moments, medical history timeline preserving life journey, memory sharing platform connecting pet lovers, memorial violet and rainbow bridge gradients, dark navy background (#0f172a), pet remembrance design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pet-care-robot' => [
                'emoji' => '🤖',
                'name_en' => 'Pet Care Robot',
                'name_ko' => '반려동물 케어 로봇',
                'bundle_title' => 'WIA Pet Care Robot Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Automated Feeding Systems, Play Interaction, Health Monitoring, Emergency Detection & Remote Owner Communication',
                'ko_title' => 'WIA 반려동물 케어 로봇 표준화 가이드',
                'ko_subtitle' => '자동 급식 시스템, 놀이 상호작용, 건강 모니터링, 비상 상황 감지 및 원격 보호자 통신',
                'en_title' => 'WIA Pet Care Robot Standard Guide',
                'en_subtitle' => 'Automated Feeding Systems, Play Interaction, Health Monitoring, Emergency Detection & Remote Owner Communication',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Friendly robotic pet companion with treat dispenser and toy launcher, health monitoring sensors scanning pet vitals, emergency detection alerting owner through smartphone, remote camera allowing owner to interact from anywhere, friendly orange and tech teal gradients, dark navy background (#0f172a), pet robot aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pet-welfare-global' => [
                'emoji' => '🌍',
                'name_en' => 'Pet Welfare Global',
                'name_ko' => '글로벌 동물복지',
                'bundle_title' => 'WIA Pet Welfare Global Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'International Standards Harmonization, Abuse Detection, Shelter Management, Adoption Protocols & Cross-Border Transfer',
                'ko_title' => 'WIA 글로벌 동물복지 표준화 가이드',
                'ko_subtitle' => '국제 표준 조화, 학대 탐지, 보호소 관리, 입양 프로토콜 및 국경 간 이동',
                'en_title' => 'WIA Pet Welfare Global Standard Guide',
                'en_subtitle' => 'International Standards Harmonization, Abuse Detection, Shelter Management, Adoption Protocols & Cross-Border Transfer',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, World map with animal welfare standards harmonization visualization, abuse detection AI analyzing pet behavior for signs of mistreatment, shelter management system optimizing adoption matches, international pet transport with welfare checkpoints, earth green and compassion blue gradients, dark navy background (#0f172a), global welfare minimalist design, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'pet-health-passport' => [
                'emoji' => '🏥',
                'name_en' => 'Pet Health Passport',
                'name_ko' => '반려동물 건강 여권',
                'bundle_title' => 'WIA Pet Health Passport Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Vaccination Records, Veterinary Visit History, Medical Imaging, International Travel Requirements & Emergency Contact Data',
                'ko_title' => 'WIA 반려동물 건강 여권 표준화 가이드',
                'ko_subtitle' => '예방접종 기록, 동물병원 방문 이력, 의료 영상, 국제 여행 요구사항 및 비상 연락처 데이터',
                'en_title' => 'WIA Pet Health Passport Standard Guide',
                'en_subtitle' => 'Vaccination Records, Veterinary Visit History, Medical Imaging, International Travel Requirements & Emergency Contact Data',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Digital pet passport with vaccination record timeline, veterinary visit history shown as medical journey map, international travel requirements checklist with country-specific rules, emergency contact with nearest vet location services, medical green and travel blue gradients, dark navy background (#0f172a), pet health documentation aesthetic, no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            'ai-city' => [
                'emoji' => '🏙️',
                'name_en' => 'AI City',
                'name_ko' => 'AI 시티',
                'bundle_title' => 'WIA AI City Standard Guide Set (KO/EN)',
                'bundle_subtitle' => 'Rust-Core Control Systems, 1% Social Contribution Engine, Citizen Data Sovereignty & AIDC-Link 4-Layer Protocol',
                'ko_title' => 'WIA AI 시티 표준화 가이드',
                'ko_subtitle' => 'Rust-Core 제어 시스템, 1% 사회 환원 엔진, 시민 데이터 주권 및 AIDC-Link 4계층 프로토콜',
                'en_title' => 'WIA AI City Standard Guide',
                'en_subtitle' => 'Rust-Core Control Systems, 1% Social Contribution Engine, Citizen Data Sovereignty & AIDC-Link 4-Layer Protocol',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Futuristic AI-powered smart city with Rust-Core neural control tower, cyan and orange tech gradients, dark navy background (#0f172a), no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
            
            'rust-learn' => [
                'emoji' => '🦀',
                'name_en' => 'Rust Learn',
                'name_ko' => 'Rust 학습',
                'bundle_title' => 'WIA Rust Learn Guide Set (KO/EN)',
                'bundle_subtitle' => 'Zero-to-Rust Programming, Ownership System, Borrowing & Lifetimes, Fearless Concurrency & Real Projects',
                'ko_title' => 'WIA Rust 학습 가이드',
                'ko_subtitle' => 'Rust 프로그래밍 입문, 소유권 시스템, 빌림과 라이프타임, 두려움 없는 동시성 및 실전 프로젝트',
                'en_title' => 'WIA Rust Learn Guide',
                'en_subtitle' => 'Zero-to-Rust Programming, Ownership System, Borrowing & Lifetimes, Fearless Concurrency & Real Projects',
                'cover_prompt' => 'Reserve top 20% with pure black (#000000) background for title text area, Rust programming crab Ferris with memory safety visualization, rust orange gradients, dark navy background (#0f172a), no text, no letters, no words, professional book cover, 4K quality. Generate this professional book cover image.'
            ],
        ];
    }
}
