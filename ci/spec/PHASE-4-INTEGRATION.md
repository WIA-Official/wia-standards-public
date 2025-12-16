# Phase 4: CI Integration Specification

## WIA-CI Ecosystem Integration

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

---

## 1. 개요

WIA-CI는 WIA 대가족 생태계와 완벽히 통합되어,
인공와우 사용자에게 최적의 경험을 제공합니다.

### 1.1 WIA 대가족 통합

```
┌──────────────────────────────────────────────────────────────────┐
│                       WIA 대가족 구조                             │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  👨 아버지 (INTENT)                                               │
│     └─→ "음악 들려줘" → CI에 최적화된 음악 스트리밍               │
│                                                                  │
│  👩 어머니 (OMNI-API)                                             │
│     └─→ 모든 CI 제조사 API를 하나로 통합                          │
│                                                                  │
│  💪 삼촌 (AIR-POWER)                                              │
│     └─→ 저전력 프로세싱으로 배터리 수명 연장                       │
│                                                                  │
│  🛡️ 이모 (AIR-SHIELD)                                            │
│     └─→ 청각 데이터 프라이버시 보호                               │
│                                                                  │
│  🌐 조카 (SOCIAL)                                                 │
│     └─→ 청각장애인 커뮤니티 연결                                   │
│                                                                  │
│  🏠 집 (HOME)                                                     │
│     └─→ CI 설정/관리 홈페이지                                     │
│                                                                  │
│  👂 CI (Cochlear Implant)                                         │
│     └─→ 옥타브 향상으로 음악 청취 개선                             │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. WIA-INTENT 통합

### 2.1 음성 명령 인터페이스

```typescript
import { WIAIntent } from '@anthropic/wia-intent';
import { WIACI } from '@anthropic/wia-ci';

// INTENT를 통한 자연어 CI 제어
const intent = new WIAIntent();
const ci = new WIACI();

// 음성 명령 처리
intent.on('command', async (cmd) => {
  switch (cmd.intent) {
    case 'music.play':
      // 음악 모드 자동 활성화
      await ci.setMode('music');
      await ci.wia.enableOctaveEnhancement();
      break;

    case 'volume.adjust':
      await ci.setVolume(cmd.parameters.level);
      break;

    case 'program.switch':
      await ci.switchProgram(cmd.parameters.program);
      break;

    case 'environment.adapt':
      // 환경에 맞게 자동 조절
      await ci.adaptToEnvironment(cmd.parameters.environment);
      break;
  }
});

// 예시 대화
// 사용자: "음악 모드로 바꿔줘"
// INTENT: { intent: 'mode.switch', parameters: { mode: 'music' } }
// → CI가 음악 모드로 전환 + 옥타브 향상 활성화
```

### 2.2 상황 인식 자동 조절

```typescript
interface ContextAwareCI {
  // INTENT가 감지한 상황
  context: {
    location: 'home' | 'office' | 'restaurant' | 'concert' | 'outdoor';
    activity: 'conversation' | 'music' | 'meeting' | 'exercise';
    noiseLevel: 'quiet' | 'moderate' | 'loud';
    companions: number;
  };

  // 자동 프로파일 적용
  autoProfile: {
    enabled: boolean;
    profiles: ContextProfile[];
  };
}

// 상황별 자동 최적화
const contextProfiles: ContextProfile[] = [
  {
    name: 'concert',
    conditions: {
      location: 'concert',
      activity: 'music'
    },
    settings: {
      mode: 'music',
      octaveEnhancement: true,
      tfsEncoding: true,
      noiseReduction: 'low',      // 음악 보존
      volume: 70
    }
  },
  {
    name: 'restaurant',
    conditions: {
      location: 'restaurant',
      noiseLevel: 'loud'
    },
    settings: {
      mode: 'speech',
      directionalMic: true,
      noiseReduction: 'high',
      beamforming: 'front'
    }
  }
];
```

---

## 3. WIA-OMNI-API 통합

### 3.1 제조사 통합 인터페이스

```typescript
import { OmniAPI } from '@anthropic/wia-omni-api';
import { CochlearAdapter, MedElAdapter, ABAdapter } from '@anthropic/wia-ci';

// 어머니(OMNI-API)가 모든 제조사를 하나로
const omni = new OmniAPI();

// 제조사별 어댑터 등록
omni.registerAdapter('cochlear', new CochlearAdapter());
omni.registerAdapter('medel', new MedElAdapter());
omni.registerAdapter('ab', new ABAdapter());

// 통합 인터페이스로 사용
const ci = await omni.getDevice('cochlear', deviceId);

// 동일한 API로 모든 제조사 제어
await ci.setVolume(80);                    // 모든 제조사 동일
await ci.setProgram('music');              // 내부적으로 변환
await ci.getElectrodeImpedances();         // 채널 수 자동 매핑
```

### 3.2 제조사별 채널 매핑

```typescript
interface ManufacturerMapping {
  cochlear: {
    electrodes: 22;
    frequencyRange: [250, 8000];
    strategies: ['ACE', 'CIS', 'MP3000'];
  };

  medel: {
    electrodes: 12;
    frequencyRange: [250, 8500];
    strategies: ['FSP', 'FS4', 'HDCIS'];
  };

  ab: {
    electrodes: 16;
    frequencyRange: [250, 8000];
    strategies: ['HiRes', 'Optima', 'ClearVoice'];
  };
}

// 22채널 표준에서 제조사별 변환
function mapTo22Channels(
  manufacturer: Manufacturer,
  electrodeData: number[]
): number[] {
  switch (manufacturer) {
    case 'cochlear':
      return electrodeData;  // 직접 매핑

    case 'medel':
      // 12 → 22 채널 업샘플링
      return interpolateChannels(electrodeData, 12, 22);

    case 'ab':
      // 16 → 22 채널 업샘플링
      return interpolateChannels(electrodeData, 16, 22);
  }
}
```

---

## 4. WIA-AIR-POWER 통합

### 4.1 저전력 처리

```typescript
import { AirPower } from '@anthropic/wia-air-power';
import { WIACI } from '@anthropic/wia-ci';

// 삼촌(AIR-POWER)의 전력 관리
const power = new AirPower();
const ci = new WIACI();

// 전력 최적화 모드
power.optimize(ci, {
  // 배터리 수명 우선
  mode: 'battery_saver',

  // 처리 분산
  processing: {
    octaveDetection: 'onDevice',      // 기기에서 처리
    harmonicAnalysis: 'onDevice',
    tfsEncoding: 'onDevice',

    // 복잡한 처리만 클라우드
    advancedMusicMode: 'cloud'
  },

  // 절전 설정
  powerSaving: {
    dimDisplayAfter: 30,              // 초
    reduceSamplingInQuiet: true,
    adaptiveProcessing: true
  }
});

// 배터리 상태 모니터링
power.on('battery_low', async (level) => {
  if (level < 10) {
    // 필수 기능만 유지
    await ci.setMode('essential');
    await ci.wia.disableOctaveEnhancement();
  }
});
```

### 4.2 처리 부하 분산

```typescript
interface ProcessingDistribution {
  // 에지 처리 (프로세서)
  edge: {
    latencySensitive: ['envelope', 'stimulation'],
    alwaysLocal: ['safety_limits', 'electrode_switching']
  };

  // 클라우드 처리
  cloud: {
    batchProcessing: ['training_data', 'analytics'],
    optional: ['advanced_octave', 'personalization']
  };

  // 하이브리드
  hybrid: {
    preference: 'edge_first',
    fallback: 'cloud',
    syncInterval: 3600               // 1시간마다 동기화
  };
}
```

---

## 5. WIA-AIR-SHIELD 통합

### 5.1 청각 데이터 프라이버시

```typescript
import { AirShield } from '@anthropic/wia-air-shield';
import { WIACI } from '@anthropic/wia-ci';

// 이모(AIR-SHIELD)가 지켜줌
const shield = new AirShield();
const ci = new WIACI();

// 프라이버시 보호 설정
shield.protect(ci, {
  // 데이터 익명화
  anonymization: {
    patientId: 'hash',                // ID 해시화
    location: 'remove',               // 위치 정보 제거
    voiceData: 'never_store'          // 음성 데이터 저장 안함
  },

  // 접근 제어
  access: {
    clinician: ['config', 'telemetry'],
    researcher: ['anonymized_stats'],
    user: ['all']
  },

  // 암호화
  encryption: {
    atRest: 'AES-256',
    inTransit: 'TLS-1.3',
    e2e: true                         // 종단간 암호화
  }
});

// 데이터 요청 시 자동 보호
ci.on('data_request', async (request) => {
  const protected = await shield.processRequest(request);
  return protected;
});
```

### 5.2 HIPAA/GDPR 준수

```typescript
interface ComplianceConfig {
  hipaa: {
    phiProtection: true;
    auditLogging: true;
    accessControl: 'role_based';
    breachNotification: true;
  };

  gdpr: {
    dataMinimization: true;
    rightToErasure: true;
    portability: true;
    consentManagement: true;
  };

  // 한국 개인정보보호법
  pipa: {
    consentRequired: true;
    purposeLimitation: true;
    dataRetentionLimit: 365           // 일
  };
}
```

---

## 6. WIA-SOCIAL 통합

### 6.1 청각장애인 커뮤니티

```typescript
import { WIASocial } from '@anthropic/wia-social';
import { WIACI } from '@anthropic/wia-ci';

// 조카(SOCIAL)로 커뮤니티 연결
const social = new WIASocial();
const ci = new WIACI();

// CI 사용자 커뮤니티
const community = await social.joinCommunity('ci_users');

// 설정 공유
await community.shareSettings({
  profile: 'concert_music',
  settings: ci.getCurrentConfig(),
  rating: 4.5,
  description: '콘서트에서 잘 들려요!'
});

// 다른 사용자 설정 가져오기
const recommendations = await community.getRecommendations({
  scenario: 'classical_concert',
  implantModel: 'CI622'
});

// 추천 설정 적용
await ci.applyConfig(recommendations[0].settings);
```

### 6.2 실시간 자막 공유

```typescript
// 그룹 대화에서 실시간 자막
const group = await social.createGroup(['user1', 'user2', 'user3']);

group.on('speech', async (event) => {
  // 다른 사용자의 발화를 자막으로
  const caption = await transcribe(event.audio);
  displayCaption(caption, event.speaker);
});

// CI 사용자 간 최적화된 통신
group.on('ci_user_speaking', async (event) => {
  // CI에 최적화된 오디오 인코딩
  const optimizedAudio = await ci.optimizeForCI(event.audio);
  broadcast(optimizedAudio);
});
```

---

## 7. WIA-HOME 통합

### 7.1 CI 관리 홈페이지

```typescript
import { WIAHome } from '@anthropic/wia-home';
import { WIACI } from '@anthropic/wia-ci';

// 집(HOME)에서 CI 관리
const home = new WIAHome({
  name: 'My CI Dashboard',
  template: 'ci_management'
});

// CI 대시보드 페이지
home.addPage('dashboard', {
  widgets: [
    // 배터리 상태
    { type: 'battery', source: ci.getBatteryLevel },

    // 현재 프로그램
    { type: 'program', source: ci.getCurrentProgram },

    // 청취 통계
    { type: 'stats', source: ci.getListeningStats },

    // 옥타브 향상 상태
    { type: 'wia_octave', source: ci.wia.getOctaveStatus }
  ]
});

// 설정 페이지
home.addPage('settings', {
  forms: [
    { type: 'volume_control', handler: ci.setVolume },
    { type: 'program_select', handler: ci.setProgram },
    { type: 'wia_toggle', handler: ci.wia.toggleOctave }
  ]
});

await home.start();
// 🏠 myci.wia.home 에서 CI 관리!
```

---

## 8. 임상 시스템 통합

### 8.1 청각사 워크스테이션

```typescript
interface AudiologistWorkstation {
  // 환자 관리
  patients: {
    list: PatientInfo[];
    appointments: Appointment[];
    history: SessionHistory[];
  };

  // 피팅 도구
  fitting: {
    measureNRT: () => NRTResult;
    adjustTCLevels: (levels: number[]) => void;
    runLiveVoice: () => void;
    saveSession: () => void;
  };

  // WIA 확장
  wiaTools: {
    testOctavePerception: () => OctaveTestResult;
    optimizeMusicMode: () => MusicOptimizationResult;
    compareWithStandard: () => ComparisonResult;
  };
}

// 임상 워크플로우
async function clinicalFitting(patientId: string): Promise<void> {
  const workstation = new AudiologistWorkstation();

  // 1. 임피던스 측정
  const impedances = await workstation.fitting.measureImpedances();
  console.log('전극 상태:', checkElectrodeStatus(impedances));

  // 2. NRT 측정
  const nrt = await workstation.fitting.measureNRT();
  console.log('신경 반응:', nrt.amplitudes);

  // 3. T/C 레벨 조정
  await workstation.fitting.adjustTCLevels({
    thresholds: nrt.suggestedT,
    comforts: nrt.suggestedC
  });

  // 4. WIA 옥타브 테스트
  const octaveTest = await workstation.wiaTools.testOctavePerception();
  console.log('옥타브 인식률:', octaveTest.accuracy);

  // 5. 음악 모드 최적화
  if (octaveTest.accuracy > 0.7) {
    await workstation.wiaTools.optimizeMusicMode();
  }

  // 6. 세션 저장
  await workstation.fitting.saveSession();
}
```

### 8.2 EHR 연동

```typescript
interface EHRIntegration {
  // HL7 FHIR 지원
  fhir: {
    endpoint: string;
    version: 'R4';
    resources: ['Patient', 'Device', 'Observation'];
  };

  // 데이터 교환
  exchange: {
    exportSession: (format: 'fhir' | 'hl7v2') => ExportData;
    importPatient: (patientId: string) => PatientData;
    syncSettings: () => void;
  };
}

// FHIR Device 리소스로 CI 표현
const ciDevice: fhir.Device = {
  resourceType: 'Device',
  identifier: [{
    system: 'urn:wia:ci:serial',
    value: 'CI-2025-001234'
  }],
  type: {
    coding: [{
      system: 'http://snomed.info/sct',
      code: '43252007',
      display: 'Cochlear implant'
    }]
  },
  patient: { reference: 'Patient/12345' },
  property: [
    {
      type: { text: 'electrodeCount' },
      valueQuantity: { value: 22 }
    },
    {
      type: { text: 'wiaOctaveEnhancement' },
      valueCode: [{ code: 'enabled' }]
    }
  ]
};
```

---

## 9. 모바일 SDK

### 9.1 iOS SDK

```swift
import WIACI

// iOS 앱에서 CI 제어
class CIViewController: UIViewController {
    let ci = WIACIManager.shared

    override func viewDidLoad() {
        super.viewDidLoad()

        // BLE 연결
        ci.connect { result in
            switch result {
            case .success(let device):
                self.setupUI(device)
            case .failure(let error):
                self.showError(error)
            }
        }
    }

    // 볼륨 조절
    @IBAction func volumeChanged(_ sender: UISlider) {
        ci.setVolume(Int(sender.value))
    }

    // 음악 모드 토글
    @IBAction func musicModeToggled(_ sender: UISwitch) {
        ci.wia.setOctaveEnhancement(enabled: sender.isOn)
    }

    // 실시간 오디오 시각화
    func setupAudioVisualization() {
        ci.subscribe(to: .audioStream) { frame in
            DispatchQueue.main.async {
                self.visualizer.update(with: frame.envelopes)
                self.octaveLabel.text = "Octave: \(frame.octaveInfo.octaveNumber)"
            }
        }
    }
}
```

### 9.2 Android SDK

```kotlin
import com.wia.ci.sdk.*

class CIActivity : AppCompatActivity() {
    private lateinit var ci: WIACIManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        ci = WIACIManager.getInstance(this)

        // 연결
        ci.connect(object : ConnectionCallback {
            override fun onConnected(device: CIDevice) {
                setupUI(device)
            }

            override fun onError(error: CIError) {
                showError(error)
            }
        })
    }

    // 프로그램 변경
    fun changeProgram(program: CIProgram) {
        ci.setProgram(program)
    }

    // WIA 옥타브 상태
    fun getOctaveStatus(): WIAOctaveStatus {
        return ci.wia.getOctaveStatus()
    }

    // 음악 모드 활성화
    fun enableMusicMode() {
        ci.wia.apply {
            setOctaveEnhancement(true)
            setTFSEncoding(true)
            setMusicMode(MusicMode.AUTO)
        }
    }
}
```

### 9.3 Flutter SDK

```dart
import 'package:wia_ci/wia_ci.dart';

class CIHomePage extends StatefulWidget {
  @override
  _CIHomePageState createState() => _CIHomePageState();
}

class _CIHomePageState extends State<CIHomePage> {
  final ci = WIACIManager();
  bool octaveEnabled = false;

  @override
  void initState() {
    super.initState();
    _connectCI();
  }

  Future<void> _connectCI() async {
    await ci.connect();
    setState(() {
      octaveEnabled = ci.wia.isOctaveEnabled;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('My CI')),
      body: Column(
        children: [
          // 배터리 표시
          BatteryWidget(level: ci.batteryLevel),

          // 볼륨 슬라이더
          VolumeSlider(
            value: ci.volume,
            onChanged: (v) => ci.setVolume(v),
          ),

          // 옥타브 향상 토글
          SwitchListTile(
            title: Text('Octave Enhancement'),
            subtitle: Text('Better music perception'),
            value: octaveEnabled,
            onChanged: (v) {
              ci.wia.setOctaveEnhancement(v);
              setState(() => octaveEnabled = v);
            },
          ),

          // 현재 모드
          ModeIndicator(mode: ci.currentMode),
        ],
      ),
    );
  }
}
```

---

## 10. 클라우드 서비스

### 10.1 WIA-CI Cloud API

```typescript
// 클라우드 API 엔드포인트
const CLOUD_API = 'https://api.wia.ci/v1';

interface WIACICloud {
  // 사용자 관리
  users: {
    register: (user: UserInfo) => Promise<UserId>;
    login: (credentials: Credentials) => Promise<AuthToken>;
    getProfile: () => Promise<UserProfile>;
  };

  // 장치 관리
  devices: {
    register: (device: DeviceInfo) => Promise<DeviceId>;
    getStatus: (deviceId: string) => Promise<DeviceStatus>;
    sync: (deviceId: string) => Promise<SyncResult>;
  };

  // 설정 동기화
  config: {
    backup: (config: CIConfiguration) => Promise<BackupId>;
    restore: (backupId: string) => Promise<CIConfiguration>;
    share: (configId: string, userId: string) => Promise<ShareResult>;
  };

  // 분석
  analytics: {
    getListeningStats: (period: DateRange) => Promise<ListeningStats>;
    getMusicUsage: (period: DateRange) => Promise<MusicUsageStats>;
    getOctavePerformance: () => Promise<OctavePerformanceStats>;
  };
}
```

### 10.2 실시간 분석

```typescript
interface RealtimeAnalytics {
  // 스트리밍 분석
  streaming: {
    // 음악 감지 정확도
    musicDetectionAccuracy: number;

    // 옥타브 인식 성능
    octaveRecognitionRate: number;

    // 하모닉 보존율
    harmonicPreservation: number;
  };

  // 일일 통계
  daily: {
    totalListeningTime: number;        // 분
    musicListeningTime: number;        // 분
    speechListeningTime: number;       // 분
    averageVolume: number;
    programChanges: number;
  };

  // 장기 추세
  trends: {
    musicEngagement: TrendData;        // 음악 청취 증가/감소
    octavePerception: TrendData;       // 옥타브 인식 향상
    satisfaction: TrendData;           // 만족도
  };
}

// 분석 대시보드
async function generateAnalyticsDashboard(
  userId: string,
  period: DateRange
): Promise<Dashboard> {
  const stats = await cloud.analytics.getListeningStats(period);
  const music = await cloud.analytics.getMusicUsage(period);
  const octave = await cloud.analytics.getOctavePerformance();

  return {
    summary: {
      totalHours: stats.totalMinutes / 60,
      musicPercentage: music.totalMinutes / stats.totalMinutes * 100,
      octaveImprovement: octave.improvementPercent
    },
    charts: {
      dailyUsage: generateDailyChart(stats.daily),
      musicGenres: generatePieChart(music.genres),
      octaveTrend: generateLineChart(octave.trend)
    },
    recommendations: generateRecommendations(stats, music, octave)
  };
}
```

---

## 11. 서드파티 통합

### 11.1 음악 스트리밍 서비스

```typescript
interface MusicStreamingIntegration {
  // 지원 서비스
  services: ['spotify', 'apple_music', 'youtube_music', 'melon', 'genie'];

  // 스트리밍 최적화
  optimization: {
    // CI에 맞게 오디오 처리
    preprocess: (audio: AudioBuffer) => ProcessedAudio;

    // 추천 음악 (CI 친화적)
    recommend: () => Track[];

    // 가사 동기화 자막
    syncLyrics: (trackId: string) => LyricSync;
  };
}

// Spotify 연동 예시
import { SpotifyApi } from 'spotify-web-api-node';

async function optimizeSpotifyForCI(
  ci: WIACI,
  spotify: SpotifyApi
): Promise<void> {
  // 현재 재생 중인 트랙
  const current = await spotify.getMyCurrentPlayingTrack();

  // CI에 최적화된 EQ 적용
  if (current.item) {
    const analysis = await spotify.getAudioAnalysis(current.item.id);

    // 음악 특성에 따른 CI 조정
    if (analysis.track.tempo > 120) {
      // 빠른 템포 - TFS 강화
      await ci.wia.setTFSStrength(0.8);
    }

    if (analysis.track.key !== null) {
      // 키 정보로 옥타브 힌트
      await ci.wia.setKeyHint(analysis.track.key);
    }
  }
}
```

### 11.2 스마트홈 통합

```typescript
interface SmartHomeIntegration {
  // 지원 플랫폼
  platforms: ['homekit', 'google_home', 'smartthings', 'alexa'];

  // 자동화
  automations: {
    // 초인종이 울리면 CI에 알림
    doorbell: {
      trigger: 'doorbell.ring',
      action: 'ci.alert.doorbell'
    };

    // 화재 경보기 알림
    smoke: {
      trigger: 'smoke_detector.alert',
      action: 'ci.alert.emergency'
    };

    // TV 시청 시 자동 프로그램 변경
    tv: {
      trigger: 'tv.power.on',
      action: 'ci.program.switch:tv'
    };
  };
}

// HomeKit 연동
import { HomeKit } from 'homekit-sdk';

const homekit = new HomeKit();

// CI를 HomeKit 액세서리로 등록
homekit.addAccessory({
  name: 'My CI',
  category: 'hearing_aid',
  services: [
    {
      type: 'hearing_aid',
      characteristics: [
        { name: 'volume', type: 'int', min: 0, max: 100 },
        { name: 'program', type: 'string' },
        { name: 'battery', type: 'int', min: 0, max: 100 }
      ]
    }
  ]
});

// 시리 명령
// "Hey Siri, CI 볼륨 올려줘"
// "Hey Siri, 음악 모드로 바꿔"
```

### 11.3 웨어러블 연동

```typescript
interface WearableIntegration {
  // 지원 기기
  devices: ['apple_watch', 'galaxy_watch', 'fitbit'];

  // 기능
  features: {
    // 손목에서 볼륨 조절
    volumeControl: boolean;

    // 프로그램 변경
    programSwitch: boolean;

    // 배터리 알림
    batteryAlert: boolean;

    // 심박수 기반 적응
    // (운동 중 자동으로 노이즈 리덕션 조절)
    heartRateAdaptation: boolean;
  };
}

// Apple Watch 컴플리케이션
struct CIComplication: View {
    @ObservedObject var ci: CIWatchManager

    var body: some View {
        VStack {
            // 배터리
            BatteryGauge(level: ci.batteryLevel)

            // 현재 모드 아이콘
            Image(systemName: ci.currentMode == .music ? "music.note" : "waveform")

            // 볼륨
            Text("\(ci.volume)%")
        }
    }
}
```

---

## 12. 개발자 도구

### 12.1 시뮬레이터

```typescript
import { CISimulator } from '@anthropic/wia-ci-simulator';

// CI 프로세싱 시뮬레이션
const sim = new CISimulator({
  electrodes: 22,
  strategy: 'WIA-OCTAVE'
});

// 오디오 파일로 테스트
const result = await sim.process('music_sample.wav');

console.log('처리 결과:');
console.log('- F0 추정:', result.f0);
console.log('- 옥타브:', result.octave);
console.log('- 선택 채널:', result.selectedChannels);

// 청각 시뮬레이션 (CI로 듣는 소리 재현)
const simulated = await sim.synthesize(result);
await playAudio(simulated);
```

### 12.2 테스트 프레임워크

```typescript
import { CITestSuite } from '@anthropic/wia-ci-test';

// 옥타브 인식 테스트
const test = new CITestSuite();

test.describe('Octave Detection', () => {
  test.it('should correctly identify C4', async () => {
    const signal = generateTone(261.63);  // C4
    const result = await ci.wia.detectOctave(signal);

    expect(result.noteName).toBe('C');
    expect(result.octaveNumber).toBe(4);
  });

  test.it('should distinguish C4 from C5', async () => {
    const c4 = await ci.wia.detectOctave(generateTone(261.63));
    const c5 = await ci.wia.detectOctave(generateTone(523.25));

    expect(c4.octaveNumber).toBe(4);
    expect(c5.octaveNumber).toBe(5);
    expect(c4.octaveNumber).not.toBe(c5.octaveNumber);
  });
});

// 테스트 실행
await test.run();
```

---

## 13. 배포 및 인증

### 13.1 앱 스토어 배포

```yaml
# iOS App Store
ios:
  bundleId: com.wia.ci
  category: Medical
  ageRating: 4+
  healthKitIntegration: true
  bluetoothUsage: "CI 프로세서와 통신합니다"

# Google Play Store
android:
  packageName: com.wia.ci
  category: Medical
  contentRating: Everyone
  permissions:
    - BLUETOOTH
    - BLUETOOTH_ADMIN
    - BLUETOOTH_CONNECT
    - FOREGROUND_SERVICE
```

### 13.2 의료기기 인증

| 인증 | 지역 | 상태 |
|------|------|------|
| FDA 510(k) | 미국 | Class II |
| CE Mark | 유럽 | Class IIa |
| KFDA | 한국 | 2등급 |
| PMDA | 일본 | Class II |
| TGA | 호주 | Class IIa |

---

## 14. 로드맵

### 14.1 향후 통합 계획

```
2025 Q1:
├── WIA-INTENT 완전 통합
├── 주요 CI 제조사 3사 지원
└── iOS/Android SDK 출시

2025 Q2:
├── 음악 스트리밍 서비스 통합
├── 스마트홈 플랫폼 연동
└── 임상 워크스테이션 v1

2025 Q3:
├── 웨어러블 연동
├── 클라우드 분석 대시보드
└── 연구자용 API

2025 Q4:
├── AI 기반 개인화
├── 양이 CI 동기화
└── 글로벌 커뮤니티 플랫폼
```

---

**Document ID**: WIA-CI-PHASE4-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License
