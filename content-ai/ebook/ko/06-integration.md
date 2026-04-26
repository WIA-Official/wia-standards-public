# 제6장: 플랫폼 통합

## 시스템 통합 가이드

### 개요

본 장에서는 WIA 콘텐츠 AI 표준을 다양한 플랫폼과 시스템에 통합하는 방법을 설명합니다. AI 생성 도구, 콘텐츠 관리 시스템, 소셜 미디어 플랫폼, 브라우저 확장 프로그램 등에 대한 통합 패턴을 다룹니다.

---

## 6.1 통합 아키텍처

### 통합 패턴 개요

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WIA 콘텐츠 AI 통합 생태계                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  AI 콘텐츠 생성                                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │  • DALL-E    • Midjourney    • Stable Diffusion    • GPT         │    │
│  │  • Sora      • ElevenLabs    • Runway             • Claude       │    │
│  └──────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                              │
│                              ▼                                              │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │                    WIA 콘텐츠 AI 미들웨어                           │    │
│  │  • 서명 서비스    • 탐지 서비스    • 검증 서비스    • 워터마크     │    │
│  └──────────────────────────┬─────────────────────────────────────────┘    │
│                              │                                              │
│        ┌─────────────────────┼─────────────────────┐                       │
│        ▼                     ▼                     ▼                       │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐               │
│  │   CMS 플러그인  │     │  소셜 플랫폼   │     │  브라우저 확장  │               │
│  │  • WordPress  │     │  통합          │     │  프로그램       │               │
│  │  • Drupal     │     │  • Facebook   │     │  • Chrome      │               │
│  │  • Ghost      │     │  • Twitter    │     │  • Firefox     │               │
│  └──────────────┘     └──────────────┘     └──────────────┘               │
│                                                                              │
│  사용자 인터페이스                                                            │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │  • 신뢰 배지    • 상세 정보 팝업    • 인증 표시    • 경고 알림     │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 통합 유형

```typescript
// 통합 유형 정의
type IntegrationType =
  | 'generator'      // AI 생성 도구
  | 'cms'            // 콘텐츠 관리 시스템
  | 'platform'       // 소셜/공유 플랫폼
  | 'browser'        // 브라우저 확장
  | 'enterprise'     // 엔터프라이즈 시스템
  | 'api';           // 직접 API 통합

interface IntegrationConfig {
  type: IntegrationType;
  name: string;
  version: string;

  // 기능 설정
  features: {
    signing: boolean;
    detection: boolean;
    verification: boolean;
    watermarking: boolean;
  };

  // API 설정
  api: {
    baseUrl: string;
    apiKey: string;
    timeout: number;
  };

  // UI 설정
  ui: {
    showBadges: boolean;
    showDetails: boolean;
    position: 'inline' | 'overlay' | 'sidebar';
  };

  // 자동화 설정
  automation: {
    autoSign: boolean;
    autoDetect: boolean;
    autoVerify: boolean;
    webhooks: WebhookConfig[];
  };
}

interface WebhookConfig {
  url: string;
  events: string[];
  secret: string;
}
```

---

## 6.2 AI 생성 도구 통합

### 생성 시점 서명

```typescript
// AI 생성기 통합 인터페이스
interface AIGeneratorIntegration {
  generatorName: string;
  generatorVersion: string;

  // 생성 후 콜백
  onContentGenerated(
    content: GeneratedContent,
    context: GenerationContext
  ): Promise<SignedContent>;
}

interface GeneratedContent {
  data: Buffer;
  contentType: 'image' | 'video' | 'audio' | 'text';
  metadata: Record<string, unknown>;
}

interface GenerationContext {
  model: string;
  prompt?: string;
  promptHash?: string;
  parameters: Record<string, unknown>;
  timestamp: Date;
  userId?: string;
}

interface SignedContent extends GeneratedContent {
  credential: ContentCredential;
  watermarked: boolean;
}

// DALL-E 통합 예제
class DALLEIntegration implements AIGeneratorIntegration {
  generatorName = 'DALL-E';
  generatorVersion = '3.0';

  private client: ContentAIClient;
  private openai: any; // OpenAI SDK

  constructor(
    contentAIClient: ContentAIClient,
    openaiClient: any
  ) {
    this.client = contentAIClient;
    this.openai = openaiClient;
  }

  async generateAndSign(
    prompt: string,
    options: GenerationOptions = {}
  ): Promise<SignedContent> {
    // 1. 이미지 생성
    const response = await this.openai.images.generate({
      model: 'dall-e-3',
      prompt: prompt,
      size: options.size || '1024x1024',
      quality: options.quality || 'standard',
      response_format: 'b64_json'
    });

    const imageData = Buffer.from(response.data[0].b64_json, 'base64');

    // 2. 프롬프트 해시 생성 (프라이버시)
    const promptHash = await this.hashPrompt(prompt);

    // 3. 콘텐츠 서명
    const signResult = await this.client.signWithAIAssertion(
      imageData,
      {
        generatorName: this.generatorName,
        modelName: 'dall-e-3',
        generationParams: {
          prompt_hash: promptHash,
          size: options.size || '1024x1024',
          quality: options.quality || 'standard',
          revised_prompt: response.data[0].revised_prompt
        }
      },
      {
        embedCredential: true,
        timestamp: true,
        watermark: {
          enabled: options.watermark !== false,
          algorithm: 'Neural',
          strength: 0.3
        }
      }
    );

    return {
      data: signResult.embeddedContent || imageData,
      contentType: 'image',
      metadata: {
        original_prompt: options.storePrompt ? prompt : undefined,
        revised_prompt: response.data[0].revised_prompt,
        size: options.size || '1024x1024'
      },
      credential: {
        id: signResult.credentialId,
        signature: signResult.signature,
        signedAt: signResult.signedAt
      },
      watermarked: options.watermark !== false
    };
  }

  async onContentGenerated(
    content: GeneratedContent,
    context: GenerationContext
  ): Promise<SignedContent> {
    const signResult = await this.client.signWithAIAssertion(
      content.data,
      {
        generatorName: this.generatorName,
        modelName: context.model,
        generationParams: {
          prompt_hash: context.promptHash,
          ...context.parameters
        }
      }
    );

    return {
      ...content,
      credential: {
        id: signResult.credentialId,
        signature: signResult.signature,
        signedAt: signResult.signedAt
      },
      watermarked: true
    };
  }

  private async hashPrompt(prompt: string): Promise<string> {
    const encoder = new TextEncoder();
    const data = encoder.encode(prompt);
    const hashBuffer = await crypto.subtle.digest('SHA-256', data);
    const hashArray = Array.from(new Uint8Array(hashBuffer));
    return 'sha256:' + hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
  }
}

interface GenerationOptions {
  size?: '1024x1024' | '1792x1024' | '1024x1792';
  quality?: 'standard' | 'hd';
  watermark?: boolean;
  storePrompt?: boolean;
}

interface ContentCredential {
  id: string;
  signature: string;
  signedAt: string;
}
```

### Stable Diffusion WebUI 통합

```python
# Stable Diffusion WebUI 확장
import gradio as gr
import numpy as np
from PIL import Image
import io
import hashlib
from typing import Optional, Dict, Any

class ContentAIExtension:
    """Stable Diffusion WebUI를 위한 콘텐츠 AI 확장"""

    def __init__(self, api_url: str, api_key: str):
        self.api_url = api_url
        self.api_key = api_key
        self.enabled = True
        self.auto_sign = True
        self.watermark_strength = 0.3

    def on_image_generated(
        self,
        image: Image.Image,
        prompt: str,
        negative_prompt: str,
        params: Dict[str, Any]
    ) -> Image.Image:
        """이미지 생성 후 콜백"""
        if not self.enabled:
            return image

        # 이미지를 바이트로 변환
        buffer = io.BytesIO()
        image.save(buffer, format='PNG')
        image_bytes = buffer.getvalue()

        # 프롬프트 해시
        prompt_hash = self._hash_prompt(prompt)
        neg_prompt_hash = self._hash_prompt(negative_prompt) if negative_prompt else None

        # 서명 요청
        if self.auto_sign:
            try:
                signed_image = self._sign_content(
                    image_bytes,
                    {
                        'generator': 'Stable Diffusion',
                        'model': params.get('model', 'unknown'),
                        'prompt_hash': prompt_hash,
                        'negative_prompt_hash': neg_prompt_hash,
                        'seed': params.get('seed'),
                        'steps': params.get('steps'),
                        'cfg_scale': params.get('cfg_scale'),
                        'sampler': params.get('sampler')
                    }
                )

                # 서명된 이미지 반환
                return Image.open(io.BytesIO(signed_image))

            except Exception as e:
                print(f"서명 실패: {e}")
                return image

        return image

    def _sign_content(
        self,
        content: bytes,
        generation_info: Dict
    ) -> bytes:
        """API를 통해 콘텐츠 서명"""
        import requests

        response = requests.post(
            f"{self.api_url}/sign",
            headers={
                'X-API-Key': self.api_key,
                'Content-Type': 'multipart/form-data'
            },
            files={
                'content': ('image.png', content, 'image/png')
            },
            data={
                'assertions': [{
                    'type': 'ai_generation',
                    'data': {
                        'ai_generated': True,
                        'generator_info': {
                            'name': generation_info['generator'],
                            'model': generation_info['model']
                        },
                        'generation_parameters': {
                            k: v for k, v in generation_info.items()
                            if k not in ['generator', 'model']
                        }
                    }
                }],
                'options': {
                    'embed_credential': True,
                    'watermark': {
                        'enabled': True,
                        'strength': self.watermark_strength
                    }
                }
            }
        )

        if response.status_code != 200:
            raise Exception(f"API 오류: {response.text}")

        return response.json().get('embedded_content')

    def _hash_prompt(self, prompt: str) -> str:
        """프롬프트 해시 생성"""
        return 'sha256:' + hashlib.sha256(prompt.encode()).hexdigest()

    # Gradio UI 컴포넌트
    def create_ui(self) -> gr.Blocks:
        """설정 UI 생성"""
        with gr.Blocks() as ui:
            gr.Markdown("## 콘텐츠 AI 인증 설정")

            enabled = gr.Checkbox(
                label="자동 서명 활성화",
                value=self.enabled
            )

            watermark = gr.Slider(
                label="워터마크 강도",
                minimum=0,
                maximum=1,
                value=self.watermark_strength,
                step=0.1
            )

            def update_settings(enabled_val, watermark_val):
                self.enabled = enabled_val
                self.watermark_strength = watermark_val
                return "설정이 업데이트되었습니다."

            gr.Button("설정 저장").click(
                update_settings,
                inputs=[enabled, watermark],
                outputs=[gr.Textbox(label="상태")]
            )

        return ui
```

---

## 6.3 CMS 플러그인

### WordPress 플러그인

```php
<?php
/**
 * Plugin Name: WIA Content AI
 * Description: AI 생성 콘텐츠 인증 및 검증
 * Version: 1.0.0
 * Author: WIA
 */

if (!defined('ABSPATH')) {
    exit;
}

class WIA_Content_AI {
    private $api_url;
    private $api_key;

    public function __construct() {
        $this->api_url = get_option('wia_content_ai_api_url');
        $this->api_key = get_option('wia_content_ai_api_key');

        // 훅 등록
        add_action('admin_menu', [$this, 'add_admin_menu']);
        add_action('add_attachment', [$this, 'on_media_upload']);
        add_filter('wp_get_attachment_image_attributes', [$this, 'add_credential_badge'], 10, 2);
        add_action('wp_ajax_verify_content', [$this, 'ajax_verify_content']);
        add_action('wp_ajax_detect_ai', [$this, 'ajax_detect_ai']);

        // 프론트엔드 스크립트
        add_action('wp_enqueue_scripts', [$this, 'enqueue_scripts']);
    }

    /**
     * 관리자 메뉴 추가
     */
    public function add_admin_menu() {
        add_options_page(
            'WIA Content AI 설정',
            'Content AI',
            'manage_options',
            'wia-content-ai',
            [$this, 'render_settings_page']
        );
    }

    /**
     * 설정 페이지 렌더링
     */
    public function render_settings_page() {
        if (isset($_POST['submit'])) {
            update_option('wia_content_ai_api_url', sanitize_url($_POST['api_url']));
            update_option('wia_content_ai_api_key', sanitize_text_field($_POST['api_key']));
            update_option('wia_content_ai_auto_detect', isset($_POST['auto_detect']));
            update_option('wia_content_ai_show_badges', isset($_POST['show_badges']));
        }

        $api_url = get_option('wia_content_ai_api_url', 'https://api.contentai.wia.org/v1');
        $api_key = get_option('wia_content_ai_api_key', '');
        $auto_detect = get_option('wia_content_ai_auto_detect', false);
        $show_badges = get_option('wia_content_ai_show_badges', true);

        ?>
        <div class="wrap">
            <h1>WIA Content AI 설정</h1>
            <form method="post">
                <table class="form-table">
                    <tr>
                        <th>API URL</th>
                        <td>
                            <input type="url" name="api_url"
                                value="<?php echo esc_attr($api_url); ?>"
                                class="regular-text">
                        </td>
                    </tr>
                    <tr>
                        <th>API 키</th>
                        <td>
                            <input type="password" name="api_key"
                                value="<?php echo esc_attr($api_key); ?>"
                                class="regular-text">
                        </td>
                    </tr>
                    <tr>
                        <th>자동 AI 탐지</th>
                        <td>
                            <input type="checkbox" name="auto_detect"
                                <?php checked($auto_detect); ?>>
                            <label>미디어 업로드 시 자동으로 AI 탐지 실행</label>
                        </td>
                    </tr>
                    <tr>
                        <th>인증 배지 표시</th>
                        <td>
                            <input type="checkbox" name="show_badges"
                                <?php checked($show_badges); ?>>
                            <label>인증된 콘텐츠에 배지 표시</label>
                        </td>
                    </tr>
                </table>
                <?php submit_button(); ?>
            </form>
        </div>
        <?php
    }

    /**
     * 미디어 업로드 시 처리
     */
    public function on_media_upload($attachment_id) {
        $auto_detect = get_option('wia_content_ai_auto_detect', false);

        if (!$auto_detect) {
            return;
        }

        $file_path = get_attached_file($attachment_id);
        $mime_type = get_post_mime_type($attachment_id);

        // 이미지만 처리
        if (strpos($mime_type, 'image/') !== 0) {
            return;
        }

        // AI 탐지 실행
        $result = $this->detect_ai_content($file_path);

        if ($result) {
            // 결과를 메타데이터로 저장
            update_post_meta($attachment_id, '_wia_ai_detection', $result);

            // 자격 증명 확인
            $verification = $this->verify_content($file_path);
            if ($verification) {
                update_post_meta($attachment_id, '_wia_credential', $verification);
            }
        }
    }

    /**
     * AI 콘텐츠 탐지
     */
    private function detect_ai_content($file_path) {
        $response = wp_remote_post($this->api_url . '/detect', [
            'headers' => [
                'X-API-Key' => $this->api_key
            ],
            'body' => [
                'content' => base64_encode(file_get_contents($file_path)),
                'content_type' => 'image'
            ],
            'timeout' => 60
        ]);

        if (is_wp_error($response)) {
            return null;
        }

        return json_decode(wp_remote_retrieve_body($response), true);
    }

    /**
     * 콘텐츠 검증
     */
    private function verify_content($file_path) {
        $response = wp_remote_post($this->api_url . '/verify', [
            'headers' => [
                'X-API-Key' => $this->api_key
            ],
            'body' => [
                'content' => base64_encode(file_get_contents($file_path))
            ],
            'timeout' => 30
        ]);

        if (is_wp_error($response)) {
            return null;
        }

        return json_decode(wp_remote_retrieve_body($response), true);
    }

    /**
     * 이미지에 인증 배지 추가
     */
    public function add_credential_badge($attr, $attachment) {
        $show_badges = get_option('wia_content_ai_show_badges', true);

        if (!$show_badges) {
            return $attr;
        }

        $credential = get_post_meta($attachment->ID, '_wia_credential', true);
        $ai_detection = get_post_meta($attachment->ID, '_wia_ai_detection', true);

        if ($credential && $credential['valid']) {
            $attr['data-wia-verified'] = 'true';
            $attr['data-wia-credential-id'] = $credential['credential_id'];
        }

        if ($ai_detection) {
            $attr['data-wia-ai-generated'] = $ai_detection['is_ai_generated'] ? 'true' : 'false';
            $attr['data-wia-ai-confidence'] = $ai_detection['confidence'];
        }

        return $attr;
    }

    /**
     * 프론트엔드 스크립트 등록
     */
    public function enqueue_scripts() {
        wp_enqueue_script(
            'wia-content-ai',
            plugins_url('assets/js/content-ai.js', __FILE__),
            ['jquery'],
            '1.0.0',
            true
        );

        wp_enqueue_style(
            'wia-content-ai',
            plugins_url('assets/css/content-ai.css', __FILE__),
            [],
            '1.0.0'
        );

        wp_localize_script('wia-content-ai', 'wiaContentAI', [
            'ajaxUrl' => admin_url('admin-ajax.php'),
            'nonce' => wp_create_nonce('wia_content_ai')
        ]);
    }

    /**
     * AJAX: 콘텐츠 검증
     */
    public function ajax_verify_content() {
        check_ajax_referer('wia_content_ai', 'nonce');

        $attachment_id = intval($_POST['attachment_id']);
        $file_path = get_attached_file($attachment_id);

        $result = $this->verify_content($file_path);

        wp_send_json($result);
    }

    /**
     * AJAX: AI 탐지
     */
    public function ajax_detect_ai() {
        check_ajax_referer('wia_content_ai', 'nonce');

        $attachment_id = intval($_POST['attachment_id']);
        $file_path = get_attached_file($attachment_id);

        $result = $this->detect_ai_content($file_path);

        wp_send_json($result);
    }
}

// 플러그인 초기화
new WIA_Content_AI();
```

---

## 6.4 브라우저 확장 프로그램

### Chrome 확장 프로그램

```typescript
// manifest.json
const manifest = {
  manifest_version: 3,
  name: "WIA Content AI Verifier",
  version: "1.0.0",
  description: "웹 콘텐츠의 AI 생성 여부 확인 및 인증 검증",
  permissions: [
    "activeTab",
    "storage",
    "contextMenus"
  ],
  host_permissions: [
    "<all_urls>"
  ],
  background: {
    service_worker: "background.js"
  },
  content_scripts: [{
    matches: ["<all_urls>"],
    js: ["content.js"],
    css: ["styles.css"]
  }],
  action: {
    default_popup: "popup.html",
    default_icon: {
      "16": "icons/icon16.png",
      "48": "icons/icon48.png",
      "128": "icons/icon128.png"
    }
  }
};

// background.js - 서비스 워커
class ContentAIBackground {
  private apiUrl: string = 'https://api.contentai.wia.org/v1';
  private apiKey: string = '';

  constructor() {
    this.initialize();
  }

  private async initialize() {
    // 저장된 설정 로드
    const settings = await chrome.storage.sync.get(['apiKey', 'apiUrl']);
    this.apiKey = settings.apiKey || '';
    this.apiUrl = settings.apiUrl || this.apiUrl;

    // 컨텍스트 메뉴 생성
    chrome.contextMenus.create({
      id: 'verify-image',
      title: 'WIA: 이미지 인증 확인',
      contexts: ['image']
    });

    chrome.contextMenus.create({
      id: 'detect-ai',
      title: 'WIA: AI 생성 여부 확인',
      contexts: ['image']
    });

    // 이벤트 리스너
    chrome.contextMenus.onClicked.addListener(this.handleContextMenu.bind(this));
    chrome.runtime.onMessage.addListener(this.handleMessage.bind(this));
  }

  private async handleContextMenu(
    info: chrome.contextMenus.OnClickData,
    tab?: chrome.tabs.Tab
  ) {
    if (!info.srcUrl || !tab?.id) return;

    switch (info.menuItemId) {
      case 'verify-image':
        await this.verifyImage(info.srcUrl, tab.id);
        break;
      case 'detect-ai':
        await this.detectAI(info.srcUrl, tab.id);
        break;
    }
  }

  private async verifyImage(imageUrl: string, tabId: number) {
    try {
      // 이미지 다운로드
      const imageData = await this.fetchImage(imageUrl);

      // API 호출
      const result = await this.callAPI('/verify', {
        content: imageData,
        content_type: 'image'
      });

      // 결과를 콘텐츠 스크립트로 전송
      chrome.tabs.sendMessage(tabId, {
        type: 'VERIFICATION_RESULT',
        imageUrl,
        result
      });

    } catch (error) {
      console.error('검증 실패:', error);
    }
  }

  private async detectAI(imageUrl: string, tabId: number) {
    try {
      const imageData = await this.fetchImage(imageUrl);

      const result = await this.callAPI('/detect', {
        content: imageData,
        content_type: 'image',
        options: {
          include_details: true
        }
      });

      chrome.tabs.sendMessage(tabId, {
        type: 'DETECTION_RESULT',
        imageUrl,
        result
      });

    } catch (error) {
      console.error('탐지 실패:', error);
    }
  }

  private async fetchImage(url: string): Promise<string> {
    const response = await fetch(url);
    const blob = await response.blob();
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onloadend = () => {
        const base64 = (reader.result as string).split(',')[1];
        resolve(base64);
      };
      reader.onerror = reject;
      reader.readAsDataURL(blob);
    });
  }

  private async callAPI(endpoint: string, data: any): Promise<any> {
    const response = await fetch(`${this.apiUrl}${endpoint}`, {
      method: 'POST',
      headers: {
        'X-API-Key': this.apiKey,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(data)
    });

    if (!response.ok) {
      throw new Error(`API 오류: ${response.status}`);
    }

    return response.json();
  }

  private handleMessage(
    message: any,
    sender: chrome.runtime.MessageSender,
    sendResponse: (response?: any) => void
  ) {
    if (message.type === 'GET_SETTINGS') {
      chrome.storage.sync.get(['apiKey', 'apiUrl', 'autoScan'])
        .then(sendResponse);
      return true;
    }

    if (message.type === 'SAVE_SETTINGS') {
      chrome.storage.sync.set(message.settings)
        .then(() => sendResponse({ success: true }));
      return true;
    }
  }
}

new ContentAIBackground();

// content.js - 콘텐츠 스크립트
class ContentAIContentScript {
  private scanningEnabled: boolean = false;
  private processedImages: Set<string> = new Set();

  constructor() {
    this.initialize();
  }

  private async initialize() {
    // 설정 로드
    const settings = await chrome.runtime.sendMessage({ type: 'GET_SETTINGS' });
    this.scanningEnabled = settings?.autoScan || false;

    // 메시지 리스너
    chrome.runtime.onMessage.addListener(this.handleMessage.bind(this));

    // 자동 스캔
    if (this.scanningEnabled) {
      this.scanPage();
    }

    // DOM 변경 감시
    this.observeDOM();
  }

  private handleMessage(message: any) {
    switch (message.type) {
      case 'VERIFICATION_RESULT':
        this.showVerificationResult(message.imageUrl, message.result);
        break;
      case 'DETECTION_RESULT':
        this.showDetectionResult(message.imageUrl, message.result);
        break;
    }
  }

  private showVerificationResult(imageUrl: string, result: any) {
    const img = document.querySelector(`img[src="${imageUrl}"]`);
    if (!img) return;

    const badge = this.createBadge(result.valid ? 'verified' : 'unverified', {
      title: result.valid ? '인증됨' : '미인증',
      details: result.valid
        ? `서명자: ${result.signer?.name || '알 수 없음'}`
        : '자격 증명을 찾을 수 없습니다'
    });

    this.attachBadge(img, badge);
  }

  private showDetectionResult(imageUrl: string, result: any) {
    const img = document.querySelector(`img[src="${imageUrl}"]`);
    if (!img) return;

    const isAI = result.is_ai_generated;
    const confidence = Math.round(result.confidence * 100);

    const badge = this.createBadge(isAI ? 'ai-generated' : 'human-created', {
      title: isAI ? 'AI 생성' : '인간 제작',
      details: `신뢰도: ${confidence}%`
    });

    this.attachBadge(img, badge);
  }

  private createBadge(
    type: 'verified' | 'unverified' | 'ai-generated' | 'human-created',
    info: { title: string; details: string }
  ): HTMLElement {
    const badge = document.createElement('div');
    badge.className = `wia-badge wia-badge-${type}`;
    badge.innerHTML = `
      <div class="wia-badge-icon"></div>
      <div class="wia-badge-tooltip">
        <div class="wia-badge-title">${info.title}</div>
        <div class="wia-badge-details">${info.details}</div>
      </div>
    `;
    return badge;
  }

  private attachBadge(img: Element, badge: HTMLElement) {
    const wrapper = document.createElement('div');
    wrapper.className = 'wia-image-wrapper';
    wrapper.style.position = 'relative';
    wrapper.style.display = 'inline-block';

    img.parentNode?.insertBefore(wrapper, img);
    wrapper.appendChild(img);
    wrapper.appendChild(badge);
  }

  private scanPage() {
    const images = document.querySelectorAll('img');

    images.forEach(img => {
      const src = img.src;
      if (!src || this.processedImages.has(src)) return;

      // 작은 이미지 스킵
      if (img.naturalWidth < 100 || img.naturalHeight < 100) return;

      this.processedImages.add(src);

      // 자격 증명 확인 (임베딩된 경우)
      chrome.runtime.sendMessage({
        type: 'SCAN_IMAGE',
        imageUrl: src
      });
    });
  }

  private observeDOM() {
    const observer = new MutationObserver((mutations) => {
      if (!this.scanningEnabled) return;

      for (const mutation of mutations) {
        mutation.addedNodes.forEach(node => {
          if (node instanceof HTMLImageElement) {
            this.scanPage();
          }
          if (node instanceof HTMLElement) {
            const images = node.querySelectorAll('img');
            if (images.length > 0) {
              this.scanPage();
            }
          }
        });
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }
}

new ContentAIContentScript();
```

---

## 6.5 통합 모범 사례

### 권장 구현 패턴

```yaml
# 통합 모범 사례
best_practices:

  error_handling:
    - pattern: "지수 백오프 재시도"
      description: "API 실패 시 2, 4, 8초 간격으로 재시도"
      max_retries: 3

    - pattern: "그레이스풀 저하"
      description: "API 불가용 시 기본 기능 유지"
      fallback: "로컬 캐시 또는 기본값 사용"

    - pattern: "회로 차단기"
      description: "연속 실패 시 일시적 API 호출 중단"
      threshold: 5
      reset_timeout: 60

  caching:
    - pattern: "결과 캐싱"
      description: "동일 콘텐츠 해시의 결과 재사용"
      ttl: 3600  # 1시간

    - pattern: "프리페칭"
      description: "예상되는 콘텐츠 미리 검증"

  security:
    - pattern: "API 키 보호"
      description: "환경 변수 또는 보안 저장소 사용"

    - pattern: "HTTPS 필수"
      description: "모든 API 통신 암호화"

    - pattern: "입력 검증"
      description: "사용자 입력 삭제 및 검증"

  performance:
    - pattern: "비동기 처리"
      description: "UI 블로킹 방지"

    - pattern: "배치 처리"
      description: "여러 콘텐츠 일괄 처리"
      max_batch_size: 10

    - pattern: "지연 로딩"
      description: "필요 시에만 검증 수행"
```

---

## 요약

WIA 콘텐츠 AI 통합 가이드는 다음을 제공합니다:

1. **AI 생성기 통합** - 생성 시점 자동 서명
2. **CMS 플러그인** - WordPress 등 주요 CMS 지원
3. **브라우저 확장** - 실시간 웹 콘텐츠 검증
4. **통합 패턴** - 오류 처리, 캐싱, 보안 모범 사례
5. **SDK 및 API** - 개발자 친화적 인터페이스

---

*© 2025 세계산업협회 (WIA). 모든 권리 보유.*
*弘益人間 (홍익인간) · 널리 인간을 이롭게 하라*
