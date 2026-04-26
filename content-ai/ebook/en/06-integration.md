# Chapter 6: Integration

## Platform and Tool Integration Patterns

This chapter provides guidance on integrating WIA Content AI capabilities into existing platforms, content management systems, AI generation tools, and distribution networks.

---

## Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Content AI Integration Points                     │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  CREATION                    DISTRIBUTION                            │
│  ┌─────────────────┐        ┌─────────────────┐                     │
│  │ AI Generation   │───────▶│ CDN/Hosting     │                     │
│  │ Tools           │        │                 │                     │
│  │ ┌─────────────┐ │        │ • C2PA embed    │                     │
│  │ │Auto-sign at │ │        │ • Verify upload │                     │
│  │ │generation   │ │        │ • Display badge │                     │
│  │ └─────────────┘ │        └─────────────────┘                     │
│  └─────────────────┘                │                               │
│           │                         │                               │
│           │                         ▼                               │
│  ┌─────────────────┐        ┌─────────────────┐                     │
│  │ Creative Tools  │        │ Social Media    │                     │
│  │ (Adobe, Canva)  │        │ Platforms       │                     │
│  │                 │        │                 │                     │
│  │ • Track edits   │        │ • Ingest check  │                     │
│  │ • Sign exports  │        │ • Label AI      │                     │
│  │ • Preserve      │        │ • User display  │                     │
│  └─────────────────┘        └─────────────────┘                     │
│           │                         │                               │
│           └────────────┬────────────┘                               │
│                        │                                            │
│                        ▼                                            │
│  ┌─────────────────────────────────────────────┐                    │
│  │              WIA Content AI Hub              │                    │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐    │                    │
│  │  │ Signing  │ │Detection │ │Verify    │    │                    │
│  │  │ Service  │ │ Service  │ │ Service  │    │                    │
│  │  └──────────┘ └──────────┘ └──────────┘    │                    │
│  └─────────────────────────────────────────────┘                    │
│                        │                                            │
│                        ▼                                            │
│  CONSUMPTION                                                        │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐       │
│  │ Browser         │ │ Mobile Apps     │ │ News/Media      │       │
│  │ Extensions      │ │                 │ │ Platforms       │       │
│  │                 │ │ • Scan QR       │ │                 │       │
│  │ • Show status   │ │ • Verify local  │ │ • Fact-check    │       │
│  │ • Quick verify  │ │ • Alert fake    │ │ • Attribution   │       │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## AI Generation Tool Integration

### Integration with LLM Providers

```typescript
// WIA Content AI - AI Tool Integration SDK
// Automatic content credentialing for AI generators

/**
 * AI Generator Integration Wrapper
 * Automatically signs AI-generated content with provenance
 */
class AIGeneratorIntegration {
  private signer: ContentSigner;
  private config: IntegrationConfig;
  private telemetry: TelemetryCollector;

  constructor(config: IntegrationConfig) {
    this.config = config;
    this.signer = new ContentSigner(config.signing);
    this.telemetry = new TelemetryCollector(config.telemetry);
  }

  /**
   * Wrap OpenAI image generation with automatic signing
   */
  async generateSignedImage(
    prompt: string,
    options: ImageGenerationOptions
  ): Promise<SignedImageResult> {
    const startTime = Date.now();

    // Generate image via OpenAI
    const generatedImage = await this.callOpenAI(prompt, options);

    // Create AI generation metadata
    const aiMetadata: AIGenerationMetadata = {
      model: "dall-e-3",
      provider: "OpenAI",
      modelVersion: "3.0",
      generatedAt: new Date().toISOString(),
      parameters: {
        prompt: this.config.redactPrompts ? undefined : prompt,
        promptHash: this.config.redactPrompts
          ? this.hashPrompt(prompt)
          : undefined,
        size: options.size,
        quality: options.quality,
        style: options.style
      }
    };

    // Sign with C2PA manifest
    const signedContent = await this.signer.signAIContent(
      generatedImage.data,
      "image/png",
      aiMetadata
    );

    // Optionally embed watermark
    let finalContent = signedContent.content;
    if (this.config.enableWatermark) {
      finalContent = await this.embedWatermark(
        signedContent.content,
        signedContent.manifestId
      );
    }

    // Log telemetry
    this.telemetry.log({
      operation: "generate_signed_image",
      model: "dall-e-3",
      duration: Date.now() - startTime,
      manifestId: signedContent.manifestId
    });

    return {
      image: finalContent,
      manifestId: signedContent.manifestId,
      metadata: {
        model: aiMetadata.model,
        timestamp: aiMetadata.generatedAt,
        signed: true,
        watermarked: this.config.enableWatermark
      }
    };
  }

  /**
   * Wrap text generation with signing
   */
  async generateSignedText(
    prompt: string,
    options: TextGenerationOptions
  ): Promise<SignedTextResult> {
    // Generate text
    const generatedText = await this.callClaude(prompt, options);

    // Create metadata document
    const metadataDoc = {
      aiGenerated: true,
      model: "claude-3-opus",
      provider: "Anthropic",
      generatedAt: new Date().toISOString(),
      promptHash: this.hashPrompt(prompt),
      parameters: {
        maxTokens: options.maxTokens,
        temperature: options.temperature
      }
    };

    // Sign the metadata
    const signedMetadata = await this.signer.signTextMetadata(
      generatedText,
      metadataDoc
    );

    return {
      text: generatedText,
      metadata: signedMetadata,
      verificationUrl: `https://verify.wia-content.org/${signedMetadata.id}`
    };
  }

  /**
   * Wrap video generation with signing
   */
  async generateSignedVideo(
    prompt: string,
    options: VideoGenerationOptions
  ): Promise<SignedVideoResult> {
    // Generate video via Sora/Runway
    const generatedVideo = await this.callVideoGenerator(prompt, options);

    // Create AI metadata
    const aiMetadata: AIGenerationMetadata = {
      model: options.model || "sora",
      provider: options.provider || "OpenAI",
      modelVersion: "1.0",
      generatedAt: new Date().toISOString(),
      parameters: {
        prompt: this.config.redactPrompts ? undefined : prompt,
        promptHash: this.config.redactPrompts ? this.hashPrompt(prompt) : undefined,
        duration: options.duration,
        resolution: options.resolution,
        aspectRatio: options.aspectRatio
      }
    };

    // Sign video
    const signedContent = await this.signer.signAIContent(
      generatedVideo.data,
      "video/mp4",
      aiMetadata
    );

    return {
      video: signedContent.content,
      manifestId: signedContent.manifestId,
      metadata: {
        model: aiMetadata.model,
        timestamp: aiMetadata.generatedAt,
        duration: options.duration,
        signed: true
      }
    };
  }

  private async callOpenAI(prompt: string, options: ImageGenerationOptions): Promise<{ data: Buffer }> {
    // OpenAI API call
    return { data: Buffer.alloc(0) };
  }

  private async callClaude(prompt: string, options: TextGenerationOptions): Promise<string> {
    // Anthropic API call
    return "";
  }

  private async callVideoGenerator(prompt: string, options: VideoGenerationOptions): Promise<{ data: Buffer }> {
    // Video generator API call
    return { data: Buffer.alloc(0) };
  }

  private hashPrompt(prompt: string): string {
    return crypto.createHash('sha256').update(prompt).digest('hex');
  }

  private async embedWatermark(content: Buffer, manifestId: string): Promise<Buffer> {
    return content;
  }
}

interface IntegrationConfig {
  signing: SigningConfig;
  telemetry: TelemetryConfig;
  redactPrompts: boolean;
  enableWatermark: boolean;
}

interface SigningConfig {
  apiKey: string;
  signerCertificate?: string;
}

interface TelemetryConfig {
  enabled: boolean;
  endpoint?: string;
}

interface ImageGenerationOptions {
  size: string;
  quality: string;
  style: string;
}

interface TextGenerationOptions {
  maxTokens: number;
  temperature: number;
}

interface VideoGenerationOptions {
  model?: string;
  provider?: string;
  duration: number;
  resolution: string;
  aspectRatio: string;
}

interface SignedImageResult {
  image: Buffer;
  manifestId: string;
  metadata: any;
}

interface SignedTextResult {
  text: string;
  metadata: any;
  verificationUrl: string;
}

interface SignedVideoResult {
  video: Buffer;
  manifestId: string;
  metadata: any;
}

interface AIGenerationMetadata {
  model: string;
  provider: string;
  modelVersion: string;
  generatedAt: string;
  parameters: any;
}

class ContentSigner {
  constructor(config: SigningConfig) {}
  async signAIContent(data: Buffer, mimeType: string, metadata: AIGenerationMetadata): Promise<{ content: Buffer; manifestId: string }> {
    return { content: data, manifestId: "" };
  }
  async signTextMetadata(text: string, metadata: any): Promise<{ id: string }> {
    return { id: "" };
  }
}

class TelemetryCollector {
  constructor(config: TelemetryConfig) {}
  log(data: any): void {}
}
```

---

## CMS Integration

### WordPress Plugin Integration

```typescript
/**
 * WordPress Content AI Plugin
 * Automatic verification and labeling for uploaded content
 */
class WordPressContentAIPlugin {
  private detector: AIDetectionService;
  private verifier: VerificationService;
  private signer: SigningService;
  private database: WordPressDB;

  constructor() {
    this.detector = new AIDetectionService(getPluginConfig('detection'));
    this.verifier = new VerificationService(getPluginConfig('verification'));
    this.signer = new SigningService(getPluginConfig('signing'));
    this.database = new WordPressDB();
  }

  /**
   * Hook into media upload process
   */
  async onMediaUpload(attachment: WPAttachment): Promise<void> {
    // Skip if already processed
    if (await this.isProcessed(attachment.id)) return;

    const filePath = attachment.file;
    const mimeType = attachment.mime_type;

    // Check for existing C2PA manifest
    const verificationResult = await this.verifier.verify(filePath, mimeType);

    // Run AI detection if no manifest or uncertain provenance
    let detectionResult = null;
    if (!verificationResult.hasManifest || !verificationResult.aiDisclosed) {
      detectionResult = await this.detector.analyze(filePath, mimeType);
    }

    // Store results as post meta
    await this.storeResults(attachment.id, {
      verification: verificationResult,
      detection: detectionResult
    });

    // Add admin notice if AI content detected
    if (detectionResult?.isAIGenerated && detectionResult.confidence > 0.8) {
      await this.addAdminNotice(attachment.id, 'AI content detected');
    }
  }

  /**
   * Filter content display to show AI labels
   */
  async filterContentDisplay(content: string, postId: number): Promise<string> {
    // Get attached images
    const attachments = await this.getPostAttachments(postId);

    for (const attachment of attachments) {
      const results = await this.getStoredResults(attachment.id);

      if (results?.verification?.aiDisclosed || results?.detection?.isAIGenerated) {
        // Add AI label to image
        content = this.addAILabel(content, attachment, results);
      }
    }

    return content;
  }

  /**
   * Add verification badge to images
   */
  private addAILabel(
    content: string,
    attachment: WPAttachment,
    results: ContentResults
  ): string {
    const badgeHtml = this.generateBadgeHtml(results);

    // Find image in content and wrap with badge
    const imgRegex = new RegExp(
      `<img[^>]+wp-image-${attachment.id}[^>]*>`,
      'g'
    );

    return content.replace(imgRegex, (match) => {
      return `<div class="wia-content-wrapper">${match}${badgeHtml}</div>`;
    });
  }

  private generateBadgeHtml(results: ContentResults): string {
    const isVerified = results.verification?.verified;
    const isAI = results.verification?.aiDisclosed || results.detection?.isAIGenerated;

    if (isVerified && isAI) {
      return `
        <div class="wia-badge wia-badge-ai-verified">
          <span class="wia-icon">🤖✓</span>
          <span class="wia-text">AI Generated (Verified)</span>
        </div>
      `;
    } else if (isAI) {
      return `
        <div class="wia-badge wia-badge-ai-detected">
          <span class="wia-icon">🤖</span>
          <span class="wia-text">AI Detected</span>
        </div>
      `;
    } else if (isVerified) {
      return `
        <div class="wia-badge wia-badge-verified">
          <span class="wia-icon">✓</span>
          <span class="wia-text">Verified</span>
        </div>
      `;
    }

    return '';
  }

  // WordPress integration methods
  private async isProcessed(attachmentId: number): Promise<boolean> {
    return false;
  }

  private async storeResults(attachmentId: number, results: any): Promise<void> {}

  private async getStoredResults(attachmentId: number): Promise<ContentResults | null> {
    return null;
  }

  private async getPostAttachments(postId: number): Promise<WPAttachment[]> {
    return [];
  }

  private async addAdminNotice(attachmentId: number, message: string): Promise<void> {}
}

interface WPAttachment {
  id: number;
  file: string;
  mime_type: string;
}

interface ContentResults {
  verification?: {
    verified: boolean;
    aiDisclosed: boolean;
    hasManifest: boolean;
  };
  detection?: {
    isAIGenerated: boolean;
    confidence: number;
  };
}

function getPluginConfig(key: string): any {
  return {};
}

class WordPressDB {
  constructor() {}
}

class AIDetectionService {
  constructor(config: any) {}
  async analyze(path: string, mimeType: string): Promise<any> { return {}; }
}

class VerificationService {
  constructor(config: any) {}
  async verify(path: string, mimeType: string): Promise<any> { return {}; }
}

class SigningService {
  constructor(config: any) {}
}
```

---

## Social Media Platform Integration

### Platform Ingestion Pipeline

```typescript
/**
 * Social Media Platform Integration
 * Content verification at upload and display
 */
class SocialPlatformIntegration {
  private contentPipeline: ContentIngestionPipeline;
  private displayService: ContentDisplayService;
  private moderationService: ModerationService;

  constructor(config: PlatformConfig) {
    this.contentPipeline = new ContentIngestionPipeline(config);
    this.displayService = new ContentDisplayService(config);
    this.moderationService = new ModerationService(config);
  }

  /**
   * Process uploaded content
   */
  async processUpload(
    content: Buffer,
    mimeType: string,
    uploaderId: string,
    postMetadata: PostMetadata
  ): Promise<ProcessedContent> {
    // Step 1: Extract any existing manifest
    const existingManifest = await this.extractManifest(content, mimeType);

    // Step 2: Run AI detection
    const detectionResult = await this.runDetection(content, mimeType);

    // Step 3: Determine content classification
    const classification = this.classifyContent(
      existingManifest,
      detectionResult
    );

    // Step 4: Apply platform policies
    const policyResult = await this.applyPolicies(
      classification,
      postMetadata
    );

    // Step 5: Store content with metadata
    const storedContent = await this.storeWithMetadata(
      content,
      {
        manifest: existingManifest,
        detection: detectionResult,
        classification,
        policy: policyResult,
        uploaderId,
        postMetadata
      }
    );

    // Step 6: Generate display metadata
    const displayInfo = this.generateDisplayInfo(classification, policyResult);

    return {
      contentId: storedContent.id,
      classification,
      displayInfo,
      moderationRequired: policyResult.requiresReview
    };
  }

  /**
   * Classify content based on provenance and detection
   */
  private classifyContent(
    manifest: ManifestInfo | null,
    detection: DetectionResult
  ): ContentClassification {
    // Verified AI content (disclosed)
    if (manifest?.valid && manifest.aiDisclosed) {
      return {
        type: "AI_VERIFIED",
        confidence: 1.0,
        label: "AI-generated content (verified)",
        icon: "✓🤖",
        source: "manifest"
      };
    }

    // Verified human content
    if (manifest?.valid && !manifest.aiDisclosed && !detection.isAIGenerated) {
      return {
        type: "HUMAN_VERIFIED",
        confidence: 1.0,
        label: "Authentic content (verified)",
        icon: "✓",
        source: "manifest"
      };
    }

    // Detected AI content (undisclosed)
    if (detection.isAIGenerated && detection.confidence > 0.85) {
      return {
        type: "AI_DETECTED",
        confidence: detection.confidence,
        label: "Likely AI-generated",
        icon: "🤖",
        source: "detection"
      };
    }

    // Possible AI content
    if (detection.isAIGenerated && detection.confidence > 0.6) {
      return {
        type: "AI_POSSIBLE",
        confidence: detection.confidence,
        label: "May contain AI content",
        icon: "?",
        source: "detection"
      };
    }

    // Unknown/unverified
    return {
      type: "UNVERIFIED",
      confidence: 0,
      label: "Unverified content",
      icon: "",
      source: "none"
    };
  }

  /**
   * Apply platform policies based on classification
   */
  private async applyPolicies(
    classification: ContentClassification,
    postMetadata: PostMetadata
  ): Promise<PolicyResult> {
    const policies: PolicyCheck[] = [];

    // Check AI disclosure requirement
    if (classification.type === "AI_DETECTED") {
      policies.push({
        policy: "AI_DISCLOSURE_REQUIRED",
        action: "ADD_LABEL",
        reason: "Detected AI content must be labeled"
      });
    }

    // Check for potential deepfake
    if (classification.type === "AI_DETECTED" &&
        postMetadata.contentType === "VIDEO" &&
        postMetadata.containsFaces) {
      policies.push({
        policy: "DEEPFAKE_REVIEW",
        action: "FLAG_FOR_REVIEW",
        reason: "Video with faces detected as AI-generated"
      });
    }

    // Check political content policies
    if (postMetadata.isPolitical &&
        (classification.type === "AI_DETECTED" || classification.type === "AI_POSSIBLE")) {
      policies.push({
        policy: "POLITICAL_AI_CONTENT",
        action: "PROMINENT_LABEL",
        reason: "AI content in political context requires prominent labeling"
      });
    }

    // Determine final actions
    const actions = this.consolidatePolicyActions(policies);

    return {
      policies,
      actions,
      requiresReview: actions.includes("FLAG_FOR_REVIEW"),
      labelRequired: actions.includes("ADD_LABEL") || actions.includes("PROMINENT_LABEL")
    };
  }

  /**
   * Generate display information for frontend
   */
  private generateDisplayInfo(
    classification: ContentClassification,
    policy: PolicyResult
  ): DisplayInfo {
    return {
      showLabel: policy.labelRequired,
      labelStyle: policy.actions.includes("PROMINENT_LABEL") ? "prominent" : "subtle",
      labelText: classification.label,
      icon: classification.icon,
      tooltipText: this.generateTooltip(classification, policy),
      infoUrl: `https://help.platform.com/ai-content#${classification.type}`,
      badge: {
        type: classification.type,
        color: this.getBadgeColor(classification.type)
      }
    };
  }

  private generateTooltip(
    classification: ContentClassification,
    policy: PolicyResult
  ): string {
    switch (classification.type) {
      case "AI_VERIFIED":
        return "This content was created with AI tools. The creator has disclosed this information through verified credentials.";
      case "AI_DETECTED":
        return "Our systems have detected that this content was likely created with AI tools.";
      case "HUMAN_VERIFIED":
        return "This content has verified credentials showing it was created by a human.";
      default:
        return "The origin of this content could not be verified.";
    }
  }

  private getBadgeColor(type: string): string {
    switch (type) {
      case "AI_VERIFIED": return "#6366f1";  // Indigo
      case "AI_DETECTED": return "#f59e0b";  // Amber
      case "HUMAN_VERIFIED": return "#10b981"; // Green
      default: return "#6b7280"; // Gray
    }
  }

  private async extractManifest(content: Buffer, mimeType: string): Promise<ManifestInfo | null> {
    return null;
  }

  private async runDetection(content: Buffer, mimeType: string): Promise<DetectionResult> {
    return { isAIGenerated: false, confidence: 0 };
  }

  private async storeWithMetadata(content: Buffer, metadata: any): Promise<{ id: string }> {
    return { id: "" };
  }

  private consolidatePolicyActions(policies: PolicyCheck[]): string[] {
    return policies.map(p => p.action);
  }
}

interface PlatformConfig {
  detection: any;
  verification: any;
  policies: any;
}

interface PostMetadata {
  contentType: string;
  containsFaces: boolean;
  isPolitical: boolean;
  tags: string[];
}

interface ProcessedContent {
  contentId: string;
  classification: ContentClassification;
  displayInfo: DisplayInfo;
  moderationRequired: boolean;
}

interface ContentClassification {
  type: string;
  confidence: number;
  label: string;
  icon: string;
  source: string;
}

interface ManifestInfo {
  valid: boolean;
  aiDisclosed: boolean;
}

interface DetectionResult {
  isAIGenerated: boolean;
  confidence: number;
}

interface PolicyCheck {
  policy: string;
  action: string;
  reason: string;
}

interface PolicyResult {
  policies: PolicyCheck[];
  actions: string[];
  requiresReview: boolean;
  labelRequired: boolean;
}

interface DisplayInfo {
  showLabel: boolean;
  labelStyle: string;
  labelText: string;
  icon: string;
  tooltipText: string;
  infoUrl: string;
  badge: {
    type: string;
    color: string;
  };
}

class ContentIngestionPipeline {
  constructor(config: PlatformConfig) {}
}

class ContentDisplayService {
  constructor(config: PlatformConfig) {}
}

class ModerationService {
  constructor(config: PlatformConfig) {}
}
```

---

## Browser Extension Integration

```typescript
/**
 * Browser Extension for Content Verification
 * Real-time verification of web content
 */
class ContentVerifierExtension {
  private apiClient: VerificationAPIClient;
  private cache: VerificationCache;
  private uiManager: UIManager;

  constructor() {
    this.apiClient = new VerificationAPIClient();
    this.cache = new VerificationCache();
    this.uiManager = new UIManager();
  }

  /**
   * Initialize extension on page load
   */
  async initialize(): Promise<void> {
    // Find all media elements on page
    const mediaElements = this.findMediaElements();

    // Process each element
    for (const element of mediaElements) {
      await this.processMediaElement(element);
    }

    // Set up mutation observer for dynamic content
    this.observeNewContent();
  }

  /**
   * Process individual media element
   */
  private async processMediaElement(element: MediaElement): Promise<void> {
    const src = element.src;

    // Check cache first
    const cached = await this.cache.get(src);
    if (cached) {
      this.displayVerificationStatus(element, cached);
      return;
    }

    // Fetch and verify
    try {
      const response = await fetch(src);
      const content = await response.arrayBuffer();

      // Quick check for C2PA manifest
      const hasManifest = await this.quickManifestCheck(content);

      let verificationResult: VerificationResult;

      if (hasManifest) {
        // Full verification
        verificationResult = await this.apiClient.verify(content);
      } else {
        // Detection only
        const detection = await this.apiClient.detect(content);
        verificationResult = {
          verified: false,
          aiDetected: detection.isAIGenerated,
          confidence: detection.confidence
        };
      }

      // Cache result
      await this.cache.set(src, verificationResult);

      // Display status
      this.displayVerificationStatus(element, verificationResult);

    } catch (error) {
      console.error('Verification failed:', error);
    }
  }

  /**
   * Display verification badge on element
   */
  private displayVerificationStatus(
    element: MediaElement,
    result: VerificationResult
  ): void {
    const badge = this.createBadge(result);
    this.uiManager.attachBadge(element, badge);
  }

  private createBadge(result: VerificationResult): BadgeConfig {
    if (result.verified && result.aiDisclosed) {
      return {
        type: "verified-ai",
        icon: "✓🤖",
        text: "AI Content (Verified)",
        color: "#6366f1"
      };
    }
    if (result.verified && !result.aiDisclosed) {
      return {
        type: "verified-human",
        icon: "✓",
        text: "Verified Authentic",
        color: "#10b981"
      };
    }
    if (result.aiDetected) {
      return {
        type: "detected-ai",
        icon: "🤖",
        text: `Likely AI (${Math.round(result.confidence * 100)}%)`,
        color: "#f59e0b"
      };
    }
    return {
      type: "unknown",
      icon: "?",
      text: "Unverified",
      color: "#6b7280"
    };
  }

  private findMediaElements(): MediaElement[] {
    return [];
  }

  private async quickManifestCheck(content: ArrayBuffer): Promise<boolean> {
    return false;
  }

  private observeNewContent(): void {}
}

interface MediaElement {
  src: string;
  type: string;
  element: HTMLElement;
}

interface VerificationResult {
  verified: boolean;
  aiDetected?: boolean;
  aiDisclosed?: boolean;
  confidence: number;
}

interface BadgeConfig {
  type: string;
  icon: string;
  text: string;
  color: string;
}

class VerificationAPIClient {
  async verify(content: ArrayBuffer): Promise<VerificationResult> {
    return { verified: false, confidence: 0 };
  }
  async detect(content: ArrayBuffer): Promise<{ isAIGenerated: boolean; confidence: number }> {
    return { isAIGenerated: false, confidence: 0 };
  }
}

class VerificationCache {
  async get(key: string): Promise<VerificationResult | null> {
    return null;
  }
  async set(key: string, value: VerificationResult): Promise<void> {}
}

class UIManager {
  attachBadge(element: MediaElement, badge: BadgeConfig): void {}
}
```

---

## Summary

| Integration Type | Key Components | Complexity |
|-----------------|----------------|------------|
| **AI Generators** | Auto-signing, metadata embedding | Medium |
| **CMS Plugins** | Upload hooks, display filters | Low |
| **Social Platforms** | Ingestion pipeline, policy engine | High |
| **Browser Extensions** | On-page verification, badges | Medium |
| **News/Media** | Editorial workflow, fact-checking | High |

---

**Next Chapter:** [Chapter 7: Security](./07-security.md) - Trust models and threat mitigation.

---

© 2025 World Industry Association (WIA). All rights reserved.
弘益人間 (홍익인간) · Benefit All Humanity
