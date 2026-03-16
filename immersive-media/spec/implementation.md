# WIA-EDU-021: Implementation Guide

**Version:** 1.0.0
**Audience:** Developers, Educators, System Administrators

## Overview

This guide provides step-by-step instructions for implementing the WIA Immersive Media Standard in educational environments.

## Prerequisites

### Hardware Requirements

**Minimum (for development)**
- PC: Intel i5/AMD Ryzen 5, 8GB RAM, GTX 1060/RX 580
- OR Mac: M1, 8GB RAM
- VR Headset: Meta Quest 2 or equivalent
- OR Smartphone: iPhone 12+/Android flagship for AR

**Recommended (for production)**
- PC: Intel i7/AMD Ryzen 7, 16GB+ RAM, RTX 3070/RX 6700 XT
- OR Mac: M1 Pro/Max, 16GB+ RAM
- VR Headsets: Meta Quest 3, PlayStation VR2
- AR Devices: iPhone 14+, iPad Pro, HoloLens 2

### Software Requirements

- Node.js 16+ (for SDK development)
- Modern web browser (Chrome 90+, Firefox 88+, Safari 15+)
- Unity 2021.3 LTS or Unreal Engine 5.1+ (for native development)
- 3D modeling software (Blender, Maya, SketchUp, etc.)
- Video editing software supporting 360° (Premiere Pro, Final Cut Pro)

## Getting Started

### 1. Installation

#### NPM Package

```bash
npm install @wia/immersive-media
```

#### CDN (for web projects)

```html
<script src="https://cdn.wia.edu/immersive-media/v1/wia-im.min.js"></script>
```

### 2. Basic Setup

#### TypeScript/JavaScript

```typescript
import { ImmersiveMedia } from '@wia/immersive-media';

// Initialize with your API key
const im = new ImmersiveMedia({
  apiKey: process.env.WIA_API_KEY,
  region: 'us-west-1',
  quality: 'auto'
});

// Verify initialization
console.log('SDK Version:', im.version);
console.log('Supported Features:', await im.getCapabilities());
```

#### HTML Embedding

```html
<!DOCTYPE html>
<html>
<head>
    <title>Immersive Learning</title>
    <script src="https://cdn.wia.edu/immersive-media/v1/wia-im.min.js"></script>
</head>
<body>
    <div id="immersive-container"></div>

    <script>
        const im = new WIA.ImmersiveMedia({
            container: '#immersive-container',
            apiKey: 'your-api-key'
        });

        im.create360Video({
            source: 'lesson.mp4',
            controls: true
        });
    </script>
</body>
</html>
```

## Implementation Patterns

### Pattern 1: 360° Video Learning Experience

Complete example for virtual field trip:

```typescript
import { ImmersiveMedia } from '@wia/immersive-media';

async function createVirtualFieldTrip() {
  const im = new ImmersiveMedia({ apiKey: API_KEY });

  // Create 360° video
  const video = im.create360Video({
    source: 'ancient-rome-8k.mp4',
    projection: 'equirectangular',
    stereoMode: 'top-bottom',
    controls: true,
    initialView: { yaw: 0, pitch: 0 }
  });

  // Add educational hotspots
  video.addHotspot({
    position: { yaw: 45, pitch: 5 },
    time: 15.5,
    content: 'The Colosseum',
    type: 'quiz',
    data: {
      question: 'When was the Colosseum built?',
      options: ['70-80 AD', '100 AD', '50 BC', '200 AD'],
      correct: 0
    }
  });

  video.addHotspot({
    position: { yaw: -30, pitch: -10 },
    time: 45.0,
    content: 'Roman Forum',
    type: 'info',
    onClick: () => {
      showAdditionalResources('roman-forum');
    }
  });

  // Track engagement
  video.on('hotspot-click', (hotspot) => {
    im.analytics.trackEvent({
      type: 'interaction',
      target: hotspot.content,
      timestamp: Date.now()
    });
  });

  video.on('ended', () => {
    im.analytics.trackProgress({
      lessonId: 'ancient-rome-tour',
      completionPercentage: 100
    });
    showAssessment();
  });

  await video.play();
}
```

### Pattern 2: Interactive 3D Model Exploration

```typescript
async function create3DMoleculeViewer() {
  const im = new ImmersiveMedia({ apiKey: API_KEY });

  // Load 3D model
  const dna = im.create3DModel({
    source: 'dna-double-helix.glb',
    position: [0, 1.5, -2],
    scale: [2, 2, 2],
    animations: ['rotate', 'unwind', 'replicate']
  });

  await dna.load();

  // Add interactive annotations
  dna.addAnnotation({
    position: [0.5, 2, 0],
    label: 'Nucleotide',
    description: 'Basic building block of DNA',
    autoShow: false
  });

  dna.addAnnotation({
    position: [0, 1.5, 0.3],
    label: 'Hydrogen Bond',
    description: 'Connects complementary base pairs',
    autoShow: false
  });

  // Interactive controls
  document.getElementById('rotate-btn').onclick = () => {
    dna.playAnimation('rotate', true);
  };

  document.getElementById('unwind-btn').onclick = () => {
    dna.playAnimation('unwind', false);
  };

  // VR mode support
  if (await im.getCapabilities().vr) {
    document.getElementById('vr-btn').onclick = async () => {
      const session = await im.startSession('immersive-vr');
      session.addObject(dna);
    };
  }
}
```

### Pattern 3: Spatial Audio Science Lab

```typescript
async function createVirtualLab() {
  const im = new ImmersiveMedia({ apiKey: API_KEY });

  // Setup spatial audio environment
  const audio = im.createSpatialAudio({
    environment: 'classroom',
    reverb: 0.2,
    attenuation: 'inverse'
  });

  // Add ambient lab sounds
  audio.addSource({
    id: 'hvac',
    position: [0, 3, 0],
    audio: 'hvac-ambient.mp3',
    volume: 0.2,
    loop: true,
    spatial: true
  });

  // Add equipment sounds at their positions
  audio.addSource({
    id: 'bunsen-burner',
    position: [-2, 1, -1],
    audio: 'bunsen-flame.mp3',
    volume: 0.5,
    loop: true
  });

  audio.addSource({
    id: 'beaker-bubbling',
    position: [1, 1.2, -0.5],
    audio: 'chemical-reaction.mp3',
    volume: 0.6,
    loop: false
  });

  // Narration follows user or comes from specific location
  const narration = audio.addSource({
    id: 'instructor',
    position: [0, 1.7, 2],
    audio: 'lab-instructions.mp3',
    volume: 0.8,
    spatial: true
  });

  // Update listener position as user moves
  im.on('user-move', (position) => {
    audio.setListenerPosition(position);
  });
}
```

### Pattern 4: Multi-Platform Deployment

```typescript
// Detect platform and optimize accordingly
async function initializeImmersiveContent() {
  const im = new ImmersiveMedia({
    apiKey: API_KEY,
    quality: 'auto' // Automatically adjusts to device
  });

  const caps = await im.getCapabilities();

  if (caps.vr) {
    // High-fidelity VR experience
    return await createVRExperience(im);
  } else if (caps.ar) {
    // AR experience for mobile
    return await createARExperience(im);
  } else {
    // Fallback to 360° video or 3D viewer
    return await create360Experience(im);
  }
}

async function createVRExperience(im) {
  const session = await im.startSession('immersive-vr');

  // Load high-quality 3D models
  const model = im.create3DModel({
    source: 'high-poly-model.glb',
    lighting: 'realistic'
  });

  session.addObject(model);
  return session;
}

async function createARExperience(im) {
  const session = await im.startSession('immersive-ar');

  // Load optimized AR models
  const model = im.create3DModel({
    source: 'mobile-optimized.usdz',
    scale: [0.5, 0.5, 0.5]
  });

  session.addObject(model);
  return session;
}

async function create360Experience(im) {
  return im.create360Video({
    source: 'fallback-360.mp4',
    controls: true
  });
}
```

## Integration with Learning Management Systems

### SCORM Integration

```typescript
// SCORM wrapper for immersive content
class ImmersiveScormWrapper {
  constructor(immersiveContent) {
    this.content = immersiveContent;
    this.scorm = window.API_1484_11; // SCORM 2004 API
  }

  initialize() {
    this.scorm.Initialize('');
    this.scorm.SetValue('cmi.completion_status', 'incomplete');
    this.scorm.SetValue('cmi.success_status', 'unknown');
  }

  trackProgress(percentage) {
    this.scorm.SetValue('cmi.progress_measure', percentage / 100);
    if (percentage >= 100) {
      this.scorm.SetValue('cmi.completion_status', 'completed');
    }
    this.scorm.Commit('');
  }

  trackScore(score) {
    this.scorm.SetValue('cmi.score.scaled', score / 100);
    this.scorm.SetValue('cmi.success_status', score >= 70 ? 'passed' : 'failed');
    this.scorm.Commit('');
  }

  terminate() {
    this.scorm.Terminate('');
  }
}

// Usage
const wrapper = new ImmersiveScormWrapper(immersiveContent);
wrapper.initialize();

immersiveContent.on('progress', (pct) => wrapper.trackProgress(pct));
immersiveContent.on('assessment-complete', (score) => wrapper.trackScore(score));
```

### xAPI (Experience API) Integration

```typescript
import { ImmersiveMedia } from '@wia/immersive-media';
import TinCan from 'tincanjs';

class ImmersiveXAPITracker {
  constructor(immersiveMedia, lrsEndpoint, auth) {
    this.im = immersiveMedia;
    this.lrs = new TinCan.LRS({ endpoint: lrsEndpoint, auth });
  }

  sendStatement(verb, object, result = null) {
    const statement = {
      actor: { mbox: 'mailto:student@example.com', name: 'Student' },
      verb: { id: `http://adlnet.gov/expapi/verbs/${verb}` },
      object: {
        id: `https://example.com/activities/${object}`,
        definition: { type: 'http://adlnet.gov/expapi/activities/lesson' }
      }
    };

    if (result) statement.result = result;

    this.lrs.saveStatement(new TinCan.Statement(statement));
  }

  trackInteraction(targetId, value) {
    this.sendStatement('interacted', targetId, {
      response: value,
      completion: true
    });
  }

  trackCompletion(lessonId, score) {
    this.sendStatement('completed', lessonId, {
      score: { scaled: score / 100 },
      completion: true,
      success: score >= 70
    });
  }
}
```

## Deployment Checklist

### Pre-Deployment

- [ ] Content tested on all target devices
- [ ] Performance meets requirements (90+ FPS for VR)
- [ ] Accessibility features implemented and tested
- [ ] Learning analytics configured
- [ ] Privacy policy compliant with GDPR/COPPA
- [ ] Content size optimized (< 500 MB for mobile)
- [ ] Bandwidth requirements documented
- [ ] Teacher training materials prepared

### Deployment

- [ ] CDN configured for global delivery
- [ ] API keys generated and secured
- [ ] LMS integration tested
- [ ] Monitoring and logging enabled
- [ ] Error tracking configured
- [ ] User feedback mechanism in place
- [ ] Documentation published
- [ ] Support channels established

### Post-Deployment

- [ ] Monitor usage analytics
- [ ] Collect user feedback
- [ ] Track learning outcomes
- [ ] Identify performance issues
- [ ] Plan content updates
- [ ] Share best practices

## Troubleshooting

### Common Issues

**Issue: Low frame rate in VR**
- Solution: Reduce polygon count, disable shadows, lower texture resolution
- Check: GPU profiler for bottlenecks

**Issue: 360° video not displaying correctly**
- Solution: Verify projection type matches source, check metadata
- Check: Video codec compatibility

**Issue: Spatial audio not working**
- Solution: Enable microphone permissions, check audio format (ambisonic)
- Check: Browser/headset audio capabilities

**Issue: Models not loading**
- Solution: Verify file paths, check CORS headers, validate glTF format
- Check: Network tab for 404/CORS errors

## Best Practices

1. **Start Simple:** Begin with basic 360° video before complex 3D interactions
2. **Test Early:** Test on target devices throughout development
3. **Optimize Always:** Prioritize performance over visual fidelity
4. **Accessibility First:** Build accessibility in from the start
5. **Measure Impact:** Track learning outcomes, not just engagement
6. **Iterate Based on Data:** Use analytics to guide improvements
7. **Document Everything:** Maintain clear documentation for future updates

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
