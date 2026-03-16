# WIA Accessible UI/UX Standard
## Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Author:** WIA Technical Committee
**Standard ID:** WIA-AAC-014

---

## Table of Contents

1. [Introduction](#introduction)
2. [Accessibility API Protocol](#accessibility-api-protocol)
3. [Screen Reader Communication](#screen-reader-communication)
4. [Keyboard Event Protocol](#keyboard-event-protocol)
5. [Focus Management Protocol](#focus-management-protocol)
6. [Live Region Protocol](#live-region-protocol)
7. [Platform Accessibility APIs](#platform-accessibility-apis)
8. [Cross-Frame Communication](#cross-frame-communication)
9. [Testing Protocol](#testing-protocol)
10. [Security Considerations](#security-considerations)

---

## 1. Introduction

This specification defines the communication protocols for implementing accessible user interfaces. It covers how components communicate accessibility information to assistive technologies, browsers, and other system components.

### 1.1 Protocol Stack

```
┌─────────────────────────────────────┐
│   Assistive Technology (AT)          │
│   (Screen readers, magnifiers, etc.) │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Platform Accessibility API         │
│   (UIAutomation, MSAA, AX API, etc.) │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Browser Accessibility Tree         │
│   (Chrome AX Tree, Firefox a11y)     │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   WIA Accessible UI/UX Layer         │
│   (This specification)                │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   DOM + ARIA                          │
└───────────────────────────────────────┘
```

### 1.2 Philosophy

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

Accessible communication protocols ensure that everyone, regardless of ability, can effectively interact with digital interfaces.

---

## 2. Accessibility API Protocol

### 2.1 Accessibility Tree Construction

The accessibility tree is a parallel structure to the DOM that represents the semantic structure for assistive technologies.

#### 2.1.1 Node Structure

```typescript
interface AccessibilityNode {
  /** Unique node ID */
  id: string;

  /** ARIA role or implicit role */
  role: string;

  /** Accessible name */
  name: string;

  /** Accessible description */
  description?: string;

  /** Node state */
  state: AccessibilityState;

  /** Children nodes */
  children: AccessibilityNode[];

  /** Parent node */
  parent?: AccessibilityNode;

  /** Bounding box */
  bounds: {
    x: number;
    y: number;
    width: number;
    height: number;
  };

  /** Actions available */
  actions: AccessibilityAction[];
}

interface AccessibilityState {
  focusable: boolean;
  focused: boolean;
  disabled: boolean;
  selected?: boolean;
  expanded?: boolean;
  checked?: boolean | "mixed";
  pressed?: boolean | "mixed";
  hidden: boolean;
  readOnly?: boolean;
  required?: boolean;
}

interface AccessibilityAction {
  name: string; // "click", "focus", "expand", etc.
  description?: string;
}
```

#### 2.1.2 Tree Construction Algorithm

```typescript
function buildAccessibilityTree(
  root: HTMLElement
): AccessibilityNode {
  const node: AccessibilityNode = {
    id: generateNodeId(root),
    role: getAriaRole(root) || getImplicitRole(root),
    name: computeAccessibleName(root),
    description: computeAccessibleDescription(root),
    state: computeAccessibilityState(root),
    bounds: root.getBoundingClientRect(),
    actions: computeAvailableActions(root),
    children: []
  };

  // Recursively build children
  const children = getAccessibleChildren(root);
  node.children = children.map(child =>
    buildAccessibilityTree(child)
  );

  return node;
}
```

### 2.2 Accessible Name Computation

Following ARIA 1.2 specification for computing accessible names:

```typescript
function computeAccessibleName(element: HTMLElement): string {
  // 1. aria-labelledby (highest priority)
  const labelledBy = element.getAttribute('aria-labelledby');
  if (labelledBy) {
    return getTextFromIds(labelledBy);
  }

  // 2. aria-label
  const ariaLabel = element.getAttribute('aria-label');
  if (ariaLabel) {
    return ariaLabel.trim();
  }

  // 3. Native label (for form controls)
  if (element instanceof HTMLInputElement) {
    const label = findAssociatedLabel(element);
    if (label) {
      return label.textContent?.trim() || '';
    }
  }

  // 4. alt attribute (for images)
  if (element instanceof HTMLImageElement) {
    return element.alt;
  }

  // 5. title attribute
  const title = element.getAttribute('title');
  if (title) {
    return title.trim();
  }

  // 6. Text content (for certain roles)
  if (shouldUseTextContent(element)) {
    return getTextContent(element);
  }

  return '';
}
```

### 2.3 State Change Notifications

```typescript
interface StateChangeEvent {
  /** Node that changed */
  nodeId: string;

  /** Property that changed */
  property: string;

  /** Old value */
  oldValue: any;

  /** New value */
  newValue: any;

  /** Timestamp */
  timestamp: number;
}

class AccessibilityTreeUpdater {
  /**
   * Notify state change to assistive technologies
   */
  notifyStateChange(event: StateChangeEvent): void {
    // Update accessibility tree
    const node = this.getNode(event.nodeId);
    if (node) {
      (node.state as any)[event.property] = event.newValue;
    }

    // Fire platform accessibility event
    this.firePlatformEvent(event);

    // Update ARIA attributes
    this.updateARIAAttributes(event);
  }

  private firePlatformEvent(event: StateChangeEvent): void {
    // Platform-specific event firing
    if (navigator.platform.includes('Win')) {
      this.fireWindowsUIAutomationEvent(event);
    } else if (navigator.platform.includes('Mac')) {
      this.fireMacAXEvent(event);
    } else if (navigator.platform.includes('Linux')) {
      this.fireLinuxATSPIEvent(event);
    }
  }
}
```

---

## 3. Screen Reader Communication

### 3.1 Live Region Protocol

#### 3.1.1 Live Region Types

```typescript
type LiveRegionPoliteness = "off" | "polite" | "assertive";

interface LiveRegionConfig {
  /** How urgent the announcement is */
  politeness: LiveRegionPoliteness;

  /** Should the entire region be read? */
  atomic: boolean;

  /** What changes should be announced */
  relevant: "additions" | "removals" | "text" | "all";

  /** Is the region currently updating? */
  busy: boolean;

  /** ARIA role for the region */
  role?: "status" | "alert" | "log" | "timer";
}
```

#### 3.1.2 Live Region Implementation

```typescript
class LiveRegionManager {
  private regions: Map<string, HTMLElement> = new Map();

  /**
   * Create a live region
   */
  createLiveRegion(
    id: string,
    config: LiveRegionConfig
  ): HTMLElement {
    const region = document.createElement('div');

    // Set ARIA attributes
    region.setAttribute('aria-live', config.politeness);
    region.setAttribute('aria-atomic', String(config.atomic));
    region.setAttribute('aria-relevant', config.relevant);
    region.setAttribute('aria-busy', String(config.busy));

    if (config.role) {
      region.setAttribute('role', config.role);
    }

    // Make visually hidden but accessible
    region.className = 'wia-sr-only';
    region.id = `wia-live-${id}`;

    document.body.appendChild(region);
    this.regions.set(id, region);

    return region;
  }

  /**
   * Announce message
   */
  announce(
    message: string,
    politeness: LiveRegionPoliteness = "polite"
  ): void {
    // Get or create announcement region
    let region = this.regions.get(`announce-${politeness}`);

    if (!region) {
      region = this.createLiveRegion(`announce-${politeness}`, {
        politeness,
        atomic: true,
        relevant: "additions",
        busy: false,
        role: politeness === "assertive" ? "alert" : "status"
      });
    }

    // Clear and set new message
    // Delay to ensure screen readers pick up the change
    region.textContent = '';
    setTimeout(() => {
      region!.textContent = message;
    }, 100);
  }
}
```

### 3.2 Screen Reader Detection

```typescript
function detectScreenReader(): {
  active: boolean;
  type?: string;
} {
  // Check for common screen reader indicators
  const indicators = {
    // NVDA
    nvda: () => {
      return window.navigator.userAgent.includes('NVDA');
    },

    // JAWS
    jaws: () => {
      return window.navigator.userAgent.includes('JAWS');
    },

    // VoiceOver (Mac/iOS)
    voiceover: () => {
      // Check for VoiceOver DOM events
      return 'ontouchstart' in window &&
             window.navigator.platform.includes('Mac');
    },

    // TalkBack (Android)
    talkback: () => {
      return window.navigator.userAgent.includes('Android') &&
             matchMedia('(prefers-reduced-motion: reduce)').matches;
    },

    // Generic detection
    generic: () => {
      // High contrast mode often indicates screen reader
      return matchMedia('(prefers-contrast: high)').matches ||
             matchMedia('(prefers-reduced-motion: reduce)').matches;
    }
  };

  for (const [type, detector] of Object.entries(indicators)) {
    if (detector()) {
      return { active: true, type };
    }
  }

  return { active: false };
}
```

---

## 4. Keyboard Event Protocol

### 4.1 Keyboard Event Handling

```typescript
interface KeyboardEventProtocol {
  /** Key combination */
  keys: string;

  /** Event type */
  type: "keydown" | "keyup" | "keypress";

  /** Modifier keys */
  modifiers: {
    ctrl: boolean;
    alt: boolean;
    shift: boolean;
    meta: boolean;
  };

  /** Should default be prevented? */
  preventDefault: boolean;

  /** Should propagation stop? */
  stopPropagation: boolean;

  /** Handler function */
  handler: (event: KeyboardEvent) => void;
}

class KeyboardEventManager {
  private shortcuts: Map<string, KeyboardEventProtocol> = new Map();

  /**
   * Register keyboard shortcut
   */
  register(protocol: KeyboardEventProtocol): void {
    const key = this.getShortcutKey(protocol);
    this.shortcuts.set(key, protocol);

    document.addEventListener(protocol.type, (e: Event) => {
      const kbEvent = e as KeyboardEvent;
      if (this.matchesProtocol(kbEvent, protocol)) {
        if (protocol.preventDefault) {
          kbEvent.preventDefault();
        }
        if (protocol.stopPropagation) {
          kbEvent.stopPropagation();
        }
        protocol.handler(kbEvent);
      }
    });
  }

  private getShortcutKey(protocol: KeyboardEventProtocol): string {
    const mods = [];
    if (protocol.modifiers.ctrl) mods.push('ctrl');
    if (protocol.modifiers.alt) mods.push('alt');
    if (protocol.modifiers.shift) mods.push('shift');
    if (protocol.modifiers.meta) mods.push('meta');
    return [...mods, protocol.keys.toLowerCase()].join('+');
  }

  private matchesProtocol(
    event: KeyboardEvent,
    protocol: KeyboardEventProtocol
  ): boolean {
    return (
      event.key.toLowerCase() === protocol.keys.toLowerCase() &&
      event.ctrlKey === protocol.modifiers.ctrl &&
      event.altKey === protocol.modifiers.alt &&
      event.shiftKey === protocol.modifiers.shift &&
      event.metaKey === protocol.modifiers.meta
    );
  }
}
```

### 4.2 Focus Management Protocol

```typescript
interface FocusProtocol {
  /** Element to focus */
  target: HTMLElement;

  /** Should scroll into view? */
  scroll: boolean;

  /** Scroll behavior */
  scrollBehavior: "auto" | "smooth";

  /** Focus options */
  options: {
    preventScroll?: boolean;
    focusVisible?: boolean;
  };
}

class FocusManager {
  private focusHistory: HTMLElement[] = [];
  private currentFocus: HTMLElement | null = null;

  /**
   * Focus element according to protocol
   */
  focus(protocol: FocusProtocol): void {
    const { target, scroll, scrollBehavior, options } = protocol;

    // Store current focus in history
    if (document.activeElement instanceof HTMLElement) {
      this.focusHistory.push(document.activeElement);
      this.currentFocus = document.activeElement;
    }

    // Apply focus
    target.focus(options);

    // Scroll into view if requested
    if (scroll && !options.preventScroll) {
      target.scrollIntoView({
        behavior: scrollBehavior,
        block: "nearest",
        inline: "nearest"
      });
    }

    // Add focus-visible class if needed
    if (options.focusVisible) {
      target.classList.add('focus-visible');
    }

    this.currentFocus = target;
  }

  /**
   * Restore previous focus
   */
  restoreFocus(): void {
    const previous = this.focusHistory.pop();
    if (previous && document.body.contains(previous)) {
      previous.focus();
      this.currentFocus = previous;
    }
  }

  /**
   * Trap focus within container
   */
  trapFocus(container: HTMLElement): FocusTrap {
    const focusable = this.getFocusableElements(container);
    const firstFocusable = focusable[0];
    const lastFocusable = focusable[focusable.length - 1];

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key !== 'Tab') return;

      if (e.shiftKey) {
        // Shift+Tab: wrap to last
        if (document.activeElement === firstFocusable) {
          e.preventDefault();
          lastFocusable.focus();
        }
      } else {
        // Tab: wrap to first
        if (document.activeElement === lastFocusable) {
          e.preventDefault();
          firstFocusable.focus();
        }
      }
    };

    container.addEventListener('keydown', handleKeyDown);

    // Focus first element
    firstFocusable.focus();

    return {
      release: () => {
        container.removeEventListener('keydown', handleKeyDown);
      }
    };
  }

  private getFocusableElements(
    container: HTMLElement
  ): HTMLElement[] {
    const selector = [
      'a[href]',
      'button:not([disabled])',
      'textarea:not([disabled])',
      'input:not([disabled])',
      'select:not([disabled])',
      '[tabindex]:not([tabindex="-1"])'
    ].join(',');

    return Array.from(container.querySelectorAll(selector));
  }
}

interface FocusTrap {
  release: () => void;
}
```

---

## 5. Live Region Protocol

### 5.1 Live Region Update Protocol

```typescript
interface LiveRegionUpdate {
  /** Region ID */
  regionId: string;

  /** Update type */
  type: "text" | "html" | "append" | "clear";

  /** Content to update */
  content: string;

  /** Delay before announcement (ms) */
  delay?: number;

  /** Should clear after announcement? */
  clearAfter?: number;
}

class LiveRegionProtocol {
  private regions: Map<string, HTMLElement> = new Map();
  private updateQueue: LiveRegionUpdate[] = [];
  private processing = false;

  /**
   * Queue live region update
   */
  queueUpdate(update: LiveRegionUpdate): void {
    this.updateQueue.push(update);
    if (!this.processing) {
      this.processQueue();
    }
  }

  /**
   * Process update queue
   */
  private async processQueue(): Promise<void> {
    this.processing = true;

    while (this.updateQueue.length > 0) {
      const update = this.updateQueue.shift()!;
      await this.processUpdate(update);
    }

    this.processing = false;
  }

  /**
   * Process single update
   */
  private async processUpdate(
    update: LiveRegionUpdate
  ): Promise<void> {
    const region = this.regions.get(update.regionId);
    if (!region) return;

    // Set busy state
    region.setAttribute('aria-busy', 'true');

    // Apply delay if specified
    if (update.delay) {
      await new Promise(resolve =>
        setTimeout(resolve, update.delay)
      );
    }

    // Apply update
    switch (update.type) {
      case 'text':
        region.textContent = update.content;
        break;

      case 'html':
        region.innerHTML = update.content;
        break;

      case 'append':
        region.insertAdjacentHTML('beforeend', update.content);
        break;

      case 'clear':
        region.textContent = '';
        break;
    }

    // Remove busy state
    region.setAttribute('aria-busy', 'false');

    // Clear after timeout if specified
    if (update.clearAfter) {
      setTimeout(() => {
        region.textContent = '';
      }, update.clearAfter);
    }
  }
}
```

---

## 6. Platform Accessibility APIs

### 6.1 Windows UI Automation

```typescript
interface UIAutomationProtocol {
  /**
   * Fire UI Automation event
   */
  fireEvent(
    element: HTMLElement,
    eventType: UIAutomationEventType,
    eventArgs?: any
  ): void;

  /**
   * Get automation element
   */
  getAutomationElement(element: HTMLElement): any;

  /**
   * Set automation properties
   */
  setAutomationProperties(
    element: HTMLElement,
    properties: UIAutomationProperties
  ): void;
}

type UIAutomationEventType =
  | "Focus"
  | "PropertyChanged"
  | "StructureChanged"
  | "SelectionChanged"
  | "TextChanged";

interface UIAutomationProperties {
  AutomationId?: string;
  Name?: string;
  ClassName?: string;
  ControlType?: string;
  HelpText?: string;
  IsEnabled?: boolean;
  IsKeyboardFocusable?: boolean;
}
```

### 6.2 macOS Accessibility API

```typescript
interface MacAXProtocol {
  /**
   * Post accessibility notification
   */
  postNotification(
    element: HTMLElement,
    notification: AXNotification
  ): void;

  /**
   * Get AX attributes
   */
  getAXAttributes(element: HTMLElement): AXAttributes;

  /**
   * Set AX attributes
   */
  setAXAttributes(
    element: HTMLElement,
    attributes: Partial<AXAttributes>
  ): void;
}

type AXNotification =
  | "AXFocusedUIElementChanged"
  | "AXValueChanged"
  | "AXSelectedTextChanged"
  | "AXUIElementDestroyed";

interface AXAttributes {
  AXRole: string;
  AXRoleDescription: string;
  AXTitle: string;
  AXDescription: string;
  AXValue: any;
  AXEnabled: boolean;
  AXFocused: boolean;
}
```

### 6.3 Linux AT-SPI Protocol

```typescript
interface ATSPIProtocol {
  /**
   * Emit AT-SPI signal
   */
  emitSignal(
    element: HTMLElement,
    signal: ATSPISignal,
    detail?: any
  ): void;

  /**
   * Get accessible object
   */
  getAccessible(element: HTMLElement): ATSPIAccessible;
}

type ATSPISignal =
  | "focus"
  | "property-change"
  | "state-changed"
  | "bounds-changed"
  | "children-changed";

interface ATSPIAccessible {
  name: string;
  description: string;
  role: number; // AT-SPI role enum
  state: number; // State bitmask
  attributes: Map<string, string>;
}
```

---

## 7. Cross-Frame Communication

### 7.1 iframe Accessibility Protocol

```typescript
interface CrossFrameMessage {
  type: "accessibility-update" | "focus-request" | "announcement";
  source: "parent" | "child";
  data: any;
}

class CrossFrameAccessibility {
  /**
   * Send message to iframe
   */
  sendToFrame(
    frame: HTMLIFrameElement,
    message: CrossFrameMessage
  ): void {
    frame.contentWindow?.postMessage(
      {
        ...message,
        wiaAccessibility: true
      },
      '*'
    );
  }

  /**
   * Send message to parent
   */
  sendToParent(message: CrossFrameMessage): void {
    if (window.parent !== window) {
      window.parent.postMessage(
        {
          ...message,
          wiaAccessibility: true
        },
        '*'
      );
    }
  }

  /**
   * Listen for cross-frame messages
   */
  listen(
    handler: (message: CrossFrameMessage) => void
  ): void {
    window.addEventListener('message', (event) => {
      if (event.data.wiaAccessibility) {
        handler(event.data as CrossFrameMessage);
      }
    });
  }
}
```

---

## 8. Testing Protocol

### 8.1 Accessibility Test Protocol

```typescript
interface AccessibilityTestProtocol {
  /** Test ID */
  id: string;

  /** Test name */
  name: string;

  /** WCAG criterion */
  wcagCriterion: string;

  /** Test function */
  test: (element: HTMLElement) => TestResult;

  /** Auto-fix function */
  fix?: (element: HTMLElement) => void;
}

interface TestResult {
  passed: boolean;
  score: number;
  issues: TestIssue[];
}

interface TestIssue {
  severity: "critical" | "serious" | "moderate" | "minor";
  message: string;
  element: HTMLElement;
  fix?: string;
}
```

---

## 9. Security Considerations

### 9.1 Input Validation

```typescript
/**
 * Sanitize ARIA label input
 */
function sanitizeARIALabel(label: string): string {
  // Remove potentially dangerous content
  return label
    .replace(/<script[^>]*>.*?<\/script>/gi, '')
    .replace(/<[^>]+>/g, '')
    .trim()
    .slice(0, 1000); // Max length
}

/**
 * Validate ARIA role
 */
function validateARIARole(role: string): boolean {
  const validRoles = [
    'alert', 'button', 'checkbox', 'dialog', 'link',
    'navigation', 'radio', 'tab', 'textbox', /* ... */
  ];
  return validRoles.includes(role);
}
```

### 9.2 Rate Limiting

```typescript
/**
 * Rate limit live region announcements
 */
class RateLimiter {
  private lastAnnouncement = 0;
  private minInterval = 500; // ms

  canAnnounce(): boolean {
    const now = Date.now();
    if (now - this.lastAnnouncement < this.minInterval) {
      return false;
    }
    this.lastAnnouncement = now;
    return true;
  }
}
```

---

## Appendix A: Protocol Examples

### A.1 Complete Button Protocol

```typescript
// Initialize button with full protocol
const button = document.getElementById('submit-btn')!;

// 1. Accessibility tree
const node = buildAccessibilityTree(button);

// 2. ARIA attributes
button.setAttribute('role', 'button');
button.setAttribute('aria-label', 'Submit form');

// 3. Keyboard handling
button.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' || e.key === ' ') {
    e.preventDefault();
    button.click();
  }
});

// 4. Focus management
button.addEventListener('focus', () => {
  button.classList.add('focus-visible');
});

// 5. State changes
function setDisabled(disabled: boolean) {
  button.setAttribute('aria-disabled', String(disabled));
  notifyStateChange({
    nodeId: node.id,
    property: 'disabled',
    oldValue: !disabled,
    newValue: disabled,
    timestamp: Date.now()
  });
}

// 6. Announcements
button.addEventListener('click', () => {
  liveRegion.announce('Form submitted successfully');
});
```

---

**弘益人間 (Hongik Ingan)** - Benefit All Humanity

© 2025 WIA - World Certification Industry Association
Licensed under MIT License
