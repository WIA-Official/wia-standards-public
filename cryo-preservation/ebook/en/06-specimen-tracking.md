# Chapter 6: Specimen Tracking and Chain of Custody

**弘益人間 (Benefit All Humanity)**

---

## Overview

This chapter covers comprehensive specimen tracking systems including barcode/RFID technologies, chain of custody documentation, location management, inventory control, and traceability throughout the specimen lifecycle. These systems are critical for regulatory compliance, quality assurance, and patient safety.

---

## Barcode and RFID Tracking Systems

### Barcode Management

```typescript
/**
 * WIA Cryo Preservation - Specimen Tracking System
 * Complete barcode and RFID implementation
 */

import { z } from 'zod';
import crypto from 'crypto';

/**
 * Barcode format standards
 */
export enum BarcodeFormat {
  CODE_128 = 'CODE_128',       // Alphanumeric, variable length
  CODE_39 = 'CODE_39',         // Alphanumeric
  QR_CODE = 'QR_CODE',         // 2D, high capacity
  DATA_MATRIX = 'DATA_MATRIX', // 2D, compact
  ISBT_128 = 'ISBT_128',       // Blood banking standard
  GS1_128 = 'GS1_128'          // Global standard
}

/**
 * Barcode schema
 */
export const BarcodeSchema = z.object({
  barcodeId: z.string().uuid(),
  code: z.string(),
  format: z.nativeEnum(BarcodeFormat),
  specimenId: z.string().uuid(),

  // Encoding
  encodedData: z.object({
    facilityCode: z.string(),
    specimenType: z.string(),
    collectionDate: z.string(),
    sequenceNumber: z.string(),
    checksum: z.string().optional()
  }),

  // Physical properties
  printedOn: z.date(),
  printedBy: z.string(),
  labelSize: z.string().optional(), // e.g., "25mm x 10mm"
  material: z.string().optional(), // e.g., "Cryogenic label"

  // Status
  status: z.enum(['ACTIVE', 'RETIRED', 'DAMAGED', 'LOST']),
  scanCount: z.number().default(0),
  lastScanned: z.date().optional(),

  // Metadata
  createdAt: z.date(),
  updatedAt: z.date()
});

export type Barcode = z.infer<typeof BarcodeSchema>;

/**
 * Barcode generator
 */
export class BarcodeGenerator {
  private facilityCode: string;
  private sequenceCounter: number = 0;

  constructor(facilityCode: string) {
    this.facilityCode = facilityCode;
  }

  /**
   * Generate unique barcode
   */
  generateBarcode(params: {
    specimenType: string;
    collectionDate: Date;
    format?: BarcodeFormat;
  }): Barcode {
    const format = params.format || BarcodeFormat.CODE_128;
    const dateStr = params.collectionDate.toISOString().split('T')[0].replace(/-/g, '');
    const sequence = String(this.sequenceCounter++).padStart(6, '0');

    // Format: FACILITY-TYPE-DATE-SEQUENCE
    const code = `${this.facilityCode}-${params.specimenType}-${dateStr}-${sequence}`;

    // Calculate checksum (simple mod 10)
    const checksum = this.calculateChecksum(code);

    return {
      barcodeId: crypto.randomUUID(),
      code: `${code}-${checksum}`,
      format,
      specimenId: '', // Will be assigned later
      encodedData: {
        facilityCode: this.facilityCode,
        specimenType: params.specimenType,
        collectionDate: dateStr,
        sequenceNumber: sequence,
        checksum
      },
      printedOn: new Date(),
      printedBy: 'SYSTEM',
      status: 'ACTIVE',
      scanCount: 0,
      createdAt: new Date(),
      updatedAt: new Date()
    };
  }

  /**
   * Calculate checksum using Luhn algorithm
   */
  private calculateChecksum(code: string): string {
    const digits = code.replace(/\D/g, '');
    let sum = 0;
    let isEven = false;

    for (let i = digits.length - 1; i >= 0; i--) {
      let digit = parseInt(digits[i]);

      if (isEven) {
        digit *= 2;
        if (digit > 9) {
          digit -= 9;
        }
      }

      sum += digit;
      isEven = !isEven;
    }

    const checksum = (10 - (sum % 10)) % 10;
    return String(checksum);
  }

  /**
   * Validate barcode checksum
   */
  validateBarcode(barcode: string): boolean {
    const parts = barcode.split('-');
    if (parts.length < 5) {
      return false;
    }

    const checksum = parts[parts.length - 1];
    const codeWithoutChecksum = parts.slice(0, -1).join('-');
    const calculatedChecksum = this.calculateChecksum(codeWithoutChecksum);

    return checksum === calculatedChecksum;
  }

  /**
   * Parse barcode
   */
  parseBarcode(barcode: string): {
    facilityCode: string;
    specimenType: string;
    collectionDate: string;
    sequenceNumber: string;
    valid: boolean;
  } | null {
    const parts = barcode.split('-');
    if (parts.length !== 5) {
      return null;
    }

    const valid = this.validateBarcode(barcode);

    return {
      facilityCode: parts[0],
      specimenType: parts[1],
      collectionDate: parts[2],
      sequenceNumber: parts[3],
      valid
    };
  }
}

/**
 * QR Code generator with enhanced data
 */
export class QRCodeGenerator {
  /**
   * Generate QR code data with embedded metadata
   */
  generateQRCodeData(params: {
    specimenId: string;
    barcode: string;
    specimenType: string;
    donorId: string;
    collectionDate: Date;
    storageLocation: string;
    facilityUrl?: string;
  }): string {
    const data = {
      v: '1.0', // Version
      sid: params.specimenId,
      bc: params.barcode,
      type: params.specimenType,
      did: params.donorId,
      date: params.collectionDate.toISOString(),
      loc: params.storageLocation,
      url: params.facilityUrl || 'https://cryo.example.com',
      ts: new Date().toISOString()
    };

    // Return JSON string to be encoded as QR code
    return JSON.stringify(data);
  }

  /**
   * Parse QR code data
   */
  parseQRCodeData(qrData: string): any {
    try {
      return JSON.parse(qrData);
    } catch (error) {
      console.error('Invalid QR code data:', error);
      return null;
    }
  }
}
```

---

### RFID Tracking System

```typescript
/**
 * RFID tag management for automated tracking
 */

export enum RFIDFrequency {
  LF_125KHZ = 'LF_125KHZ',     // Low frequency, short range
  HF_13_56MHZ = 'HF_13_56MHZ', // High frequency, ISO 15693
  UHF_865_868MHZ = 'UHF_865_868MHZ', // Ultra-high frequency, long range
  UHF_902_928MHZ = 'UHF_902_928MHZ'  // UHF (North America)
}

export const RFIDTagSchema = z.object({
  tagId: z.string().uuid(),
  epc: z.string(), // Electronic Product Code
  tid: z.string(), // Tag ID (manufacturer)
  frequency: z.nativeEnum(RFIDFrequency),
  specimenId: z.string().uuid(),

  // Tag properties
  manufacturer: z.string(),
  model: z.string(),
  memorySize: z.number(), // bytes
  cryogenicRated: z.boolean(),
  operatingTempMin: z.number(), // Celsius
  operatingTempMax: z.number(), // Celsius

  // Data storage
  userData: z.record(z.any()).optional(),
  lockStatus: z.enum(['UNLOCKED', 'LOCKED', 'PERMALOCKED']),

  // Status
  status: z.enum(['ACTIVE', 'INACTIVE', 'DAMAGED', 'LOST']),
  attachedTo: z.string(), // Container type
  attachedDate: z.date(),
  lastRead: z.date().optional(),
  readCount: z.number().default(0),

  // Metadata
  createdAt: z.date(),
  updatedAt: z.date()
});

export type RFIDTag = z.infer<typeof RFIDTagSchema>;

/**
 * RFID reader integration
 */
export class RFIDReader {
  private readerName: string;
  private frequency: RFIDFrequency;
  private readRange: number; // meters

  constructor(readerName: string, frequency: RFIDFrequency, readRange: number) {
    this.readerName = readerName;
    this.frequency = frequency;
    this.readRange = readRange;
  }

  /**
   * Simulate RFID tag read
   */
  async readTag(): Promise<{
    epc: string;
    tid: string;
    rssi: number; // Signal strength
    timestamp: Date;
  }> {
    // In production, this would interface with actual RFID hardware
    return new Promise((resolve) => {
      setTimeout(() => {
        resolve({
          epc: this.generateEPC(),
          tid: this.generateTID(),
          rssi: -45 + Math.random() * 20, // -45 to -25 dBm
          timestamp: new Date()
        });
      }, 100);
    });
  }

  /**
   * Write data to RFID tag
   */
  async writeTag(epc: string, userData: Record<string, any>): Promise<boolean> {
    // In production, this would write to actual RFID hardware
    console.log(`Writing to tag ${epc}:`, userData);
    return true;
  }

  /**
   * Bulk read multiple tags
   */
  async bulkRead(durationMs: number = 5000): Promise<Array<{
    epc: string;
    tid: string;
    rssi: number;
    count: number;
  }>> {
    // Simulate reading multiple tags
    const tags = new Map<string, { epc: string; tid: string; rssi: number; count: number }>();

    const startTime = Date.now();
    while (Date.now() - startTime < durationMs) {
      const tag = await this.readTag();

      if (tags.has(tag.epc)) {
        tags.get(tag.epc)!.count++;
      } else {
        tags.set(tag.epc, { ...tag, count: 1 });
      }

      await new Promise(resolve => setTimeout(resolve, 50));
    }

    return Array.from(tags.values());
  }

  /**
   * Generate EPC (Electronic Product Code)
   */
  private generateEPC(): string {
    // SGTIN-96 format: Header + Filter + Partition + Company + Item + Serial
    const header = '30'; // SGTIN-96
    const filter = '3'; // Unit of trade
    const company = '0614141'; // Example company prefix
    const item = '012345'; // Item reference
    const serial = Math.floor(Math.random() * 1000000);

    return `${header}${filter}${company}${item}${serial}`;
  }

  /**
   * Generate TID (Tag ID)
   */
  private generateTID(): string {
    const manufacturer = 'E280'; // NXP example
    const model = '1160';
    const serial = crypto.randomBytes(4).toString('hex').toUpperCase();

    return `${manufacturer}${model}${serial}`;
  }
}

/**
 * RFID inventory management system
 */
export class RFIDInventorySystem {
  private reader: RFIDReader;
  private tagRegistry: Map<string, RFIDTag> = new Map();

  constructor(reader: RFIDReader) {
    this.reader = reader;
  }

  /**
   * Register RFID tag
   */
  registerTag(tag: RFIDTag): void {
    this.tagRegistry.set(tag.epc, tag);
  }

  /**
   * Perform inventory scan
   */
  async performInventoryScan(
    tankId: string,
    expectedCount?: number
  ): Promise<{
    scannedTags: RFIDTag[];
    missingTags: RFIDTag[];
    unexpectedTags: string[];
    totalScanned: number;
    matchRate: number;
  }> {
    console.log(`Performing inventory scan for tank ${tankId}...`);

    // Read all tags in range
    const readTags = await this.reader.bulkRead(5000);

    const scannedTags: RFIDTag[] = [];
    const unexpectedTags: string[] = [];

    readTags.forEach(read => {
      const tag = this.tagRegistry.get(read.epc);

      if (tag) {
        tag.lastRead = new Date();
        tag.readCount++;
        scannedTags.push(tag);
      } else {
        unexpectedTags.push(read.epc);
      }
    });

    // Find missing tags
    const scannedEPCs = new Set(scannedTags.map(t => t.epc));
    const missingTags = Array.from(this.tagRegistry.values()).filter(
      tag => tag.status === 'ACTIVE' && !scannedEPCs.has(tag.epc)
    );

    const matchRate = expectedCount
      ? (scannedTags.length / expectedCount) * 100
      : 0;

    return {
      scannedTags,
      missingTags,
      unexpectedTags,
      totalScanned: readTags.length,
      matchRate
    };
  }

  /**
   * Locate specific specimen
   */
  async locateSpecimen(specimenId: string): Promise<{
    found: boolean;
    tag?: RFIDTag;
    rssi?: number;
    estimatedDistance?: number;
  }> {
    // Find tag associated with specimen
    const tag = Array.from(this.tagRegistry.values()).find(
      t => t.specimenId === specimenId
    );

    if (!tag) {
      return { found: false };
    }

    // Read tag to verify presence
    const reads = await this.reader.bulkRead(2000);
    const read = reads.find(r => r.epc === tag.epc);

    if (read) {
      // Estimate distance based on RSSI
      // Simplified model: distance ≈ 10 ^ ((TxPower - RSSI) / (10 * n))
      // where n is path loss exponent (typically 2-4)
      const txPower = 30; // Typical transmit power in dBm
      const pathLossExponent = 2.5;
      const estimatedDistance =
        Math.pow(10, (txPower - read.rssi) / (10 * pathLossExponent));

      return {
        found: true,
        tag,
        rssi: read.rssi,
        estimatedDistance
      };
    }

    return { found: false, tag };
  }
}
```

---

## Chain of Custody System

### Custody Event Tracking

```typescript
/**
 * Chain of custody event tracking
 */

export enum CustodyEventType {
  COLLECTION = 'COLLECTION',
  RECEIVED = 'RECEIVED',
  PROCESSED = 'PROCESSED',
  TESTED = 'TESTED',
  FROZEN = 'FROZEN',
  STORED = 'STORED',
  MOVED = 'MOVED',
  RETRIEVED = 'RETRIEVED',
  THAWED = 'THAWED',
  USED = 'USED',
  DISPOSED = 'DISPOSED',
  TRANSFERRED = 'TRANSFERRED',
  QUARANTINED = 'QUARANTINED',
  RELEASED = 'RELEASED'
}

export const CustodyEventSchema = z.object({
  eventId: z.string().uuid(),
  specimenId: z.string().uuid(),

  // Event details
  eventType: z.nativeEnum(CustodyEventType),
  timestamp: z.date(),
  location: z.string(),
  facility: z.string(),

  // Personnel
  performedBy: z.object({
    personId: z.string().uuid(),
    name: z.string(),
    role: z.string(),
    signature: z.string().optional() // Digital signature hash
  }),
  witnessedBy: z.object({
    personId: z.string().uuid(),
    name: z.string(),
    role: z.string(),
    signature: z.string().optional()
  }).optional(),
  authorizedBy: z.object({
    personId: z.string().uuid(),
    name: z.string(),
    role: z.string(),
    approvalDate: z.date()
  }).optional(),

  // Event specifics
  fromLocation: z.string().optional(),
  toLocation: z.string().optional(),
  temperature: z.number().optional(), // Celsius
  duration: z.number().optional(), // minutes

  // Documentation
  documentation: z.array(z.object({
    documentId: z.string().uuid(),
    type: z.string(),
    url: z.string().url(),
    uploadedAt: z.date()
  })).optional(),

  // Quality checks
  qualityChecks: z.array(z.object({
    parameter: z.string(),
    value: z.string(),
    passFail: z.enum(['PASS', 'FAIL', 'N/A'])
  })).optional(),

  // Notes and deviations
  notes: z.string().optional(),
  deviations: z.array(z.string()).optional(),

  // Blockchain verification (optional)
  blockchainHash: z.string().optional(),
  blockchainTimestamp: z.date().optional(),

  // Metadata
  createdAt: z.date(),
  verifiedAt: z.date().optional(),
  verifiedBy: z.string().optional()
});

export type CustodyEvent = z.infer<typeof CustodyEventSchema>;

/**
 * Chain of custody manager
 */
export class ChainOfCustodyManager {
  private events: Map<string, CustodyEvent[]> = new Map();

  /**
   * Record custody event
   */
  recordEvent(event: CustodyEvent): void {
    const specimenEvents = this.events.get(event.specimenId) || [];
    specimenEvents.push(event);
    this.events.set(event.specimenId, specimenEvents);

    console.log(`Custody event recorded: ${event.eventType} for specimen ${event.specimenId}`);
  }

  /**
   * Get complete custody chain
   */
  getCustodyChain(specimenId: string): CustodyEvent[] {
    return this.events.get(specimenId) || [];
  }

  /**
   * Verify custody chain integrity
   */
  verifyCustodyChain(specimenId: string): {
    valid: boolean;
    issues: string[];
    gaps: Array<{ from: Date; to: Date; duration: number }>;
  } {
    const events = this.getCustodyChain(specimenId);
    const issues: string[] = [];
    const gaps: Array<{ from: Date; to: Date; duration: number }> = [];

    if (events.length === 0) {
      return { valid: false, issues: ['No custody events found'], gaps: [] };
    }

    // Sort events chronologically
    events.sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime());

    // Check for required events
    const hasCollection = events.some(e => e.eventType === CustodyEventType.COLLECTION);
    const hasStorage = events.some(e => e.eventType === CustodyEventType.STORED);

    if (!hasCollection) {
      issues.push('Missing collection event');
    }

    if (!hasStorage) {
      issues.push('Missing storage event');
    }

    // Check for time gaps (> 24 hours between events)
    for (let i = 1; i < events.length; i++) {
      const prevEvent = events[i - 1];
      const currEvent = events[i];
      const gapMs = currEvent.timestamp.getTime() - prevEvent.timestamp.getTime();
      const gapHours = gapMs / (1000 * 60 * 60);

      if (gapHours > 24) {
        gaps.push({
          from: prevEvent.timestamp,
          to: currEvent.timestamp,
          duration: gapHours
        });
        issues.push(
          `Time gap of ${gapHours.toFixed(1)} hours between ${prevEvent.eventType} and ${currEvent.eventType}`
        );
      }
    }

    // Check for missing witnesses on critical events
    const criticalEvents = events.filter(e =>
      [CustodyEventType.FROZEN, CustodyEventType.TRANSFERRED, CustodyEventType.DISPOSED].includes(e.eventType)
    );

    criticalEvents.forEach(event => {
      if (!event.witnessedBy) {
        issues.push(`Critical event ${event.eventType} missing witness`);
      }
    });

    return {
      valid: issues.length === 0,
      issues,
      gaps
    };
  }

  /**
   * Generate custody report
   */
  generateCustodyReport(specimenId: string): string {
    const events = this.getCustodyChain(specimenId);
    const verification = this.verifyCustodyChain(specimenId);

    let report = `
Chain of Custody Report
=======================
Specimen ID: ${specimenId}
Total Events: ${events.length}
Custody Status: ${verification.valid ? 'VALID' : 'ISSUES FOUND'}

Events:
-------
`;

    events.forEach((event, index) => {
      report += `
${index + 1}. ${event.eventType}
   Timestamp: ${event.timestamp.toISOString()}
   Location: ${event.location}
   Performed By: ${event.performedBy.name} (${event.performedBy.role})
   ${event.witnessedBy ? `Witnessed By: ${event.witnessedBy.name}` : ''}
   ${event.notes ? `Notes: ${event.notes}` : ''}
   ${event.deviations && event.deviations.length > 0 ? `⚠️ Deviations: ${event.deviations.join(', ')}` : ''}
`;
    });

    if (!verification.valid) {
      report += `\n\nIssues Found:\n`;
      verification.issues.forEach((issue, index) => {
        report += `${index + 1}. ${issue}\n`;
      });
    }

    if (verification.gaps.length > 0) {
      report += `\n\nTime Gaps:\n`;
      verification.gaps.forEach((gap, index) => {
        report += `${index + 1}. ${gap.duration.toFixed(1)} hours (${gap.from.toISOString()} to ${gap.to.toISOString()})\n`;
      });
    }

    return report;
  }

  /**
   * Transfer custody
   */
  transferCustody(params: {
    specimenId: string;
    fromPersonId: string;
    toPersonId: string;
    fromLocation: string;
    toLocation: string;
    authorizedBy: string;
    notes?: string;
  }): CustodyEvent {
    const event: CustodyEvent = {
      eventId: crypto.randomUUID(),
      specimenId: params.specimenId,
      eventType: CustodyEventType.TRANSFERRED,
      timestamp: new Date(),
      location: params.toLocation,
      facility: params.toLocation.split('/')[0],
      performedBy: {
        personId: params.toPersonId,
        name: 'Transfer Recipient',
        role: 'TECHNICIAN'
      },
      witnessedBy: {
        personId: params.fromPersonId,
        name: 'Transfer Sender',
        role: 'TECHNICIAN'
      },
      authorizedBy: {
        personId: params.authorizedBy,
        name: 'Authorized Person',
        role: 'ADMINISTRATOR',
        approvalDate: new Date()
      },
      fromLocation: params.fromLocation,
      toLocation: params.toLocation,
      notes: params.notes,
      createdAt: new Date()
    };

    this.recordEvent(event);
    return event;
  }
}
```

---

## Location Management System

### Storage Location Hierarchy

```typescript
/**
 * Hierarchical storage location management
 */

export interface StorageLocation {
  tankId: string;
  canister?: string;
  goblet?: string;
  cane?: string;
  box?: string;
  position: string;
  level?: string;
}

export class LocationManager {
  /**
   * Parse location string
   */
  parseLocation(locationString: string): StorageLocation {
    // Format: "TANK-001/CAN-05/GOB-03/POS-A4"
    const parts = locationString.split('/');

    const location: StorageLocation = {
      tankId: parts[0],
      position: parts[parts.length - 1]
    };

    parts.forEach(part => {
      if (part.startsWith('CAN-')) location.canister = part;
      if (part.startsWith('GOB-')) location.goblet = part;
      if (part.startsWith('CANE-')) location.cane = part;
      if (part.startsWith('BOX-')) location.box = part;
      if (part.startsWith('LEVEL-')) location.level = part;
    });

    return location;
  }

  /**
   * Format location as string
   */
  formatLocation(location: StorageLocation): string {
    const parts = [location.tankId];

    if (location.level) parts.push(location.level);
    if (location.canister) parts.push(location.canister);
    if (location.goblet) parts.push(location.goblet);
    if (location.cane) parts.push(location.cane);
    if (location.box) parts.push(location.box);
    parts.push(location.position);

    return parts.join('/');
  }

  /**
   * Validate location format
   */
  validateLocation(location: StorageLocation): {
    valid: boolean;
    errors: string[];
  } {
    const errors: string[] = [];

    if (!location.tankId || location.tankId.trim().length === 0) {
      errors.push('Tank ID is required');
    }

    if (!location.position || location.position.trim().length === 0) {
      errors.push('Position is required');
    }

    return {
      valid: errors.length === 0,
      errors
    };
  }

  /**
   * Calculate storage capacity
   */
  calculateCapacity(params: {
    tankType: string;
    canistersPerTank: number;
    gobletsPerCanister: number;
    straWsPerGoblet: number;
  }): {
    totalPositions: number;
    capacityBreakdown: {
      canisters: number;
      gobletsPerCanister: number;
      strawsPerGoblet: number;
    };
  } {
    const totalPositions =
      params.canistersPerTank *
      params.gobletsPerCanister *
      params.straWsPerGoblet;

    return {
      totalPositions,
      capacityBreakdown: {
        canisters: params.canistersPerTank,
        gobletsPerCanister: params.gobletsPerCanister,
        strawsPerGoblet: params.straWsPerGoblet
      }
    };
  }

  /**
   * Find nearest available location
   */
  findNearestAvailableLocation(
    currentLocation: StorageLocation,
    occupiedLocations: Set<string>
  ): StorageLocation | null {
    // Simplified example - in production, use actual tank layout
    const candidates: StorageLocation[] = [];

    // Generate nearby locations (same canister, different goblets)
    if (currentLocation.canister) {
      for (let i = 1; i <= 10; i++) {
        const candidate: StorageLocation = {
          ...currentLocation,
          goblet: `GOB-${String(i).padStart(2, '0')}`,
          position: 'A1'
        };

        const locationStr = this.formatLocation(candidate);
        if (!occupiedLocations.has(locationStr)) {
          candidates.push(candidate);
        }
      }
    }

    return candidates[0] || null;
  }
}
```

---

## Inventory Management System

```typescript
/**
 * Complete inventory management
 */

export interface InventorySnapshot {
  snapshotId: string;
  timestamp: Date;
  facility: string;
  tankId: string;

  summary: {
    totalCapacity: number;
    totalOccupied: number;
    totalAvailable: number;
    utilizationRate: number;
  };

  byCategory: Map<string, {
    count: number;
    percentage: number;
  }>;

  byStatus: Map<string, number>;

  specimens: Array<{
    specimenId: string;
    barcode: string;
    category: string;
    location: string;
    status: string;
    storageDuration: number; // days
  }>;

  discrepancies?: Array<{
    type: 'MISSING' | 'UNEXPECTED' | 'MISLOCATION';
    description: string;
    specimenId?: string;
  }>;
}

export class InventoryManager {
  /**
   * Perform inventory count
   */
  async performInventoryCount(
    tankId: string,
    specimens: Array<any>,
    rfidSystem?: RFIDInventorySystem
  ): Promise<InventorySnapshot> {
    const snapshot: InventorySnapshot = {
      snapshotId: crypto.randomUUID(),
      timestamp: new Date(),
      facility: 'Main Facility',
      tankId,
      summary: {
        totalCapacity: 1000,
        totalOccupied: 0,
        totalAvailable: 0,
        utilizationRate: 0
      },
      byCategory: new Map(),
      byStatus: new Map(),
      specimens: [],
      discrepancies: []
    };

    // Use RFID if available
    let scannedSpecimens = specimens;
    if (rfidSystem) {
      const scanResult = await rfidSystem.performInventoryScan(tankId, specimens.length);
      scannedSpecimens = scanResult.scannedTags.map(tag =>
        specimens.find(s => s.specimenId === tag.specimenId)
      ).filter(s => s);

      // Check for discrepancies
      if (scanResult.missingTags.length > 0) {
        scanResult.missingTags.forEach(tag => {
          snapshot.discrepancies?.push({
            type: 'MISSING',
            description: `Specimen ${tag.specimenId} not found during RFID scan`,
            specimenId: tag.specimenId
          });
        });
      }
    }

    // Count specimens
    snapshot.summary.totalOccupied = scannedSpecimens.length;
    snapshot.summary.totalAvailable = snapshot.summary.totalCapacity - snapshot.summary.totalOccupied;
    snapshot.summary.utilizationRate =
      (snapshot.summary.totalOccupied / snapshot.summary.totalCapacity) * 100;

    // Group by category
    scannedSpecimens.forEach(specimen => {
      const category = specimen.category;
      const current = snapshot.byCategory.get(category) || { count: 0, percentage: 0 };
      current.count++;
      snapshot.byCategory.set(category, current);

      // Group by status
      const status = specimen.status;
      snapshot.byStatus.set(status, (snapshot.byStatus.get(status) || 0) + 1);

      // Add to specimen list
      const storageDuration = Math.floor(
        (Date.now() - new Date(specimen.storage.storageDate).getTime()) / (1000 * 60 * 60 * 24)
      );

      snapshot.specimens.push({
        specimenId: specimen.specimenId,
        barcode: specimen.barcode,
        category: specimen.category,
        location: specimen.storage.location.position,
        status: specimen.status,
        storageDuration
      });
    });

    // Calculate percentages
    snapshot.byCategory.forEach((value, key) => {
      value.percentage = (value.count / snapshot.summary.totalOccupied) * 100;
    });

    return snapshot;
  }

  /**
   * Generate inventory report
   */
  generateInventoryReport(snapshot: InventorySnapshot): string {
    let report = `
Inventory Snapshot Report
=========================
Snapshot ID: ${snapshot.snapshotId}
Timestamp: ${snapshot.timestamp.toISOString()}
Facility: ${snapshot.facility}
Tank: ${snapshot.tankId}

Summary:
--------
Total Capacity: ${snapshot.summary.totalCapacity}
Total Occupied: ${snapshot.summary.totalOccupied}
Total Available: ${snapshot.summary.totalAvailable}
Utilization Rate: ${snapshot.summary.utilizationRate.toFixed(2)}%

By Category:
------------
`;

    snapshot.byCategory.forEach((value, category) => {
      report += `${category}: ${value.count} (${value.percentage.toFixed(2)}%)\n`;
    });

    report += `\nBy Status:\n----------\n`;
    snapshot.byStatus.forEach((count, status) => {
      report += `${status}: ${count}\n`;
    });

    if (snapshot.discrepancies && snapshot.discrepancies.length > 0) {
      report += `\n⚠️ Discrepancies Found: ${snapshot.discrepancies.length}\n`;
      snapshot.discrepancies.forEach((disc, index) => {
        report += `${index + 1}. [${disc.type}] ${disc.description}\n`;
      });
    }

    return report;
  }

  /**
   * Compare two inventory snapshots
   */
  compareSnapshots(
    previous: InventorySnapshot,
    current: InventorySnapshot
  ): {
    added: number;
    removed: number;
    moved: number;
    unchanged: number;
    changes: Array<{
      specimenId: string;
      changeType: 'ADDED' | 'REMOVED' | 'MOVED';
      details: string;
    }>;
  } {
    const prevMap = new Map(previous.specimens.map(s => [s.specimenId, s]));
    const currMap = new Map(current.specimens.map(s => [s.specimenId, s]));

    const changes: Array<{
      specimenId: string;
      changeType: 'ADDED' | 'REMOVED' | 'MOVED';
      details: string;
    }> = [];

    let added = 0, removed = 0, moved = 0, unchanged = 0;

    // Check for added and moved specimens
    currMap.forEach((currSpec, id) => {
      const prevSpec = prevMap.get(id);

      if (!prevSpec) {
        added++;
        changes.push({
          specimenId: id,
          changeType: 'ADDED',
          details: `New specimen at ${currSpec.location}`
        });
      } else if (prevSpec.location !== currSpec.location) {
        moved++;
        changes.push({
          specimenId: id,
          changeType: 'MOVED',
          details: `Moved from ${prevSpec.location} to ${currSpec.location}`
        });
      } else {
        unchanged++;
      }
    });

    // Check for removed specimens
    prevMap.forEach((prevSpec, id) => {
      if (!currMap.has(id)) {
        removed++;
        changes.push({
          specimenId: id,
          changeType: 'REMOVED',
          details: `Removed from ${prevSpec.location}`
        });
      }
    });

    return { added, removed, moved, unchanged, changes };
  }
}
```

---

## Summary

This chapter provides comprehensive specimen tracking and chain of custody systems:

- **Barcode Systems**: Code 128, QR codes, data matrix with checksum validation
- **RFID Tracking**: Cryogenic-rated tags, automated inventory, location finding
- **Chain of Custody**: Complete event tracking, verification, digital signatures
- **Location Management**: Hierarchical storage structure, capacity planning
- **Inventory Management**: Real-time counts, discrepancy detection, reporting

Key Features:
- Multiple tracking methods (barcode, RFID, QR code)
- Automated inventory reconciliation
- Complete custody trail with witnesses
- Location validation and optimization
- Discrepancy detection and reporting
- Compliance with regulatory requirements

---

**弘益人間 (Benefit All Humanity)**

*Comprehensive tracking systems ensure specimen safety, regulatory compliance, and patient confidence in cryopreservation services.*
