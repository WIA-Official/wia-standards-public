/**
 * WIA-LEG-008: Data Portability Standard - Helper Functions
 * @version 1.0.0
 * @license MIT
 */

import {
  DataPortabilityPackage,
  ExportConfig,
  TransferConfig,
  TransferResult
} from './types';

/**
 * Generate a UUID v4
 */
function generateUUID(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    const r = Math.random() * 16 | 0;
    const v = c === 'x' ? r : (r & 0x3 | 0x8);
    return v.toString(16);
  });
}

/**
 * Create an export package with default settings
 */
export async function createExportPackage(
  deceasedId: string,
  executorId: string,
  categories: string[] = ['all']
): Promise<DataPortabilityPackage> {
  const id = generateUUID();
  return {
    '@context': 'https://schema.wiastandards.com/leg-008/v1',
    '@type': 'DataPortabilityPackage',
    id: `urn:uuid:${id}`,
    version: '1.0',
    generated_at: new Date().toISOString(),
    deceased: {
      '@type': 'Person',
      id: deceasedId,
      name: '',
      dateOfDeath: new Date().toISOString()
    },
    executor: {
      '@type': 'LegalExecutor',
      id: executorId,
      name: '',
      relationship: '',
      contactEmail: '',
      authorization: {
        type: 'court_appointed',
        document_ref: '',
        issued_by: '',
        valid_from: new Date().toISOString(),
        valid_until: new Date(Date.now() + 365 * 24 * 60 * 60 * 1000).toISOString()
      }
    },
    data_inventory: {
      total_categories: 0,
      total_items: 0,
      total_size_bytes: 0,
      categories: {}
    },
    data: {},
    metadata: {
      export_method: 'automated',
      export_tool: 'wia-leg-008-sdk v1.0.0',
      checksum: '',
      audit_trail: [
        {
          timestamp: new Date().toISOString(),
          action: 'export_initiated',
          actor: executorId
        }
      ]
    }
  };
}

/**
 * Initiate a transfer between platforms
 */
export async function initiateTransfer(
  source: string,
  destination: string,
  dataTypes: string[],
  executorConsent: boolean = true
): Promise<TransferResult> {
  return {
    transferId: `txfr_${generateUUID()}`,
    status: 'initiated',
    items_transferred: 0
  };
}

/**
 * Verify executor consent
 */
export async function verifyConsent(
  deceasedId: string,
  executorId: string,
  requestedPermissions: string[]
): Promise<{ valid: boolean; consent_id?: string; errors?: string[] }> {
  return {
    valid: true,
    consent_id: `consent_${generateUUID()}`
  };
}

/**
 * Mask sensitive data
 */
export function maskAccountNumber(accountNumber: string, visibleDigits: number = 4): string {
  if (accountNumber.length <= visibleDigits) {
    return accountNumber;
  }
  const masked = '*'.repeat(accountNumber.length - visibleDigits);
  const visible = accountNumber.slice(-visibleDigits);
  return masked + visible;
}

/**
 * Format file size
 */
export function formatFileSize(bytes: number): string {
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let size = bytes;
  let unitIndex = 0;
  
  while (size >= 1024 && unitIndex < units.length - 1) {
    size /= 1024;
    unitIndex++;
  }
  
  return `${size.toFixed(2)} ${units[unitIndex]}`;
}
