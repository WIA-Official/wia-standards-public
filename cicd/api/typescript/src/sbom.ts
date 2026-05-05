/**
 * SBOM helpers — minimum CycloneDX 1.5 + SPDX 2.3 emitters.
 */

export const SBOM_FORMATS = ['CYCLONEDX', 'SPDX', 'SPDX_JSON'] as const;
export const SLSA_LEVELS  = ['L0', 'L1', 'L2', 'L3'] as const;
export type SbomFormat    = typeof SBOM_FORMATS[number];
export type SlsaLevel     = typeof SLSA_LEVELS[number];

export interface SbomComponent {
  type:     'library' | 'application' | 'container' | 'framework' | 'os';
  name:     string;
  version:  string;
  purl?:    string;
  licenses?: string[];
}

export interface SbomMeta {
  serialNumber: string;
  timestamp:    string;
  toolVendor:   string;
  toolName:     string;
  toolVersion:  string;
}

/** Emit a minimum CycloneDX 1.5 JSON document. */
export function emitCycloneDx(meta: SbomMeta, components: SbomComponent[]): object {
  return {
    bomFormat:    'CycloneDX',
    specVersion:  '1.5',
    serialNumber: meta.serialNumber,
    version:      1,
    metadata: {
      timestamp: meta.timestamp,
      tools: [{ vendor: meta.toolVendor, name: meta.toolName, version: meta.toolVersion }]
    },
    components: components.map(c => ({
      type:    c.type,
      name:    c.name,
      version: c.version,
      ...(c.purl ? { purl: c.purl } : {}),
      ...(c.licenses ? { licenses: c.licenses.map(id => ({ license: { id } })) } : {})
    }))
  };
}

/** Emit a minimum SPDX 2.3 tag-value document. */
export function emitSpdxTagValue(meta: SbomMeta, components: SbomComponent[]): string {
  const lines: string[] = [
    'SPDXVersion: SPDX-2.3',
    'DataLicense: CC0-1.0',
    `SPDXID: SPDXRef-DOCUMENT`,
    `DocumentName: ${meta.toolName}-sbom`,
    `Created: ${meta.timestamp}`,
    `Creator: Tool: ${meta.toolVendor} ${meta.toolName} ${meta.toolVersion}`
  ];
  components.forEach((c, i) => {
    lines.push('');
    lines.push(`PackageName: ${c.name}`);
    lines.push(`SPDXID: SPDXRef-Package-${i}`);
    lines.push(`PackageVersion: ${c.version}`);
    lines.push(`PackageDownloadLocation: NOASSERTION`);
    lines.push(`PackageLicenseConcluded: ${(c.licenses ?? ['NOASSERTION'])[0]}`);
  });
  return lines.join('\n');
}

/** Determine if a (slsaLevel, signed) pair satisfies WIA-CICD's L3 target. */
export function meetsSlsaTarget(level: SlsaLevel, signed: boolean): boolean {
  return level === 'L3' && signed;
}
