/**
 * WIA-BIO-021: Synthetic Biology Registry SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Biotechnology Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides tools for synthetic biology part registration, search,
 * and characterization, with support for multiple sequence formats and
 * integration with existing registries.
 */

import {
  BiologicalPart,
  PartRegistrationRequest,
  PartRegistrationResponse,
  PartSearchQuery,
  PartSearchResult,
  PartSearchResultItem,
  PartUpdateRequest,
  CharacterizationUpdateRequest,
  CharacterizationData,
  FormatConversionRequest,
  FormatConversionResponse,
  SequenceData,
  SequenceFormat,
  PartType,
  PartStatus,
  SafetyLevel,
  BioErrorCode,
  BioRegistryError,
  VersionHistory,
  CompositePart,
  ReusabilityMetrics,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-021 Synthetic Biology Registry SDK
 */
export class SyntheticBioRegistrySDK {
  private version = '1.0.0';
  private initialized = false;
  private parts: Map<string, BiologicalPart> = new Map();

  constructor() {
    this.initialized = true;
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Register a new biological part
   *
   * @param request - Part registration request
   * @returns Registration response with assigned part ID
   */
  async registerPart(
    request: PartRegistrationRequest
  ): Promise<PartRegistrationResponse> {
    try {
      // Validate part ID format
      let partId = request.partId;
      if (!partId) {
        partId = this.generatePartId(request.type);
      } else {
        this.validatePartId(partId);
      }

      // Check if part already exists
      if (this.parts.has(partId)) {
        throw new BioRegistryError(
          BioErrorCode.PART_EXISTS,
          `Part ${partId} already exists in registry`
        );
      }

      // Validate sequence
      const sequenceData = this.parseSequence(request.sequence);
      this.validateSequence(sequenceData);

      // Create part object
      const part: BiologicalPart = {
        partId,
        partName: request.partName,
        type: request.type,
        subtype: request.subtype,
        status: 'available',
        safetyLevel: request.safetyLevel || 'BSL-1',
        sequence: sequenceData,
        organism: request.organism,
        biologicalContext: request.biologicalContext,
        author: request.author,
        version: '1.0.0',
        license: request.license,
        created: new Date().toISOString(),
        modified: new Date().toISOString(),
        description: request.description,
      };

      // Store part
      this.parts.set(partId, part);

      return {
        partId,
        version: '1.0.0',
        url: `https://registry.wiastandards.com/parts/${partId}`,
        created: new Date().toISOString(),
        success: true,
      };
    } catch (error) {
      if (error instanceof BioRegistryError) {
        throw error;
      }
      throw new BioRegistryError(
        BioErrorCode.INVALID_SEQUENCE,
        `Registration failed: ${error}`
      );
    }
  }

  /**
   * Get part details by ID
   *
   * @param partId - Part identifier
   * @param version - Specific version (optional)
   * @returns Part details
   */
  async getPartDetails(partId: string, version?: string): Promise<BiologicalPart> {
    const part = this.parts.get(partId);

    if (!part) {
      throw new BioRegistryError(
        BioErrorCode.PART_NOT_FOUND,
        `Part ${partId} not found in registry`
      );
    }

    // If version specified, filter by version
    if (version && part.version !== version) {
      throw new BioRegistryError(
        BioErrorCode.PART_NOT_FOUND,
        `Part ${partId} version ${version} not found`
      );
    }

    return part;
  }

  /**
   * Search for parts in the registry
   *
   * @param query - Search query parameters
   * @returns Search results
   */
  async searchParts(query: PartSearchQuery): Promise<PartSearchResult> {
    const allParts = Array.from(this.parts.values());
    let filtered = allParts;

    // Apply filters
    if (query.type) {
      filtered = filtered.filter((p) => p.type === query.type);
    }

    if (query.organism) {
      filtered = filtered.filter(
        (p) => p.organism?.toLowerCase().includes(query.organism!.toLowerCase())
      );
    }

    if (query.author) {
      filtered = filtered.filter(
        (p) => p.author.name.toLowerCase().includes(query.author!.toLowerCase())
      );
    }

    if (query.safetyLevel) {
      filtered = filtered.filter((p) => p.safetyLevel === query.safetyLevel);
    }

    if (query.status) {
      filtered = filtered.filter((p) => p.status === query.status);
    }

    if (query.keywords) {
      const keywords = query.keywords.toLowerCase();
      filtered = filtered.filter(
        (p) =>
          p.partName.toLowerCase().includes(keywords) ||
          p.description?.toLowerCase().includes(keywords)
      );
    }

    if (query.partIdPrefix) {
      filtered = filtered.filter((p) => p.partId.startsWith(query.partIdPrefix!));
    }

    // Sort results
    const sortBy = query.sortBy || 'relevance';
    const sortOrder = query.sortOrder || 'desc';

    filtered.sort((a, b) => {
      let comparison = 0;

      switch (sortBy) {
        case 'date':
          comparison =
            new Date(a.created).getTime() - new Date(b.created).getTime();
          break;
        case 'name':
          comparison = a.partName.localeCompare(b.partName);
          break;
        case 'popularity':
          comparison =
            (a.reusability?.timesUsed || 0) - (b.reusability?.timesUsed || 0);
          break;
        default:
          comparison = 0;
      }

      return sortOrder === 'asc' ? comparison : -comparison;
    });

    // Pagination
    const page = query.page || 1;
    const pageSize = query.pageSize || 20;
    const startIdx = (page - 1) * pageSize;
    const endIdx = startIdx + pageSize;
    const paginatedResults = filtered.slice(startIdx, endIdx);

    // Create result items
    const results: PartSearchResultItem[] = paginatedResults.map((part) => ({
      partId: part.partId,
      name: part.partName,
      type: part.type,
      description: part.description,
      author: part.author.name,
      version: part.version,
      relevance: 1.0,
      snippet: part.description?.substring(0, 150),
    }));

    return {
      results,
      total: filtered.length,
      page,
      pageSize,
      totalPages: Math.ceil(filtered.length / pageSize),
    };
  }

  /**
   * Update part characterization data
   *
   * @param request - Characterization update request
   * @returns Updated part with new version
   */
  async updatePartCharacterization(
    request: CharacterizationUpdateRequest
  ): Promise<BiologicalPart> {
    const part = this.parts.get(request.partId);

    if (!part) {
      throw new BioRegistryError(
        BioErrorCode.PART_NOT_FOUND,
        `Part ${request.partId} not found`
      );
    }

    // Update characterization data
    const updatedCharacterization: CharacterizationData = {
      measurements: [
        ...(part.characterization?.measurements || []),
        ...(request.measurements || []),
      ],
      growthCurve: request.growthCurve || part.characterization?.growthCurve,
      expression: request.expression || part.characterization?.expression,
      activity: request.activity || part.characterization?.activity,
    };

    // Increment version (minor)
    const newVersion = this.incrementVersion(part.version, 'minor');

    // Update part
    const updatedPart: BiologicalPart = {
      ...part,
      characterization: updatedCharacterization,
      version: newVersion,
      modified: new Date().toISOString(),
    };

    this.parts.set(request.partId, updatedPart);

    return updatedPart;
  }

  /**
   * Update part details
   *
   * @param request - Part update request
   * @returns Updated part
   */
  async updatePart(request: PartUpdateRequest): Promise<BiologicalPart> {
    const part = this.parts.get(request.partId);

    if (!part) {
      throw new BioRegistryError(
        BioErrorCode.PART_NOT_FOUND,
        `Part ${request.partId} not found`
      );
    }

    // Increment version
    const newVersion = this.incrementVersion(part.version, request.versionIncrement);

    // Update part
    const updatedPart: BiologicalPart = {
      ...part,
      ...request.updates,
      version: newVersion,
      modified: new Date().toISOString(),
    };

    this.parts.set(request.partId, updatedPart);

    return updatedPart;
  }

  /**
   * Convert sequence format
   *
   * @param request - Format conversion request
   * @returns Converted sequence
   */
  async convertFormat(
    request: FormatConversionRequest
  ): Promise<FormatConversionResponse> {
    try {
      let output: string;

      // Parse input
      const parsed = this.parseSequence(request.input, request.inputFormat);

      // Convert to output format
      switch (request.outputFormat) {
        case 'raw':
          output = parsed.nucleotides;
          break;

        case 'fasta':
          output = this.toFasta(parsed, request.metadata);
          break;

        case 'genbank':
          output = this.toGenBank(parsed, request.metadata);
          break;

        case 'sbol':
          output = this.toSBOL(parsed, request.metadata);
          break;

        default:
          throw new Error(`Unsupported output format: ${request.outputFormat}`);
      }

      return {
        output,
        format: request.outputFormat,
        success: true,
      };
    } catch (error) {
      return {
        output: '',
        format: request.outputFormat,
        success: false,
        error: `Format conversion failed: ${error}`,
      };
    }
  }

  /**
   * Calculate sequence statistics
   *
   * @param sequence - DNA/RNA sequence
   * @returns Sequence statistics
   */
  calculateSequenceStats(sequence: string): {
    length: number;
    gcContent: number;
    atContent: number;
    molecularWeight: number;
  } {
    const seq = sequence.toUpperCase();
    const length = seq.length;

    let gc = 0;
    let at = 0;

    for (const base of seq) {
      if (base === 'G' || base === 'C') gc++;
      if (base === 'A' || base === 'T' || base === 'U') at++;
    }

    const gcContent = length > 0 ? gc / length : 0;
    const atContent = length > 0 ? at / length : 0;

    // Approximate molecular weight (Da)
    const molecularWeight = length * 330; // Average MW per nucleotide

    return {
      length,
      gcContent,
      atContent,
      molecularWeight,
    };
  }

  /**
   * Validate part ID format
   *
   * @param partId - Part identifier to validate
   */
  validatePartId(partId: string): void {
    const partIdRegex = /^BBa_[PRBCETKV][0-9]{5,6}$/;

    if (!partIdRegex.test(partId)) {
      throw new BioRegistryError(
        BioErrorCode.INVALID_PART_ID,
        `Invalid part ID format: ${partId}. Expected format: BBa_[TYPE][NUMBER]`
      );
    }
  }

  /**
   * Generate a new part ID
   *
   * @param type - Part type
   * @returns Generated part ID
   */
  private generatePartId(type: PartType): string {
    const prefix = this.getPartIdPrefix(type);
    const number = Math.floor(Math.random() * 900000) + 100000;
    return `BBa_${prefix}${number}`;
  }

  /**
   * Get part ID prefix based on type
   */
  private getPartIdPrefix(type: PartType): string {
    const prefixMap: Record<PartType, string> = {
      promoter: 'P',
      rbs: 'R',
      cds: 'C',
      terminator: 'T',
      plasmid: 'V',
      composite: 'K',
      regulatory: 'R',
      primer: 'P',
      scar: 'S',
      tag: 'T',
      other: 'X',
    };

    return prefixMap[type] || 'X';
  }

  /**
   * Parse sequence from string
   */
  private parseSequence(
    sequence: string,
    format: SequenceFormat = 'raw'
  ): SequenceData {
    let nucleotides = '';

    if (format === 'raw') {
      nucleotides = sequence.replace(/\s/g, '').toUpperCase();
    } else if (format === 'fasta') {
      const lines = sequence.split('\n');
      nucleotides = lines
        .filter((line) => !line.startsWith('>'))
        .join('')
        .replace(/\s/g, '')
        .toUpperCase();
    } else {
      // For GenBank and SBOL, extract sequence
      nucleotides = sequence.replace(/\s/g, '').toUpperCase();
    }

    return {
      nucleotides,
      length: nucleotides.length,
      format,
      sequenceType: 'dna',
      checksum: this.calculateChecksum(nucleotides),
    };
  }

  /**
   * Validate sequence
   */
  private validateSequence(sequence: SequenceData): void {
    const dnaRegex = /^[ATCGNatcgn]+$/;
    const rnaRegex = /^[AUCGNaucgn]+$/;

    if (sequence.sequenceType === 'dna' && !dnaRegex.test(sequence.nucleotides)) {
      throw new BioRegistryError(
        BioErrorCode.INVALID_SEQUENCE,
        'Invalid DNA sequence: contains invalid characters'
      );
    }

    if (sequence.sequenceType === 'rna' && !rnaRegex.test(sequence.nucleotides)) {
      throw new BioRegistryError(
        BioErrorCode.INVALID_SEQUENCE,
        'Invalid RNA sequence: contains invalid characters'
      );
    }

    if (sequence.length < 1 || sequence.length > 1000000) {
      throw new BioRegistryError(
        BioErrorCode.INVALID_SEQUENCE,
        'Sequence length out of bounds (1-1,000,000 bp)'
      );
    }
  }

  /**
   * Calculate sequence checksum
   */
  private calculateChecksum(sequence: string): string {
    // Simple hash for demonstration (use crypto in production)
    let hash = 0;
    for (let i = 0; i < sequence.length; i++) {
      hash = (hash << 5) - hash + sequence.charCodeAt(i);
      hash |= 0;
    }
    return `sha256:${Math.abs(hash).toString(16)}`;
  }

  /**
   * Increment version number
   */
  private incrementVersion(
    currentVersion: string,
    incrementType: 'major' | 'minor' | 'patch'
  ): string {
    const [major, minor, patch] = currentVersion.split('.').map(Number);

    switch (incrementType) {
      case 'major':
        return `${major + 1}.0.0`;
      case 'minor':
        return `${major}.${minor + 1}.0`;
      case 'patch':
        return `${major}.${minor}.${patch + 1}`;
      default:
        return currentVersion;
    }
  }

  /**
   * Convert to FASTA format
   */
  private toFasta(sequence: SequenceData, metadata?: Partial<BiologicalPart>): string {
    const header = metadata?.partId || 'Unknown';
    const description = metadata?.partName || '';
    let fasta = `>${header} ${description}\n`;

    // Split sequence into 60-character lines
    const seq = sequence.nucleotides;
    for (let i = 0; i < seq.length; i += 60) {
      fasta += seq.substring(i, i + 60) + '\n';
    }

    return fasta;
  }

  /**
   * Convert to GenBank format (simplified)
   */
  private toGenBank(
    sequence: SequenceData,
    metadata?: Partial<BiologicalPart>
  ): string {
    const partId = metadata?.partId || 'Unknown';
    const name = metadata?.partName || 'Unknown';
    const length = sequence.length;

    let genbank = `LOCUS       ${partId.padEnd(16)} ${length} bp    DNA     linear   SYN ${new Date().toISOString().split('T')[0]}\n`;
    genbank += `DEFINITION  ${name}\n`;
    genbank += `ACCESSION   ${partId}\n`;
    genbank += `VERSION     ${partId}.${metadata?.version || '1'}\n`;
    genbank += `KEYWORDS    .\n`;
    genbank += `SOURCE      synthetic construct\n`;
    genbank += `  ORGANISM  synthetic construct\n`;
    genbank += `ORIGIN\n`;

    // Format sequence with line numbers
    const seq = sequence.nucleotides.toLowerCase();
    for (let i = 0; i < seq.length; i += 60) {
      const lineNum = (i + 1).toString().padStart(9);
      const chunk = seq.substring(i, i + 60);
      const formatted = chunk.match(/.{1,10}/g)?.join(' ') || chunk;
      genbank += `${lineNum} ${formatted}\n`;
    }

    genbank += '//\n';
    return genbank;
  }

  /**
   * Convert to SBOL format (simplified XML)
   */
  private toSBOL(sequence: SequenceData, metadata?: Partial<BiologicalPart>): string {
    const partId = metadata?.partId || 'Unknown';
    const name = metadata?.partName || 'Unknown';
    const version = metadata?.version || '1.0.0';

    return `<?xml version="1.0" encoding="UTF-8"?>
<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
         xmlns:sbol="http://sbols.org/v2#"
         xmlns:dcterms="http://purl.org/dc/terms/">
  <sbol:ComponentDefinition rdf:about="http://wiastandards.com/${partId}">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/${partId}"/>
    <sbol:displayId>${partId}</sbol:displayId>
    <sbol:version>${version}</sbol:version>
    <dcterms:title>${name}</dcterms:title>
    <sbol:type rdf:resource="http://www.biopax.org/release/biopax-level3.owl#DnaRegion"/>
    <sbol:sequence rdf:resource="http://wiastandards.com/${partId}_sequence"/>
  </sbol:ComponentDefinition>

  <sbol:Sequence rdf:about="http://wiastandards.com/${partId}_sequence">
    <sbol:persistentIdentity rdf:resource="http://wiastandards.com/${partId}_sequence"/>
    <sbol:displayId>${partId}_sequence</sbol:displayId>
    <sbol:version>${version}</sbol:version>
    <sbol:elements>${sequence.nucleotides}</sbol:elements>
    <sbol:encoding rdf:resource="http://www.chem.qmul.ac.uk/iubmb/misc/naseq.html"/>
  </sbol:Sequence>
</rdf:RDF>`;
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Register a biological part (standalone function)
 */
export async function registerPart(
  request: PartRegistrationRequest
): Promise<PartRegistrationResponse> {
  const sdk = new SyntheticBioRegistrySDK();
  return sdk.registerPart(request);
}

/**
 * Search for parts (standalone function)
 */
export async function searchParts(query: PartSearchQuery): Promise<PartSearchResult> {
  const sdk = new SyntheticBioRegistrySDK();
  return sdk.searchParts(query);
}

/**
 * Get part details (standalone function)
 */
export async function getPartDetails(
  partId: string,
  version?: string
): Promise<BiologicalPart> {
  const sdk = new SyntheticBioRegistrySDK();
  return sdk.getPartDetails(partId, version);
}

/**
 * Update part characterization (standalone function)
 */
export async function updatePartCharacterization(
  request: CharacterizationUpdateRequest
): Promise<BiologicalPart> {
  const sdk = new SyntheticBioRegistrySDK();
  return sdk.updatePartCharacterization(request);
}

/**
 * Convert sequence format (standalone function)
 */
export async function convertFormat(
  request: FormatConversionRequest
): Promise<FormatConversionResponse> {
  const sdk = new SyntheticBioRegistrySDK();
  return sdk.convertFormat(request);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { SyntheticBioRegistrySDK };
export default SyntheticBioRegistrySDK;
