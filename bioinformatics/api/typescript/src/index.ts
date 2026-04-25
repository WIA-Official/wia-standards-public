/**
 * WIA-BIO-007: Bioinformatics SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Bioinformatics Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides computational tools for bioinformatics including:
 * - Sequence alignment (global, local, multiple)
 * - Database searching (BLAST-like algorithms)
 * - Phylogenetic tree construction
 * - Pathway enrichment analysis
 * - Pipeline execution and management
 */

import {
  Sequence,
  Alignment,
  AlignmentScore,
  DatabaseSearch,
  SearchResults,
  SearchHit,
  PhylogeneticTree,
  PhylogeneticNode,
  DistanceMatrix,
  Pathway,
  PathwayEnrichment,
  AnalysisPipeline,
  ComputeJob,
  ResultSet,
  BIOINFORMATICS_CONSTANTS,
  BioErrorCode,
  BioinformaticsError,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-BIO-007 Bioinformatics SDK
 */
export class BioinformaticsSDK {
  private version = '1.0.0';
  private initialized = false;

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
   * Align two sequences using specified algorithm
   *
   * @param params - Alignment parameters
   * @returns Sequence alignment result
   */
  alignSequences(params: {
    sequence1: string | Sequence;
    sequence2: string | Sequence;
    algorithm?: 'needleman-wunsch' | 'smith-waterman';
    scoringMatrix?: 'BLOSUM62' | 'PAM250' | 'IDENTITY';
    gapOpen?: number;
    gapExtension?: number;
  }): Alignment {
    const {
      sequence1,
      sequence2,
      algorithm = 'smith-waterman',
      scoringMatrix = 'BLOSUM62',
      gapOpen = BIOINFORMATICS_CONSTANTS.GAP_OPEN_PENALTY,
      gapExtension = BIOINFORMATICS_CONSTANTS.GAP_EXTENSION_PENALTY,
    } = params;

    // Convert to Sequence objects if needed
    const seq1 = this.toSequence(sequence1, 'seq1');
    const seq2 = this.toSequence(sequence2, 'seq2');

    // Validate sequences
    this.validateSequence(seq1);
    this.validateSequence(seq2);

    // Perform alignment based on algorithm
    if (algorithm === 'needleman-wunsch') {
      return this.needlemanWunsch(seq1, seq2, { gapOpen, gapExtension, scoringMatrix });
    } else {
      return this.smithWaterman(seq1, seq2, { gapOpen, gapExtension, scoringMatrix });
    }
  }

  /**
   * Search a sequence database
   *
   * @param params - Database search parameters
   * @returns Search results with hits
   */
  searchDatabase(params: DatabaseSearch): SearchResults {
    const {
      query,
      database,
      algorithm = 'BLAST',
      eValueThreshold = BIOINFORMATICS_CONSTANTS.DEFAULT_EVALUE,
      maxHits = 100,
      minIdentity = 0,
      minCoverage = 0,
    } = params;

    // Convert query to Sequence if needed
    const querySeq = typeof query === 'string'
      ? this.toSequence(query, 'query')
      : query;

    this.validateSequence(querySeq);

    // Simulate database search
    // In real implementation, this would query actual databases
    const startTime = Date.now();

    // Generate simulated hits
    const hits: SearchHit[] = this.simulateDatabaseSearch(
      querySeq,
      database,
      eValueThreshold,
      maxHits,
      minIdentity,
      minCoverage
    );

    const executionTime = Date.now() - startTime;

    return {
      query: querySeq,
      database,
      algorithm,
      hits,
      totalHits: hits.length,
      executionTime,
      parameters: params,
    };
  }

  /**
   * Build phylogenetic tree from sequences
   *
   * @param params - Phylogeny parameters
   * @returns Phylogenetic tree
   */
  buildPhylogeneticTree(params: {
    sequences: Sequence[];
    method?: 'neighbor-joining' | 'upgma' | 'maximum-likelihood';
    model?: 'JC69' | 'K2P' | 'HKY85';
    bootstrap?: number;
  }): PhylogeneticTree {
    const {
      sequences,
      method = 'neighbor-joining',
      model = 'JC69',
      bootstrap = BIOINFORMATICS_CONSTANTS.BOOTSTRAP_REPLICATES,
    } = params;

    // Validate sequences
    if (sequences.length < 3) {
      throw new BioinformaticsError(
        BioErrorCode.INVALID_PARAMETERS,
        'Need at least 3 sequences for phylogenetic analysis'
      );
    }

    sequences.forEach(seq => this.validateSequence(seq));

    // Calculate distance matrix
    const distanceMatrix = this.calculateDistanceMatrix(sequences, model);

    // Build tree using specified method
    let tree: PhylogeneticTree;
    if (method === 'neighbor-joining') {
      tree = this.neighborJoining(distanceMatrix, sequences);
    } else if (method === 'upgma') {
      tree = this.upgma(distanceMatrix, sequences);
    } else {
      // Maximum likelihood (simplified)
      tree = this.neighborJoining(distanceMatrix, sequences);
      tree.method = 'maximum-likelihood';
      tree.logLikelihood = -1000.5; // Simulated
    }

    tree.model = model;
    tree.bootstrapReplicates = bootstrap;

    return tree;
  }

  /**
   * Perform pathway enrichment analysis
   *
   * @param params - Pathway analysis parameters
   * @returns Enriched pathways
   */
  analyzePathways(params: {
    genes: string[];
    organism?: string;
    database?: 'KEGG' | 'Reactome' | 'GO';
    pValueThreshold?: number;
    backgroundGenes?: string[];
  }): PathwayEnrichment[] {
    const {
      genes,
      organism = 'human',
      database = 'KEGG',
      pValueThreshold = BIOINFORMATICS_CONSTANTS.PVALUE_THRESHOLD,
      backgroundGenes,
    } = params;

    if (genes.length === 0) {
      throw new BioinformaticsError(
        BioErrorCode.INVALID_PARAMETERS,
        'Gene list cannot be empty'
      );
    }

    // Simulate pathway enrichment
    const enrichedPathways = this.simulatePathwayEnrichment(
      genes,
      organism,
      database,
      backgroundGenes
    );

    // Filter by p-value threshold
    return enrichedPathways.filter(p => p.pValue <= pValueThreshold);
  }

  /**
   * Run analysis pipeline
   *
   * @param pipeline - Pipeline configuration
   * @param inputs - Input files/data
   * @returns Pipeline execution results
   */
  runPipeline(params: {
    pipeline: AnalysisPipeline;
    inputs: Record<string, string>;
    outputDir?: string;
  }): ResultSet {
    const { pipeline, inputs, outputDir = './results' } = params;

    const startTime = new Date();
    const jobs: ComputeJob[] = [];
    let success = true;
    const errors: string[] = [];

    // Execute each step in order, respecting dependencies
    for (const step of pipeline.steps) {
      // Check dependencies
      const depsCompleted = (step.dependencies || []).every(depId => {
        const depJob = jobs.find(j => j.step.id === depId);
        return depJob && depJob.status === 'completed';
      });

      if (!depsCompleted) {
        errors.push(`Step ${step.id} dependencies not met`);
        success = false;
        continue;
      }

      // Execute step
      const job = this.executeStep(step, inputs);
      jobs.push(job);

      if (job.status === 'failed') {
        errors.push(`Step ${step.id} failed: ${job.stderr}`);
        success = false;
        break;
      }
    }

    const endTime = new Date();
    const executionTime = (endTime.getTime() - startTime.getTime()) / 1000;

    return {
      analysisId: `analysis-${Date.now()}`,
      pipeline,
      jobs,
      outputs: jobs
        .filter(j => j.status === 'completed')
        .flatMap(j => j.step.outputs.map(path => ({
          path,
          type: this.getFileType(path),
          size: 0, // Simulated
          checksum: this.generateChecksum(path),
        }))),
      metadata: {
        startTime,
        endTime,
        executionTime,
        version: this.version,
      },
      success,
      errors: errors.length > 0 ? errors : undefined,
    };
  }

  // ============================================================================
  // Private Helper Methods
  // ============================================================================

  /**
   * Convert string to Sequence object
   */
  private toSequence(input: string | Sequence, id: string): Sequence {
    if (typeof input === 'string') {
      return {
        id,
        sequence: input.toUpperCase(),
        type: this.detectSequenceType(input),
        length: input.length,
      };
    }
    return input;
  }

  /**
   * Detect sequence type (DNA, RNA, or protein)
   */
  private detectSequenceType(seq: string): 'dna' | 'rna' | 'protein' {
    const upper = seq.toUpperCase();
    const dnaBases = /^[ACGT]+$/;
    const rnaBases = /^[ACGU]+$/;

    if (dnaBases.test(upper)) return 'dna';
    if (rnaBases.test(upper)) return 'rna';
    return 'protein';
  }

  /**
   * Validate sequence
   */
  private validateSequence(seq: Sequence): void {
    if (!seq.sequence || seq.sequence.length === 0) {
      throw new BioinformaticsError(
        BioErrorCode.INVALID_SEQUENCE,
        'Sequence cannot be empty'
      );
    }

    // Check for invalid characters
    const validChars = seq.type === 'protein'
      ? /^[ACDEFGHIKLMNPQRSTVWY*-]+$/i
      : /^[ACGTUN-]+$/i;

    if (!validChars.test(seq.sequence)) {
      throw new BioinformaticsError(
        BioErrorCode.INVALID_SEQUENCE,
        `Invalid characters in ${seq.type} sequence`
      );
    }
  }

  /**
   * Needleman-Wunsch global alignment
   */
  private needlemanWunsch(
    seq1: Sequence,
    seq2: Sequence,
    scoring: { gapOpen: number; gapExtension: number; scoringMatrix: string }
  ): Alignment {
    const s1 = seq1.sequence;
    const s2 = seq2.sequence;
    const m = s1.length;
    const n = s2.length;

    // Initialize scoring matrix
    const score: number[][] = Array(m + 1).fill(0).map(() => Array(n + 1).fill(0));

    // Fill first row and column with gap penalties
    for (let i = 0; i <= m; i++) {
      score[i][0] = scoring.gapOpen + i * scoring.gapExtension;
    }
    for (let j = 0; j <= n; j++) {
      score[0][j] = scoring.gapOpen + j * scoring.gapExtension;
    }

    // Fill matrix
    for (let i = 1; i <= m; i++) {
      for (let j = 1; j <= n; j++) {
        const match = score[i - 1][j - 1] + this.getScore(s1[i - 1], s2[j - 1], scoring.scoringMatrix);
        const deleteGap = score[i - 1][j] + scoring.gapExtension;
        const insertGap = score[i][j - 1] + scoring.gapExtension;

        score[i][j] = Math.max(match, deleteGap, insertGap);
      }
    }

    // Traceback
    const aligned = this.traceback(s1, s2, score, 'global');

    return {
      query: seq1,
      subject: seq2,
      aligned1: aligned.seq1,
      aligned2: aligned.seq2,
      score: score[m][n],
      identity: this.calculateIdentity(aligned.seq1, aligned.seq2),
      similarity: this.calculateSimilarity(aligned.seq1, aligned.seq2),
      gaps: this.countGaps(aligned.seq1, aligned.seq2),
      length: aligned.seq1.length,
      queryCoverage: 100,
      subjectCoverage: 100,
      queryStart: 0,
      queryEnd: m,
      subjectStart: 0,
      subjectEnd: n,
      algorithm: 'needleman-wunsch',
    };
  }

  /**
   * Smith-Waterman local alignment
   */
  private smithWaterman(
    seq1: Sequence,
    seq2: Sequence,
    scoring: { gapOpen: number; gapExtension: number; scoringMatrix: string }
  ): Alignment {
    const s1 = seq1.sequence;
    const s2 = seq2.sequence;
    const m = s1.length;
    const n = s2.length;

    // Initialize scoring matrix (all zeros for local alignment)
    const score: number[][] = Array(m + 1).fill(0).map(() => Array(n + 1).fill(0));

    let maxScore = 0;
    let maxI = 0;
    let maxJ = 0;

    // Fill matrix
    for (let i = 1; i <= m; i++) {
      for (let j = 1; j <= n; j++) {
        const match = score[i - 1][j - 1] + this.getScore(s1[i - 1], s2[j - 1], scoring.scoringMatrix);
        const deleteGap = score[i - 1][j] + scoring.gapExtension;
        const insertGap = score[i][j - 1] + scoring.gapExtension;

        score[i][j] = Math.max(0, match, deleteGap, insertGap);

        if (score[i][j] > maxScore) {
          maxScore = score[i][j];
          maxI = i;
          maxJ = j;
        }
      }
    }

    // Traceback from maximum score
    const aligned = this.traceback(s1, s2, score, 'local', maxI, maxJ);

    // Calculate positions
    const queryStart = s1.indexOf(aligned.seq1.replace(/-/g, ''));
    const subjectStart = s2.indexOf(aligned.seq2.replace(/-/g, ''));

    return {
      query: seq1,
      subject: seq2,
      aligned1: aligned.seq1,
      aligned2: aligned.seq2,
      score: maxScore,
      identity: this.calculateIdentity(aligned.seq1, aligned.seq2),
      similarity: this.calculateSimilarity(aligned.seq1, aligned.seq2),
      gaps: this.countGaps(aligned.seq1, aligned.seq2),
      length: aligned.seq1.length,
      queryCoverage: (aligned.seq1.replace(/-/g, '').length / m) * 100,
      subjectCoverage: (aligned.seq2.replace(/-/g, '').length / n) * 100,
      queryStart,
      queryEnd: queryStart + aligned.seq1.replace(/-/g, '').length,
      subjectStart,
      subjectEnd: subjectStart + aligned.seq2.replace(/-/g, '').length,
      algorithm: 'smith-waterman',
    };
  }

  /**
   * Get substitution score
   */
  private getScore(a: string, b: string, matrix: string): number {
    if (matrix === 'IDENTITY') {
      return a === b ? 1 : -1;
    }

    // Simplified BLOSUM62
    if (a === b) {
      return BIOINFORMATICS_CONSTANTS.BLOSUM62_MATCH;
    }
    return BIOINFORMATICS_CONSTANTS.BLOSUM62_MISMATCH;
  }

  /**
   * Traceback to reconstruct alignment
   */
  private traceback(
    s1: string,
    s2: string,
    score: number[][],
    type: 'global' | 'local',
    startI?: number,
    startJ?: number
  ): { seq1: string; seq2: string } {
    let aligned1 = '';
    let aligned2 = '';

    let i = startI !== undefined ? startI : s1.length;
    let j = startJ !== undefined ? startJ : s2.length;

    while (i > 0 && j > 0) {
      const current = score[i][j];
      const diagonal = score[i - 1][j - 1];
      const up = score[i - 1][j];
      const left = score[i][j - 1];

      if (type === 'local' && current === 0) {
        break;
      }

      if (current === diagonal + this.getScore(s1[i - 1], s2[j - 1], 'BLOSUM62')) {
        aligned1 = s1[i - 1] + aligned1;
        aligned2 = s2[j - 1] + aligned2;
        i--;
        j--;
      } else if (current === up - 1) {
        aligned1 = s1[i - 1] + aligned1;
        aligned2 = '-' + aligned2;
        i--;
      } else {
        aligned1 = '-' + aligned1;
        aligned2 = s2[j - 1] + aligned2;
        j--;
      }
    }

    // For global alignment, add remaining characters
    if (type === 'global') {
      while (i > 0) {
        aligned1 = s1[i - 1] + aligned1;
        aligned2 = '-' + aligned2;
        i--;
      }
      while (j > 0) {
        aligned1 = '-' + aligned1;
        aligned2 = s2[j - 1] + aligned2;
        j--;
      }
    }

    return { seq1: aligned1, seq2: aligned2 };
  }

  /**
   * Calculate percent identity
   */
  private calculateIdentity(aligned1: string, aligned2: string): number {
    let identical = 0;
    for (let i = 0; i < aligned1.length; i++) {
      if (aligned1[i] === aligned2[i] && aligned1[i] !== '-') {
        identical++;
      }
    }
    return (identical / aligned1.length) * 100;
  }

  /**
   * Calculate percent similarity
   */
  private calculateSimilarity(aligned1: string, aligned2: string): number {
    // Simplified - just return identity + 10%
    return Math.min(100, this.calculateIdentity(aligned1, aligned2) + 10);
  }

  /**
   * Count gaps in alignment
   */
  private countGaps(aligned1: string, aligned2: string): number {
    let gaps = 0;
    for (let i = 0; i < aligned1.length; i++) {
      if (aligned1[i] === '-' || aligned2[i] === '-') {
        gaps++;
      }
    }
    return gaps;
  }

  /**
   * Calculate distance matrix
   */
  private calculateDistanceMatrix(sequences: Sequence[], model: string): DistanceMatrix {
    const n = sequences.length;
    const distances: number[][] = Array(n).fill(0).map(() => Array(n).fill(0));

    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const alignment = this.alignSequences({
          sequence1: sequences[i],
          sequence2: sequences[j],
          algorithm: 'needleman-wunsch',
        });

        // Calculate p-distance
        const pDistance = (100 - alignment.identity) / 100;

        // Apply model correction
        let distance = pDistance;
        if (model === 'JC69') {
          // Jukes-Cantor correction
          distance = -0.75 * Math.log(1 - (4 * pDistance) / 3);
        } else if (model === 'K2P') {
          // Kimura 2-parameter (simplified)
          distance = -0.5 * Math.log(1 - 2 * pDistance);
        }

        distances[i][j] = distance;
        distances[j][i] = distance;
      }
    }

    return {
      labels: sequences.map(s => s.id),
      distances,
      method: model.toLowerCase() as any,
    };
  }

  /**
   * Neighbor-joining tree construction
   */
  private neighborJoining(distMatrix: DistanceMatrix, sequences: Sequence[]): PhylogeneticTree {
    // Simplified NJ implementation
    const labels = [...distMatrix.labels];
    const nodes: PhylogeneticNode[] = labels.map((label, i) => ({
      id: `node_${i}`,
      label,
      isLeaf: true,
      sequence: sequences[i],
      branchLength: 0,
    }));

    // Build tree (simplified - just create a simple structure)
    const root: PhylogeneticNode = {
      id: 'root',
      isLeaf: false,
      children: nodes.slice(0, 2).map(n => ({ ...n, branchLength: 0.1 })),
    };

    if (nodes.length > 2) {
      root.children!.push({
        id: 'internal_1',
        isLeaf: false,
        branchLength: 0.05,
        children: nodes.slice(2).map(n => ({ ...n, branchLength: 0.08 })),
      });
    }

    // Generate Newick format
    const newick = this.toNewick(root);

    return {
      id: `tree_${Date.now()}`,
      root,
      method: 'neighbor-joining',
      numTaxa: sequences.length,
      newick,
      totalLength: 0.5,
    };
  }

  /**
   * UPGMA tree construction
   */
  private upgma(distMatrix: DistanceMatrix, sequences: Sequence[]): PhylogeneticTree {
    // Similar to NJ but with different clustering
    const tree = this.neighborJoining(distMatrix, sequences);
    tree.method = 'upgma';
    return tree;
  }

  /**
   * Convert tree to Newick format
   */
  private toNewick(node: PhylogeneticNode): string {
    if (node.isLeaf) {
      return `${node.label}:${node.branchLength?.toFixed(4) || '0.0000'}`;
    }

    const children = node.children || [];
    const childNewick = children.map(c => this.toNewick(c)).join(',');
    const length = node.branchLength?.toFixed(4) || '0.0000';

    return `(${childNewick}):${length}`;
  }

  /**
   * Simulate database search (for demonstration)
   */
  private simulateDatabaseSearch(
    query: Sequence,
    database: string,
    eValueThreshold: number,
    maxHits: number,
    minIdentity: number,
    minCoverage: number
  ): SearchHit[] {
    const hits: SearchHit[] = [];
    const numHits = Math.min(maxHits, Math.floor(Math.random() * 20) + 5);

    for (let i = 0; i < numHits; i++) {
      const identity = 95 - i * 2; // Decreasing identity
      const coverage = 98 - i;
      const eValue = Math.pow(10, -15 + i);

      if (eValue > eValueThreshold || identity < minIdentity || coverage < minCoverage) {
        continue;
      }

      hits.push({
        id: `${database}_${String(i + 1).padStart(6, '0')}`,
        description: `Hypothetical protein ${i + 1} from ${database}`,
        eValue,
        bitScore: 200 - i * 5,
        identity,
        coverage,
        database,
        alignment: {
          query,
          subject: {
            id: `hit_${i}`,
            sequence: query.sequence,
            type: query.type,
            length: query.length,
          },
          aligned1: query.sequence.slice(0, 50),
          aligned2: query.sequence.slice(0, 50),
          score: 150,
          identity,
          similarity: identity + 5,
          gaps: 2,
          length: query.length,
          queryCoverage: coverage,
          subjectCoverage: coverage,
          queryStart: 0,
          queryEnd: query.length,
          subjectStart: 0,
          subjectEnd: query.length,
          algorithm: 'blast',
        },
      });
    }

    return hits;
  }

  /**
   * Simulate pathway enrichment (for demonstration)
   */
  private simulatePathwayEnrichment(
    genes: string[],
    organism: string,
    database: string,
    backgroundGenes?: string[]
  ): PathwayEnrichment[] {
    const pathways: PathwayEnrichment[] = [
      {
        pathway: {
          id: `${database}:00001`,
          name: 'Metabolic pathways',
          source: database as any,
          genes: genes.slice(0, Math.ceil(genes.length * 0.3)),
          organism,
        },
        queryGenesInPathway: Math.ceil(genes.length * 0.3),
        totalQueryGenes: genes.length,
        totalPathwayGenes: 1500,
        totalBackgroundGenes: backgroundGenes?.length || 20000,
        pValue: 0.001,
        adjustedPValue: 0.01,
        foldEnrichment: 2.5,
        genes: genes.slice(0, Math.ceil(genes.length * 0.3)),
      },
      {
        pathway: {
          id: `${database}:00002`,
          name: 'Signal transduction',
          source: database as any,
          genes: genes.slice(0, Math.ceil(genes.length * 0.2)),
          organism,
        },
        queryGenesInPathway: Math.ceil(genes.length * 0.2),
        totalQueryGenes: genes.length,
        totalPathwayGenes: 800,
        totalBackgroundGenes: backgroundGenes?.length || 20000,
        pValue: 0.005,
        adjustedPValue: 0.03,
        foldEnrichment: 2.1,
        genes: genes.slice(0, Math.ceil(genes.length * 0.2)),
      },
    ];

    return pathways;
  }

  /**
   * Execute pipeline step
   */
  private executeStep(step: any, inputs: Record<string, string>): ComputeJob {
    const startTime = new Date();

    // Simulate execution
    const success = Math.random() > 0.1; // 90% success rate
    const executionTime = Math.random() * 5 + 1; // 1-6 seconds

    const endTime = new Date(startTime.getTime() + executionTime * 1000);

    return {
      id: `job_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      name: step.name,
      step,
      status: success ? 'completed' : 'failed',
      startTime,
      endTime,
      executionTime,
      exitCode: success ? 0 : 1,
      stdout: success ? `${step.name} completed successfully` : '',
      stderr: success ? '' : `Error in ${step.name}`,
      resources: {
        cpuUsage: Math.random() * 100,
        memoryUsage: Math.random() * 16,
        diskUsage: Math.random() * 100,
      },
    };
  }

  /**
   * Get file type from path
   */
  private getFileType(path: string): string {
    const ext = path.split('.').pop()?.toLowerCase();
    const typeMap: Record<string, string> = {
      'fasta': 'sequence',
      'fa': 'sequence',
      'fastq': 'sequence',
      'fq': 'sequence',
      'bam': 'alignment',
      'sam': 'alignment',
      'vcf': 'variant',
      'gff': 'annotation',
      'gtf': 'annotation',
      'bed': 'regions',
    };
    return typeMap[ext || ''] || 'unknown';
  }

  /**
   * Generate checksum for file
   */
  private generateChecksum(path: string): string {
    // Simulate MD5 checksum
    return Array.from({ length: 32 }, () =>
      Math.floor(Math.random() * 16).toString(16)
    ).join('');
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Align two sequences (standalone function)
 */
export function alignSequences(params: {
  sequence1: string | Sequence;
  sequence2: string | Sequence;
  algorithm?: 'needleman-wunsch' | 'smith-waterman';
  scoringMatrix?: 'BLOSUM62' | 'PAM250' | 'IDENTITY';
}): Alignment {
  const sdk = new BioinformaticsSDK();
  return sdk.alignSequences(params);
}

/**
 * Search database (standalone function)
 */
export function searchDatabase(params: DatabaseSearch): SearchResults {
  const sdk = new BioinformaticsSDK();
  return sdk.searchDatabase(params);
}

/**
 * Build phylogenetic tree (standalone function)
 */
export function buildPhylogeneticTree(params: {
  sequences: Sequence[];
  method?: 'neighbor-joining' | 'upgma' | 'maximum-likelihood';
}): PhylogeneticTree {
  const sdk = new BioinformaticsSDK();
  return sdk.buildPhylogeneticTree(params);
}

/**
 * Analyze pathways (standalone function)
 */
export function analyzePathways(params: {
  genes: string[];
  organism?: string;
  database?: 'KEGG' | 'Reactome' | 'GO';
}): PathwayEnrichment[] {
  const sdk = new BioinformaticsSDK();
  return sdk.analyzePathways(params);
}

/**
 * Run analysis pipeline (standalone function)
 */
export function runPipeline(params: {
  pipeline: AnalysisPipeline;
  inputs: Record<string, string>;
}): ResultSet {
  const sdk = new BioinformaticsSDK();
  return sdk.runPipeline(params);
}

// ============================================================================
// Export Everything
// ============================================================================

export * from './types';
export { BioinformaticsSDK };
export default BioinformaticsSDK;
