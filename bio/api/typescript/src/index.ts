/**
 * WIA Bio Standard - SDK Implementation
 *
 * @packageDocumentation
 * @module wia-bio
 */

import { EventEmitter } from 'events';
import * as types from './types';

export * from './types';

/**
 * Main Bio Platform class
 */
export class WIABio extends EventEmitter {
  private config: types.BioConfig;
  private sequences: Map<string, types.Sequence> = new Map();
  private samples: Map<string, types.Sample> = new Map();
  private experiments: Map<string, types.Experiment> = new Map();
  private genes: Map<string, types.Gene> = new Map();
  private proteins: Map<string, types.Protein> = new Map();

  constructor(config: types.BioConfig) {
    super();
    this.config = config;
  }

  async addSequence(sequence: Omit<types.Sequence, 'id' | 'length'>): Promise<types.Sequence> {
    const newSeq: types.Sequence = {
      ...sequence,
      id: `seq-${Date.now()}`,
      length: sequence.sequence.length
    };

    if (this.config.enableQualityChecks) {
      newSeq.quality = this.calculateQuality(newSeq);
    }

    this.sequences.set(newSeq.id, newSeq);
    this.emit('sequence-added', newSeq);
    return newSeq;
  }

  private calculateQuality(sequence: types.Sequence): types.SequenceQuality {
    const seq = sequence.sequence.toUpperCase();
    const gcCount = (seq.match(/[GC]/g) || []).length;
    const nCount = (seq.match(/N/g) || []).length;

    return {
      level: types.QualityLevel.Raw,
      meanQuality: 30,
      gcContent: gcCount / seq.length,
      nContent: nCount / seq.length
    };
  }

  getSequence(sequenceId: string): types.Sequence | undefined {
    return this.sequences.get(sequenceId);
  }

  searchSequences(query: { organism?: string; type?: types.SequenceType }): types.Sequence[] {
    return Array.from(this.sequences.values()).filter(seq => {
      if (query.organism && !seq.organism.scientificName.toLowerCase().includes(query.organism.toLowerCase())) {
        return false;
      }
      if (query.type && seq.type !== query.type) {
        return false;
      }
      return true;
    });
  }

  async translateDNA(sequenceId: string): Promise<string> {
    const sequence = this.sequences.get(sequenceId);
    if (!sequence || sequence.type !== types.SequenceType.DNA) {
      throw new Error('Invalid DNA sequence');
    }

    const codonTable: Record<string, string> = {
      'ATG': 'M', 'TGG': 'W', 'TTT': 'F', 'TTC': 'F',
      'TTA': 'L', 'TTG': 'L', 'CTT': 'L', 'CTC': 'L', 'CTA': 'L', 'CTG': 'L',
      'ATT': 'I', 'ATC': 'I', 'ATA': 'I', 'GTT': 'V', 'GTC': 'V', 'GTA': 'V', 'GTG': 'V',
      'TAA': '*', 'TAG': '*', 'TGA': '*'
    };

    let protein = '';
    for (let i = 0; i < sequence.sequence.length - 2; i += 3) {
      const codon = sequence.sequence.substring(i, i + 3).toUpperCase();
      protein += codonTable[codon] || 'X';
    }

    return protein;
  }

  async findORFs(sequenceId: string, minLength: number = 100): Promise<types.Feature[]> {
    const sequence = this.sequences.get(sequenceId);
    if (!sequence || sequence.type !== types.SequenceType.DNA) {
      throw new Error('Invalid DNA sequence');
    }

    const orfs: types.Feature[] = [];
    const seq = sequence.sequence.toUpperCase();

    let inORF = false;
    let orfStart = 0;

    for (let i = 0; i < seq.length - 2; i += 3) {
      const codon = seq.substring(i, i + 3);
      if (codon === 'ATG' && !inORF) {
        inORF = true;
        orfStart = i;
      } else if ((codon === 'TAA' || codon === 'TAG' || codon === 'TGA') && inORF) {
        if (i - orfStart >= minLength) {
          orfs.push({
            id: `orf-${orfs.length}`,
            type: 'cds',
            name: `ORF${orfs.length + 1}`,
            start: orfStart,
            end: i + 3,
            strand: '+'
          });
        }
        inORF = false;
      }
    }

    return orfs;
  }

  async align(queryId: string, targetId: string): Promise<types.Alignment> {
    const query = this.sequences.get(queryId);
    const target = this.sequences.get(targetId);
    if (!query || !target) throw new Error('Sequences not found');

    // Simplified alignment simulation
    const minLen = Math.min(query.length, target.length);
    let matches = 0;
    for (let i = 0; i < minLen; i++) {
      if (query.sequence[i] === target.sequence[i]) matches++;
    }

    return {
      id: `align-${Date.now()}`,
      queryId,
      targetId,
      queryStart: 0,
      queryEnd: query.length,
      targetStart: 0,
      targetEnd: target.length,
      score: matches,
      identity: matches / minLen,
      coverage: minLen / query.length,
      eValue: Math.pow(10, -matches)
    };
  }

  async registerGene(gene: Omit<types.Gene, 'id'>): Promise<types.Gene> {
    const newGene: types.Gene = {
      ...gene,
      id: `gene-${Date.now()}`
    };

    this.genes.set(newGene.id, newGene);
    this.emit('gene-registered', newGene);
    return newGene;
  }

  getGene(geneId: string): types.Gene | undefined {
    return this.genes.get(geneId);
  }

  async registerProtein(protein: Omit<types.Protein, 'id'>): Promise<types.Protein> {
    const newProtein: types.Protein = {
      ...protein,
      id: `prot-${Date.now()}`
    };

    this.proteins.set(newProtein.id, newProtein);
    this.emit('protein-registered', newProtein);
    return newProtein;
  }

  getProtein(proteinId: string): types.Protein | undefined {
    return this.proteins.get(proteinId);
  }

  async createExperiment(experiment: Omit<types.Experiment, 'id' | 'results' | 'status' | 'startDate'>): Promise<types.Experiment> {
    const newExperiment: types.Experiment = {
      ...experiment,
      id: `exp-${Date.now()}`,
      results: [],
      status: 'pending',
      startDate: new Date()
    };

    this.experiments.set(newExperiment.id, newExperiment);
    this.emit('experiment-created', newExperiment);
    return newExperiment;
  }

  async startExperiment(experimentId: string): Promise<void> {
    const experiment = this.experiments.get(experimentId);
    if (!experiment) throw new Error('Experiment not found');

    experiment.status = 'running';
    this.emit('experiment-started', experiment);
  }

  async addExperimentResult(experimentId: string, result: Omit<types.ExperimentResult, 'id' | 'timestamp'>): Promise<void> {
    const experiment = this.experiments.get(experimentId);
    if (!experiment) throw new Error('Experiment not found');

    experiment.results.push({
      ...result,
      id: `result-${experiment.results.length}`,
      timestamp: new Date()
    });

    this.emit('result-added', { experimentId, result });
  }

  calculateGCContent(sequence: string): number {
    const upper = sequence.toUpperCase();
    const gc = (upper.match(/[GC]/g) || []).length;
    return gc / sequence.length;
  }

  reverseComplement(sequence: string): string {
    const complement: Record<string, string> = { 'A': 'T', 'T': 'A', 'G': 'C', 'C': 'G', 'N': 'N' };
    return sequence.split('').reverse().map(base => complement[base.toUpperCase()] || 'N').join('');
  }

  checkCompliance(targetLevel: types.CertificationLevel): types.ComplianceReport {
    const tests: types.TestResult[] = [];

    tests.push({
      testName: 'Configuration Validation',
      passed: this.config.apiEndpoint !== undefined,
      notes: 'API endpoint must be defined'
    });

    tests.push({
      testName: 'Sequence Support',
      passed: true,
      notes: 'SDK supports sequence management'
    });

    if (targetLevel !== types.CertificationLevel.Bronze) {
      tests.push({
        testName: 'Quality Checks',
        passed: this.config.enableQualityChecks === true,
        notes: 'Quality checks required for Silver/Gold'
      });
    }

    const passed = tests.every(t => t.passed);

    return {
      standard: 'WIA-BIO',
      testDate: new Date().toISOString(),
      config: this.config,
      targetLevel,
      tests,
      passed,
      achievedLevel: passed ? targetLevel : undefined
    };
  }
}

export default { WIABio };
