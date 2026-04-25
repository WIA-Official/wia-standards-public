# Chapter 4: Genomic and DNA Sequencing

## Molecular Tools for Marine Biodiversity

DNA sequencing has revolutionized marine biology, enabling species identification from microscopic larvae, revealing cryptic species, detecting organisms from water samples, and reconstructing evolutionary relationships. This chapter explores how genomic data is generated, standardized, and integrated with traditional observational data.

### The DNA Revolution in Marine Biology

Traditional marine taxonomy relies on morphological features - often challenging for microscopic organisms, larval stages, or cryptic species. DNA sequencing provides molecular fingerprints that enable:

**Species Identification:** Match unknown specimens to reference sequences
**Cryptic Species Discovery:** Reveal hidden diversity in morphologically similar groups
**Environmental DNA (eDNA):** Detect species from water samples without capture
**Gut Content Analysis:** Identify prey items in predator stomachs
**Population Genetics:** Track connectivity and migration patterns
**Phylogenetics:** Reconstruct evolutionary relationships

### DNA Barcoding for Species Identification

The **Barcode of Life** initiative established standardized genetic markers for species identification:

#### Standard Barcode Genes

```typescript
interface BarcodeGene {
  gene: string;
  targetGroup: string;
  length: number;              // Base pairs
  variability: "low" | "medium" | "high";
  universality: number;        // 0-1 scale (how many species amplify)
}

const marineBarcodesGenes: BarcodeGene[] = [
  {
    gene: "COI",                 // Cytochrome c oxidase I
    targetGroup: "Animals (Metazoa)",
    length: 658,
    variability: "high",
    universality: 0.85
  },
  {
    gene: "16S rRNA",
    targetGroup: "All life (universal)",
    length: 1500,
    variability: "medium",
    universality: 0.99
  },
  {
    gene: "18S rRNA",
    targetGroup: "Eukaryotes",
    length: 1800,
    variability: "low",
    universality: 0.95
  },
  {
    gene: "rbcL",                // Ribulose-bisphosphate carboxylase
    targetGroup: "Algae, plants",
    length: 600,
    variability: "medium",
    universality: 0.75
  },
  {
    gene: "ITS",                 // Internal transcribed spacer
    targetGroup: "Fungi, some algae",
    length: 600,
    variability: "high",
    universality: 0.80
  }
];
```

#### DNA Barcode Data Structure

```typescript
interface DNABarcodeRecord {
  // Identifiers
  processID: string;           // BOLD Process ID
  genBankID?: string;          // GenBank accession number
  aphiaID?: number;            // WoRMS taxonomic ID

  // Taxonomy
  scientificName: string;
  kingdom: string;
  phylum: string;
  class: string;
  order: string;
  family: string;
  genus: string;
  species: string;
  identificationMethod: "morphology" | "DNA" | "both";
  identificationConfidence: number;  // 0-1 scale
  identifier: string;          // Expert who identified specimen

  // Specimen data
  specimen: {
    catalogNumber: string;
    institution: string;       // e.g., "Smithsonian NMNH"
    collectionCode: string;
    fieldNumber?: string;
    lifeStage: "adult" | "juvenile" | "larva" | "egg" | "unknown";
    sex?: "male" | "female" | "hermaphrodite" | "unknown";
    tissueType: string;        // "muscle", "fin_clip", "whole_organism"
    preservation: string;      // "ethanol", "frozen", "dried"
    voucherPhoto?: string;     // URL to specimen image
  };

  // Collection event
  collection: {
    eventID: string;
    collectors: string[];
    collectionDate: Date;
    location: {
      locality: string;        // Free text description
      latitude: number;
      longitude: number;
      coordinateUncertainty: number;
      depth?: number;          // Meters
      habitat: string;         // "coral_reef", "rocky_shore", "deep_sea"
    };
  };

  // Sequence data
  sequence: {
    markerCode: string;        // "COI-5P", "16S", "18S"
    nucleotides: string;       // FASTA sequence (A,T,G,C,N)
    sequenceLength: number;
    ambiguousBases: number;    // Count of 'N' (unknown bases)
    GC_content: number;        // Percentage

    primers: {
      forward: string;         // Primer name/sequence
      reverse: string;
    };

    sequencingFacility: string;
    sequencingMethod: "Sanger" | "Illumina" | "PacBio" | "Nanopore";
    sequencingDate: Date;

    quality: {
      averageQuality: number;  // Phred score
      traceFileAvailable: boolean;
      contigs?: number;        // If assembled from multiple reads
    };
  };

  // Analysis
  identification: {
    method: "BLAST" | "tree_based" | "ML_classifier";
    database: string;          // "BOLD", "GenBank", "custom"
    databaseVersion: string;
    matchType: "exact" | "close" | "ambiguous";
    similarity: number;        // 0-100% to nearest match
    nearestSpecies: string;
    geneticDistance: number;   // K2P distance or similar
  };

  // Metadata
  publicReleaseDate?: Date;
  dataUseLicense: "CC0" | "CC-BY" | "CC-BY-NC";
  funding: string[];
  references: string[];        // DOIs of publications
}

// Real example: Coral grouper DNA barcode
const coralGrouperBarcode: DNABarcodeRecord = {
  processID: "FISBO001-18",
  genBankID: "MH123456",
  aphiaID: 212844,

  scientificName: "Plectropomus leopardus",
  kingdom: "Animalia",
  phylum: "Chordata",
  class: "Actinopterygii",
  order: "Perciformes",
  family: "Serranidae",
  genus: "Plectropomus",
  species: "leopardus",
  identificationMethod: "both",
  identificationConfidence: 1.0,
  identifier: "Dr. John Smith",

  specimen: {
    catalogNumber: "BPBM 41234",
    institution: "Bernice Pauahi Bishop Museum",
    collectionCode: "FISH",
    fieldNumber: "JRS-2018-045",
    lifeStage: "adult",
    sex: "female",
    tissueType: "muscle",
    preservation: "ethanol",
    voucherPhoto: "https://example.org/specimen/BPBM41234.jpg"
  },

  collection: {
    eventID: "Hawaii_2018_Dive_023",
    collectors: ["J. Smith", "A. Wong"],
    collectionDate: new Date("2018-06-15"),
    location: {
      locality: "Molokini Crater, Maui, Hawaii",
      latitude: 20.6283,
      longitude: -156.4950,
      coordinateUncertainty: 100,
      depth: 18,
      habitat: "coral_reef"
    }
  },

  sequence: {
    markerCode: "COI-5P",
    nucleotides: "ATGGCACACCTCCGAACCCTTTACTTGATTTTCGGAGCGTGGGCCGGAATAGTAGGAACCGCCCTTAGCCTCCTCATTCGGGCCGAACTAAGTCAACCCGGGTCCCTCCTAGGGGACGATCAAATCTACAATGTCATCGTTACCGCACACGCCTTTGTAATAATTTTCTTTATAGTTATACCAATTATAATTGGAGGATTTGGAAACTGACTTGTACCACTTATGATTGGAGCTCCCGATATAGCCTTCCCCCGAATAAATAATATAAGCTTTTGACTCCTACCCCCCTCATTCACACTCCTTCTAGCCTCCTCCGGTGTAGAAGCCGGTGCTGGCACAGGATGAACTGTTTATCCTCCCCTAGCCGGCAATCTTGCCCACGCCGGAGCCTCCGTTGACCTAACCATTTTCTCCCTACACTTAGCAGGTATTTCATCAATTTTAGGGGCCATTAATTTTATCACAACAATTATTAATATACGAATCCCAGGAATAACCCTAGACCGTATACCCCTATTTGTATGATCCGTTTTAATTACAGCAGTCCTTCTCCTACTATCCCTACCAGTACTAGCCGGGGCTATTACTATACTATTAACAGACCGAAACCTAAATACAACCTTCTTTGACCCTGCCGGCGGAGGAGACCCTATCCTTTACCAACATTTATTC",
    sequenceLength: 658,
    ambiguousBases: 0,
    GC_content: 47.2,

    primers: {
      forward: "FishF2_t1 (TGTAAAACGACGGCCAGTCGACTAATCATAAAGATATCGGCAC)",
      reverse: "FishR2_t1 (CAGGAAACAGCTATGACACTTCAGGGTGACCGAAGAATCAGAA)"
    },

    sequencingFacility: "University of Hawaii Sequencing Core",
    sequencingMethod: "Sanger",
    sequencingDate: new Date("2018-07-01"),

    quality: {
      averageQuality: 58,      // High quality Sanger read
      traceFileAvailable: true,
      contigs: 1
    }
  },

  identification: {
    method: "BLAST",
    database: "BOLD",
    databaseVersion: "v4.0",
    matchType: "exact",
    similarity: 99.85,
    nearestSpecies: "Plectropomus leopardus",
    geneticDistance: 0.0015
  },

  publicReleaseDate: new Date("2018-08-15"),
  dataUseLicense: "CC0",
  funding: ["NSF OCE-1234567"],
  references: ["10.1234/example.doi"]
};
```

### Environmental DNA (eDNA) Metabarcoding

Rather than capturing organisms, eDNA methods sequence DNA from water samples to detect all species present:

```typescript
interface eDNASample {
  sampleID: string;
  project: string;

  sampling: {
    date: Date;
    location: GeographicCoordinate;
    depth: number;              // Meters
    habitat: string;
    sampleVolume: number;       // Liters filtered
    replicates: number;

    filtration: {
      filterType: string;       // "Sterivex", "membrane_filter"
      poreSize: number;         // Micrometers (typically 0.22 or 0.45)
      filtrationMethod: "vacuum" | "peristaltic_pump";
      preservative: string;     // "ethanol", "RNAlater", "frozen"
    };

    environmental: {
      temperature: number;      // Celsius
      salinity: number;         // PSU
      pH?: number;
      turbidity?: number;       // NTU
    };

    controls: {
      fieldBlank: boolean;      // Filtered clean water at site
      extractionBlank: boolean; // No-template DNA extraction
      PCR_negative: boolean;    // No-template PCR
    };
  };

  laboratory: {
    extractionMethod: string;   // "DNeasy Blood & Tissue Kit"
    extractionDate: Date;
    DNAConcentration: number;   // ng/μL
    260_280_ratio: number;      // DNA purity indicator

    PCR: {
      target: string;           // "COI", "16S", "18S", "12S"
      primers: {
        forward: string;
        reverse: string;
      };
      cycles: number;
      annealingTemp: number;    // Celsius
      replicates: number;
    };

    sequencing: {
      platform: "Illumina MiSeq" | "Illumina NovaSeq" | "Ion Torrent" | "PacBio";
      readType: "paired_end" | "single_end";
      readLength: number;       // Base pairs
      indexing: "single" | "dual";
      runID: string;
    };
  };

  bioinformatics: {
    rawReads: number;
    qualityFilteredReads: number;
    mergedReads: number;        // Paired-end only
    clusteredOTUs: number;      // Operational Taxonomic Units

    pipeline: string;           // "QIIME2", "mothur", "DADA2"
    version: string;
    steps: string[];            // ["demultiplex", "quality_filter", "denoise", "cluster"]

    taxonomicAssignment: {
      method: "BLAST" | "ML_classifier" | "phylogenetic";
      database: string;         // "SILVA", "MIDORI", "custom"
      databaseVersion: string;
      confidenceThreshold: number;  // 0-1 scale
    };
  };

  results: eDNADetection[];
}

interface eDNADetection {
  OTU_ID: string;               // Operational Taxonomic Unit identifier
  sequence: string;             // Representative sequence
  readCount: number;            // Number of sequence reads
  relativeAbundance: number;    // Proportion of total reads

  taxonomy: {
    kingdom: string;
    phylum: string;
    class: string;
    order: string;
    family: string;
    genus: string;
    species: string;
    confidence: number;         // Assignment confidence
  };

  matchQuality: {
    similarity: number;         // Percent similarity to reference
    coverage: number;           // Percent of query sequence aligned
    eValue: number;             // BLAST E-value
    topHit: string;             // Best match species name
  };

  annotation: {
    trophicLevel?: string;      // "producer", "herbivore", "carnivore"
    habitat?: string;           // Expected habitat
    invasiveStatus?: "native" | "invasive" | "cryptogenic";
    conservationStatus?: string;
  };
}

// Real example: Coral reef eDNA survey
const coralReefEDNA: eDNASample = {
  sampleID: "HI_Reef_001_Rep1",
  project: "Hawaii Reef Biodiversity Assessment 2024",

  sampling: {
    date: new Date("2024-06-15T10:30:00Z"),
    location: {
      latitude: 21.4389,
      longitude: -157.7881,
      datum: "WGS84",
      uncertainty: 10
    },
    depth: 10,
    habitat: "coral_reef",
    sampleVolume: 2.0,          // 2 liters
    replicates: 3,

    filtration: {
      filterType: "Sterivex-GP 0.22μm",
      poreSize: 0.22,
      filtrationMethod: "peristaltic_pump",
      preservative: "RNAlater"
    },

    environmental: {
      temperature: 26.5,
      salinity: 35.2,
      pH: 8.1,
      turbidity: 2.5
    },

    controls: {
      fieldBlank: true,
      extractionBlank: true,
      PCR_negative: true
    }
  },

  laboratory: {
    extractionMethod: "DNeasy PowerWater Kit (Qiagen)",
    extractionDate: new Date("2024-06-20"),
    DNAConcentration: 12.5,
    260_280_ratio: 1.85,

    PCR: {
      target: "COI",
      primers: {
        forward: "mlCOIintF (GGWACWGGWTGAACWGTWTAYCCYCC)",
        reverse: "jgHCO2198 (TAIACYTCIGGRTGICCRAARAAYCA)"
      },
      cycles: 35,
      annealingTemp: 46,
      replicates: 3
    },

    sequencing: {
      platform: "Illumina MiSeq",
      readType: "paired_end",
      readLength: 300,
      indexing: "dual",
      runID: "MiSeq_2024_06_Run145"
    }
  },

  bioinformatics: {
    rawReads: 245678,
    qualityFilteredReads: 198234,
    mergedReads: 187456,
    clusteredOTUs: 342,

    pipeline: "QIIME2",
    version: "2024.2",
    steps: [
      "demultiplex",
      "quality_filter_q30",
      "denoise_dada2",
      "merge_paired_ends",
      "cluster_97_percent",
      "chimera_removal",
      "taxonomic_assignment"
    ],

    taxonomicAssignment: {
      method: "BLAST",
      database: "MIDORI2",
      databaseVersion: "GB250",
      confidenceThreshold: 0.97
    }
  },

  results: [
    {
      OTU_ID: "OTU_0001",
      sequence: "ATGGCACACCTCCGAACCCTTTAC...",  // Truncated
      readCount: 15234,
      relativeAbundance: 0.081,

      taxonomy: {
        kingdom: "Animalia",
        phylum: "Chordata",
        class: "Actinopterygii",
        order: "Perciformes",
        family: "Acanthuridae",
        genus: "Acanthurus",
        species: "triostegus",
        confidence: 0.99
      },

      matchQuality: {
        similarity: 99.2,
        coverage: 100,
        eValue: 0,
        topHit: "Acanthurus triostegus"
      },

      annotation: {
        trophicLevel: "herbivore",
        habitat: "coral_reef",
        invasiveStatus: "native",
        conservationStatus: "Least Concern"
      }
    },
    // ... 341 more OTUs detected
  ]
};
```

### Whole Genome Sequencing

For detailed evolutionary studies and population genomics:

```typescript
interface GenomeAssembly {
  assemblyID: string;           // GenBank assembly accession
  aphiaID: number;
  scientificName: string;

  specimen: {
    catalogNumber: string;
    institution: string;
    sex?: string;
    tissue: string;
    locality: string;
  };

  sequencing: {
    strategy: "whole_genome_shotgun" | "hierarchical" | "hybrid";
    platform: string[];         // ["Illumina NovaSeq", "PacBio HiFi"]
    coverage: number;           // Fold coverage (e.g., 100x)
    readLength: number;         // Base pairs
    insertSize: number;         // For paired-end libraries
    totalBases: number;         // Gigabases sequenced
  };

  assembly: {
    assembler: string;          // "SPAdes", "Canu", "FALCON"
    version: string;
    assemblyDate: Date;

    statistics: {
      genomeSize: number;       // Base pairs
      numberOfContigs: number;
      numberOfScaffolds: number;
      N50: number;              // Scaffold N50 length
      L50: number;              // Number of scaffolds in N50
      longestScaffold: number;  // Base pairs
      GC_content: number;       // Percentage
      completeness: number;     // BUSCO score (0-100%)
    };

    chromosomes?: {
      count: number;
      names: string[];
      lengths: number[];
    };
  };

  annotation: {
    method: string;             // "MAKER", "BRAKER", "GeneMark"
    geneCount: number;
    proteinCodingGenes: number;
    tRNA_genes: number;
    rRNA_genes: number;

    functionalAnnotation: {
      GOterms: number;          // Gene Ontology terms
      KOterms: number;          // KEGG Orthology
      InterProDomains: number;
    };
  };

  quality: {
    completeness: number;       // BUSCO complete (%)
    contamination: number;      // Percentage
    heterozygosity: number;     // Percentage
  };

  availability: {
    genBankAccession: string;
    SRAAccession: string;       // Raw reads
    publicationDOI?: string;
    assemblyFTP: string;
    annotationFTP: string;
  };
}

// Example: Crown-of-thorns starfish genome
const cotsGenome: GenomeAssembly = {
  assemblyID: "GCA_902459465.1",
  aphiaID: 213276,
  scientificName: "Acanthaster planci",

  specimen: {
    catalogNumber: "AIMS_COTs_2018_001",
    institution: "Australian Institute of Marine Science",
    sex: "female",
    tissue: "gonad",
    locality: "Great Barrier Reef, Australia"
  },

  sequencing: {
    strategy: "hybrid",
    platform: ["Illumina NovaSeq", "PacBio Sequel II"],
    coverage: 120,
    readLength: 15000,          // PacBio HiFi average
    insertSize: 500,
    totalBases: 50              // 50 Gb sequenced
  },

  assembly: {
    assembler: "Canu",
    version: "2.1",
    assemblyDate: new Date("2020-03-15"),

    statistics: {
      genomeSize: 420000000,    // 420 Mb
      numberOfContigs: 1245,
      numberOfScaffolds: 856,
      N50: 2500000,             // 2.5 Mb
      L50: 52,
      longestScaffold: 15700000, // 15.7 Mb
      GC_content: 37.8,
      completeness: 96.5        // BUSCO
    }
  },

  annotation: {
    method: "MAKER",
    geneCount: 23456,
    proteinCodingGenes: 22134,
    tRNA_genes: 456,
    rRNA_genes: 89,

    functionalAnnotation: {
      GOterms: 18765,
      KOterms: 6543,
      InterProDomains: 12345
    }
  },

  quality: {
    completeness: 96.5,
    contamination: 0.8,
    heterozygosity: 1.2
  },

  availability: {
    genBankAccession: "GCA_902459465.1",
    SRAAccession: "SRR10123456",
    publicationDOI: "10.1038/s41586-020-example",
    assemblyFTP: "ftp://ftp.ncbi.nlm.nih.gov/genomes/...",
    annotationFTP: "ftp://ftp.ncbi.nlm.nih.gov/genomes/..."
  }
};
```

### Genomic Data Quality Control

```typescript
class GenomicQualityControl {

  // Check for contamination from other species
  checkContamination(assembly: GenomeAssembly): {
    status: "pass" | "warning" | "fail";
    issues: string[];
  } {
    const issues: string[] = [];

    if (assembly.quality.contamination > 5) {
      issues.push(`High contamination: ${assembly.quality.contamination}%`);
      return { status: "fail", issues };
    }

    if (assembly.quality.contamination > 2) {
      issues.push(`Moderate contamination: ${assembly.quality.contamination}%`);
      return { status: "warning", issues };
    }

    return { status: "pass", issues: [] };
  }

  // Assess assembly completeness using BUSCO
  assessCompleteness(buscoScore: number): {
    quality: "excellent" | "good" | "fair" | "poor";
    recommendation: string;
  } {
    if (buscoScore >= 95) {
      return {
        quality: "excellent",
        recommendation: "Assembly suitable for publication and detailed analysis"
      };
    } else if (buscoScore >= 90) {
      return {
        quality: "good",
        recommendation: "Assembly acceptable for most analyses"
      };
    } else if (buscoScore >= 80) {
      return {
        quality: "fair",
        recommendation: "Consider additional sequencing or assembly optimization"
      };
    } else {
      return {
        quality: "poor",
        recommendation: "Assembly incomplete; requires additional work"
      };
    }
  }

  // Validate barcode sequence quality
  validateBarcodeQuality(record: DNABarcodeRecord): {
    valid: boolean;
    issues: string[];
  } {
    const issues: string[] = [];

    // Check sequence length
    if (record.sequence.sequenceLength < 500) {
      issues.push(`Short sequence: ${record.sequence.sequenceLength}bp (minimum 500bp recommended)`);
    }

    // Check ambiguous bases
    const ambiguityPercent = (record.sequence.ambiguousBases / record.sequence.sequenceLength) * 100;
    if (ambiguityPercent > 2) {
      issues.push(`High ambiguity: ${ambiguityPercent.toFixed(1)}% (maximum 2% recommended)`);
    }

    // Check quality scores
    if (record.sequence.quality.averageQuality < 40) {
      issues.push(`Low quality score: ${record.sequence.quality.averageQuality} (minimum 40 recommended)`);
    }

    // Check if trace files available (important for validation)
    if (!record.sequence.quality.traceFileAvailable) {
      issues.push("No trace files available for quality verification");
    }

    return {
      valid: issues.length === 0,
      issues
    };
  }
}
```

### Best Practices for Genomic Data

**1. Deposit in Public Repositories**
- DNA barcodes → BOLD Systems and GenBank
- Raw sequence reads → NCBI SRA, ENA, or DDBJ
- Genome assemblies → GenBank/RefSeq

**2. Link to Physical Specimens**
Include museum catalog numbers and voucher photos.

**3. Provide Complete Metadata**
Locality, depth, environmental conditions, collection methods.

**4. Follow Naming Conventions**
Use standardized gene names (COI, not COX1 or COXI).

**5. Include Quality Metrics**
Phred scores, BUSCO completeness, contamination estimates.

**6. Document Bioinformatics**
Pipeline versions, parameters, reference databases used.

**7. Open Data**
Use CC0 or CC-BY licenses for maximum reusability.

### Philosophy: The Molecular Commons

Genomic data represents a shared resource for all humanity (弘益人間). The entire GenBank database - containing 250+ million sequences - is freely accessible worldwide. This enables:

- Species identification in any country
- Disease outbreak tracking
- Biodiversity conservation
- Climate change research
- Drug discovery from marine organisms

When you deposit a sequence, you contribute to this global commons. When you download a sequence, you benefit from millions of hours of work by researchers worldwide. Use it wisely, share freely, and contribute back.

---

**Next Chapter:** Ecosystem Modeling - integrating biological and environmental data to understand and predict marine ecosystem dynamics.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (Hongik Ingan) · Benefit All Humanity
