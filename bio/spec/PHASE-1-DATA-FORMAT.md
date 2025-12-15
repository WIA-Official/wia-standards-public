# WIA Biotech Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12

---

## 1. Overview

### 1.1 Purpose

WIA Biotech Data Format은 바이오테크놀로지 연구 및 산업 데이터의 저장, 전송, 교환을 위한 통합 표준입니다. 이 표준은 유전자 편집, 합성생물학, 단백질 공학 등 다양한 분야의 데이터 상호운용성을 보장하고, 연구 데이터의 재현성을 높이기 위해 설계되었습니다.

### 1.2 Scope

- **In Scope**:
  - 유전자 서열 데이터 (Gene Sequences)
  - CRISPR 편집 실험 데이터
  - 단백질 구조 예측 데이터
  - 합성생물학 부품 데이터
  - 실험 메타데이터

- **Out of Scope** (Phase 1):
  - DNA 컴퓨팅/저장 데이터 (Phase 2+)
  - 실시간 바이오센서 스트리밍 (Phase 3)
  - 바이오파운드리 자동화 통합 (Phase 4)

### 1.3 Design Principles

1. **Interoperability**: 기존 표준(FASTA, FASTQ, SBOL, PDB)과 호환
2. **Extensibility**: 새로운 기술/모달리티 추가 용이
3. **Self-describing**: 메타데이터를 통한 자기 기술적 형식
4. **Traceability**: 실험 재현성을 위한 완전한 추적 가능성
5. **Accessibility**: JSON 기반의 인간 가독성

---

## 2. Data Structure

### 2.1 Top-Level Structure

```
wia-biotech-project/
├── project.json            # Project metadata
├── sequences/
│   ├── sequence.json       # Sequence metadata
│   └── sequence.fasta      # Sequence data (FASTA format)
├── editing/
│   ├── crispr-experiment.json  # CRISPR experiment data
│   ├── guide-rnas.json     # gRNA definitions
│   └── results.json        # Editing results
├── structures/
│   ├── structure.json      # Structure metadata
│   ├── structure.pdb       # PDB format
│   └── confidence.json     # Confidence metrics (pLDDT, PAE)
├── parts/
│   ├── part.json           # BioBrick/SBOL part definition
│   └── assembly.json       # Assembly design
└── experiments/
    ├── experiment.json     # Experiment metadata
    └── results/            # Experimental results
```

### 2.2 File Naming Convention

```
{project}_{type}_{identifier}.{extension}

Examples:
- proj-001_seq_gene-abc.fasta
- proj-001_crispr_exp-001.json
- proj-001_struct_protein-xyz.pdb
```

---

## 3. Project Metadata

### 3.1 project.json

```json
{
  "$schema": "https://wia.live/schemas/bio/project.schema.json",
  "wia_version": "1.0.0",
  "format_version": "1.0.0",
  "project_id": "proj-20251215-001",

  "project_info": {
    "name": "CRISPR Gene Therapy Development",
    "description": "Development of CRISPR-based therapy for genetic disorder X",
    "start_date": "2025-01-01",
    "status": "active",
    "category": "gene_therapy"
  },

  "organization": {
    "name": "Research Institute",
    "department": "Molecular Biology",
    "principal_investigator": "Dr. Kim",
    "contact_email": "contact@example.org"
  },

  "data_files": {
    "sequences": ["sequences/sequence.json"],
    "editing": ["editing/crispr-experiment.json"],
    "structures": ["structures/structure.json"],
    "parts": ["parts/part.json"]
  },

  "ethics": {
    "irb_approval": "IRB-2025-0001",
    "biosafety_level": "BSL-2",
    "gmo_approval": "GMO-2025-001"
  },

  "created_at": "2025-12-15T10:00:00Z",
  "updated_at": "2025-12-15T10:00:00Z"
}
```

### 3.2 Project Categories

| Category | Code | Description |
|----------|------|-------------|
| Gene Therapy | `gene_therapy` | 유전자 치료 연구 |
| Synthetic Biology | `synthetic_biology` | 합성생물학 |
| Protein Engineering | `protein_engineering` | 단백질 공학 |
| Drug Discovery | `drug_discovery` | 신약 개발 |
| Diagnostics | `diagnostics` | 진단 기술 |
| Agriculture | `agriculture` | 농업 바이오텍 |
| Industrial | `industrial` | 산업 바이오텍 |

---

## 4. Sequence Data

### 4.1 sequence.json

```json
{
  "$schema": "https://wia.live/schemas/bio/sequence.schema.json",
  "sequence_id": "seq-001",

  "sequence_info": {
    "name": "Target Gene ABC",
    "description": "Human gene ABC associated with disease X",
    "organism": "Homo sapiens",
    "taxonomy_id": 9606,
    "gene_symbol": "ABC1",
    "gene_id": "12345"
  },

  "sequence_type": "dna",
  "length_bp": 2500,
  "gc_content": 0.52,

  "source": {
    "database": "NCBI GenBank",
    "accession": "NM_001234567",
    "version": "NM_001234567.2",
    "retrieved_date": "2025-12-01"
  },

  "annotations": [
    {
      "feature": "exon",
      "start": 1,
      "end": 500,
      "strand": "+",
      "name": "Exon 1"
    },
    {
      "feature": "cds",
      "start": 100,
      "end": 2400,
      "strand": "+",
      "name": "Coding sequence"
    }
  ],

  "files": {
    "fasta": "sequence.fasta",
    "genbank": "sequence.gb"
  },

  "checksum": {
    "algorithm": "sha256",
    "value": "abc123..."
  }
}
```

### 4.2 Sequence Types

| Type | Code | Description |
|------|------|-------------|
| DNA | `dna` | Deoxyribonucleic acid |
| RNA | `rna` | Ribonucleic acid |
| mRNA | `mrna` | Messenger RNA |
| Protein | `protein` | Amino acid sequence |
| Synthetic | `synthetic` | Synthetic construct |

### 4.3 FASTA File Format

```fasta
>seq-001 Target Gene ABC | Homo sapiens | 2500bp
ATGCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCG
ATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGATCGA
...
```

---

## 5. CRISPR Editing Data

### 5.1 crispr-experiment.json

```json
{
  "$schema": "https://wia.live/schemas/bio/crispr-experiment.schema.json",
  "experiment_id": "crispr-exp-001",

  "experiment_info": {
    "name": "Gene ABC Knockout Study",
    "description": "CRISPR-Cas9 mediated knockout of gene ABC",
    "date": "2025-12-15",
    "experimenter": "Dr. Lee"
  },

  "target": {
    "gene_symbol": "ABC1",
    "gene_id": "12345",
    "organism": "Homo sapiens",
    "cell_line": "HEK293T",
    "sequence_ref": "seq-001"
  },

  "editing_system": {
    "type": "crispr_cas9",
    "cas_variant": "SpCas9",
    "pam_sequence": "NGG",
    "delivery_method": "lipofection"
  },

  "guide_rnas": [
    {
      "grna_id": "grna-001",
      "name": "gRNA-ABC-1",
      "sequence": "ATCGATCGATCGATCGATCG",
      "target_start": 500,
      "target_end": 519,
      "strand": "+",
      "pam": "TGG",
      "off_target_score": 0.95,
      "on_target_score": 0.88
    }
  ],

  "editing_type": "knockout",
  "repair_template": null,

  "protocol": {
    "cas9_concentration_nm": 100,
    "grna_concentration_nm": 200,
    "transfection_reagent": "Lipofectamine 3000",
    "incubation_hours": 48
  },

  "results_ref": "editing/results.json"
}
```

### 5.2 Editing Types

| Type | Code | Description |
|------|------|-------------|
| Knockout | `knockout` | Gene disruption via NHEJ |
| Knock-in | `knockin` | Gene insertion via HDR |
| Base Edit | `base_edit` | Single base conversion |
| Prime Edit | `prime_edit` | Precision editing |
| CRISPRi | `crispri` | Transcription repression |
| CRISPRa | `crispra` | Transcription activation |
| Epigenome Edit | `epigenome` | Epigenetic modification |

### 5.3 CRISPR Systems

| System | Code | PAM | Description |
|--------|------|-----|-------------|
| SpCas9 | `crispr_cas9` | NGG | Standard Cas9 |
| SaCas9 | `crispr_saCas9` | NNGRRT | Smaller Cas9 |
| Cas12a (Cpf1) | `crispr_cas12a` | TTTV | T-rich PAM |
| Cas13 | `crispr_cas13` | - | RNA targeting |
| Base Editor | `base_editor` | NGG | ABE/CBE |
| Prime Editor | `prime_editor` | NGG | pegRNA system |

### 5.4 results.json

```json
{
  "$schema": "https://wia.live/schemas/bio/crispr-results.schema.json",
  "results_id": "results-001",
  "experiment_ref": "crispr-exp-001",

  "analysis_date": "2025-12-16",
  "analysis_method": "NGS",

  "editing_efficiency": {
    "overall_efficiency": 0.78,
    "indel_frequency": 0.75,
    "hdr_frequency": null,
    "wild_type_frequency": 0.22
  },

  "indel_profile": [
    {
      "type": "deletion",
      "size_bp": -1,
      "frequency": 0.35,
      "sequence": "ATCGATCGATCGATCGAT-G"
    },
    {
      "type": "insertion",
      "size_bp": 1,
      "frequency": 0.25,
      "sequence": "ATCGATCGATCGATCGATAG"
    },
    {
      "type": "deletion",
      "size_bp": -5,
      "frequency": 0.15,
      "sequence": "ATCGATCGATCGAT-----G"
    }
  ],

  "off_target_analysis": {
    "method": "GUIDE-seq",
    "sites_detected": 3,
    "sites": [
      {
        "chromosome": "chr3",
        "position": 12345678,
        "mismatches": 3,
        "frequency": 0.002
      }
    ]
  },

  "quality_metrics": {
    "sequencing_depth": 10000,
    "mapping_rate": 0.98,
    "confidence_score": 0.95
  },

  "files": {
    "raw_fastq": "results/raw.fastq.gz",
    "aligned_bam": "results/aligned.bam",
    "analysis_report": "results/report.pdf"
  }
}
```

---

## 6. Protein Structure Data

### 6.1 structure.json

```json
{
  "$schema": "https://wia.live/schemas/bio/structure.schema.json",
  "structure_id": "struct-001",

  "structure_info": {
    "name": "ABC1 Protein",
    "description": "3D structure of human ABC1 protein",
    "uniprot_id": "P12345",
    "pdb_id": null
  },

  "prediction": {
    "method": "alphafold",
    "version": "AlphaFold 2.3",
    "date": "2025-12-15",
    "source": "local_prediction"
  },

  "protein": {
    "name": "ABC1",
    "organism": "Homo sapiens",
    "length_aa": 500,
    "molecular_weight_da": 55000,
    "sequence_ref": "seq-002"
  },

  "quality": {
    "mean_plddt": 85.5,
    "confident_residues_pct": 0.78,
    "disordered_residues_pct": 0.12
  },

  "domains": [
    {
      "name": "N-terminal domain",
      "start": 1,
      "end": 150,
      "mean_plddt": 92.3,
      "pfam_id": "PF00001"
    },
    {
      "name": "Catalytic domain",
      "start": 151,
      "end": 400,
      "mean_plddt": 88.7,
      "pfam_id": "PF00002"
    }
  ],

  "files": {
    "pdb": "structure.pdb",
    "mmcif": "structure.cif",
    "confidence": "confidence.json"
  }
}
```

### 6.2 confidence.json (AlphaFold Metrics)

```json
{
  "$schema": "https://wia.live/schemas/bio/structure-confidence.schema.json",
  "structure_ref": "struct-001",

  "plddt": {
    "description": "predicted Local Distance Difference Test",
    "range": [0, 100],
    "values": [92.1, 89.5, 88.2, 85.0, ...]
  },

  "pae": {
    "description": "Predicted Aligned Error",
    "unit": "angstrom",
    "max_value": 31.75,
    "matrix": [
      [0.5, 1.2, 2.3, ...],
      [1.2, 0.4, 1.8, ...],
      ...
    ]
  },

  "interpretation": {
    "high_confidence_regions": [
      {"start": 1, "end": 150, "mean_plddt": 92.3},
      {"start": 200, "end": 350, "mean_plddt": 90.1}
    ],
    "low_confidence_regions": [
      {"start": 151, "end": 199, "mean_plddt": 45.2, "likely_disordered": true}
    ]
  }
}
```

### 6.3 Prediction Methods

| Method | Code | Description |
|--------|------|-------------|
| AlphaFold | `alphafold` | DeepMind AI prediction |
| AlphaFold3 | `alphafold3` | Complex prediction |
| RoseTTAFold | `rosettafold` | Baker Lab prediction |
| ESMFold | `esmfold` | Meta AI prediction |
| Experimental | `experimental` | X-ray, NMR, Cryo-EM |
| Homology | `homology` | Template-based modeling |

---

## 7. Synthetic Biology Parts

### 7.1 part.json

```json
{
  "$schema": "https://wia.live/schemas/bio/part.schema.json",
  "part_id": "part-001",

  "part_info": {
    "name": "Strong Constitutive Promoter",
    "description": "High-expression promoter for E. coli",
    "short_name": "pConst-Strong",
    "part_type": "promoter"
  },

  "registry": {
    "source": "iGEM Registry",
    "registry_id": "BBa_J23100",
    "url": "http://parts.igem.org/Part:BBa_J23100"
  },

  "sequence": {
    "dna": "TTGACGGCTAGCTCAGTCCTAGGTACAGTGCTAGC",
    "length_bp": 35
  },

  "assembly": {
    "standard": "biobrick",
    "prefix": "GAATTCGCGGCCGCTTCTAGAG",
    "suffix": "TACTAGTAGCGGCCGCTGCAG",
    "compatible_standards": ["biobrick", "golden_gate", "gibson"]
  },

  "characterization": {
    "organism": "Escherichia coli",
    "strain": "DH5alpha",
    "relative_strength": 1.0,
    "measurement_method": "fluorescence",
    "reference_part": "BBa_J23100"
  },

  "performance": {
    "expression_level": "high",
    "reliability": 0.95,
    "tested_conditions": ["LB", "M9", "37C", "30C"]
  },

  "files": {
    "genbank": "part.gb",
    "sbol": "part.sbol"
  }
}
```

### 7.2 Part Types

| Type | Code | Description |
|------|------|-------------|
| Promoter | `promoter` | Transcription initiation |
| RBS | `rbs` | Ribosome binding site |
| CDS | `cds` | Coding sequence |
| Terminator | `terminator` | Transcription termination |
| Operator | `operator` | Regulatory element |
| Reporter | `reporter` | GFP, RFP, etc. |
| Origin | `origin` | Replication origin |
| Resistance | `resistance` | Antibiotic resistance |
| Tag | `tag` | Protein tag (His, FLAG) |
| Composite | `composite` | Multi-part device |

### 7.3 Assembly Standards

| Standard | Code | Description |
|----------|------|-------------|
| BioBrick | `biobrick` | iGEM standard |
| BioBrick RFC10 | `biobrick_rfc10` | Standard BioBrick |
| Golden Gate | `golden_gate` | Type IIs assembly |
| Gibson | `gibson` | Isothermal assembly |
| MoClo | `moclo` | Modular Cloning |
| SEVA | `seva` | European Vector Standard |
| BASIC | `basic` | Idempotent cloning |

### 7.4 assembly.json

```json
{
  "$schema": "https://wia.live/schemas/bio/assembly.schema.json",
  "assembly_id": "asm-001",

  "assembly_info": {
    "name": "GFP Expression Cassette",
    "description": "Constitutive GFP expression construct",
    "assembly_date": "2025-12-15"
  },

  "assembly_method": "golden_gate",
  "backbone": {
    "name": "pSEVA331",
    "registry_id": "SEVA-331",
    "selection_marker": "chloramphenicol"
  },

  "parts": [
    {
      "position": 1,
      "part_ref": "part-001",
      "name": "pConst-Strong",
      "type": "promoter"
    },
    {
      "position": 2,
      "part_ref": "part-002",
      "name": "RBS-Strong",
      "type": "rbs"
    },
    {
      "position": 3,
      "part_ref": "part-003",
      "name": "GFP",
      "type": "cds"
    },
    {
      "position": 4,
      "part_ref": "part-004",
      "name": "dblTerm",
      "type": "terminator"
    }
  ],

  "final_construct": {
    "length_bp": 2500,
    "sequence_ref": "seq-asm-001"
  },

  "verification": {
    "method": "sanger_sequencing",
    "status": "verified",
    "date": "2025-12-16"
  }
}
```

---

## 8. Experiment Metadata

### 8.1 experiment.json

```json
{
  "$schema": "https://wia.live/schemas/bio/experiment.schema.json",
  "experiment_id": "exp-001",

  "experiment_info": {
    "name": "Gene Expression Analysis",
    "description": "RNA-seq analysis of edited cells",
    "type": "transcriptomics",
    "date": "2025-12-15"
  },

  "protocol": {
    "name": "Standard RNA-seq Protocol v2",
    "version": "2.0",
    "steps": [
      "RNA extraction (TRIzol)",
      "Quality control (Bioanalyzer)",
      "Library preparation (Illumina TruSeq)",
      "Sequencing (NovaSeq 6000)"
    ]
  },

  "samples": [
    {
      "sample_id": "sample-001",
      "name": "Control",
      "condition": "wild_type",
      "replicates": 3
    },
    {
      "sample_id": "sample-002",
      "name": "CRISPR Edited",
      "condition": "abc_knockout",
      "replicates": 3
    }
  ],

  "equipment": {
    "sequencer": "Illumina NovaSeq 6000",
    "read_length": 150,
    "read_type": "paired_end"
  },

  "analysis": {
    "pipeline": "nf-core/rnaseq",
    "version": "3.12.0",
    "reference_genome": "GRCh38",
    "annotation": "GENCODE v44"
  },

  "results_location": "experiments/results/"
}
```

### 8.2 Experiment Types

| Type | Code | Description |
|------|------|-------------|
| Transcriptomics | `transcriptomics` | RNA-seq, microarray |
| Proteomics | `proteomics` | Mass spectrometry |
| Genomics | `genomics` | WGS, WES |
| Epigenomics | `epigenomics` | ChIP-seq, ATAC-seq |
| Metabolomics | `metabolomics` | Metabolite profiling |
| Cell Assay | `cell_assay` | Cell-based experiments |
| Biochemistry | `biochemistry` | In vitro assays |

---

## 9. Validation Rules

### 9.1 Required Fields

| File | Required Fields |
|------|-----------------|
| project.json | wia_version, project_id, project_info.name |
| sequence.json | sequence_id, sequence_type, length_bp |
| crispr-experiment.json | experiment_id, target.gene_symbol, editing_system.type, guide_rnas |
| structure.json | structure_id, prediction.method, protein.name |
| part.json | part_id, part_info.part_type, sequence.dna |

### 9.2 Value Constraints

```yaml
sequence_type:
  type: string
  enum: [dna, rna, mrna, protein, synthetic]

editing_type:
  type: string
  enum: [knockout, knockin, base_edit, prime_edit, crispri, crispra, epigenome]

crispr_system:
  type: string
  enum: [crispr_cas9, crispr_saCas9, crispr_cas12a, crispr_cas13, base_editor, prime_editor]

part_type:
  type: string
  enum: [promoter, rbs, cds, terminator, operator, reporter, origin, resistance, tag, composite]

prediction_method:
  type: string
  enum: [alphafold, alphafold3, rosettafold, esmfold, experimental, homology]

plddt_score:
  type: number
  minimum: 0
  maximum: 100
```

---

## 10. Compatibility

### 10.1 Import from FASTA

```python
# Pseudocode for FASTA to WIA-Biotech conversion
def convert_fasta_to_wia(fasta_file):
    records = parse_fasta(fasta_file)

    sequences = []
    for record in records:
        seq = {
            "sequence_id": generate_id(),
            "sequence_info": {
                "name": record.description,
                "description": record.description
            },
            "sequence_type": infer_type(record.sequence),
            "length_bp": len(record.sequence),
            "gc_content": calculate_gc(record.sequence) if is_dna(record.sequence) else None,
            "files": {
                "fasta": fasta_file
            }
        }
        sequences.append(seq)

    return sequences
```

### 10.2 Export to SBOL

```python
# Pseudocode for WIA-Biotech to SBOL conversion
def convert_wia_to_sbol(part_json, sbol_file):
    part = load_json(part_json)

    # Create SBOL document
    doc = sbol.Document()

    # Create component
    component = doc.createComponent(
        displayId=part['part_id'],
        name=part['part_info']['name'],
        type=map_part_type_to_sbol(part['part_info']['part_type'])
    )

    # Add sequence
    component.addSequence(
        encoding=sbol.IUPAC_DNA,
        elements=part['sequence']['dna']
    )

    doc.write(sbol_file)
    return sbol_file
```

### 10.3 Import from AlphaFold DB

```python
# Pseudocode for AlphaFold to WIA-Biotech conversion
def convert_alphafold_to_wia(uniprot_id):
    # Fetch from AlphaFold DB
    pdb_data = fetch_alphafold_pdb(uniprot_id)
    confidence_data = fetch_alphafold_confidence(uniprot_id)

    structure = {
        "structure_id": f"struct-{uniprot_id}",
        "structure_info": {
            "name": get_protein_name(uniprot_id),
            "uniprot_id": uniprot_id
        },
        "prediction": {
            "method": "alphafold",
            "source": "alphafold_db"
        },
        "quality": {
            "mean_plddt": calculate_mean_plddt(confidence_data)
        }
    }

    return structure, pdb_data, confidence_data
```

---

## 11. Examples

### 11.1 Minimal CRISPR Experiment

```json
// crispr-experiment.json (minimal)
{
  "wia_version": "1.0.0",
  "experiment_id": "crispr-001",
  "target": {
    "gene_symbol": "ABC1"
  },
  "editing_system": {
    "type": "crispr_cas9"
  },
  "guide_rnas": [
    {
      "grna_id": "grna-001",
      "sequence": "ATCGATCGATCGATCGATCG"
    }
  ]
}
```

### 11.2 Full BioBrick Part

See full example in `/bio/examples/biobrick-gfp/`

### 11.3 AlphaFold Structure Import

See full example in `/bio/examples/alphafold-import/`

---

## 12. Schema Files

All JSON Schema files are available at:

- `schemas/project.schema.json`
- `schemas/sequence.schema.json`
- `schemas/crispr-experiment.schema.json`
- `schemas/crispr-results.schema.json`
- `schemas/structure.schema.json`
- `schemas/structure-confidence.schema.json`
- `schemas/part.schema.json`
- `schemas/assembly.schema.json`
- `schemas/experiment.schema.json`

Online: `https://wia.live/schemas/bio/`

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12 | Initial specification |

---

## 14. Acknowledgments

This specification is informed by existing standards including:
- FASTA/FASTQ (Sequence data)
- GenBank/EMBL (Annotated sequences)
- SBOL (Synthetic Biology Open Language)
- PDB/mmCIF (Protein structures)
- BioBricks (Modular DNA parts)
- AlphaFold (Structure prediction)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12
**Author**: WIA Biotech Working Group

---

弘益人間 - *Benefit All Humanity*
