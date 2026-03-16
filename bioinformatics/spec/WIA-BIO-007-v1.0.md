# WIA-BIO-007: Bioinformatics Specification v1.0

> **Standard ID:** WIA-BIO-007
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Bioinformatics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Sequence Analysis Algorithms](#2-sequence-analysis-algorithms)
3. [Data Formats and Standards](#3-data-formats-and-standards)
4. [Database Integration](#4-database-integration)
5. [Phylogenetic Analysis](#5-phylogenetic-analysis)
6. [Pathway Analysis](#6-pathway-analysis)
7. [Machine Learning Integration](#7-machine-learning-integration)
8. [Pipeline Architecture](#8-pipeline-architecture)
9. [Reproducibility Requirements](#9-reproducibility-requirements)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the computational framework for bioinformatics analysis, including sequence alignment, database searching, phylogenetic inference, pathway analysis, and machine learning applications.

### 1.2 Scope

The standard covers:
- Sequence alignment algorithms (global, local, multiple)
- Database search methods (BLAST, FASTA, HMMER)
- Phylogenetic tree construction and analysis
- Pathway enrichment and network analysis
- Machine learning for biological predictions
- Data format specifications and conversions
- Reproducibility and workflow management

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize bioinformatics by providing accessible, reproducible, and scientifically rigorous computational tools that advance human health and biological understanding.

### 1.4 Terminology

- **Sequence Alignment**: Arrangement of sequences to identify regions of similarity
- **E-value**: Expected value; probability of finding alignment by chance
- **Phylogeny**: Evolutionary relationships among species or genes
- **Ortholog**: Genes in different species evolved from common ancestor
- **Pathway**: Series of molecular interactions leading to biological outcome
- **FASTA**: Text-based format for nucleotide/protein sequences

---

## 2. Sequence Analysis Algorithms

### 2.1 Global Alignment (Needleman-Wunsch)

#### 2.1.1 Algorithm

Dynamic programming algorithm for optimal global alignment:

```
Initialization:
F(0,0) = 0
F(i,0) = γ × i  for i > 0
F(0,j) = γ × j  for j > 0

Recursion:
F(i,j) = max {
  F(i-1,j-1) + σ(xi, yj),    # Match/mismatch
  F(i-1,j) + γ,              # Deletion
  F(i,j-1) + γ               # Insertion
}

Traceback: Start at F(m,n), follow maximum scores to F(0,0)
```

Where:
- `F(i,j)` = Score matrix at position (i,j)
- `σ(xi, yj)` = Substitution score from scoring matrix
- `γ` = Gap penalty (negative value)
- `m, n` = Sequence lengths

#### 2.1.2 Scoring Matrices

**BLOSUM62** (BLOcks SUbstitution Matrix):
```
   A  R  N  D  C  Q  E  G  H  I  L  K  M  F  P  S  T  W  Y  V
A  4 -1 -2 -2  0 -1 -1  0 -2 -1 -1 -1 -1 -2 -1  1  0 -3 -2  0
R -1  5  0 -2 -3  1  0 -2  0 -3 -2  2 -1 -3 -2 -1 -1 -3 -2 -3
...
```

**PAM250** (Point Accepted Mutation):
- Used for distant evolutionary relationships
- Higher numbers = greater evolutionary distance

#### 2.1.3 Gap Penalties

**Linear gap penalty**:
```
γ(k) = -gₒ × k
```

**Affine gap penalty** (preferred):
```
γ(k) = -gₒ - gₑ × (k - 1)
```

Where:
- `gₒ` = Gap opening penalty (typically -10)
- `gₑ` = Gap extension penalty (typically -0.5)
- `k` = Gap length

### 2.2 Local Alignment (Smith-Waterman)

#### 2.2.1 Algorithm

Modification of Needleman-Wunsch for local alignment:

```
Initialization:
F(0,j) = 0  for all j
F(i,0) = 0  for all i

Recursion:
F(i,j) = max {
  0,                         # New alignment
  F(i-1,j-1) + σ(xi, yj),   # Match/mismatch
  F(i-1,j) + γ,             # Deletion
  F(i,j-1) + γ              # Insertion
}

Traceback: Start at maximum F(i,j), end when reaching 0
```

#### 2.2.2 Performance Metrics

**Identity**:
```
Identity = (Number of identical residues / Alignment length) × 100%
```

**Similarity**:
```
Similarity = (Number of similar residues / Alignment length) × 100%
```

**Coverage**:
```
Query Coverage = (Aligned query length / Total query length) × 100%
Subject Coverage = (Aligned subject length / Total subject length) × 100%
```

### 2.3 BLAST (Basic Local Alignment Search Tool)

#### 2.3.1 Algorithm Steps

1. **Word Finding**: Identify short matches (words) of length W
2. **Word Extension**: Extend words to high-scoring segment pairs (HSPs)
3. **Evaluation**: Calculate E-value for statistical significance

#### 2.3.2 E-value Calculation

```
E = K × m × n × e^(-λS)
```

Where:
- `K`, `λ` = Statistical parameters (depend on scoring system)
- `m` = Query sequence length
- `n` = Total database length
- `S` = Alignment score

For `E < 0.001`: Highly significant
For `E < 0.01`: Significant
For `E > 0.1`: Not significant

#### 2.3.3 Bit Score

```
S' = (λS - ln K) / ln 2
```

Where:
- `S'` = Bit score (normalized, database-independent)
- `S` = Raw alignment score

#### 2.3.4 BLAST Variants

| Program | Query | Database | Purpose |
|---------|-------|----------|---------|
| BLASTN | Nucleotide | Nucleotide | DNA vs DNA |
| BLASTP | Protein | Protein | Protein vs protein |
| BLASTX | Nucleotide (6-frame) | Protein | DNA vs protein |
| TBLASTN | Protein | Nucleotide (6-frame) | Protein vs DNA |
| TBLASTX | Nucleotide (6-frame) | Nucleotide (6-frame) | DNA vs DNA (protein level) |

### 2.4 Multiple Sequence Alignment

#### 2.4.1 Progressive Alignment (ClustalW/MUSCLE)

```
1. Calculate pairwise distances between all sequences
2. Build guide tree from distance matrix
3. Progressively align sequences following tree topology
4. Refine alignment iteratively
```

#### 2.4.2 Sum-of-Pairs Score

```
SP = ∑∑ σ(sᵢⱼ, sₖⱼ)
     i<k j
```

Where:
- `sᵢⱼ` = Character at position j in sequence i
- Sum over all sequence pairs and positions

#### 2.4.3 Profile-Profile Alignment

Used in MUSCLE and MAFFT:
```
Score(profile₁, profile₂) = ∑ fᵢ(a) × fⱼ(b) × σ(a,b)
                            a,b
```

Where:
- `fᵢ(a)` = Frequency of residue a at position i in profile 1
- `fⱼ(b)` = Frequency of residue b at position j in profile 2

---

## 3. Data Formats and Standards

### 3.1 FASTA Format

```
>sp|P12345|PROT_HUMAN Protein name OS=Homo sapiens GN=GENE PE=1 SV=1
MKFLKFSLLTAVLLSVVFAFSSCGDDDDTGYLPPSQAIQDLLKRGGIVDQCCTSICSLYQ
LENYCNFVNQHLCGSHLVEALYLVCGERGFFYTPKA
```

**Header format**:
- `>` = Header marker
- `sp|P12345|PROT_HUMAN` = Database|Accession|Entry name
- Followed by description

**Sequence**:
- Single-letter amino acid or nucleotide codes
- Typically 60-80 characters per line
- May include line breaks (ignored in parsing)

### 3.2 GenBank Format

```
LOCUS       NM_001301717            2891 bp    mRNA    linear   PRI 15-JAN-2024
DEFINITION  Homo sapiens breast cancer 1 (BRCA1), transcript variant 1, mRNA.
ACCESSION   NM_001301717
VERSION     NM_001301717.2
KEYWORDS    RefSeq.
SOURCE      Homo sapiens (human)
  ORGANISM  Homo sapiens
            Eukaryota; Metazoa; Chordata; Craniata; Vertebrata; Euteleostomi;
            Mammalia; Eutheria; Euarchontoglires; Primates; Haplorrhini;
            Catarrhini; Hominidae; Homo.
FEATURES             Location/Qualifiers
     source          1..2891
                     /organism="Homo sapiens"
                     /mol_type="mRNA"
                     /db_xref="taxon:9606"
     gene            1..2891
                     /gene="BRCA1"
ORIGIN
        1 atggatttat ctgctcttcg cg...
```

### 3.3 PDB Format (Protein Data Bank)

```
HEADER    TRANSFERASE                             04-JAN-24   8RAI
TITLE     CRYSTAL STRUCTURE OF HUMAN CDK2
ATOM      1  N   MET A   1      20.154  29.699   5.276  1.00 49.05           N
ATOM      2  CA  MET A   1      21.413  30.430   5.360  1.00 43.14           C
ATOM      3  C   MET A   1      21.296  31.546   6.394  1.00 35.88           C
...
```

**Fields**:
- Record name, atom serial, atom name, residue name, chain ID
- Coordinates (x, y, z), occupancy, temperature factor, element

### 3.4 VCF Format (Variant Call Format)

```
##fileformat=VCFv4.2
##reference=GRCh38
#CHROM  POS     ID      REF  ALT     QUAL   FILTER  INFO           FORMAT  SAMPLE1
chr1    12345   rs123   A    G       99     PASS    DP=100;AF=0.5  GT:DP   0/1:50
chr1    67890   .       TC   T       85     PASS    DP=80;AF=0.3   GT:DP   0/0:80
```

**Fields**:
- CHROM: Chromosome
- POS: Position
- REF: Reference allele
- ALT: Alternative allele
- QUAL: Quality score
- FORMAT: Genotype format

---

## 4. Database Integration

### 4.1 UniProt (Protein Sequences)

**REST API**:
```
GET https://rest.uniprot.org/uniprotkb/P12345.fasta
GET https://rest.uniprot.org/uniprotkb/search?query=gene:BRCA1+AND+organism_id:9606
```

**Response format**: FASTA, XML, JSON, TSV

### 4.2 NCBI Databases

#### 4.2.1 E-utilities API

```
# Search
https://eutils.ncbi.nlm.nih.gov/entrez/eutils/esearch.fcgi?db=protein&term=BRCA1

# Fetch
https://eutils.ncbi.nlm.nih.gov/entrez/eutils/efetch.fcgi?db=protein&id=NP_009225&rettype=fasta
```

#### 4.2.2 BLAST API

```
# Submit BLAST search
POST https://blast.ncbi.nlm.nih.gov/Blast.cgi
  CMD=Put&PROGRAM=blastp&DATABASE=nr&QUERY=MKFLKFSLLT...

# Check status
GET https://blast.ncbi.nlm.nih.gov/Blast.cgi?CMD=Get&RID={request_id}

# Retrieve results
GET https://blast.ncbi.nlm.nih.gov/Blast.cgi?CMD=Get&RID={request_id}&FORMAT_TYPE=XML
```

### 4.3 Ensembl (Genomic Data)

**REST API**:
```
GET https://rest.ensembl.org/sequence/id/ENSG00000139618?type=genomic
GET https://rest.ensembl.org/lookup/id/ENSG00000139618?expand=1
```

### 4.4 PDB (Protein Structures)

```
# Download structure
GET https://files.rcsb.org/download/1ABC.pdb
GET https://files.rcsb.org/download/1ABC.cif

# Search API
GET https://search.rcsb.org/rcsbsearch/v2/query?json={query}
```

---

## 5. Phylogenetic Analysis

### 5.1 Distance-Based Methods

#### 5.1.1 Jukes-Cantor Distance

```
d = -¾ ln(1 - 4p/3)
```

Where:
- `d` = Evolutionary distance
- `p` = Proportion of sites that differ

#### 5.1.2 Kimura 2-Parameter Distance

```
d = -½ ln[(1 - 2P - Q) × √(1 - 2Q)]
```

Where:
- `P` = Transition frequency
- `Q` = Transversion frequency

#### 5.1.3 Neighbor-Joining Algorithm

```
1. Calculate distance matrix D
2. For each pair (i,j), compute:
   Sᵢⱼ = dᵢⱼ - (rᵢ + rⱼ)
   where rᵢ = ∑ₖ dᵢₖ / (n - 2)
3. Join pair with minimum Sᵢⱼ
4. Compute new distances to joined node
5. Repeat until tree is complete
```

### 5.2 Maximum Likelihood

#### 5.2.1 Likelihood Calculation

```
L = P(Data | Tree, Model)
  = ∏ᵢ P(columnᵢ | Tree, Model)
```

#### 5.2.2 Substitution Models

**JC69 (Jukes-Cantor)**:
```
P(i→j|t) = {
  ¼ + ¾e^(-4μt/3)    if i = j
  ¼ - ¼e^(-4μt/3)    if i ≠ j
}
```

**HKY85** (Hasegawa-Kishino-Yano):
- Allows different base frequencies
- Transition/transversion ratio (κ)

**GTR** (General Time Reversible):
- Most complex model
- 6 substitution rates + 4 base frequencies

### 5.3 Bootstrap Analysis

```
1. Resample alignment columns with replacement
2. Build tree from resampled alignment
3. Repeat B times (typically B = 1000)
4. Calculate support: frequency each branch appears
```

**Bootstrap support interpretation**:
- ≥ 95%: Very high support
- 70-95%: Moderate to high support
- < 70%: Low support

---

## 6. Pathway Analysis

### 6.1 Pathway Enrichment

#### 6.1.1 Fisher's Exact Test

```
           In pathway  Not in pathway
Selected      k            n - k           n
Not selected  m - k        N - n - m + k   N - n
              m            N - m           N
```

**P-value**:
```
P = (m choose k) × (N-m choose n-k) / (N choose n)
```

#### 6.1.2 Hypergeometric Test

```
P(X = k) = (m choose k) × (N-m choose n-k) / (N choose n)
```

Where:
- `N` = Total genes in background
- `m` = Genes in pathway
- `n` = Genes in query set
- `k` = Overlap between query and pathway

#### 6.1.3 Gene Set Enrichment Analysis (GSEA)

```
ES = max|∑(hits) 1/Nₕ - ∑(misses) 1/Nₘ|
```

Where:
- `ES` = Enrichment score
- `Nₕ` = Number of hits
- `Nₘ` = Number of misses

### 6.2 Network Analysis

#### 6.2.1 Centrality Measures

**Degree centrality**:
```
Cᴅ(v) = deg(v) / (n - 1)
```

**Betweenness centrality**:
```
Cʙ(v) = ∑ₛ≠ᵥ≠ₜ (σₛₜ(v) / σₛₜ)
```

Where:
- `σₛₜ` = Number of shortest paths from s to t
- `σₛₜ(v)` = Number passing through v

**Closeness centrality**:
```
Cᴄ(v) = (n - 1) / ∑ᵤ d(v,u)
```

### 6.3 Pathway Databases

| Database | Type | Coverage |
|----------|------|----------|
| KEGG | Metabolic, signaling | Comprehensive |
| Reactome | Biological pathways | Human-focused |
| GO | Gene ontology | Hierarchical |
| WikiPathways | Community-curated | Growing |
| BioCyc | Metabolic | Multi-organism |

---

## 7. Machine Learning Integration

### 7.1 Protein Structure Prediction

#### 7.1.1 AlphaFold2 Architecture

```
Input: Amino acid sequence
↓
MSA Generation (Multiple Sequence Alignment)
↓
Template Search (PDB)
↓
Evoformer Network (48 blocks)
↓
Structure Module
↓
Output: 3D coordinates + confidence scores (pLDDT)
```

**pLDDT Score** (predicted LDDT):
- > 90: Very high confidence
- 70-90: Confident
- 50-70: Low confidence
- < 50: Very low confidence

#### 7.1.2 Contact Prediction

```
Precision = TP / (TP + FP)
```

Where contacts defined as Cβ-Cβ distance < 8Å

### 7.2 Variant Effect Prediction

#### 7.2.1 SIFT (Sorting Intolerant From Tolerant)

```
SIFT score = (# seqs with observed residue) / (# seqs in MSA)
```

- Score < 0.05: Deleterious
- Score ≥ 0.05: Tolerated

#### 7.2.2 PolyPhen-2

Machine learning classifier using:
- Sequence-based features
- Structure-based features
- Conservation scores

**Prediction**:
- Probably damaging: > 0.85
- Possibly damaging: 0.15-0.85
- Benign: < 0.15

### 7.3 Deep Learning Models

#### 7.3.1 Convolutional Neural Networks

Used for:
- Protein secondary structure prediction
- DNA-protein binding site prediction
- RNA structure prediction

#### 7.3.2 Recurrent Neural Networks

Used for:
- Sequence generation
- Splice site prediction
- Protein function annotation

#### 7.3.3 Transformer Models

**ESM-2** (Evolutionary Scale Modeling):
- Protein language model
- 15B parameters
- Pre-trained on UniRef

**ProtTrans**:
- BERT-based protein model
- Transfer learning for various tasks

---

## 8. Pipeline Architecture

### 8.1 Workflow Management

#### 8.1.1 Nextflow Example

```groovy
process ALIGN {
  input:
    path sequences

  output:
    path 'alignment.fasta'

  script:
  """
  muscle -in ${sequences} -out alignment.fasta
  """
}

process TREE {
  input:
    path alignment

  output:
    path 'tree.nwk'

  script:
  """
  iqtree -s ${alignment} -bb 1000 -nt AUTO
  """
}

workflow {
  sequences_ch = Channel.fromPath('*.fasta')
  alignment_ch = ALIGN(sequences_ch)
  TREE(alignment_ch)
}
```

#### 8.1.2 Snakemake Example

```python
rule all:
    input:
        "results/tree.nwk"

rule align:
    input:
        "data/sequences.fasta"
    output:
        "results/alignment.fasta"
    shell:
        "muscle -in {input} -out {output}"

rule tree:
    input:
        "results/alignment.fasta"
    output:
        "results/tree.nwk"
    shell:
        "iqtree -s {input} -bb 1000"
```

### 8.2 Containerization

#### 8.2.1 Docker Example

```dockerfile
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    muscle \
    iqtree \
    python3 \
    python3-pip

COPY scripts/ /app/scripts/
COPY requirements.txt /app/

RUN pip3 install -r /app/requirements.txt

WORKDIR /app
ENTRYPOINT ["python3", "scripts/pipeline.py"]
```

#### 8.2.2 Singularity Example

```
Bootstrap: docker
From: ubuntu:22.04

%post
    apt-get update
    apt-get install -y muscle iqtree blast+

%runscript
    exec "$@"
```

### 8.3 Cloud Computing

#### 8.3.1 AWS Batch

```yaml
jobDefinition:
  type: container
  image: bioinformatics-pipeline:latest
  vcpus: 8
  memory: 32768
  command:
    - python3
    - pipeline.py
    - --input
    - s3://bucket/input
    - --output
    - s3://bucket/output
```

#### 8.3.2 Google Cloud Life Sciences

```json
{
  "pipeline": {
    "actions": [
      {
        "imageUri": "gcr.io/project/bioinformatics",
        "commands": ["blastp", "-query", "input.fasta"],
        "mounts": [
          {
            "disk": "work",
            "path": "/mnt/work"
          }
        ]
      }
    ]
  }
}
```

---

## 9. Reproducibility Requirements

### 9.1 Version Control

**Required metadata**:
```yaml
analysis:
  name: "Phylogenetic analysis of BRCA1 orthologs"
  version: "1.0.0"
  date: "2025-12-26"
  author: "Jane Doe"

software:
  - name: "MUSCLE"
    version: "5.1"
  - name: "IQ-TREE"
    version: "2.2.0"
  - name: "Python"
    version: "3.11.0"

data:
  - name: "input_sequences.fasta"
    md5: "5d41402abc4b2a76b9719d911017c592"
  - name: "alignment.fasta"
    md5: "7d793037a0760186574b0282f2f435e7"

parameters:
  alignment:
    algorithm: "muscle"
    iterations: 2
  tree:
    model: "GTR+I+G"
    bootstrap: 1000
    seed: 12345
```

### 9.2 Random Seed Management

All stochastic processes must use documented random seeds:
```python
import random
import numpy as np

def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    # For ML frameworks:
    # torch.manual_seed(seed)
    # tf.random.set_seed(seed)
```

### 9.3 Computational Environment

**Conda environment**:
```yaml
name: bioinformatics
channels:
  - bioconda
  - conda-forge
  - defaults
dependencies:
  - python=3.11
  - biopython=1.81
  - muscle=5.1
  - iqtree=2.2.0
  - blast=2.14.0
  - pip:
    - numpy==1.24.0
    - pandas==2.0.0
```

### 9.4 Data Provenance

Track all data transformations:
```json
{
  "provenance": [
    {
      "step": 1,
      "operation": "download",
      "source": "UniProt",
      "accessions": ["P38398", "Q9Y6K9"],
      "timestamp": "2025-12-26T10:00:00Z"
    },
    {
      "step": 2,
      "operation": "align",
      "tool": "muscle",
      "version": "5.1",
      "parameters": {"maxiters": 2},
      "input": "sequences.fasta",
      "output": "alignment.fasta",
      "timestamp": "2025-12-26T10:05:00Z"
    }
  ]
}
```

---

## 10. References

### 10.1 Key Papers

1. Altschul, S.F. et al. (1990). "Basic local alignment search tool." *J Mol Biol* 215:403-410.
2. Needleman, S.B. & Wunsch, C.D. (1970). "A general method applicable to the search for similarities in the amino acid sequence of two proteins." *J Mol Biol* 48:443-453.
3. Smith, T.F. & Waterman, M.S. (1981). "Identification of common molecular subsequences." *J Mol Biol* 147:195-197.
4. Thompson, J.D. et al. (1994). "CLUSTAL W: improving the sensitivity of progressive multiple sequence alignment." *Nucleic Acids Res* 22:4673-4680.
5. Jumper, J. et al. (2021). "Highly accurate protein structure prediction with AlphaFold." *Nature* 596:583-589.

### 10.2 Databases

| Database | URL | Purpose |
|----------|-----|---------|
| UniProt | uniprot.org | Protein sequences |
| NCBI | ncbi.nlm.nih.gov | Genes, proteins, literature |
| Ensembl | ensembl.org | Genome annotations |
| PDB | rcsb.org | Protein structures |
| KEGG | genome.jp/kegg | Pathways |
| GO | geneontology.org | Gene ontology |

### 10.3 Software Tools

| Tool | Purpose | Reference |
|------|---------|-----------|
| BLAST+ | Sequence search | NCBI |
| MUSCLE | Multiple alignment | Edgar 2004 |
| MAFFT | Multiple alignment | Katoh et al. 2002 |
| IQ-TREE | Phylogeny | Nguyen et al. 2015 |
| RAxML | Phylogeny | Stamatakis 2014 |
| HMMER | Profile HMMs | Eddy 2011 |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based bioinformatics queries
- WIA-OMNI-API: Universal API gateway
- WIA-CLOUD: Distributed computing standards
- WIA-DATA: Data management and sharing

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-BIO-007 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
