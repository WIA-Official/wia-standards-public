# WIA-BIO-001: Data Formats Specification

## Version 1.0 | 2025-01-15

---

## Table of Contents
1. [Overview](#overview)
2. [FASTQ Format](#fastq-format)
3. [SAM/BAM/CRAM Formats](#sambamcram-formats)
4. [VCF/BCF Format](#vcfbcf-format)
5. [BED Format](#bed-format)
6. [GFF/GTF Formats](#gffgtf-formats)
7. [Custom JSON Schema](#custom-json-schema)
8. [Compression Standards](#compression-standards)

---

## Overview

This specification defines mandatory and recommended data formats for genome sequencing data interchange under WIA-BIO-001. All formats must comply with their respective community standards while adhering to additional WIA requirements for metadata and quality.

### Format Priority

| Stage | Primary Format | Alternative | Compression |
|-------|----------------|-------------|-------------|
| **Raw Reads** | FASTQ | uBAM | gzip/bgzip |
| **Aligned Reads** | BAM | CRAM | BAM (BGZF), CRAM |
| **Variants** | VCF | BCF | bgzip, BCF |
| **Regions** | BED | Interval List | gzip |
| **Annotations** | GFF3 | GTF | gzip |
| **Metadata** | JSON | XML | - |

---

## FASTQ Format

### Standard Specification

FASTQ is the de facto standard for storing raw sequencing reads. Each record consists of four lines:

```
@<sequence_identifier>
<nucleotide_sequence>
+
<quality_scores>
```

### Example

```fastq
@SRR12345678.1 1 length=150
GATTTGGGGTTCAAAGCAGTATCGATCAAATAGTAAATCCATTTGTTCAACTCACAGTTTACAATGGGAACATGCGACAGCTCGATGCCTTAAACGCATCCGATGCTAGCTAGCTAGCTAGCTAGCTAGCTAGCTAGCTACGATGCGATCGATCGAT
+
IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
```

### Header Line Requirements

**Mandatory Fields:**
```
@<instrument>:<run_number>:<flowcell_id>:<lane>:<tile>:<x_pos>:<y_pos> <read>:<is_filtered>:<control_number>:<sample_barcode>
```

**Illumina Example:**
```
@A00123:456:HNGMTDSXY:1:1101:10123:1000 1:N:0:ATCACG
```

**WIA-BIO-001 Extended Header (Optional):**
```
@READ_ID sample=SAMPLE_001 platform=ILLUMINA date=2025-01-15 lib=LIB_001
```

### Quality Score Encoding

**Phred+33 (Sanger/Illumina 1.8+):**
```
Quality = ASCII_value - 33
```

**ASCII Range:** `!` (33) to `~` (126)
**Quality Range:** 0 to 93

| Symbol | ASCII | Phred Score | Error Probability |
|--------|-------|-------------|-------------------|
| ! | 33 | 0 | 1.0 (100%) |
| + | 43 | 10 | 0.1 (10%) |
| 5 | 53 | 20 | 0.01 (1%) |
| ? | 63 | 30 | 0.001 (0.1%) |
| I | 73 | 40 | 0.0001 (0.01%) |

### Paired-End Naming Convention

```
# Read 1
@READ_001/1
ATCGATCGATCGATCG
+
IIIIIIIIIIIIIIII

# Read 2 (same fragment)
@READ_001/2
GCTAGCTAGCTAGCTA
+
IIIIIIIIIIIIIIII
```

**Or Modern Illumina:**
```
@READ_001 1:N:0:ATCACG
@READ_001 2:N:0:ATCACG
```

### Compression

**Required:** gzip (`.fastq.gz`) or bgzip (`.fastq.bgz`)

**Recommended bgzip for parallel processing:**
```bash
bgzip -@ 8 -c input.fastq > output.fastq.bgz
```

---

## SAM/BAM/CRAM Formats

### SAM (Sequence Alignment/Map)

**Header Section:**
```sam
@HD	VN:1.6	SO:coordinate
@SQ	SN:chr1	LN:248956422	M5:6aef897c3d6ff0c78aff06ac189178dd	UR:file:///reference/GRCh38.fa
@RG	ID:SAMPLE_001	SM:PATIENT_123	PL:ILLUMINA	LB:LIB_001	PU:HNGMTDSXY.1.ATCACG
@PG	ID:bwa	PN:bwa	VN:0.7.17	CL:bwa mem -t 8 -R '@RG\tID:SAMPLE_001' reference.fa reads.fastq
```

**Alignment Section:**
```sam
READ_001	99	chr1	10000	60	150M	=	10200	350	ATCGATCG...	IIIIIIII...	NM:i:0	MD:Z:150	AS:i:150	XS:i:0
```

### SAM Fields

| Col | Field | Description | Example |
|-----|-------|-------------|---------|
| 1 | QNAME | Query name | READ_001 |
| 2 | FLAG | Bitwise flag | 99 (paired, proper, first) |
| 3 | RNAME | Reference name | chr1 |
| 4 | POS | 1-based position | 10000 |
| 5 | MAPQ | Mapping quality | 60 |
| 6 | CIGAR | Alignment string | 150M |
| 7 | RNEXT | Mate reference | = (same) |
| 8 | PNEXT | Mate position | 10200 |
| 9 | TLEN | Template length | 350 |
| 10 | SEQ | Read sequence | ATCGATCG... |
| 11 | QUAL | Base qualities | IIIIIIII... |
| 12+ | TAG | Optional fields | NM:i:0 |

### FLAG Values

| Flag | Hex | Meaning |
|------|-----|---------|
| 1 | 0x1 | Paired-end read |
| 2 | 0x2 | Properly paired |
| 4 | 0x4 | Unmapped |
| 8 | 0x8 | Mate unmapped |
| 16 | 0x10 | Reverse strand |
| 32 | 0x20 | Mate reverse strand |
| 64 | 0x40 | First in pair |
| 128 | 0x80 | Second in pair |
| 256 | 0x100 | Secondary alignment |
| 512 | 0x200 | QC fail |
| 1024 | 0x400 | PCR/optical duplicate |
| 2048 | 0x800 | Supplementary alignment |

**Example:** FLAG=99 = 1+2+32+64 (paired, proper, mate reverse, first)

### Required Tags (WIA-BIO-001)

| Tag | Type | Description | Example |
|-----|------|-------------|---------|
| **RG** | Z | Read group | RG:Z:SAMPLE_001 |
| **NM** | i | Edit distance | NM:i:0 |
| **MD** | Z | Mismatch string | MD:Z:150 |
| **AS** | i | Alignment score | AS:i:150 |
| **XS** | i | Suboptimal score | XS:i:0 |

### BAM Format

**Binary, compressed version of SAM.**

**Creation:**
```bash
samtools view -bS input.sam > output.bam
samtools sort -@ 8 -o sorted.bam output.bam
samtools index sorted.bam
```

**Advantages:**
- 5-10x smaller than SAM
- Faster to parse
- Random access via index

### CRAM Format

**Reference-based compression (20-60% smaller than BAM).**

**Creation:**
```bash
samtools view -T reference.fa -C -o output.cram input.bam
samtools index output.cram
```

**Requirements:**
- Must specify reference genome (`-T` flag)
- Reference must be accessible during decompression
- Use for long-term archival

**Compression Comparison:**
```
FASTQ:  100 GB
BAM:     30 GB (3x compression)
CRAM:    10 GB (10x compression from FASTQ, 3x from BAM)
```

---

## VCF/BCF Format

### VCF (Variant Call Format) v4.3

**Header:**
```vcf
##fileformat=VCFv4.3
##fileDate=20250115
##source=GATK_HaplotypeCaller_v4.3.0
##reference=file:///data/reference/GRCh38.fa
##contig=<ID=chr1,length=248956422,assembly=GRCh38>
##INFO=<ID=DP,Number=1,Type=Integer,Description="Total Depth">
##INFO=<ID=AF,Number=A,Type=Float,Description="Allele Frequency">
##FORMAT=<ID=GT,Number=1,Type=String,Description="Genotype">
##FORMAT=<ID=GQ,Number=1,Type=Integer,Description="Genotype Quality">
##FORMAT=<ID=DP,Number=1,Type=Integer,Description="Read Depth">
##FORMAT=<ID=AD,Number=R,Type=Integer,Description="Allelic Depths">
#CHROM	POS	ID	REF	ALT	QUAL	FILTER	INFO	FORMAT	SAMPLE_001
```

**Variant Records:**
```vcf
chr1	12345	rs12345	A	G	99.0	PASS	DP=100;AF=0.5	GT:GQ:DP:AD	0/1:99:100:50,50
chr1	67890	.	ATG	A	85.0	PASS	DP=85;AF=1.0	GT:GQ:DP:AD	1/1:85:85:0,85
chr2	11223	rs11223	C	T,G	120.0	PASS	DP=120;AF=0.4,0.1	GT:GQ:DP:AD	1/2:99:120:60,48,12
```

### VCF Fixed Fields

| Field | Description | Example |
|-------|-------------|---------|
| **CHROM** | Chromosome | chr1 |
| **POS** | 1-based position | 12345 |
| **ID** | Variant identifier | rs12345 |
| **REF** | Reference allele | A |
| **ALT** | Alternate allele(s) | G or G,T (multi-allelic) |
| **QUAL** | Phred quality score | 99.0 |
| **FILTER** | Filter status | PASS, LowQual |
| **INFO** | Variant-level info | DP=100;AF=0.5 |
| **FORMAT** | Sample field format | GT:GQ:DP:AD |
| **SAMPLE** | Sample genotype data | 0/1:99:100:50,50 |

### Genotype Encoding

| GT | Meaning | Alleles |
|----|---------|---------|
| **0/0** | Homozygous reference | REF/REF |
| **0/1** | Heterozygous | REF/ALT |
| **1/1** | Homozygous alternate | ALT/ALT |
| **1/2** | Compound heterozygous | ALT1/ALT2 |
| **./.** | Missing/no call | Unknown |
| **0\|1** | Phased heterozygous | REF\|ALT |

### Required INFO Fields (WIA-BIO-001)

```vcf
##INFO=<ID=DP,Number=1,Type=Integer,Description="Total read depth">
##INFO=<ID=AF,Number=A,Type=Float,Description="Allele frequency (0-1)">
##INFO=<ID=AC,Number=A,Type=Integer,Description="Allele count">
##INFO=<ID=AN,Number=1,Type=Integer,Description="Total number of alleles">
##INFO=<ID=FS,Number=1,Type=Float,Description="Fisher strand bias">
##INFO=<ID=QD,Number=1,Type=Float,Description="Quality by depth">
```

### Required FORMAT Fields

```vcf
##FORMAT=<ID=GT,Number=1,Type=String,Description="Genotype">
##FORMAT=<ID=GQ,Number=1,Type=Integer,Description="Genotype quality">
##FORMAT=<ID=DP,Number=1,Type=Integer,Description="Read depth">
##FORMAT=<ID=AD,Number=R,Type=Integer,Description="Allelic depths (ref, alt1, alt2, ...)">
```

### Variant Types Examples

**SNP (Single Nucleotide Polymorphism):**
```vcf
chr1	100	.	A	G	99	PASS	DP=50	GT:DP	0/1:50
```

**Insertion:**
```vcf
chr1	200	.	T	TAAA	85	PASS	DP=60	GT:DP	0/1:60
```

**Deletion:**
```vcf
chr1	300	.	AGGG	A	90	PASS	DP=70	GT:DP	0/1:70
```

**Complex/MNP:**
```vcf
chr1	400	.	ATG	GCA	75	PASS	DP=80	GT:DP	0/1:80
```

**Structural Variant:**
```vcf
chr1	500	.	N	<DEL>	100	PASS	SVTYPE=DEL;SVLEN=-5000;END=5500	GT	0/1
```

### BCF Format

**Binary, indexed version of VCF (5-10x faster parsing).**

```bash
# Convert VCF to BCF
bcftools view -Ob -o output.bcf input.vcf.gz

# Index BCF
bcftools index output.bcf

# Query BCF
bcftools view -r chr1:10000-20000 output.bcf
```

---

## BED Format

### Standard BED Format

**Minimal 3-column:**
```bed
chr1	10000	20000
chr2	30000	40000
```

**Extended 12-column (BED12):**
```bed
chr1	10000	20000	GENE1	100	+	10050	19950	255,0,0	3	500,400,600	0,5000,9400
```

### BED Field Definitions

| Column | Name | Description | Required |
|--------|------|-------------|----------|
| 1 | chrom | Chromosome | Yes |
| 2 | chromStart | Start position (0-based) | Yes |
| 3 | chromEnd | End position (exclusive) | Yes |
| 4 | name | Feature name | No |
| 5 | score | Score (0-1000) | No |
| 6 | strand | +/- | No |
| 7 | thickStart | Coding start | No |
| 8 | thickEnd | Coding end | No |
| 9 | itemRgb | RGB color | No |
| 10 | blockCount | Number of blocks | No |
| 11 | blockSizes | Block sizes | No |
| 12 | blockStarts | Block starts | No |

### WIA-BIO-001 BED Requirements

**For target regions (exome/panel):**
```bed
# BED format with gene names
chr1	100	200	GENE1_exon1	.	+
chr1	500	700	GENE1_exon2	.	+
chr2	1000	1200	GENE2_exon1	.	+
```

**Header (optional but recommended):**
```bed
#track name="WIA_Panel_v1" description="WIA-BIO-001 Target Regions" version=1.0
chr1	100	200	GENE1_exon1
```

---

## GFF/GTF Formats

### GFF3 Format

```gff3
##gff-version 3
##sequence-region chr1 1 248956422
chr1	HAVANA	gene	11869	14409	.	+	.	ID=ENSG00000223972;Name=DDX11L1;biotype=transcribed_unprocessed_pseudogene
chr1	HAVANA	transcript	11869	14409	.	+	.	ID=ENST00000456328;Parent=ENSG00000223972;Name=DDX11L1-202
chr1	HAVANA	exon	11869	12227	.	+	.	ID=exon1;Parent=ENST00000456328
```

### GTF Format (Gene Transfer Format)

```gtf
chr1	HAVANA	gene	11869	14409	.	+	.	gene_id "ENSG00000223972"; gene_name "DDX11L1"; gene_biotype "transcribed_unprocessed_pseudogene";
chr1	HAVANA	transcript	11869	14409	.	+	.	gene_id "ENSG00000223972"; transcript_id "ENST00000456328"; gene_name "DDX11L1";
chr1	HAVANA	exon	11869	12227	.	+	.	gene_id "ENSG00000223972"; transcript_id "ENST00000456328"; exon_number "1";
```

---

## Custom JSON Schema

### WIA-BIO-001 Sample Metadata

```json
{
  "sampleId": "SAMPLE_001",
  "patientId": "PATIENT_123",
  "sequencing": {
    "platform": "ILLUMINA",
    "instrument": "NovaSeq6000",
    "runDate": "2025-01-15",
    "flowcellId": "HNGMTDSXY",
    "lane": 1,
    "library": {
      "id": "LIB_001",
      "prepKit": "TruSeq DNA PCR-Free",
      "insertSize": 350,
      "barcode": "ATCACG"
    }
  },
  "reference": {
    "genome": "GRCh38",
    "version": "GRCh38.p14",
    "source": "NCBI"
  },
  "quality": {
    "meanCoverage": 45.2,
    "pctQ30": 95.3,
    "pctMapped": 98.1,
    "contamination": 0.002
  }
}
```

---

## Compression Standards

### Mandatory Compression

| Format | Compression | Extension | Tool |
|--------|-------------|-----------|------|
| FASTQ | gzip/bgzip | `.fastq.gz` | `gzip -c` or `bgzip` |
| SAM | BGZF (via BAM) | `.bam` | `samtools view -b` |
| VCF | bgzip | `.vcf.gz` | `bgzip` |
| BED | gzip | `.bed.gz` | `gzip -c` |

### bgzip vs gzip

**bgzip:** Block-gzip (BGZF) allows random access when indexed.

```bash
# Compress with bgzip
bgzip -c input.vcf > output.vcf.gz

# Index for random access
tabix -p vcf output.vcf.gz

# Query specific region
tabix output.vcf.gz chr1:10000-20000
```

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA BIO Working Group
**License:** Apache 2.0
