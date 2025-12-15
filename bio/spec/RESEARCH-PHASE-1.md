# WIA Biotech Research Report - Phase 1

**Biotechnology Standards Research**

---

## 1. Executive Summary

바이오테크놀로지는 생명공학, 유전자 편집, 합성생물학, 단백질 공학의 융합으로 급속히 발전하고 있습니다. CRISPR 유전자 편집 시장은 2024년 40.1억 달러에서 2033년 135억 달러로 성장할 것으로 전망됩니다 (CAGR 14.7%). DNA 컴퓨팅 시장은 2024년 2.2억 달러에서 2032년 26.8억 달러로 급성장이 예상됩니다 (CAGR 36.76%).

본 연구 보고서는 WIA Biotech Standard 개발을 위한 기초 조사로서, CRISPR 유전자 편집, 합성생물학, DNA 컴퓨팅, 단백질 구조 예측, 이종장기이식, mRNA 기술의 현황과 표준화 기회를 분석합니다.

---

## 2. Technology Landscape

### 2.1 CRISPR Gene Editing

#### Overview
- **원리**: Cas9 단백질과 가이드 RNA를 이용한 정밀 유전자 편집
- **발견**: 2012년 Jennifer Doudna & Emmanuelle Charpentier
- **노벨상**: 2020년 화학상 수상

#### Market Overview

| Year | Market Size | Growth |
|------|-------------|--------|
| 2024 | $4.01B | - |
| 2025 | $4.60B | 14.7% |
| 2033 | $13.50B (projected) | CAGR 14.7% |

#### Key Technologies

**CRISPR-Cas9 (Classic)**
- 표적 유전자 절단 및 편집
- 이중 가닥 절단 (DSB) 유도
- 비표적 효과 (off-target) 가능성

**Base Editing**
- DSB 없이 단일 염기 변환
- 더 정밀한 편집 가능
- 제한된 편집 유형

**Prime Editing**
- 대규모 유전자 삭제 가능
- DSB 없이 정밀 편집
- 비표적 돌연변이 감소
- 치료 응용에 안전한 옵션

#### 2025 Major Developments

**AI Integration**
- CRISPR-GPT: Stanford Medicine 개발 LLM 기반 유전자 편집 보조 도구
- 실험 설계 자동화, 데이터 분석, 설계 결함 문제 해결
- 윤리적 요청에 대한 안전장치 내장

**Delivery Technology**
- Lipid Nanoparticle Spherical Nucleic Acids (LNP-SNAs)
- DNA 코팅 나노입자로 편집 성공률 3배 향상
- 정밀도 개선 및 독성 감소

**Clinical Progress**
- Casgevy: 최초 FDA 승인 CRISPR 치료제 (겸상적혈구병, 베타-지중해빈혈)
- Eli Lilly, Verve Therapeutics 13억 달러 인수 (2025.06)
- 인도, CRISPR 편집 쌀 품종 최초 도입 (2025.05)

#### Major Companies

| Company | Product/Focus | Status |
|---------|---------------|--------|
| Editas Medicine | 유전자 치료 | Clinical Trials |
| Intellia Therapeutics | In vivo 편집 | Clinical Trials |
| CRISPR Therapeutics | Casgevy | FDA Approved |
| Verve Therapeutics | 심혈관 유전자 편집 | Acquired by Lilly |
| Beam Therapeutics | Base editing | Clinical Trials |
| Prime Medicine | Prime editing | Development |

#### Regulatory Framework
- FDA Breakthrough Device Designation 활발
- 인간 생식세포 편집: 국제적 금지 유지 (Third International Summit 2023)
- GMO 규제: 국가별 상이한 프레임워크

---

### 2.2 Synthetic Biology

#### Overview
- **정의**: 생물학적 시스템의 설계, 제작, 재프로그래밍
- **핵심 개념**: 모듈화, 표준화, 추상화

#### Standard Assembly Methods

**BioBricks**
- 제한효소 조립 표준
- Registry of Standard Biological Parts (20,000+ 부품)
- iGEM 대회 표준 플랫폼

**Assembly Methods Comparison**

| Method | Description | Accuracy |
|--------|-------------|----------|
| 3A Assembly | BioBrick 기반 제한효소 조립 | 90%+ |
| BASIC | Biopart Assembly Standard for Idempotent Cloning | 93-99.7% |
| Golden Gate | Type IIs 제한효소 기반 | 95%+ |
| Gibson Assembly | 오버랩 기반 등온 조립 | 90%+ |
| SEVA 3.1 | SEVA, BioBricks, Type IIs 통합 플랫폼 | 95%+ |

**SEVA 3.1 Platform**
- SEVA, BioBricks, Type IIs 표준 간 호환성 제공
- Golden Gate 기반 'SevaBrick Assembly'
- 표준 SevaBrick 프라이머로 멀티파트 조립

#### Key Databases & Resources

| Resource | Description | URL |
|----------|-------------|-----|
| Registry of Standard Biological Parts | iGEM BioBrick 저장소 | parts.igem.org |
| SEVA Database | Synthetic Biology Vector Archive | seva.cnb.csic.es |
| Addgene | Plasmid repository | addgene.org |
| SynBioHub | 표준 부품 데이터베이스 | synbiohub.org |

#### Applications
- 바이오연료 생산
- 의약품 합성 (아르테미시닌 등)
- 바이오센서
- 세포 공장

---

### 2.3 DNA Computing & Data Storage

#### Overview
- **원리**: DNA 분자를 컴퓨팅 기질 및 데이터 저장 매체로 활용
- **저장 밀도**: 1 exabyte/mm³ (이론적)

#### Market Growth

| Year | Market Size | Growth |
|------|-------------|--------|
| 2024 | $219.79M | - |
| 2025 | $293.70M | 33.6% |
| 2032 | $2.68B (projected) | CAGR 36.76% |

#### Encoding Standards

**ADS Codex (Adaptive DNA Storage Codex)**
- Los Alamos National Laboratory 개발
- 바이너리 → DNA 4염기 코드 변환
- IARPA MIST (Molecular Information Storage) 프로그램 핵심

**Key Technologies**

| Technology | Description | Status |
|------------|-------------|--------|
| Nanopore Sequencing | 실시간 DNA 읽기 (SUSTag-ORCtrL) | Available |
| DNA Synthesis | 염기 추가 (~1초/염기) | Development |
| Error Correction | 데이터 무결성 보장 | Research |
| Massive Parallel | 대규모 병렬 처리 | Development |

**2025 Developments**
- SNIA DNA Data Storage White Papers 발행 (2025.07)
  - "DNA Data Storage Technology Review"
  - "DNA Data Storage Codecs - Examples, Requirements, and Metrics"
- Storage and Computing with DNA Conference 2025 (Paris)
- SMALL 전략: 기능 전환을 위한 최소 아키텍처 변경

#### Challenges
- 읽기/쓰기 속도 (현재 병목)
- 비용 (합성 및 시퀀싱)
- 랜덤 액세스
- 장기 보존 안정성

---

### 2.4 Protein Structure Prediction (AlphaFold)

#### Overview
- **개발**: Google DeepMind
- **원리**: AI 기반 단백질 3D 구조 예측
- **영향**: 2024 노벨 화학상 (David Baker, Demis Hassabis, John Jumper)

#### AlphaFold Database

| Metric | Value |
|--------|-------|
| 총 구조 수 | 214+ million |
| 확장 규모 | 500x (2021 초기 대비) |
| 라이선스 | CC-BY-4.0 |

#### Standard File Formats

**Structure Files**
- **PDB** (Protein Data Bank): 전통적 구조 형식
- **mmCIF**: macromolecular Crystallographic Information File
- **binaryCIF**: 압축된 CIF 형식
- **modelCIF**: AlphaFold DB 표준 준수 형식

**Confidence Metrics (JSON)**

| Metric | Range | Description |
|--------|-------|-------------|
| pLDDT | 0-100 | predicted Local Distance Difference Test |
| PAE | 0-31.75 Å | Predicted Aligned Error |

**pLDDT Interpretation**

| Score | Confidence | Recommendation |
|-------|------------|----------------|
| >90 | Very high | 결합 사이트 분석 적합 |
| 70-90 | Good | 신뢰할 수 있는 백본 예측 |
| 50-70 | Low | 주의해서 사용 |
| <50 | Very low | 무질서 영역 가능성 |

**B-factor Column**
- PDB 파일의 B-factor 컬럼에 pLDDT 저장
- mmCIF: `_ma_qa_metric_local` 카테고리

#### AlphaFold3
- 단백질-리간드, 단백질-DNA/RNA 복합체 예측
- CIF 형식 출력 (수소 위치 미포함)

#### 2025 Publication Standards
- 예측 구조 발표 시 신뢰도 메트릭 필수 포함
- 관련 파일 공개 권장
- 인용: Fleming J. et al. (2025) Journal of Molecular Biology

---

### 2.5 Xenotransplantation

#### Overview
- **정의**: 동물 장기를 인간에게 이식
- **주요 원천**: 유전자 편집 돼지

#### 2025 Regulatory Milestones

| Date | Event |
|------|-------|
| 2025.02 | FDA 최초 임상시험 승인 (eGenesis, United Therapeutics) |
| 2025.10 | 첫 임상시험 돼지 신장 이식 (뉴욕) |
| 2025 | WHO 지침 업데이트 예정 (IXA 제안) |

#### Clinical Progress

**Compassionate Use Cases (2024-2025)**
- 돼지 신장, 심장, 간 이식 동정적 사용 사례 다수
- Peer-reviewed 결과 발표

**Clinical Trials**
- United Therapeutics Corporation
- eGenesis
- 6명 대상 돼지 신장 임상시험 진행 중

#### Genetic Modifications

**Required Gene Knockouts**
- GGTA1 (glycoprotein α-galactosyltransferase 1): 초급성 거부 유발
- CMAH (cytidine monophosphate-N-acetylneuraminic acid hydroxylase)

**Human Transgenes**
- 면역 호환성 향상을 위한 인간 유전자 삽입
- 응고 조절 유전자
- 보체 조절 유전자

#### International Guidelines

| Organization | Role |
|--------------|------|
| IXA (International Xenotransplantation Association) | 국제 가이드라인 주도 |
| TTS (Transplantation Society) | 협력 기관 |
| WHO | 글로벌 가이던스 |

#### Challenges
- 규제 프레임워크 표준화 필요
- 동물 복지 우려
- 장기 면역 억제
- 인수공통감염병 위험

---

### 2.6 mRNA Technology

#### Overview
- **원리**: 메신저 RNA를 통한 단백질 발현 유도
- **플랫폼**: Lipid Nanoparticle (LNP) 기반 전달

#### Standard LNP Composition

| Component | Function |
|-----------|----------|
| Ionizable Lipids | mRNA 캡슐화, 엔도솜 탈출 |
| Helper/Neutral Lipids | 구조 안정화 |
| Cholesterol | 막 안정성 |
| PEG-Lipids | 면역 회피, 안정성 |

#### 2025 Technology Advances

**Reduced Lipid Platforms**
- 망간 이온 활용 mRNA 농축 코어 형성
- 필요 지질 양 감소, 독성 저감
- 생체내 형질전환 및 면역 반응 향상

**PEG Alternatives**
- Poly(carboxybetaine) (PCB): 쯔비터이온 폴리머
- PEG 대체로 면역 반응 감소
- Cornell University 개발

**Improved Delivery**
- MIT: 1/100 용량으로 동일 면역 반응 달성
- 백신 비용 절감 가능성

**Modular Vaccine Platform (MVP)**
- 키메라 항원 모듈식 조립
- 다양한 병원체 대응 가능

**On-Chip Production**
- 마이크로플루이딕 플랫폼
- 주문형 개인화 mRNA 합성
- LNP 캡슐화 통합

#### Applications

| Application | Status |
|-------------|--------|
| COVID-19 Vaccines | Approved |
| Influenza Vaccines | Clinical Trials |
| Cancer Immunotherapy | Clinical Trials |
| Autoimmune Disorders | Research |
| Genetic Diseases | Research |

#### Challenges
- PEG 면역원성 (Anti-PEG 항체)
- 저온 유통 필요성
- 반복 투여 시 면역 반응
- 장기 안전성 데이터 부족

---

## 3. Existing Standards and Protocols

### 3.1 Sequence Data Formats

| Format | Description | Use Case |
|--------|-------------|----------|
| **FASTA** | 간단한 서열 형식 | 범용 서열 저장 |
| **FASTQ** | 서열 + 품질 점수 | NGS 데이터 |
| **GenBank** | NCBI 서열 형식 | 주석이 있는 서열 |
| **EMBL** | 유럽 서열 형식 | 유럽 DB |
| **SBOL** | Synthetic Biology Open Language | 합성생물학 설계 |

### 3.2 Structure Data Formats

| Format | Description | Use Case |
|--------|-------------|----------|
| **PDB** | Protein Data Bank | 단백질 구조 |
| **mmCIF** | macromolecular CIF | 복잡한 구조 |
| **PDBX/mmCIF** | PDB 확장 형식 | PDB 아카이브 표준 |
| **SDF/MOL** | Structure Data File | 소분자 |
| **SMILES** | 분자 표현 문자열 | 화합물 검색 |

### 3.3 Bioinformatics Data Standards

| Standard | Organization | Description |
|----------|--------------|-------------|
| **MAGE-TAB** | EMBL-EBI | 마이크로어레이 데이터 |
| **MIAME** | FGED | 마이크로어레이 실험 최소 정보 |
| **MINSEQE** | FGED | 시퀀싱 실험 최소 정보 |
| **ISA-Tab** | ISA Commons | 조사-연구-분석 메타데이터 |

### 3.4 Synthetic Biology Standards

| Standard | Description |
|----------|-------------|
| **BioBricks** | 모듈식 DNA 부품 표준 |
| **SBOL** | Synthetic Biology Open Language |
| **SEVA** | Standard European Vector Architecture |
| **MoClo** | Modular Cloning System |

---

## 4. Key Challenges

### 4.1 Technical Challenges
- **데이터 상호운용성**: 다양한 형식 간 변환 복잡
- **확장성**: 대용량 시퀀싱 데이터 처리
- **품질 관리**: 실험 간 재현성
- **표준화 부재**: 새로운 기술 영역 (DNA 컴퓨팅 등)

### 4.2 Regulatory Challenges
- 국가별 상이한 규제 프레임워크
- GMO/유전자 편집 규제 불확실성
- 이종장기이식 가이드라인 개발 중
- 데이터 공유 및 프라이버시

### 4.3 Ethical Challenges
- 인간 생식세포 편집 논쟁
- 합성생물학 이중 사용 (Dual-use)
- 바이오안전성 (Biosafety)
- 공평한 접근성

---

## 5. Standardization Opportunities

### 5.1 Data Format Standardization
- 통합 바이오테크 데이터 형식 필요
- CRISPR 편집 실험 메타데이터 표준
- DNA 저장 코덱 표준
- 단백질 구조 예측 신뢰도 메트릭 표준

### 5.2 API Standardization
- 유전자 편집 설계 도구 API
- 서열 분석 파이프라인
- 구조 예측 서비스
- 부품 라이브러리 접근

### 5.3 Protocol Standardization
- 합성생물학 조립 프로토콜
- 유전자 편집 검증 워크플로우
- 품질 관리 체크리스트
- 데이터 교환 프로토콜

### 5.4 Integration Standardization
- 연구소 정보 관리 시스템 (LIMS) 연동
- 바이오파운드리 자동화 통합
- AI/ML 파이프라인 연결
- 클라우드 컴퓨팅 플랫폼

---

## 6. Recommendations for WIA Biotech Standard

### 6.1 Immediate Priorities (Phase 1-2)
1. 유전자 서열 및 편집 데이터 형식 표준화
2. 기존 FASTA, FASTQ, SBOL 형식과의 호환성 확보
3. CRISPR 편집 결과 메타데이터 스키마 정의
4. 단백질 구조 예측 데이터 통합 형식

### 6.2 Medium-term Goals (Phase 3)
1. 합성생물학 조립 프로토콜 표준
2. DNA 저장/컴퓨팅 데이터 형식
3. 이종장기이식 추적 데이터
4. 바이오안전성 메타데이터

### 6.3 Long-term Vision (Phase 4)
1. 글로벌 바이오테크 데이터 플랫폼 통합
2. AI/ML 기반 설계 도구 API
3. 규제 준수 자동화
4. 바이오파운드리 네트워크 연결

---

## 7. References

### Industry Reports
- [CRISPR Gene Editing Market Report 2024-2033](https://www.grandviewresearch.com/industry-analysis/crispr-based-gene-editing-market-report)
- [DNA Computing Market Global Forecast 2025-2032](https://www.globenewswire.com/news-release/2025/12/12/3204781/28124/en/DNA-Computing-Market-Global-Forecast-2025-2032.html)
- [SNIA DNA Data Storage White Papers 2025](https://www.storagenewsletter.com/2025/10/14/snia-2025-storage-and-computing-with-dna-conference-presentations-available/)

### Academic Publications
- [Challenges and opportunities in DNA computing and data storage (Nature Nanotechnology 2025)](https://www.nature.com/articles/s41565-025-01937-w)
- [Emerging frontiers in protein structure prediction following the AlphaFold revolution (RSIF 2025)](https://royalsocietypublishing.org/doi/10.1098/rsif.2024.0886)
- [Xenotransplantation Literature Update 2025 (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12391572/)
- [Precision gene editing: CRISPR-Cas in modern genetics (PMC 2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12590234/)

### Technical Documentation
- [AlphaFold Protein Structure Database](https://alphafold.ebi.ac.uk/)
- [Registry of Standard Biological Parts](https://parts.igem.org/)
- [SEVA 3.1 Platform](https://pubmed.ncbi.nlm.nih.gov/32710525/)
- [BioBrick Assembly Standards](https://pubmed.ncbi.nlm.nih.gov/24395353/)

### Company Resources
- [Stanford Medicine CRISPR-GPT](https://med.stanford.edu/news/all-news/2025/09/ai-crispr-gene-therapy.html)
- [MIT LNP Research](https://news.mit.edu/2025/particles-enhance-mrna-delivery-could-reduce-vaccine-dosage-costs-1107)
- [Cornell PCB-LNP Research](https://news.cornell.edu/stories/2025/05/stealthy-lipid-nanoparticles-give-mrna-vaccines-makeover)

---

**Document Version**: 1.0
**Last Updated**: 2025-12
**Author**: WIA Biotech Working Group

---

弘益人間 - *Benefit All Humanity*
