# Phase 1 사전 조사 결과

## WIA Nano Data Format Standard Research

---

## 1. 기존 표준 조사

### 1.1 ISO TC 229 - Nanotechnologies

**출처**: [ISO TC 229 Official](https://www.iso.org/committee/381983.html)

- **설립**: 2005년
- **현황**: 114개 발행 표준, 40개 개발 중
- **범위**: 100nm 이하 나노스케일 물질 및 프로세스

**Working Groups**:
| WG | 분야 | 담당국 |
|---|------|-------|
| WG1 | Terminology and nomenclature | Canada |
| WG2 | Measurement and characterization | Japan |
| WG3 | Health, safety and environment | USA |
| WG4 | Materials specifications | China |
| WG5 | Products and applications | South Korea |

**주요 표준**:
- ISO/TS 13329:2024 - 나노물질 안전 데이터 시트 (SDS)
- ISO/TS 80004 시리즈 - 나노기술 용어 정의

**WIA Nano 적용**:
- 용어 및 명명법 ISO 표준 준수
- 나노물질 특성 기술 시 ISO 기준 참조

---

### 1.2 IEEE 1906.1 - Nanoscale Communication

**출처**: [IEEE 1906.1 Standard](https://standards.ieee.org/ieee/1906.1/5171/)

- **목적**: 나노스케일 및 분자 통신 프레임워크 표준화
- **범위**: 인체 내 통신, 스마트 소재, 분자 수준 센싱

**핵심 구성요소**:
1. 나노스케일 통신 네트워킹 정의
2. 개념적 모델
3. 공통 용어
4. 성능 메트릭

**IEEE 1906.1.1-2020 Data Model**:
- YANG 모듈 기반 데이터 모델
- 나노스케일 통신 시스템 기술
- 원격 시뮬레이션/운영/분석 지원

**WIA Nano 적용**:
- Phase 3 통신 프로토콜에서 IEEE 1906.1 참조
- YANG 모듈 데이터 구조 참고

---

### 1.3 PDBx/mmCIF - Molecular Structure Format

**출처**: [PDBx/mmCIF Dictionary](https://mmcif.wwpdb.org/)

- **개발**: IUCr (International Union of Crystallography) + PDB
- **용도**: 거대분자 구조 데이터 표현
- **형식**: ASCII 텍스트, 카테고리 기반 테이블

**구조**:
```
_atom_site.group_PDB
_atom_site.id
_atom_site.type_symbol
_atom_site.label_atom_id
_atom_site.Cartn_x
_atom_site.Cartn_y
_atom_site.Cartn_z
```

**JSON 변환**:
- mmJSON: PDBj에서 사용하는 JSON 형식
- WebGL 분자 뷰어용 최적화 형식

**WIA Nano 적용**:
- 분자 조립기의 분자 구조 데이터에 PDB 호환 좌표계 사용
- 원자 위치는 Angstrom 단위

---

## 2. 분야별 데이터 형식 조사

### 2.1 Molecular Assembler (분자 조립기)

**개념**: 원자/분자 수준에서 정밀하게 물질을 조립하는 시스템

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| target_molecule | 목표 분자 구조 | PDB/SMILES |
| building_blocks | 빌딩 블록 원자/분자 | - |
| assembly_progress | 조립 진행률 | % |
| workspace_coordinates | 작업 공간 좌표 | nm |
| bond_energy | 결합 에너지 | kJ/mol |

**참조 데이터 형식**:
- SMILES (Simplified Molecular Input Line Entry System)
- InChI (International Chemical Identifier)
- XYZ 분자 좌표 형식

---

### 2.2 Nano Machine (나노 머신)

**개념**: 분자 모터, 분자 스위치 등 나노스케일 기계 장치

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| position | 3D 위치 | nm |
| orientation | 방향 (쿼터니언) | - |
| state | 작동 상태 | enum |
| energy_source | 에너지원 (ATP 등) | - |
| cycles_completed | 완료 사이클 수 | count |
| torque | 토크 | pN·nm |

**참조**:
- 분자 모터 (F1-ATPase): 회전속도 ~130 rev/s
- 키네신: 이동속도 ~800 nm/s

---

### 2.3 Molecular Memory (분자 메모리)

**개념**: DNA, 단백질 등 분자 기반 데이터 저장

**출처**: [DNA Digital Data Storage](https://en.wikipedia.org/wiki/DNA_digital_data_storage)

**DNA 인코딩 방식**:
| 방식 | 매핑 | 저장 밀도 |
|-----|------|----------|
| 2-bit | 00→A, 01→G, 10→C, 11→T | 2 bits/nucleotide |
| Ternary | Base-3 to nucleotide | ~1.6 bits/nucleotide |
| Goldman | Huffman + trits | ~1.77 bits/nucleotide |

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| address | 논리 주소 | - |
| encoding_method | 인코딩 방식 | enum |
| strand_sequence | DNA 서열 | ATCG |
| error_correction | 오류 정정 코드 | - |
| storage_density | 저장 밀도 | bits/nt |

**저장 용량**: 1g DNA ≈ 455 EB (이론적 최대)

---

### 2.4 Nanomedicine (나노의약)

**개념**: 나노입자 기반 약물 전달 시스템

**출처**: [Liposome Drug Delivery](https://pmc.ncbi.nlm.nih.gov/articles/PMC8879473/)

**리포좀 특성**:
- 크기: 20-1000 nm (임상 승인: 50-300 nm)
- 구조: 인지질 이중층 구형 소포
- FDA 첫 승인: Doxil (1995)

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| carrier_type | 운반체 유형 | enum |
| diameter | 직경 | nm |
| drug_payload | 약물 적재량 | pg |
| target_receptor | 표적 수용체 | string |
| release_trigger | 방출 트리거 | enum |
| ph_threshold | pH 임계값 | pH |

**트리거 유형**:
- pH 감응 (종양 미세환경: pH 5.5-6.5)
- 온도 감응
- 효소 감응
- 자기장/초음파

---

### 2.5 Nanorobotics (나노로봇)

**개념**: 나노스케일 자율 로봇 시스템

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| position | 3D 위치 | μm (체내) |
| dimensions | 크기 | nm |
| propulsion_type | 추진 방식 | enum |
| speed | 이동 속도 | μm/s |
| sensors | 센서 목록 | array |
| actuators | 액추에이터 목록 | array |
| power_source | 전원 | enum |
| power_level | 전력 수준 | % |
| mission_status | 미션 상태 | enum |

**추진 방식**:
- 편모 (Flagellar): 박테리아 모방
- 화학적 (Chemical): 촉매 반응 이용
- 자기 (Magnetic): 외부 자기장 제어
- 초음파 (Acoustic): 음향파 추진

---

### 2.6 Nanosensor (나노센서)

**개념**: 나노스케일 감지 장치 (양자점, 나노와이어 등)

**데이터 요구사항**:
| 필드 | 설명 | 단위 |
|-----|------|-----|
| sensor_type | 센서 유형 | enum |
| target_analyte | 검출 대상 | string |
| measurement_value | 측정값 | varies |
| detection_limit | 검출 한계 | varies |
| snr | 신호대잡음비 | dB |
| wavelength | 파장 (광학) | nm |
| calibration_date | 캘리브레이션 일시 | ISO8601 |

**센서 유형**:
- 양자점 (Quantum Dot): 형광 기반
- 나노와이어 (Nanowire): 전기 전도도 변화
- 플라즈몬 (Plasmonic): 표면 플라즈몬 공명
- MEMS/NEMS: 기계적 공진

---

## 3. 공통 필드 분석

### 3.1 모든 나노시스템 공통

| 필드 | 설명 | 필수 |
|-----|------|:---:|
| system_type | 시스템 유형 | ✓ |
| device_info | 장치 정보 | ✓ |
| timestamp | 타임스탬프 | ✓ |
| sequence | 시퀀스 번호 | ✓ |
| environment | 환경 조건 | ✓ |
| meta | 메타데이터 | - |

### 3.2 환경 조건 (Environment)

| 필드 | 설명 | 단위 |
|-----|------|-----|
| temperature | 온도 | K |
| pressure | 압력 | Pa |
| medium | 매질 | enum |
| ph | pH (수용액) | - |
| ionic_strength | 이온 강도 | M |

### 3.3 좌표 시스템

| 스케일 | 단위 | 사용처 |
|-------|-----|-------|
| 원자 | pm (피코미터) | 원자 반경, 결합 길이 |
| 분자 | Å (옹스트롬) | 분자 구조 |
| 나노입자 | nm (나노미터) | 나노머신, 센서 |
| 세포 | μm (마이크로미터) | 나노로봇 위치 |

---

## 4. 결론

### 4.1 표준 형식 설계 방향

1. **계층적 구조**: 공통 필드 + 도메인별 데이터
2. **단위 명시**: 모든 물리량에 단위 포함
3. **확장성**: 새로운 나노시스템 유형 추가 용이
4. **호환성**:
   - ISO TC 229 용어 준수
   - IEEE 1906.1 통신 모델 호환
   - PDB 분자 좌표 호환

### 4.2 JSON Schema 적용

- JSON Schema draft-07 사용
- 필수/선택 필드 명확히 구분
- 값 범위 및 형식 검증

### 4.3 참고문헌

1. ISO/TC 229 Nanotechnologies - https://www.iso.org/committee/381983.html
2. IEEE 1906.1-2015 - https://standards.ieee.org/ieee/1906.1/5171/
3. IEEE 1906.1.1-2020 Data Model - https://standards.ieee.org/standard/1906_1_1-2020.html
4. PDBx/mmCIF - https://mmcif.wwpdb.org/
5. DNA Digital Data Storage - https://en.wikipedia.org/wiki/DNA_digital_data_storage
6. Liposome Drug Delivery - https://pmc.ncbi.nlm.nih.gov/articles/PMC8879473/

---

<div align="center">

**WIA Nano Standard - Phase 1 Research**

弘益人間

</div>
