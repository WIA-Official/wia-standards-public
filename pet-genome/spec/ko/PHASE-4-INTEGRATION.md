# WIA-PET-004: Pet Genome 통합 명세서
## Phase 4: 시스템 통합 및 구현

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. 소개

### 1.1 목적
본 명세서는 반려동물 유전체 데이터 시스템을 위한 통합 패턴, 구현 가이드라인 및 배포 전략을 정의합니다. WIA Pet Genome 표준을 기존 수의학 진료 관리 시스템, 번식 데이터베이스, 연구 플랫폼 및 반려동물 보험 시스템과 통합하기 위한 포괄적인 지침을 제공합니다.

### 1.2 적용 범위
본 표준은 다음을 다룹니다:
- 통합 아키텍처 패턴 (Integration Architecture Patterns)
- 레거시 시스템 마이그레이션 전략 (Legacy System Migration)
- 수의학 진료 관리 시스템(VPMS) 통합 (Veterinary Practice Management System Integration)
- 번식 데이터베이스 연결 (Breeding Database Connectivity)
- 반려동물 보험 시스템 통합 (Pet Insurance System Integration)
- 연구 플랫폼 데이터 공유 (Research Platform Data Sharing)
- 모바일 애플리케이션 통합 (Mobile Application Integration)
- 타사 서비스 통합 (Third-party Service Integrations)
- 테스트 및 검증 프레임워크 (Testing and Validation Frameworks)
- 배포 및 롤아웃 전략 (Deployment and Rollout Strategies)
- 성능 최적화 (Performance Optimization)
- 모니터링 및 관찰성 (Monitoring and Observability)

### 1.3 통합 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                     통합 계층 (Integration Layer)                │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │   VPMS   │  │  번식    │  │  보험    │  │  연구    │       │
│  │  어댑터  │  │ 어댑터   │  │ 어댑터   │  │ 어댑터   │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │               │
├───────┴─────────────┴─────────────┴─────────────┴───────────────┤
│              WIA Pet Genome API Gateway                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ 프로파일 │  │  변이    │  │  품종    │  │  건강    │       │
│  │ 서비스   │  │ 서비스   │  │ 서비스   │  │ 서비스   │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │               │
├───────┴─────────────┴─────────────┴─────────────┴───────────────┤
│                    데이터 저장 계층 (Data Storage Layer)         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ 프로파일 │  │  유전체  │  │   동의   │  │  감사    │       │
│  │    DB    │  │  데이터  │  │    DB    │  │  로그    │       │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 수의학 진료 관리 시스템 통합

### 2.1 VPMS 통합 아키텍처

#### 지원되는 VPMS 플랫폼

| VPMS 플랫폼 | 통합 방법 | 데이터 동기화 | 인증 |
|--------------|-------------------|-----------|----------------|
| Avimark | REST API + HL7 FHIR | 실시간 | OAuth 2.0 |
| Cornerstone | SOAP Web Services | 예약 | API Key + JWT |
| ezyVet | REST API | 실시간 | OAuth 2.0 |
| Impromed | HL7 v2.x Messages | 실시간 | SAML 2.0 |
| IDEXX Neo | REST API | 실시간 | OAuth 2.0 |
| VetView | Custom API | 예약 | API Key |

### 2.2 VPMS 통합 어댑터

VPMS 통합 어댑터는 다양한 수의학 시스템과의 표준화된 인터페이스를 제공합니다.

```python
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
from datetime import datetime
import requests

class VPMSAdapter(ABC):
    """VPMS 통합을 위한 추상 기본 클래스"""

    def __init__(self, config: Dict):
        self.config = config
        self.base_url = config['baseUrl']
        self.credentials = config['credentials']

    @abstractmethod
    def authenticate(self) -> str:
        """VPMS 인증 및 액세스 토큰 반환"""
        pass

    @abstractmethod
    def get_patient(self, patient_id: str) -> Dict:
        """VPMS에서 환자 기록 조회"""
        pass

    @abstractmethod
    def create_genomic_result(self, patient_id: str, genomic_data: Dict) -> Dict:
        """VPMS에 유전체 검사 결과 생성"""
        pass

    @abstractmethod
    def sync_patient_data(self, profile_id: str) -> Dict:
        """VPMS와 유전체 시스템 간 환자 데이터 동기화"""
        pass


class AvimarkAdapter(VPMSAdapter):
    """Avimark VPMS 통합 어댑터"""

    def authenticate(self) -> str:
        """OAuth 2.0을 사용한 인증"""
        auth_url = f"{self.base_url}/oauth/token"

        response = requests.post(
            auth_url,
            data={
                'grant_type': 'client_credentials',
                'client_id': self.credentials['clientId'],
                'client_secret': self.credentials['clientSecret'],
                'scope': 'patient:read patient:write lab:write'
            }
        )

        response.raise_for_status()
        return response.json()['access_token']

    def get_patient(self, patient_id: str) -> Dict:
        """Avimark에서 환자 조회"""
        token = self.authenticate()

        response = requests.get(
            f"{self.base_url}/api/v1/patients/{patient_id}",
            headers={'Authorization': f'Bearer {token}'}
        )

        response.raise_for_status()
        return self._transform_patient_data(response.json())

    def create_genomic_result(self, patient_id: str, genomic_data: Dict) -> Dict:
        """Avimark에 유전체 검사 결과 생성"""
        token = self.authenticate()

        # 유전체 데이터를 Avimark 실험실 결과 형식으로 변환
        lab_result = self._transform_genomic_to_lab_result(genomic_data)

        response = requests.post(
            f"{self.base_url}/api/v1/patients/{patient_id}/lab-results",
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': 'application/json'
            },
            json=lab_result
        )

        response.raise_for_status()
        return response.json()

    def sync_patient_data(self, profile_id: str) -> Dict:
        """환자 데이터 동기화"""
        # VPMS에서 환자 조회
        vpms_patient = self.get_patient(profile_id)

        # 유전체 프로파일 조회
        genomic_profile = self._get_genomic_profile(profile_id)

        # 병합 및 업데이트
        merged_data = self._merge_patient_data(vpms_patient, genomic_profile)

        return merged_data

    def _transform_patient_data(self, vpms_data: Dict) -> Dict:
        """VPMS 환자 데이터를 WIA 형식으로 변환"""
        return {
            'petIdentification': {
                'registeredName': vpms_data.get('name'),
                'microchipId': vpms_data.get('microchipNumber'),
                'birthDate': vpms_data.get('dateOfBirth'),
                'sex': vpms_data.get('gender', '').upper()
            },
            'vpmsPatientId': vpms_data.get('id'),
            'ownerInformation': {
                'ownerId': vpms_data.get('ownerId'),
                'ownerName': vpms_data.get('ownerName')
            }
        }

    def _transform_genomic_to_lab_result(self, genomic_data: Dict) -> Dict:
        """유전체 데이터를 VPMS 실험실 결과 형식으로 변환"""
        return {
            'testName': '유전 건강 검사',
            'testDate': datetime.utcnow().isoformat(),
            'testType': 'GENOMIC',
            'laboratory': 'WIA Pet Genome Laboratory',
            'results': [
                {
                    'testComponent': '품종 식별',
                    'value': genomic_data.get('breedInformation', {}).get('primaryBreed', {}).get('breedName'),
                    'unit': 'breed',
                    'referenceRange': 'N/A',
                    'status': 'FINAL'
                },
                {
                    'testComponent': '유전 질환 검사',
                    'value': self._format_disease_results(genomic_data.get('healthMarkers', {})),
                    'notes': self._generate_clinical_notes(genomic_data),
                    'status': 'FINAL'
                }
            ],
            'attachments': [
                {
                    'fileName': 'genomic_report.pdf',
                    'url': genomic_data.get('reportUrl'),
                    'type': 'application/pdf'
                }
            ]
        }

    def _format_disease_results(self, health_markers: Dict) -> str:
        """질병 검사 결과 형식화"""
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if not diseases:
            return '병원성 변이가 감지되지 않음'

        high_risk = [d for d in diseases if d.get('riskLevel') in ['AFFECTED', 'HIGH']]

        if high_risk:
            return f"경고: {len(high_risk)}개 고위험 상태 감지됨"
        else:
            carriers = [d for d in diseases if d.get('riskLevel') == 'CARRIER']
            if carriers:
                return f"{len(carriers)}개 상태에 대한 보인자"
            else:
                return '저위험 프로파일'

    def _generate_clinical_notes(self, genomic_data: Dict) -> str:
        """수의사를 위한 임상 노트 생성"""
        notes = ["유전 건강 검사 결과:\n"]

        # 품종 정보
        breed_info = genomic_data.get('breedInformation', {})
        primary_breed = breed_info.get('primaryBreed', {})
        notes.append(f"품종: {primary_breed.get('breedName')} ({primary_breed.get('percentage')}%)")

        # 질병 위험
        health_markers = genomic_data.get('healthMarkers', {})
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if diseases:
            notes.append("\n유전 건강 소견:")
            for disease in diseases[:5]:  # 상위 5개
                notes.append(f"  - {disease.get('diseaseName')}: {disease.get('riskLevel')}")

        # 약물유전체학
        pharma = health_markers.get('pharmacogenomics', [])
        if pharma:
            notes.append("\n약물 반응 예측:")
            for drug in pharma:
                if drug.get('responseType') != 'NORMAL_METABOLIZER':
                    notes.append(f"  - {drug.get('drugName')}: {drug.get('responseType')}")

        notes.append("\n자세한 결과는 첨부된 유전체 보고서를 참조하십시오.")

        return '\n'.join(notes)

    def _get_genomic_profile(self, profile_id: str) -> Dict:
        """WIA API에서 유전체 프로파일 조회"""
        # WIA Pet Genome API 호출 구현
        pass

    def _merge_patient_data(self, vpms_data: Dict, genomic_data: Dict) -> Dict:
        """VPMS 및 유전체 데이터 병합"""
        return {
            **vpms_data,
            'genomicProfile': genomic_data
        }


class EzyVetAdapter(VPMSAdapter):
    """ezyVet VPMS 통합 어댑터"""

    def authenticate(self) -> str:
        """ezyVet OAuth 2.0 인증"""
        auth_url = f"{self.base_url}/v1/oauth/token"

        response = requests.post(
            auth_url,
            data={
                'grant_type': 'client_credentials',
                'client_id': self.credentials['clientId'],
                'client_secret': self.credentials['clientSecret'],
                'scope': 'full'
            }
        )

        response.raise_for_status()
        return response.json()['access_token']

    def get_patient(self, patient_id: str) -> Dict:
        """ezyVet에서 환자 조회"""
        token = self.authenticate()

        response = requests.get(
            f"{self.base_url}/v1/animal/{patient_id}",
            headers={
                'Authorization': f'Bearer {token}',
                'Accept': 'application/json'
            }
        )

        response.raise_for_status()
        return self._transform_ezyvet_patient(response.json())

    def create_genomic_result(self, patient_id: str, genomic_data: Dict) -> Dict:
        """ezyVet에 유전체 결과 생성"""
        token = self.authenticate()

        # PDF가 첨부된 임상 노트로 생성
        clinical_note = {
            'animal_id': patient_id,
            'note_type': 'Clinical',
            'subject': '유전 건강 검사 결과',
            'description': self._generate_clinical_notes(genomic_data),
            'created_date': datetime.utcnow().isoformat()
        }

        response = requests.post(
            f"{self.base_url}/v1/clinicalnote",
            headers={
                'Authorization': f'Bearer {token}',
                'Content-Type': 'application/json'
            },
            json=clinical_note
        )

        response.raise_for_status()
        return response.json()

    def sync_patient_data(self, profile_id: str) -> Dict:
        """ezyVet 환자 데이터 동기화"""
        # Avimark와 유사한 구현
        pass

    def _transform_ezyvet_patient(self, ezyvet_data: Dict) -> Dict:
        """ezyVet 데이터를 WIA 형식으로 변환"""
        animal = ezyvet_data.get('animal', {})

        return {
            'petIdentification': {
                'registeredName': animal.get('name'),
                'microchipId': animal.get('microchip_number'),
                'birthDate': animal.get('date_of_birth'),
                'sex': animal.get('sex', '').upper()
            },
            'vpmsPatientId': animal.get('id'),
            'species': animal.get('species_name')
        }

    def _generate_clinical_notes(self, genomic_data: Dict) -> str:
        """임상 노트 생성 (Avimark와 유사)"""
        # Avimark 어댑터의 구현 재사용
        pass

# 사용 예제
config = {
    'baseUrl': 'https://api.avimark.com',
    'credentials': {
        'clientId': 'your-client-id',
        'clientSecret': 'your-client-secret'
    }
}

avimark = AvimarkAdapter(config)

# 환자 조회
patient = avimark.get_patient('PATIENT-12345')

# 유전체 결과 생성
genomic_data = {
    'profileId': 'PGP-ABCD12345678',
    'breedInformation': {
        'primaryBreed': {
            'breedName': 'Labrador Retriever',
            'percentage': 87.5
        }
    },
    'healthMarkers': {
        'geneticDiseaseRisks': [
            {
                'diseaseName': 'Progressive Retinal Atrophy',
                'riskLevel': 'CARRIER'
            }
        ]
    },
    'reportUrl': 'https://reports.pet-genome.wia.org/report.pdf'
}

result = avimark.create_genomic_result('PATIENT-12345', genomic_data)
print(f"실험실 결과 생성됨: {result['id']}")
```

---

## 3. 번식 데이터베이스 통합

### 3.1 혈통 관리 시스템 통합

번식 데이터베이스 통합은 유전 정보를 번식 결정에 통합하여 건강한 자손을 생산하는 데 중요합니다.

```python
from typing import List, Dict, Optional
from datetime import datetime

class PedigreeSystemIntegration:
    """번식 및 혈통 관리 시스템과의 통합"""

    def __init__(self, api_config: Dict):
        self.api_config = api_config
        self.base_url = api_config['baseUrl']

    def export_to_pedigree_system(self, genomic_profile: Dict) -> Dict:
        """
        유전체 데이터를 혈통 관리 시스템으로 내보내기

        Args:
            genomic_profile: WIA 유전체 프로파일

        Returns:
            혈통 시스템 기록
        """
        pedigree_record = {
            'registrationId': genomic_profile['profileId'],
            'individualData': {
                'name': genomic_profile['petIdentification']['registeredName'],
                'microchipId': genomic_profile['petIdentification']['microchipId'],
                'birthDate': genomic_profile['petIdentification']['birthDate'],
                'sex': genomic_profile['petIdentification']['sex'],
                'breed': genomic_profile['breedInformation']['primaryBreed']['breedName']
            },
            'geneticData': {
                'breedComposition': self._format_breed_composition(genomic_profile),
                'inbreedingCoefficient': self._extract_inbreeding_coefficient(genomic_profile),
                'geneticDiversity': self._extract_diversity_metrics(genomic_profile),
                'healthScreening': self._format_health_screening(genomic_profile)
            },
            'breedingRecommendations': self._generate_breeding_recommendations(genomic_profile),
            'exportDate': datetime.utcnow().isoformat() + 'Z',
            'dataVersion': '1.0'
        }

        return pedigree_record

    def calculate_breeding_compatibility(self,
                                        profile1_id: str,
                                        profile2_id: str) -> Dict:
        """
        두 동물 간의 번식 호환성 계산

        Args:
            profile1_id: 첫 번째 부모 프로파일 ID
            profile2_id: 두 번째 부모 프로파일 ID

        Returns:
            호환성 평가
        """
        # 두 프로파일 모두 가져오기
        profile1 = self._get_profile(profile1_id)
        profile2 = self._get_profile(profile2_id)

        # 잠재적 자손의 근친교배 계수 계산
        offspring_inbreeding = self._predict_offspring_inbreeding(profile1, profile2)

        # 호환 가능한 질병 보인자 확인
        disease_compatibility = self._check_disease_compatibility(profile1, profile2)

        # 자손의 유전적 다양성 계산
        predicted_diversity = self._predict_offspring_diversity(profile1, profile2)

        # 품종 표준 준수
        breed_compliance = self._check_breed_standard_compliance(profile1, profile2)

        compatibility_score = self._calculate_compatibility_score(
            offspring_inbreeding,
            disease_compatibility,
            predicted_diversity,
            breed_compliance
        )

        return {
            'parent1': profile1_id,
            'parent2': profile2_id,
            'compatibilityScore': compatibility_score,
            'recommendedBreeding': compatibility_score >= 0.7,
            'predictions': {
                'offspringInbreeding': offspring_inbreeding,
                'offspringDiversity': predicted_diversity,
                'diseaseRisks': disease_compatibility['risks']
            },
            'breedStandardCompliance': breed_compliance,
            'recommendations': self._generate_pairing_recommendations(
                offspring_inbreeding,
                disease_compatibility,
                predicted_diversity
            )
        }

    def _format_breed_composition(self, profile: Dict) -> List[Dict]:
        """혈통 시스템을 위한 품종 구성 형식화"""
        composition = profile.get('breedInformation', {}).get('ancestryComposition', [])
        return [
            {
                'breed': item['breedName'],
                'percentage': item['percentage'],
                'confidence': item.get('confidence', 0.95)
            }
            for item in composition
        ]

    def _extract_inbreeding_coefficient(self, profile: Dict) -> float:
        """근친교배 계수 추출"""
        diversity = profile.get('breedInformation', {}).get('geneticDiversity', {})
        return diversity.get('inbreedingCoefficient', 0.0)

    def _extract_diversity_metrics(self, profile: Dict) -> Dict:
        """유전적 다양성 지표 추출"""
        diversity = profile.get('breedInformation', {}).get('geneticDiversity', {})
        return {
            'heterozygosity': diversity.get('heterozygosity', 0.0),
            'effectivePopulationSize': diversity.get('effectivePopulationSize', 0)
        }

    def _format_health_screening(self, profile: Dict) -> Dict:
        """건강 검사 결과 형식화"""
        health = profile.get('healthMarkers', {})
        diseases = health.get('geneticDiseaseRisks', [])

        return {
            'affected': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'AFFECTED'],
            'carrier': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'CARRIER'],
            'clear': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'LOW']
        }

    def _generate_breeding_recommendations(self, profile: Dict) -> List[str]:
        """번식 권장사항 생성"""
        recommendations = []

        inbreeding = self._extract_inbreeding_coefficient(profile)
        if inbreeding > 0.125:
            recommendations.append("높은 근친교배 계수 - 외래교배 권장")

        health = self._format_health_screening(profile)
        if health['affected']:
            recommendations.append(f"번식 금지 - 발병: {', '.join(health['affected'])}")
        elif health['carrier']:
            recommendations.append(f"보인자: {', '.join(health['carrier'])} - 교배 상대 검사 필요")

        return recommendations

    def _predict_offspring_inbreeding(self, profile1: Dict, profile2: Dict) -> float:
        """자손의 근친교배 계수 예측"""
        # 간소화된 계산
        f1 = self._extract_inbreeding_coefficient(profile1)
        f2 = self._extract_inbreeding_coefficient(profile2)

        # 부모 근친교배 평균 + 친족 계수
        kinship = self._calculate_kinship(profile1, profile2)

        return (f1 + f2) / 2 + kinship

    def _check_disease_compatibility(self, profile1: Dict, profile2: Dict) -> Dict:
        """질병 보인자 호환성 확인"""
        health1 = self._format_health_screening(profile1)
        health2 = self._format_health_screening(profile2)

        # 공통 보인자 찾기 (열성 질환에 위험)
        common_carriers = set(health1['carrier']) & set(health2['carrier'])

        risks = []
        if common_carriers:
            for disease in common_carriers:
                risks.append({
                    'disease': disease,
                    'riskLevel': 'HIGH',
                    'probability': 0.25,  # 자손이 발병할 확률 25%
                    'recommendation': f"{disease} 발병 자손이 25% 확률로 나타날 수 있음"
                })

        return {
            'compatible': len(common_carriers) == 0,
            'risks': risks,
            'commonCarriers': list(common_carriers)
        }

    def _predict_offspring_diversity(self, profile1: Dict, profile2: Dict) -> float:
        """자손의 유전적 다양성 예측"""
        div1 = self._extract_diversity_metrics(profile1)
        div2 = self._extract_diversity_metrics(profile2)

        # 부모 이형접합도 평균
        return (div1['heterozygosity'] + div2['heterozygosity']) / 2

    def _check_breed_standard_compliance(self, profile1: Dict, profile2: Dict) -> Dict:
        """품종 표준 준수 확인"""
        breed1 = profile1.get('breedInformation', {}).get('primaryBreed', {})
        breed2 = profile2.get('breedInformation', {}).get('primaryBreed', {})

        if breed1.get('breedName') != breed2.get('breedName'):
            return {
                'compliant': False,
                'reason': '교잡 교배'
            }

        # 순혈 확인
        purebred1 = breed1.get('percentage', 0) >= 87.5
        purebred2 = breed2.get('percentage', 0) >= 87.5

        return {
            'compliant': purebred1 and purebred2,
            'parent1Purebred': purebred1,
            'parent2Purebred': purebred2,
            'expectedOffspringPurity': (breed1.get('percentage', 0) + breed2.get('percentage', 0)) / 2
        }

    def _calculate_kinship(self, profile1: Dict, profile2: Dict) -> float:
        """두 동물 간의 친족 계수 계산"""
        # 간소화 - 실제 유전체 데이터를 사용
        return 0.0

    def _calculate_compatibility_score(self,
                                      inbreeding: float,
                                      disease_compat: Dict,
                                      diversity: float,
                                      breed_compliance: Dict) -> float:
        """전체 호환성 점수 계산 (0-1)"""
        score = 1.0

        # 높은 근친교배에 페널티
        if inbreeding > 0.125:
            score -= 0.3
        elif inbreeding > 0.0625:
            score -= 0.1

        # 질병 비호환성에 페널티
        if not disease_compat['compatible']:
            score -= 0.4

        # 높은 다양성에 보상
        if diversity > 0.35:
            score += 0.1

        # 품종 표준 비준수에 페널티
        if not breed_compliance['compliant']:
            score -= 0.2

        return max(0.0, min(1.0, score))

    def _generate_pairing_recommendations(self,
                                         inbreeding: float,
                                         disease_compat: Dict,
                                         diversity: float) -> List[str]:
        """교배 특정 권장사항 생성"""
        recommendations = []

        if inbreeding > 0.125:
            recommendations.append("경고: 높은 예측 근친교배 계수")

        if not disease_compat['compatible']:
            recommendations.append(f"경고: 모두 {', '.join(disease_compat['commonCarriers'])}의 보인자")
            recommendations.append("발병 자손을 피하기 위해 대체 교배 고려")

        if diversity < 0.30:
            recommendations.append("낮은 유전적 다양성 예측 - 외래교배 고려")

        if not recommendations:
            recommendations.append("유전 분석에 기반한 적합한 교배")

        return recommendations

    def _get_profile(self, profile_id: str) -> Dict:
        """유전체 프로파일 조회"""
        # API에서 가져오기 구현
        pass

# 사용 예제
pedigree_integration = PedigreeSystemIntegration({
    'baseUrl': 'https://pedigree-system.example.com'
})

# 번식 호환성 계산
compatibility = pedigree_integration.calculate_breeding_compatibility(
    profile1_id='PGP-SIRE-001',
    profile2_id='PGP-DAM-002'
)

print(f"호환성 점수: {compatibility['compatibilityScore']}")
print(f"권장: {compatibility['recommendedBreeding']}")
print(f"예측 자손 근친교배: {compatibility['predictions']['offspringInbreeding']}")
```

---

## 4. 테스트 및 검증 프레임워크

### 4.1 통합 테스트 스위트

```python
import unittest
from typing import Dict
import json

class PetGenomeIntegrationTests(unittest.TestCase):
    """포괄적인 통합 테스트 스위트"""

    def setUp(self):
        """테스트 픽스처 설정"""
        self.test_profile = self._load_test_profile()
        self.vpms_adapter = None  # 테스트 구성으로 초기화

    def test_vpms_patient_sync(self):
        """VPMS 환자 동기화 테스트"""
        # VPMS에서 테스트 환자 생성
        vpms_patient = {
            'id': 'TEST-PATIENT-001',
            'name': 'Test Dog',
            'microchipNumber': '123456789012345',
            'dateOfBirth': '2023-01-01',
            'gender': 'MALE'
        }

        # 유전체 시스템으로 동기화
        genomic_profile = self._sync_patient_to_genomic(vpms_patient)

        # 어설션
        self.assertEqual(
            genomic_profile['petIdentification']['microchipId'],
            vpms_patient['microchipNumber']
        )
        self.assertEqual(
            genomic_profile['petIdentification']['registeredName'],
            vpms_patient['name']
        )

    def test_genomic_result_export(self):
        """유전체 결과의 VPMS 내보내기 테스트"""
        # 유전체 결과 생성
        result = self._create_genomic_result(self.test_profile)

        # VPMS로 내보내기
        vpms_result = self._export_to_vpms(result)

        # 내보내기 검증
        self.assertIsNotNone(vpms_result)
        self.assertEqual(vpms_result['testType'], 'GENOMIC')

    def test_breeding_compatibility_calculation(self):
        """번식 호환성 계산 테스트"""
        pedigree = PedigreeSystemIntegration({})

        compatibility = pedigree.calculate_breeding_compatibility(
            'PGP-TEST-001',
            'PGP-TEST-002'
        )

        # 계산 검증
        self.assertIn('compatibilityScore', compatibility)
        self.assertGreaterEqual(compatibility['compatibilityScore'], 0.0)
        self.assertLessEqual(compatibility['compatibilityScore'], 1.0)

    def test_data_encryption_roundtrip(self):
        """암호화 및 복호화 테스트"""
        import os
        from pet_genome_sdk import GenomicDataEncryption

        key = os.urandom(32)
        encryptor = GenomicDataEncryption(key)

        # 암호화
        encrypted = encryptor.encrypt_profile(self.test_profile)

        # 복호화
        decrypted = encryptor.decrypt_profile(encrypted)

        # 검증
        self.assertEqual(decrypted, self.test_profile)

    def _load_test_profile(self) -> Dict:
        """테스트 유전체 프로파일 로드"""
        return {
            'profileId': 'PGP-TEST-001',
            'speciesCode': 'CANIS_FAMILIARIS',
            'petIdentification': {
                'registeredName': 'Test Dog',
                'microchipId': '123456789012345',
                'birthDate': '2023-01-01',
                'sex': 'MALE'
            }
        }

    def _sync_patient_to_genomic(self, vpms_patient: Dict) -> Dict:
        """VPMS 환자를 유전체 시스템으로 동기화"""
        # 구현
        pass

    def _create_genomic_result(self, profile: Dict) -> Dict:
        """유전체 검사 결과 생성"""
        # 구현
        pass

    def _export_to_vpms(self, result: Dict) -> Dict:
        """결과를 VPMS로 내보내기"""
        # 구현
        pass

# 테스트 실행
if __name__ == '__main__':
    unittest.main()
```

---

## 5. 배포 전략

### 5.1 단계별 롤아웃 계획

| 단계 | 기간 | 활동 | 성공 기준 |
|-------|----------|------------|------------------|
| 단계 1: 파일럿 | 4주 | 5개 수의 클리닉, 100건 검사 | >95% 데이터 정확도, <500ms API 응답 |
| 단계 2: 제한적 | 8주 | 25개 클리닉, 1000건 검사 | >99% 가동 시간, 성공적인 VPMS 통합 |
| 단계 3: 지역적 | 12주 | 100개 클리닉, 10,000건 검사 | <1% 오류율, 전체 기능 동등성 |
| 단계 4: 전체 출시 | 진행 중 | 모든 참여자 | 99.9% SLA 준수 |

### 5.2 배포 체크리스트

배포 전 준비, 배포 실행, 배포 후 검증 단계로 구성됩니다.

---

## 6. 모니터링 및 관찰성

### 6.1 주요 지표

| 지표 카테고리 | 구체적 지표 | 목표 | 경고 임계값 |
|----------------|------------------|--------|-----------------|
| API 성능 | 응답 시간 (p95) | <500ms | >1000ms |
| API 성능 | 응답 시간 (p99) | <1000ms | >2000ms |
| 가용성 | 가동 시간 | 99.9% | <99.5% |
| 오류율 | HTTP 5xx 오류 | <0.1% | >0.5% |
| 데이터 처리 | VCF 처리 시간 | <30분 | >60분 |
| 보안 | 인증 실패 시도 | <1% | >5% |
| 비즈니스 | 일일 활성 사용자 | 성장 | >10% 감소 |
| 비즈니스 | 처리된 검사/일 | 성장 | >10% 감소 |

---

## 7. 버전 이력

| Version | Date | 변경사항 | 작성자 |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | 초기 통합 명세서 | WIA 표준 위원회 |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
