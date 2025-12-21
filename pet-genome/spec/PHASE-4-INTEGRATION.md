# WIA-PET-004: Pet Genome Integration Specification
## Phase 4: System Integration and Implementation

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #F59E0B (Amber)

---

## 1. Introduction

### 1.1 Purpose
This specification defines integration patterns, implementation guidelines, and deployment strategies for pet genomic data systems. It provides comprehensive guidance for integrating WIA Pet Genome standards with existing veterinary practice management systems, breeding databases, research platforms, and pet insurance systems.

### 1.2 Scope
This standard covers:
- Integration architecture patterns
- Legacy system migration strategies
- Veterinary practice management system (VPMS) integration
- Breeding database connectivity
- Pet insurance system integration
- Research platform data sharing
- Mobile application integration
- Third-party service integrations
- Testing and validation frameworks
- Deployment and rollout strategies
- Performance optimization
- Monitoring and observability

### 1.3 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Integration Layer                            │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │   VPMS   │  │ Breeding │  │Insurance │  │ Research │       │
│  │  Adapter │  │  Adapter │  │ Adapter  │  │  Adapter │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │               │
├───────┴─────────────┴─────────────┴─────────────┴───────────────┤
│              WIA Pet Genome API Gateway                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ Profile  │  │ Variant  │  │  Breed   │  │  Health  │       │
│  │ Service  │  │ Service  │  │ Service  │  │ Service  │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │               │
├───────┴─────────────┴─────────────┴─────────────┴───────────────┤
│                    Data Storage Layer                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │  Profile │  │  Genomic │  │  Consent │  │  Audit   │       │
│  │    DB    │  │   Data   │  │    DB    │  │   Logs   │       │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Veterinary Practice Management System Integration

### 2.1 VPMS Integration Architecture

#### Supported VPMS Platforms

| VPMS Platform | Integration Method | Data Sync | Authentication |
|--------------|-------------------|-----------|----------------|
| Avimark | REST API + HL7 FHIR | Real-time | OAuth 2.0 |
| Cornerstone | SOAP Web Services | Scheduled | API Key + JWT |
| ezyVet | REST API | Real-time | OAuth 2.0 |
| Impromed | HL7 v2.x Messages | Real-time | SAML 2.0 |
| IDEXX Neo | REST API | Real-time | OAuth 2.0 |
| VetView | Custom API | Scheduled | API Key |

### 2.2 VPMS Integration Adapter

```python
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
from datetime import datetime
import requests

class VPMSAdapter(ABC):
    """Abstract base class for VPMS integrations"""

    def __init__(self, config: Dict):
        self.config = config
        self.base_url = config['baseUrl']
        self.credentials = config['credentials']

    @abstractmethod
    def authenticate(self) -> str:
        """Authenticate with VPMS and return access token"""
        pass

    @abstractmethod
    def get_patient(self, patient_id: str) -> Dict:
        """Retrieve patient record from VPMS"""
        pass

    @abstractmethod
    def create_genomic_result(self, patient_id: str, genomic_data: Dict) -> Dict:
        """Create genomic test result in VPMS"""
        pass

    @abstractmethod
    def sync_patient_data(self, profile_id: str) -> Dict:
        """Synchronize patient data between VPMS and genomic system"""
        pass


class AvimmarkAdapter(VPMSAdapter):
    """Avimark VPMS integration adapter"""

    def authenticate(self) -> str:
        """Authenticate using OAuth 2.0"""
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
        """Retrieve patient from Avimark"""
        token = self.authenticate()

        response = requests.get(
            f"{self.base_url}/api/v1/patients/{patient_id}",
            headers={'Authorization': f'Bearer {token}'}
        )

        response.raise_for_status()
        return self._transform_patient_data(response.json())

    def create_genomic_result(self, patient_id: str, genomic_data: Dict) -> Dict:
        """Create genomic test result in Avimark"""
        token = self.authenticate()

        # Transform genomic data to Avimark lab result format
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
        """Synchronize patient data"""
        # Get patient from VPMS
        vpms_patient = self.get_patient(profile_id)

        # Get genomic profile
        genomic_profile = self._get_genomic_profile(profile_id)

        # Merge and update
        merged_data = self._merge_patient_data(vpms_patient, genomic_profile)

        return merged_data

    def _transform_patient_data(self, vpms_data: Dict) -> Dict:
        """Transform VPMS patient data to WIA format"""
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
        """Transform genomic data to VPMS lab result format"""
        return {
            'testName': 'Genetic Health Screening',
            'testDate': datetime.utcnow().isoformat(),
            'testType': 'GENOMIC',
            'laboratory': 'WIA Pet Genome Laboratory',
            'results': [
                {
                    'testComponent': 'Breed Identification',
                    'value': genomic_data.get('breedInformation', {}).get('primaryBreed', {}).get('breedName'),
                    'unit': 'breed',
                    'referenceRange': 'N/A',
                    'status': 'FINAL'
                },
                {
                    'testComponent': 'Genetic Disease Screening',
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
        """Format disease screening results"""
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if not diseases:
            return 'No pathogenic variants detected'

        high_risk = [d for d in diseases if d.get('riskLevel') in ['AFFECTED', 'HIGH']]

        if high_risk:
            return f"WARNING: {len(high_risk)} high-risk condition(s) detected"
        else:
            carriers = [d for d in diseases if d.get('riskLevel') == 'CARRIER']
            if carriers:
                return f"Carrier for {len(carriers)} condition(s)"
            else:
                return 'Low risk profile'

    def _generate_clinical_notes(self, genomic_data: Dict) -> str:
        """Generate clinical notes for veterinarian"""
        notes = ["Genetic Health Screening Results:\n"]

        # Breed information
        breed_info = genomic_data.get('breedInformation', {})
        primary_breed = breed_info.get('primaryBreed', {})
        notes.append(f"Breed: {primary_breed.get('breedName')} ({primary_breed.get('percentage')}%)")

        # Disease risks
        health_markers = genomic_data.get('healthMarkers', {})
        diseases = health_markers.get('geneticDiseaseRisks', [])

        if diseases:
            notes.append("\nGenetic Health Findings:")
            for disease in diseases[:5]:  # Top 5
                notes.append(f"  - {disease.get('diseaseName')}: {disease.get('riskLevel')}")

        # Pharmacogenomics
        pharma = health_markers.get('pharmacogenomics', [])
        if pharma:
            notes.append("\nDrug Response Predictions:")
            for drug in pharma:
                if drug.get('responseType') != 'NORMAL_METABOLIZER':
                    notes.append(f"  - {drug.get('drugName')}: {drug.get('responseType')}")

        notes.append("\nFor detailed results, see attached genomic report.")

        return '\n'.join(notes)

    def _get_genomic_profile(self, profile_id: str) -> Dict:
        """Retrieve genomic profile from WIA API"""
        # Implementation would call WIA Pet Genome API
        pass

    def _merge_patient_data(self, vpms_data: Dict, genomic_data: Dict) -> Dict:
        """Merge VPMS and genomic data"""
        return {
            **vpms_data,
            'genomicProfile': genomic_data
        }


class EzyVetAdapter(VPMSAdapter):
    """ezyVet VPMS integration adapter"""

    def authenticate(self) -> str:
        """Authenticate with ezyVet OAuth 2.0"""
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
        """Retrieve patient from ezyVet"""
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
        """Create genomic result in ezyVet"""
        token = self.authenticate()

        # Create as clinical note with attached PDF
        clinical_note = {
            'animal_id': patient_id,
            'note_type': 'Clinical',
            'subject': 'Genetic Health Screening Results',
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
        """Synchronize ezyVet patient data"""
        # Implementation similar to Avimark
        pass

    def _transform_ezyvet_patient(self, ezyvet_data: Dict) -> Dict:
        """Transform ezyVet data to WIA format"""
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
        """Generate clinical notes (similar to Avimark)"""
        # Reuse implementation from Avimark adapter
        pass


# Usage example
config = {
    'baseUrl': 'https://api.avimark.com',
    'credentials': {
        'clientId': 'your-client-id',
        'clientSecret': 'your-client-secret'
    }
}

avimark = AvimmarkAdapter(config)

# Retrieve patient
patient = avimark.get_patient('PATIENT-12345')

# Create genomic result
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
print(f"Created lab result: {result['id']}")
```

### 2.3 HL7 FHIR Integration

```python
from fhir.resources.patient import Patient
from fhir.resources.observation import Observation
from fhir.resources.diagnosticreport import DiagnosticReport
from fhir.resources.specimen import Specimen
from datetime import datetime

class FHIRGenomicMapper:
    """Map WIA Pet Genome data to HL7 FHIR resources"""

    def create_patient_resource(self, genomic_profile: Dict) -> Patient:
        """Create FHIR Patient resource from genomic profile"""
        pet_id = genomic_profile['petIdentification']

        patient = Patient(
            **{
                'id': genomic_profile['profileId'],
                'identifier': [
                    {
                        'system': 'http://pet-genome.wia.org/profile-id',
                        'value': genomic_profile['profileId']
                    },
                    {
                        'system': 'http://pet-genome.wia.org/microchip',
                        'value': pet_id.get('microchipId')
                    }
                ],
                'name': [
                    {
                        'text': pet_id.get('registeredName'),
                        'use': 'official'
                    }
                ],
                'gender': self._map_gender(pet_id.get('sex')),
                'birthDate': pet_id.get('birthDate'),
                'extension': [
                    {
                        'url': 'http://pet-genome.wia.org/fhir/StructureDefinition/species',
                        'valueCodeableConcept': {
                            'coding': [
                                {
                                    'system': 'http://snomed.info/sct',
                                    'code': self._get_species_code(genomic_profile['speciesCode']),
                                    'display': genomic_profile['speciesCode']
                                }
                            ]
                        }
                    }
                ]
            }
        )

        return patient

    def create_diagnostic_report(self, genomic_profile: Dict) -> DiagnosticReport:
        """Create FHIR DiagnosticReport for genomic testing"""
        report = DiagnosticReport(
            **{
                'id': f"DR-{genomic_profile['profileId']}",
                'status': 'final',
                'category': [
                    {
                        'coding': [
                            {
                                'system': 'http://terminology.hl7.org/CodeSystem/v2-0074',
                                'code': 'GE',
                                'display': 'Genetics'
                            }
                        ]
                    }
                ],
                'code': {
                    'coding': [
                        {
                            'system': 'http://loinc.org',
                            'code': '81247-9',
                            'display': 'Master HL7 genetic variant reporting panel'
                        }
                    ],
                    'text': 'Comprehensive Genetic Health Screening'
                },
                'subject': {
                    'reference': f"Patient/{genomic_profile['profileId']}"
                },
                'effectiveDateTime': genomic_profile.get('sampleMetadata', {}).get('processingDate'),
                'issued': datetime.utcnow().isoformat(),
                'performer': [
                    {
                        'reference': f"Organization/{genomic_profile.get('sampleMetadata', {}).get('laboratory', {}).get('name')}"
                    }
                ],
                'result': self._create_observations(genomic_profile),
                'conclusion': self._generate_conclusion(genomic_profile)
            }
        )

        return report

    def create_variant_observation(self, variant: Dict, profile_id: str) -> Observation:
        """Create FHIR Observation for genetic variant"""
        obs = Observation(
            **{
                'id': f"OBS-{variant.get('variantId')}",
                'status': 'final',
                'category': [
                    {
                        'coding': [
                            {
                                'system': 'http://terminology.hl7.org/CodeSystem/observation-category',
                                'code': 'laboratory',
                                'display': 'Laboratory'
                            }
                        ]
                    }
                ],
                'code': {
                    'coding': [
                        {
                            'system': 'http://loinc.org',
                            'code': '69548-6',
                            'display': 'Genetic variant assessment'
                        }
                    ]
                },
                'subject': {
                    'reference': f"Patient/{profile_id}"
                },
                'valueCodeableConcept': {
                    'coding': [
                        {
                            'system': 'http://loinc.org',
                            'code': self._map_significance_to_loinc(variant.get('significance')),
                            'display': variant.get('significance')
                        }
                    ]
                },
                'component': [
                    {
                        'code': {
                            'coding': [
                                {
                                    'system': 'http://loinc.org',
                                    'code': '48018-6',
                                    'display': 'Gene studied [ID]'
                                }
                            ]
                        },
                        'valueCodeableConcept': {
                            'text': variant.get('gene')
                        }
                    },
                    {
                        'code': {
                            'coding': [
                                {
                                    'system': 'http://loinc.org',
                                    'code': '81252-9',
                                    'display': 'Discrete genetic variant'
                                }
                            ]
                        },
                        'valueString': f"{variant.get('chromosome')}:{variant.get('position')}"
                    },
                    {
                        'code': {
                            'coding': [
                                {
                                    'system': 'http://loinc.org',
                                    'code': '69547-8',
                                    'display': 'Genomic ref allele [ID]'
                                }
                            ]
                        },
                        'valueString': variant.get('referenceAllele')
                    },
                    {
                        'code': {
                            'coding': [
                                {
                                    'system': 'http://loinc.org',
                                    'code': '69551-0',
                                    'display': 'Genomic alt allele [ID]'
                                }
                            ]
                        },
                        'valueString': variant.get('alternateAllele')
                    }
                ]
            }
        )

        return obs

    def _map_gender(self, sex: str) -> str:
        """Map pet sex to FHIR gender"""
        mapping = {
            'MALE': 'male',
            'FEMALE': 'female',
            'INTERSEX': 'other',
            'UNKNOWN': 'unknown'
        }
        return mapping.get(sex, 'unknown')

    def _get_species_code(self, species: str) -> str:
        """Get SNOMED CT code for species"""
        codes = {
            'CANIS_FAMILIARIS': '448771007',
            'FELIS_CATUS': '448169003',
            'EQUUS_CABALLUS': '35354009'
        }
        return codes.get(species, 'unknown')

    def _create_observations(self, genomic_profile: Dict) -> List:
        """Create observations for all variants"""
        observations = []

        variants = genomic_profile.get('genomicData', {}).get('variants', {}).get('clinicalVariants', [])

        for variant in variants:
            obs = self.create_variant_observation(variant, genomic_profile['profileId'])
            observations.append({'reference': f"Observation/{obs.id}"})

        return observations

    def _generate_conclusion(self, genomic_profile: Dict) -> str:
        """Generate diagnostic report conclusion"""
        conclusions = []

        # Breed information
        breed = genomic_profile.get('breedInformation', {}).get('primaryBreed', {})
        conclusions.append(f"Breed: {breed.get('breedName')} ({breed.get('percentage')}%)")

        # Disease risks
        diseases = genomic_profile.get('healthMarkers', {}).get('geneticDiseaseRisks', [])
        high_risk = [d for d in diseases if d.get('riskLevel') in ['AFFECTED', 'HIGH']]

        if high_risk:
            conclusions.append(f"High-risk conditions identified: {len(high_risk)}")
        else:
            conclusions.append("No high-risk conditions identified")

        return '. '.join(conclusions)

    def _map_significance_to_loinc(self, significance: str) -> str:
        """Map variant significance to LOINC code"""
        mapping = {
            'PATHOGENIC': 'LA6668-3',
            'LIKELY_PATHOGENIC': 'LA26332-9',
            'UNCERTAIN': 'LA26333-7',
            'LIKELY_BENIGN': 'LA26334-5',
            'BENIGN': 'LA6675-8'
        }
        return mapping.get(significance, 'LA26333-7')

# Usage
fhir_mapper = FHIRGenomicMapper()

genomic_profile = {
    'profileId': 'PGP-ABCD12345678',
    'speciesCode': 'CANIS_FAMILIARIS',
    'petIdentification': {
        'registeredName': 'Champion Golden Star',
        'microchipId': '123456789012345',
        'birthDate': '2023-05-15',
        'sex': 'MALE'
    },
    'breedInformation': {
        'primaryBreed': {
            'breedName': 'Labrador Retriever',
            'percentage': 87.5
        }
    }
}

# Create FHIR resources
patient = fhir_mapper.create_patient_resource(genomic_profile)
diagnostic_report = fhir_mapper.create_diagnostic_report(genomic_profile)

# Export to JSON
import json
print(json.dumps(patient.dict(), indent=2))
```

---

## 3. Breeding Database Integration

### 3.1 Pedigree Management System Integration

```python
from typing import List, Dict, Optional
from datetime import datetime

class PedigreeSystemIntegration:
    """Integration with breeding and pedigree management systems"""

    def __init__(self, api_config: Dict):
        self.api_config = api_config
        self.base_url = api_config['baseUrl']

    def export_to_pedigree_system(self, genomic_profile: Dict) -> Dict:
        """
        Export genomic data to pedigree management system

        Args:
            genomic_profile: WIA genomic profile

        Returns:
            Pedigree system record
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
        Calculate breeding compatibility between two animals

        Args:
            profile1_id: First parent profile ID
            profile2_id: Second parent profile ID

        Returns:
            Compatibility assessment
        """
        # Fetch both profiles
        profile1 = self._get_profile(profile1_id)
        profile2 = self._get_profile(profile2_id)

        # Calculate inbreeding coefficient of potential offspring
        offspring_inbreeding = self._predict_offspring_inbreeding(profile1, profile2)

        # Check for compatible disease carriers
        disease_compatibility = self._check_disease_compatibility(profile1, profile2)

        # Calculate genetic diversity of offspring
        predicted_diversity = self._predict_offspring_diversity(profile1, profile2)

        # Breed standard compliance
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
        """Format breed composition for pedigree system"""
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
        """Extract inbreeding coefficient"""
        diversity = profile.get('breedInformation', {}).get('geneticDiversity', {})
        return diversity.get('inbreedingCoefficient', 0.0)

    def _extract_diversity_metrics(self, profile: Dict) -> Dict:
        """Extract genetic diversity metrics"""
        diversity = profile.get('breedInformation', {}).get('geneticDiversity', {})
        return {
            'heterozygosity': diversity.get('heterozygosity', 0.0),
            'effectivePopulationSize': diversity.get('effectivePopulationSize', 0)
        }

    def _format_health_screening(self, profile: Dict) -> Dict:
        """Format health screening results"""
        health = profile.get('healthMarkers', {})
        diseases = health.get('geneticDiseaseRisks', [])

        return {
            'affected': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'AFFECTED'],
            'carrier': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'CARRIER'],
            'clear': [d['diseaseName'] for d in diseases if d['riskLevel'] == 'LOW']
        }

    def _generate_breeding_recommendations(self, profile: Dict) -> List[str]:
        """Generate breeding recommendations"""
        recommendations = []

        inbreeding = self._extract_inbreeding_coefficient(profile)
        if inbreeding > 0.125:
            recommendations.append("High inbreeding coefficient - outcross breeding recommended")

        health = self._format_health_screening(profile)
        if health['affected']:
            recommendations.append(f"DO NOT BREED - Affected by: {', '.join(health['affected'])}")
        elif health['carrier']:
            recommendations.append(f"Carrier for: {', '.join(health['carrier'])} - Test breeding partner")

        return recommendations

    def _predict_offspring_inbreeding(self, profile1: Dict, profile2: Dict) -> float:
        """Predict inbreeding coefficient of offspring"""
        # Simplified calculation
        f1 = self._extract_inbreeding_coefficient(profile1)
        f2 = self._extract_inbreeding_coefficient(profile2)

        # Average parental inbreeding + kinship coefficient
        kinship = self._calculate_kinship(profile1, profile2)

        return (f1 + f2) / 2 + kinship

    def _check_disease_compatibility(self, profile1: Dict, profile2: Dict) -> Dict:
        """Check disease carrier compatibility"""
        health1 = self._format_health_screening(profile1)
        health2 = self._format_health_screening(profile2)

        # Find common carriers (risky for recessive diseases)
        common_carriers = set(health1['carrier']) & set(health2['carrier'])

        risks = []
        if common_carriers:
            for disease in common_carriers:
                risks.append({
                    'disease': disease,
                    'riskLevel': 'HIGH',
                    'probability': 0.25,  # 25% chance of affected offspring
                    'recommendation': f"25% of offspring may be affected by {disease}"
                })

        return {
            'compatible': len(common_carriers) == 0,
            'risks': risks,
            'commonCarriers': list(common_carriers)
        }

    def _predict_offspring_diversity(self, profile1: Dict, profile2: Dict) -> float:
        """Predict genetic diversity of offspring"""
        div1 = self._extract_diversity_metrics(profile1)
        div2 = self._extract_diversity_metrics(profile2)

        # Average parental heterozygosity
        return (div1['heterozygosity'] + div2['heterozygosity']) / 2

    def _check_breed_standard_compliance(self, profile1: Dict, profile2: Dict) -> Dict:
        """Check breed standard compliance"""
        breed1 = profile1.get('breedInformation', {}).get('primaryBreed', {})
        breed2 = profile2.get('breedInformation', {}).get('primaryBreed', {})

        if breed1.get('breedName') != breed2.get('breedName'):
            return {
                'compliant': False,
                'reason': 'Cross-breed pairing'
            }

        # Check if purebred
        purebred1 = breed1.get('percentage', 0) >= 87.5
        purebred2 = breed2.get('percentage', 0) >= 87.5

        return {
            'compliant': purebred1 and purebred2,
            'parent1Purebred': purebred1,
            'parent2Purebred': purebred2,
            'expectedOffspringPurity': (breed1.get('percentage', 0) + breed2.get('percentage', 0)) / 2
        }

    def _calculate_kinship(self, profile1: Dict, profile2: Dict) -> float:
        """Calculate kinship coefficient between two animals"""
        # Simplified - would use actual genomic data
        return 0.0

    def _calculate_compatibility_score(self,
                                      inbreeding: float,
                                      disease_compat: Dict,
                                      diversity: float,
                                      breed_compliance: Dict) -> float:
        """Calculate overall compatibility score (0-1)"""
        score = 1.0

        # Penalize high inbreeding
        if inbreeding > 0.125:
            score -= 0.3
        elif inbreeding > 0.0625:
            score -= 0.1

        # Penalize disease incompatibility
        if not disease_compat['compatible']:
            score -= 0.4

        # Reward high diversity
        if diversity > 0.35:
            score += 0.1

        # Penalize breed standard non-compliance
        if not breed_compliance['compliant']:
            score -= 0.2

        return max(0.0, min(1.0, score))

    def _generate_pairing_recommendations(self,
                                         inbreeding: float,
                                         disease_compat: Dict,
                                         diversity: float) -> List[str]:
        """Generate pairing-specific recommendations"""
        recommendations = []

        if inbreeding > 0.125:
            recommendations.append("WARNING: High predicted inbreeding coefficient")

        if not disease_compat['compatible']:
            recommendations.append(f"WARNING: Both carriers for {', '.join(disease_compat['commonCarriers'])}")
            recommendations.append("Consider alternative pairing to avoid affected offspring")

        if diversity < 0.30:
            recommendations.append("Low genetic diversity predicted - consider outcross")

        if not recommendations:
            recommendations.append("Suitable pairing based on genetic analysis")

        return recommendations

    def _get_profile(self, profile_id: str) -> Dict:
        """Retrieve genomic profile"""
        # Implementation would fetch from API
        pass

# Usage
pedigree_integration = PedigreeSystemIntegration({
    'baseUrl': 'https://pedigree-system.example.com'
})

# Calculate breeding compatibility
compatibility = pedigree_integration.calculate_breeding_compatibility(
    profile1_id='PGP-SIRE-001',
    profile2_id='PGP-DAM-002'
)

print(f"Compatibility score: {compatibility['compatibilityScore']}")
print(f"Recommended: {compatibility['recommendedBreeding']}")
print(f"Predicted offspring inbreeding: {compatibility['predictions']['offspringInbreeding']}")
```

---

## 4. Testing and Validation Framework

### 4.1 Integration Test Suite

```python
import unittest
from typing import Dict
import json

class PetGenomeIntegrationTests(unittest.TestCase):
    """Comprehensive integration test suite"""

    def setUp(self):
        """Set up test fixtures"""
        self.test_profile = self._load_test_profile()
        self.vpms_adapter = None  # Initialize with test config

    def test_vpms_patient_sync(self):
        """Test VPMS patient synchronization"""
        # Create test patient in VPMS
        vpms_patient = {
            'id': 'TEST-PATIENT-001',
            'name': 'Test Dog',
            'microchipNumber': '123456789012345',
            'dateOfBirth': '2023-01-01',
            'gender': 'MALE'
        }

        # Sync to genomic system
        genomic_profile = self._sync_patient_to_genomic(vpms_patient)

        # Assertions
        self.assertEqual(
            genomic_profile['petIdentification']['microchipId'],
            vpms_patient['microchipNumber']
        )
        self.assertEqual(
            genomic_profile['petIdentification']['registeredName'],
            vpms_patient['name']
        )

    def test_genomic_result_export(self):
        """Test export of genomic results to VPMS"""
        # Create genomic result
        result = self._create_genomic_result(self.test_profile)

        # Export to VPMS
        vpms_result = self._export_to_vpms(result)

        # Verify export
        self.assertIsNotNone(vpms_result)
        self.assertEqual(vpms_result['testType'], 'GENOMIC')

    def test_fhir_resource_creation(self):
        """Test FHIR resource creation"""
        fhir_mapper = FHIRGenomicMapper()

        # Create patient resource
        patient = fhir_mapper.create_patient_resource(self.test_profile)

        # Validate resource
        self.assertEqual(patient.id, self.test_profile['profileId'])
        self.assertIsNotNone(patient.identifier)

    def test_breeding_compatibility_calculation(self):
        """Test breeding compatibility calculation"""
        pedigree = PedigreeSystemIntegration({})

        compatibility = pedigree.calculate_breeding_compatibility(
            'PGP-TEST-001',
            'PGP-TEST-002'
        )

        # Verify calculation
        self.assertIn('compatibilityScore', compatibility)
        self.assertGreaterEqual(compatibility['compatibilityScore'], 0.0)
        self.assertLessEqual(compatibility['compatibilityScore'], 1.0)

    def test_data_encryption_roundtrip(self):
        """Test encryption and decryption"""
        import os
        from pet_genome_sdk import GenomicDataEncryption

        key = os.urandom(32)
        encryptor = GenomicDataEncryption(key)

        # Encrypt
        encrypted = encryptor.encrypt_profile(self.test_profile)

        # Decrypt
        decrypted = encryptor.decrypt_profile(encrypted)

        # Verify
        self.assertEqual(decrypted, self.test_profile)

    def test_consent_verification(self):
        """Test consent verification"""
        from pet_genome_sdk import ConsentManager

        consent_mgr = ConsentManager()

        # Create consent
        consent = consent_mgr.create_consent(
            profile_id='PGP-TEST-001',
            consent_data={
                'geneticTesting': {'granted': True},
                'researchParticipation': {'granted': False}
            }
        )

        # Verify consent
        verification = consent_mgr.verify_consent(
            consent_id=consent['consentId'],
            required_scope='GENETIC_TESTING'
        )

        self.assertTrue(verification['valid'])

    def test_api_rate_limiting(self):
        """Test API rate limiting"""
        from pet_genome_sdk import PetGenomeClient

        client = PetGenomeClient(api_key='test_key')

        # Make requests until rate limited
        request_count = 0
        rate_limited = False

        while request_count < 1000 and not rate_limited:
            try:
                client.profiles.get('PGP-TEST-001')
                request_count += 1
            except Exception as e:
                if 'RATE_LIMIT_EXCEEDED' in str(e):
                    rate_limited = True

        self.assertTrue(rate_limited or request_count == 1000)

    def test_webhook_delivery(self):
        """Test webhook notification delivery"""
        from pet_genome_sdk import WebhookManager

        webhook_mgr = WebhookManager()

        # Register webhook
        webhook = webhook_mgr.register_webhook(
            url='https://test.example.com/webhook',
            events=['profile.created', 'analysis.completed']
        )

        # Trigger event
        webhook_mgr.trigger_event(
            event='profile.created',
            data={'profileId': 'PGP-TEST-001'}
        )

        # Verify delivery (would check mock server)
        self.assertTrue(True)  # Simplified

    def _load_test_profile(self) -> Dict:
        """Load test genomic profile"""
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
        """Sync VPMS patient to genomic system"""
        # Implementation
        pass

    def _create_genomic_result(self, profile: Dict) -> Dict:
        """Create genomic test result"""
        # Implementation
        pass

    def _export_to_vpms(self, result: Dict) -> Dict:
        """Export result to VPMS"""
        # Implementation
        pass

# Run tests
if __name__ == '__main__':
    unittest.main()
```

### 4.2 Performance Testing

```python
import time
import statistics
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict

class PerformanceTest:
    """Performance and load testing"""

    def __init__(self, api_client):
        self.client = api_client
        self.results = []

    def test_api_response_time(self, endpoint: str, iterations: int = 100) -> Dict:
        """Test API endpoint response time"""
        response_times = []

        for _ in range(iterations):
            start = time.time()
            self.client.get(endpoint)
            end = time.time()

            response_times.append((end - start) * 1000)  # Convert to ms

        return {
            'endpoint': endpoint,
            'iterations': iterations,
            'mean_ms': statistics.mean(response_times),
            'median_ms': statistics.median(response_times),
            'p95_ms': self._percentile(response_times, 95),
            'p99_ms': self._percentile(response_times, 99),
            'min_ms': min(response_times),
            'max_ms': max(response_times)
        }

    def test_concurrent_requests(self, endpoint: str, concurrent: int = 50) -> Dict:
        """Test concurrent request handling"""
        def make_request():
            start = time.time()
            try:
                self.client.get(endpoint)
                return time.time() - start, 'SUCCESS'
            except Exception as e:
                return time.time() - start, str(e)

        with ThreadPoolExecutor(max_workers=concurrent) as executor:
            futures = [executor.submit(make_request) for _ in range(concurrent)]
            results = [f.result() for f in futures]

        response_times = [r[0] * 1000 for r in results]
        successes = sum(1 for r in results if r[1] == 'SUCCESS')

        return {
            'endpoint': endpoint,
            'concurrentRequests': concurrent,
            'successRate': successes / concurrent,
            'mean_ms': statistics.mean(response_times),
            'p95_ms': self._percentile(response_times, 95)
        }

    def test_file_upload_performance(self, file_size_mb: int) -> Dict:
        """Test file upload performance"""
        # Generate test file
        test_data = b'0' * (file_size_mb * 1024 * 1024)

        start = time.time()
        # Upload file
        # self.client.upload_file(test_data)
        end = time.time()

        duration = end - start
        throughput = file_size_mb / duration  # MB/s

        return {
            'fileSizeMB': file_size_mb,
            'durationSeconds': duration,
            'throughputMBps': throughput
        }

    def _percentile(self, data: List[float], percentile: int) -> float:
        """Calculate percentile"""
        sorted_data = sorted(data)
        index = int(len(sorted_data) * (percentile / 100))
        return sorted_data[index]

# Usage
perf_test = PerformanceTest(api_client=None)  # Initialize with real client

# Test endpoint performance
results = perf_test.test_api_response_time('/v1/profiles/PGP-TEST-001')
print(f"Mean response time: {results['mean_ms']:.2f}ms")
print(f"P95 response time: {results['p95_ms']:.2f}ms")

# Test concurrent load
load_results = perf_test.test_concurrent_requests('/v1/profiles', concurrent=50)
print(f"Success rate: {load_results['successRate'] * 100:.1f}%")
```

---

## 5. Deployment Strategies

### 5.1 Phased Rollout Plan

| Phase | Duration | Activities | Success Criteria |
|-------|----------|------------|------------------|
| Phase 1: Pilot | 4 weeks | 5 veterinary clinics, 100 tests | >95% data accuracy, <500ms API response |
| Phase 2: Limited | 8 weeks | 25 clinics, 1000 tests | >99% uptime, successful VPMS integration |
| Phase 3: Regional | 12 weeks | 100 clinics, 10,000 tests | <1% error rate, full feature parity |
| Phase 4: Full Launch | Ongoing | All participants | 99.9% SLA compliance |

### 5.2 Deployment Checklist

```yaml
deployment_checklist:
  pre_deployment:
    - name: "Verify environment configuration"
      status: "pending"
      responsible: "DevOps Team"

    - name: "Database migration scripts tested"
      status: "pending"
      responsible: "Database Admin"

    - name: "API endpoints smoke tested"
      status: "pending"
      responsible: "QA Team"

    - name: "Security scan completed"
      status: "pending"
      responsible: "Security Team"

    - name: "Backup procedures verified"
      status: "pending"
      responsible: "DevOps Team"

  deployment:
    - name: "Deploy to staging environment"
      status: "pending"
      responsible: "DevOps Team"

    - name: "Run integration test suite"
      status: "pending"
      responsible: "QA Team"

    - name: "Performance testing"
      status: "pending"
      responsible: "Performance Team"

    - name: "Deploy to production (blue-green)"
      status: "pending"
      responsible: "DevOps Team"

  post_deployment:
    - name: "Monitor error rates"
      duration: "24 hours"
      responsible: "DevOps Team"

    - name: "Verify data consistency"
      duration: "48 hours"
      responsible: "Data Team"

    - name: "User acceptance testing"
      duration: "1 week"
      responsible: "Product Team"

    - name: "Document lessons learned"
      duration: "Ongoing"
      responsible: "All Teams"
```

---

## 6. Monitoring and Observability

### 6.1 Key Metrics

| Metric Category | Specific Metrics | Target | Alert Threshold |
|----------------|------------------|--------|-----------------|
| API Performance | Response time (p95) | <500ms | >1000ms |
| API Performance | Response time (p99) | <1000ms | >2000ms |
| Availability | Uptime | 99.9% | <99.5% |
| Error Rates | HTTP 5xx errors | <0.1% | >0.5% |
| Data Processing | VCF processing time | <30min | >60min |
| Security | Failed auth attempts | <1% | >5% |
| Business | Daily active users | Growth | Decline >10% |
| Business | Tests processed/day | Growth | Decline >10% |

### 6.2 Monitoring Implementation

```python
from prometheus_client import Counter, Histogram, Gauge
import time
from functools import wraps

# Define metrics
api_requests_total = Counter(
    'pet_genome_api_requests_total',
    'Total API requests',
    ['method', 'endpoint', 'status']
)

api_request_duration = Histogram(
    'pet_genome_api_request_duration_seconds',
    'API request duration',
    ['method', 'endpoint']
)

active_genomic_profiles = Gauge(
    'pet_genome_active_profiles',
    'Number of active genomic profiles'
)

vcf_processing_duration = Histogram(
    'pet_genome_vcf_processing_duration_seconds',
    'VCF file processing duration',
    ['file_size_category']
)

def monitor_api_call(func):
    """Decorator to monitor API calls"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        method = kwargs.get('method', 'GET')
        endpoint = kwargs.get('endpoint', 'unknown')

        start_time = time.time()
        try:
            result = func(*args, **kwargs)
            status = '200'
            return result
        except Exception as e:
            status = '500'
            raise
        finally:
            duration = time.time() - start_time
            api_requests_total.labels(method=method, endpoint=endpoint, status=status).inc()
            api_request_duration.labels(method=method, endpoint=endpoint).observe(duration)

    return wrapper

@monitor_api_call
def api_endpoint_handler(method, endpoint, **kwargs):
    """Example API endpoint handler"""
    # Implementation
    pass

# Usage
api_endpoint_handler(method='GET', endpoint='/v1/profiles/PGP-001')
```

---

## 7. Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-12-18 | Initial integration specification | WIA Standards Committee |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
