# WIA-REFUGEE-CREDENTIAL: Phase 2 - Algorithms

**난민 자격증명 알고리즘**
*Verification, Matching, and Assessment Algorithms*

> "국가가 무너져도, 사람의 가치는 무너지지 않습니다."

홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document specifies algorithms for:
1. **Confidence Calculation** - Overall credential reliability
2. **Peer Matching** - Finding verifiers from the same background
3. **Competency Assessment** - Evaluating skills without documents
4. **Document Verification** - AI-based authenticity checking
5. **Credential Scoring** - Institution acceptance scoring
6. **Privacy-Preserving Verification** - Anonymous proofs

---

## 2. Confidence Calculation Algorithm

### 2.1 Overall Confidence Score

```python
def calculate_overall_confidence(credential: RefugeeCredential) -> ConfidenceResult:
    """
    Calculate overall confidence score for a credential.

    Confidence ranges from 0.0 to 1.0
    Based on verification levels and corroborating evidence.
    """

    # Base confidence from verification level
    level_confidence = {
        1: 0.30,  # Self-declaration
        2: 0.60,  # Peer verification
        3: 0.80,  # Assessment
        4: 0.95   # Document verification
    }

    base_confidence = level_confidence.get(credential.verification_level, 0.30)

    # Calculate component scores
    education_score = calculate_education_confidence(credential.education)
    competency_score = calculate_competency_confidence(credential.competencies)
    experience_score = calculate_experience_confidence(credential.experience)

    # Weighted combination
    weights = {
        'education': 0.35,
        'competency': 0.40,
        'experience': 0.25
    }

    component_score = (
        education_score * weights['education'] +
        competency_score * weights['competency'] +
        experience_score * weights['experience']
    )

    # Combine base and component scores
    # Base verification level has more weight
    overall = base_confidence * 0.6 + component_score * 0.4

    # Apply bonuses
    bonus = calculate_confidence_bonus(credential)
    overall = min(0.99, overall + bonus)

    # Apply penalties
    penalty = calculate_confidence_penalty(credential)
    overall = max(0.10, overall - penalty)

    return ConfidenceResult(
        overall_confidence=round(overall, 2),
        base_confidence=base_confidence,
        education_confidence=education_score,
        competency_confidence=competency_score,
        experience_confidence=experience_score,
        factors=get_confidence_factors(credential)
    )


def calculate_education_confidence(education: List[EducationRecord]) -> float:
    """Calculate confidence for education records."""

    if not education:
        return 0.0

    scores = []
    for edu in education:
        score = edu.verification.confidence_score

        # Bonus for higher education
        level_bonus = {
            'doctorate': 0.1,
            'masters': 0.08,
            'professional': 0.08,
            'bachelors': 0.05
        }
        score += level_bonus.get(edu.level.value, 0.0)

        # Bonus for evidence
        if edu.evidence:
            evidence_bonus = min(len(edu.evidence) * 0.05, 0.15)
            score += evidence_bonus

        scores.append(min(1.0, score))

    return sum(scores) / len(scores)


def calculate_competency_confidence(competencies: List[CompetencyRecord]) -> float:
    """Calculate confidence for competency records."""

    if not competencies:
        return 0.0

    scores = []
    for comp in competencies:
        score = comp.verification.confidence_score

        # Bonus for formal assessment
        if comp.assessment.assessment_type in ['practical_test', 'simulation']:
            score += 0.1

        # Bonus for WIA-certified assessor
        if comp.assessment.assessor_type == 'wia_certified':
            score += 0.1

        # Bonus for high scores
        if comp.assessment.score and comp.assessment.score > 80:
            score += 0.05

        scores.append(min(1.0, score))

    return sum(scores) / len(scores)


def calculate_confidence_bonus(credential: RefugeeCredential) -> float:
    """Calculate bonus factors that increase confidence."""

    bonus = 0.0

    # Multiple verification methods
    methods = set(v.method for v in credential.verifications)
    if len(methods) >= 3:
        bonus += 0.05

    # Multiple peer verifiers
    peer_verifications = [v for v in credential.verifications
                         if v.method == 'peer_attestation']
    if len(peer_verifications) >= 3:
        bonus += 0.05

    # Consistent timeline
    if verify_timeline_consistency(credential):
        bonus += 0.03

    # Cross-verification (education matches competency)
    if verify_cross_consistency(credential):
        bonus += 0.05

    return bonus


def calculate_confidence_penalty(credential: RefugeeCredential) -> float:
    """Calculate penalty factors that decrease confidence."""

    penalty = 0.0

    # Inconsistencies detected
    if detect_inconsistencies(credential):
        penalty += 0.15

    # Very old unverified claims
    for edu in credential.education:
        if edu.verification.level == 1:  # Self-declared
            age_years = (datetime.utcnow() - edu.verification.verification_date).days / 365
            if age_years > 2:
                penalty += 0.05

    # No evidence at all
    has_any_evidence = any(
        bool(edu.evidence) for edu in credential.education
    ) or any(
        bool(comp.evidence) for comp in credential.competencies
    )
    if not has_any_evidence:
        penalty += 0.10

    return penalty
```

### 2.2 Confidence Level Classification

| Confidence | Level | Use Cases |
|------------|-------|-----------|
| 0.90-0.99 | Very High | Direct credential recognition |
| 0.75-0.89 | High | Conditional recognition |
| 0.60-0.74 | Moderate | Requires additional verification |
| 0.40-0.59 | Low | Limited acceptance |
| 0.00-0.39 | Very Low | Initial claims only |

---

## 3. Peer Matching Algorithm

### 3.1 Overview

Finds potential peer verifiers from the same institution, workplace, or professional network.

### 3.2 Implementation

```python
def find_peer_matches(
    credential: RefugeeCredential,
    peer_network: PeerNetwork
) -> List[PeerMatch]:
    """
    Find peers who can verify the credential holder's claims.

    Matching criteria:
    - Same institution (education)
    - Same employer (work experience)
    - Same professional community
    - Geographic/temporal overlap
    """

    matches = []

    # Match by education
    for edu in credential.education:
        institution_peers = peer_network.find_by_institution(
            institution_name=edu.institution_name,
            country=edu.institution_country,
            year_range=(edu.year_start - 2, (edu.year_end or edu.year_start) + 2)
        )

        for peer in institution_peers:
            match_score = calculate_education_match_score(edu, peer)
            if match_score >= 0.5:
                matches.append(PeerMatch(
                    peer_id=peer.id,
                    match_type='education',
                    match_score=match_score,
                    shared_context={
                        'institution': edu.institution_name,
                        'years': f"{edu.year_start}-{edu.year_end}",
                        'field': edu.field_of_study
                    }
                ))

    # Match by work experience
    for exp in credential.experience:
        employer_peers = peer_network.find_by_employer(
            employer_name=exp.employer_name,
            country=exp.employer_country,
            date_range=(exp.start_date, exp.end_date)
        )

        for peer in employer_peers:
            match_score = calculate_work_match_score(exp, peer)
            if match_score >= 0.5:
                matches.append(PeerMatch(
                    peer_id=peer.id,
                    match_type='employment',
                    match_score=match_score,
                    shared_context={
                        'employer': exp.employer_name,
                        'dates': f"{exp.start_date}-{exp.end_date}",
                        'role': exp.job_title
                    }
                ))

    # Match by professional domain
    domains = set(comp.domain for comp in credential.competencies)
    for domain in domains:
        domain_peers = peer_network.find_by_domain(
            domain=domain,
            nationality=credential.identity.nationality_claimed
        )

        for peer in domain_peers:
            match_score = calculate_domain_match_score(domain, peer)
            if match_score >= 0.3:
                matches.append(PeerMatch(
                    peer_id=peer.id,
                    match_type='professional',
                    match_score=match_score,
                    shared_context={
                        'domain': domain,
                        'nationality': credential.identity.nationality_claimed
                    }
                ))

    # Sort by match score and deduplicate
    matches = deduplicate_matches(matches)
    matches.sort(key=lambda m: m.match_score, reverse=True)

    return matches[:50]  # Return top 50 matches


def calculate_education_match_score(
    education: EducationRecord,
    peer: PeerProfile
) -> float:
    """Calculate how well a peer matches for education verification."""

    score = 0.0

    # Same institution
    if fuzzy_match(education.institution_name, peer.institutions):
        score += 0.4

    # Overlapping years
    year_overlap = calculate_year_overlap(
        (education.year_start, education.year_end),
        peer.education_years
    )
    score += year_overlap * 0.3

    # Same field
    if education.field_of_study in peer.fields:
        score += 0.2

    # Same country
    if education.institution_country == peer.country:
        score += 0.1

    return min(1.0, score)


def calculate_year_overlap(range1: tuple, range2: tuple) -> float:
    """Calculate overlap ratio between two year ranges."""

    start1, end1 = range1
    start2, end2 = range2

    if end1 is None:
        end1 = start1 + 4  # Assume 4 years
    if end2 is None:
        end2 = start2 + 4

    overlap_start = max(start1, start2)
    overlap_end = min(end1, end2)

    if overlap_start > overlap_end:
        return 0.0

    overlap = overlap_end - overlap_start
    total = max(end1 - start1, end2 - start2)

    return overlap / total if total > 0 else 0.0
```

### 3.3 Peer Network Growth

```python
def grow_peer_network(credential: RefugeeCredential) -> None:
    """
    As credentials are verified, grow the peer network.
    Each verified person becomes a potential verifier.
    """

    if credential.verification_level >= 2:
        peer_network.add_peer(PeerProfile(
            id=credential.identity.holder_did,
            institutions=[edu.institution_name for edu in credential.education],
            employers=[exp.employer_name for exp in credential.experience],
            fields=[edu.field_of_study for edu in credential.education],
            domains=[comp.domain for comp in credential.competencies],
            country=credential.identity.nationality_claimed,
            education_years=[(edu.year_start, edu.year_end) for edu in credential.education],
            verification_level=credential.verification_level
        ))
```

---

## 4. Competency Assessment Algorithm

### 4.1 Assessment Matching

```python
def select_assessment(
    competency: CompetencyRecord,
    available_assessments: List[Assessment]
) -> AssessmentSelection:
    """
    Select appropriate assessment for a claimed competency.
    """

    # Filter by domain
    domain_assessments = [a for a in available_assessments
                          if a.domain == competency.domain]

    if not domain_assessments:
        # Fall back to generic assessment
        return AssessmentSelection(
            assessment_type='generic_interview',
            reason='No domain-specific assessment available'
        )

    # Match by specific skill
    skill_assessments = [a for a in domain_assessments
                         if competency.skill_name.lower() in a.skills_covered]

    if skill_assessments:
        # Prefer practical over theoretical
        practical = [a for a in skill_assessments
                     if a.assessment_type in ['practical_test', 'simulation']]
        if practical:
            return AssessmentSelection(
                assessment=practical[0],
                reason='Direct skill match with practical assessment'
            )
        return AssessmentSelection(
            assessment=skill_assessments[0],
            reason='Direct skill match'
        )

    # Match by proficiency level
    level_appropriate = [a for a in domain_assessments
                          if competency.level.value in a.target_levels]

    if level_appropriate:
        return AssessmentSelection(
            assessment=level_appropriate[0],
            reason='Level-appropriate assessment'
        )

    return AssessmentSelection(
        assessment=domain_assessments[0],
        reason='Domain match'
    )


def calculate_assessment_result(
    raw_score: float,
    assessment: Assessment,
    claimed_level: ProficiencyLevel
) -> AssessmentResult:
    """
    Convert raw assessment score to verified proficiency level.
    """

    # Normalize score to 0-100
    normalized_score = (raw_score / assessment.max_score) * 100

    # Determine achieved level
    level_thresholds = {
        'master': 95,
        'expert': 85,
        'proficient': 70,
        'competent': 55,
        'beginner': 40,
        'novice': 0
    }

    achieved_level = 'novice'
    for level, threshold in level_thresholds.items():
        if normalized_score >= threshold:
            achieved_level = level
            break

    # Compare with claimed level
    level_order = ['novice', 'beginner', 'competent', 'proficient', 'expert', 'master']
    claimed_idx = level_order.index(claimed_level.value)
    achieved_idx = level_order.index(achieved_level)

    if achieved_idx >= claimed_idx:
        verification_status = 'verified'
        confidence = 0.8 + (normalized_score - 70) / 100 * 0.15
    elif achieved_idx == claimed_idx - 1:
        verification_status = 'partially_verified'
        confidence = 0.6
    else:
        verification_status = 'not_verified'
        confidence = 0.3

    return AssessmentResult(
        raw_score=raw_score,
        normalized_score=normalized_score,
        claimed_level=claimed_level,
        achieved_level=achieved_level,
        verification_status=verification_status,
        confidence=min(0.95, confidence)
    )
```

### 4.2 Domain-Specific Assessments

```python
ASSESSMENT_REGISTRY = {
    'medicine': {
        'assessments': [
            {
                'id': 'med-clinical-001',
                'name': 'Clinical Knowledge Assessment',
                'type': 'written_test',
                'duration_minutes': 120,
                'skills_covered': ['diagnosis', 'treatment', 'pharmacology'],
                'target_levels': ['competent', 'proficient', 'expert']
            },
            {
                'id': 'med-practical-001',
                'name': 'Clinical Simulation',
                'type': 'simulation',
                'duration_minutes': 60,
                'skills_covered': ['patient_care', 'emergency_response'],
                'target_levels': ['proficient', 'expert']
            }
        ],
        'partner_institutions': ['WIA Medical Assessment Center', 'Partner Hospitals']
    },
    'software_engineering': {
        'assessments': [
            {
                'id': 'swe-code-001',
                'name': 'Coding Assessment',
                'type': 'practical_test',
                'duration_minutes': 90,
                'skills_covered': ['programming', 'algorithms', 'debugging'],
                'target_levels': ['beginner', 'competent', 'proficient', 'expert']
            },
            {
                'id': 'swe-portfolio-001',
                'name': 'Portfolio Review',
                'type': 'portfolio_review',
                'duration_minutes': 30,
                'skills_covered': ['software_design', 'project_management'],
                'target_levels': ['proficient', 'expert', 'master']
            }
        ]
    },
    'teaching': {
        'assessments': [
            {
                'id': 'teach-demo-001',
                'name': 'Teaching Demonstration',
                'type': 'simulation',
                'duration_minutes': 45,
                'skills_covered': ['lesson_delivery', 'classroom_management'],
                'target_levels': ['competent', 'proficient', 'expert']
            }
        ]
    }
}
```

---

## 5. Document Verification Algorithm

### 5.1 AI-Based Authenticity Check

```python
def verify_document(
    document: Evidence,
    claimed_education: EducationRecord
) -> DocumentVerificationResult:
    """
    Verify document authenticity using AI and cross-referencing.
    """

    results = []

    # 1. Image quality check
    quality_result = check_image_quality(document.file_hash)
    results.append(quality_result)

    # 2. Tampering detection
    tampering_result = detect_tampering(document.file_hash)
    results.append(tampering_result)

    # 3. Text extraction and validation
    extracted_text = extract_text(document.file_hash)
    text_result = validate_extracted_text(extracted_text, claimed_education)
    results.append(text_result)

    # 4. Template matching (known document formats)
    template_result = match_document_template(
        document.file_hash,
        claimed_education.institution_country
    )
    results.append(template_result)

    # 5. Cross-reference with external sources
    cross_ref_result = cross_reference_document(
        extracted_text,
        claimed_education
    )
    results.append(cross_ref_result)

    # Calculate overall authenticity
    authenticity_score = calculate_authenticity_score(results)

    return DocumentVerificationResult(
        document_id=document.id,
        authenticity_score=authenticity_score,
        tampering_detected=tampering_result.tampering_detected,
        text_verified=text_result.verified,
        template_matched=template_result.matched,
        cross_reference_found=cross_ref_result.found,
        confidence=calculate_document_confidence(results),
        verification_date=datetime.utcnow(),
        details=results
    )


def detect_tampering(file_hash: str) -> TamperingResult:
    """
    Detect image manipulation using forensic analysis.
    """

    image_data = storage.get_file(file_hash)

    checks = {
        'ela_analysis': perform_ela_analysis(image_data),      # Error Level Analysis
        'metadata_check': check_metadata_consistency(image_data),
        'noise_analysis': analyze_noise_patterns(image_data),
        'compression_analysis': analyze_compression_artifacts(image_data),
        'copy_move_detection': detect_copy_move(image_data),
        'splicing_detection': detect_splicing(image_data)
    }

    tampering_indicators = sum(1 for check in checks.values() if check.suspicious)
    tampering_detected = tampering_indicators >= 2

    return TamperingResult(
        tampering_detected=tampering_detected,
        confidence=1.0 - (tampering_indicators / len(checks)),
        suspicious_areas=get_suspicious_areas(checks),
        checks=checks
    )


def cross_reference_document(
    extracted_text: str,
    claimed: EducationRecord
) -> CrossReferenceResult:
    """
    Cross-reference document content with external sources.
    """

    matches = []

    # Check institution existence
    institution_exists = verify_institution_exists(
        claimed.institution_name,
        claimed.institution_country
    )
    if institution_exists:
        matches.append(CrossReferenceMatch(
            source='institution_database',
            field='institution_name',
            confidence=0.9
        ))

    # Check against archived web pages
    wayback_result = search_wayback_machine(
        claimed.institution_name,
        claimed.year_end
    )
    if wayback_result.found:
        matches.append(CrossReferenceMatch(
            source='wayback_machine',
            field='institution_website',
            confidence=0.7,
            url=wayback_result.url
        ))

    # Check against known graduate lists (if available)
    graduate_list_result = search_graduate_databases(
        institution=claimed.institution_name,
        year=claimed.year_end,
        field=claimed.field_of_study
    )
    if graduate_list_result.found:
        matches.append(CrossReferenceMatch(
            source='graduate_database',
            field='graduation_record',
            confidence=0.95
        ))

    return CrossReferenceResult(
        found=len(matches) > 0,
        matches=matches,
        confidence=max([m.confidence for m in matches]) if matches else 0.0
    )
```

---

## 6. Credential Scoring for Institutions

### 6.1 Institution Acceptance Score

```python
def calculate_acceptance_score(
    credential: RefugeeCredential,
    institution: InstitutionProfile,
    purpose: AcceptancePurpose
) -> AcceptanceScore:
    """
    Calculate how likely an institution is to accept this credential.
    """

    # Get institution requirements
    requirements = institution.get_requirements(purpose)

    scores = []

    # Verification level requirement
    if credential.verification_level >= requirements.min_verification_level:
        scores.append(('verification_level', 1.0))
    else:
        scores.append(('verification_level', 0.3))

    # Confidence requirement
    confidence = calculate_overall_confidence(credential).overall_confidence
    if confidence >= requirements.min_confidence:
        scores.append(('confidence', 1.0))
    else:
        scores.append(('confidence', confidence / requirements.min_confidence))

    # Education level match
    highest_education = get_highest_education(credential.education)
    if highest_education.level.value in requirements.accepted_education_levels:
        scores.append(('education_level', 1.0))
    else:
        scores.append(('education_level', 0.5))

    # Field of study match
    fields = [edu.field_of_study for edu in credential.education]
    if any(field in requirements.accepted_fields for field in fields):
        scores.append(('field_match', 1.0))
    else:
        scores.append(('field_match', 0.3))

    # Language requirements
    language_score = check_language_requirements(
        credential.languages,
        requirements.language_requirements
    )
    scores.append(('language', language_score))

    # Calculate overall
    weights = requirements.score_weights
    overall = sum(
        score * weights.get(name, 0.2)
        for name, score in scores
    )

    # Determine recommendation
    if overall >= 0.85:
        recommendation = 'strong_accept'
    elif overall >= 0.70:
        recommendation = 'accept'
    elif overall >= 0.55:
        recommendation = 'conditional_accept'
    elif overall >= 0.40:
        recommendation = 'further_review'
    else:
        recommendation = 'not_recommended'

    return AcceptanceScore(
        overall_score=overall,
        component_scores=dict(scores),
        recommendation=recommendation,
        missing_requirements=get_missing_requirements(scores, requirements),
        improvement_suggestions=get_improvement_suggestions(credential, requirements)
    )
```

---

## 7. Privacy-Preserving Verification

### 7.1 Zero-Knowledge Proofs

```python
def generate_anonymous_proof(
    credential: RefugeeCredential,
    claim: str
) -> AnonymousProof:
    """
    Generate a zero-knowledge proof for a specific claim
    without revealing the holder's identity.

    Examples:
    - "Has medical degree" (without revealing name/institution)
    - "Has 10+ years experience" (without revealing employer)
    - "Speaks language at C1 level" (without revealing identity)
    """

    # Parse claim
    claim_type, claim_value = parse_claim(claim)

    # Verify claim is true
    claim_verified = verify_claim_against_credential(credential, claim_type, claim_value)

    if not claim_verified:
        raise ClaimNotVerifiableError(f"Claim '{claim}' cannot be verified")

    # Generate ZK proof
    proof = zk_generate_proof(
        credential_commitment=hash_credential(credential),
        claim_type=claim_type,
        claim_value=claim_value,
        proving_key=get_proving_key(claim_type)
    )

    return AnonymousProof(
        claim=claim,
        proof=proof,
        verification_key=get_verification_key(claim_type),
        valid_until=datetime.utcnow() + timedelta(days=30),
        wia_signature=sign_proof(proof)
    )


def verify_anonymous_proof(proof: AnonymousProof) -> bool:
    """
    Verify a zero-knowledge proof without learning the holder's identity.
    """

    # Check WIA signature
    if not verify_wia_signature(proof.wia_signature, proof.proof):
        return False

    # Check expiration
    if datetime.utcnow() > proof.valid_until:
        return False

    # Verify ZK proof
    return zk_verify_proof(
        proof=proof.proof,
        verification_key=proof.verification_key
    )
```

### 7.2 Selective Disclosure

```python
def create_selective_disclosure(
    credential: RefugeeCredential,
    fields_to_reveal: List[str]
) -> SelectiveDisclosure:
    """
    Create a credential presentation that reveals only selected fields.
    """

    revealed_data = {}

    for field in fields_to_reveal:
        if field == 'education_level':
            revealed_data['education_level'] = credential.education[0].level
        elif field == 'highest_degree':
            revealed_data['highest_degree'] = get_highest_education(credential.education)
        elif field == 'competency_domains':
            revealed_data['competency_domains'] = [c.domain for c in credential.competencies]
        elif field == 'verification_level':
            revealed_data['verification_level'] = credential.verification_level
        elif field == 'languages':
            revealed_data['languages'] = credential.languages
        # ... more fields

    # Generate proof that revealed data is from valid credential
    proof = generate_disclosure_proof(credential, fields_to_reveal)

    return SelectiveDisclosure(
        revealed_data=revealed_data,
        proof=proof,
        hidden_field_count=count_hidden_fields(credential, fields_to_reveal),
        overall_confidence=calculate_overall_confidence(credential).overall_confidence,
        wia_signature=sign_disclosure(revealed_data, proof)
    )
```

---

## 8. Timeline Consistency Verification

### 8.1 Implementation

```python
def verify_timeline_consistency(credential: RefugeeCredential) -> TimelineResult:
    """
    Verify that education and work timelines are consistent.
    """

    events = []

    # Collect education events
    for edu in credential.education:
        events.append(TimelineEvent(
            type='education_start',
            date=f"{edu.year_start}-09-01",  # Assume September start
            description=f"Started {edu.level} at {edu.institution_name}"
        ))
        if edu.year_end:
            events.append(TimelineEvent(
                type='education_end',
                date=f"{edu.year_end}-06-30",  # Assume June end
                description=f"Completed {edu.level}"
            ))

    # Collect work events
    for exp in credential.experience:
        events.append(TimelineEvent(
            type='work_start',
            date=exp.start_date,
            description=f"Started as {exp.job_title} at {exp.employer_name}"
        ))
        if exp.end_date:
            events.append(TimelineEvent(
                type='work_end',
                date=exp.end_date,
                description=f"Left {exp.employer_name}"
            ))

    # Sort by date
    events.sort(key=lambda e: e.date)

    # Check for inconsistencies
    issues = []

    # Check age consistency (not working before age 15, etc.)
    birth_year = credential.identity.birth_year
    for event in events:
        event_year = int(event.date[:4])
        age_at_event = event_year - birth_year

        if event.type == 'work_start' and age_at_event < 15:
            issues.append(TimelineIssue(
                type='age_inconsistency',
                description=f"Work started at age {age_at_event}",
                severity='high'
            ))

        if event.type == 'education_start' and 'doctorate' in event.description.lower():
            if age_at_event < 22:
                issues.append(TimelineIssue(
                    type='age_inconsistency',
                    description=f"Doctorate started at age {age_at_event}",
                    severity='medium'
                ))

    # Check for overlapping full-time activities
    # (Some overlap is OK - part-time work during study)
    overlaps = find_significant_overlaps(events)
    for overlap in overlaps:
        issues.append(TimelineIssue(
            type='overlap',
            description=overlap.description,
            severity='low'
        ))

    # Check for large gaps
    gaps = find_large_gaps(events, threshold_months=24)
    for gap in gaps:
        issues.append(TimelineIssue(
            type='gap',
            description=gap.description,
            severity='info'
        ))

    return TimelineResult(
        events=events,
        issues=issues,
        consistent=len([i for i in issues if i.severity == 'high']) == 0,
        confidence_adjustment=calculate_timeline_confidence_adjustment(issues)
    )
```

---

**Document ID**: WIA-REFUGEE-CREDENTIAL-PHASE-2
**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft

*"시리아 의사는 여전히 의사입니다. 우크라이나 교수는 여전히 교수입니다."*
