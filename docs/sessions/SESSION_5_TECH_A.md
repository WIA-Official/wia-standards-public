# Session 5: Technology Misc A (70개 표준)

**테마:** 바이오, 의료, 기초 기술
**Primary Color:** #8B5CF6 (purple)

---

## 작업 목록 (70개)

```
3d-image-sensor
3d-printing-construction
5g-6g-spectrum
6g-communication
adas
additive-manufacturing
anti-gravity
api-gateway
artificial-organ
augmentation-ethics
augmentation-safety
autonomous-ship
autonomous-weapon-ethics
battery-management-system
beauty-tech
bio-banking
bio-ethics
bio-integration
bio-manufacturing
bio-safety
biodiversity-index
bioinformatics
biomarker-data
bionic-ear
bionic-limb
biopharma
biosensor
building-energy-management
cdn
cellular-therapy
ci-cd
circular-economy
clinical-trial-data
cloud-computing
code-quality
cognitive-enhancement
connected-car
container-technology
cosmetics-data
crispr-protocol
cyber-defense
cyber-weapon
cybernetic-implant
dark-energy-research
dark-matter-detection
data-center
deep-sea-exploration
delivery-drone
devops
digital-factory
dimension-portal
distributed-computing
drought-monitoring
drug-discovery
e-waste-management
ecosystem-monitoring
edge-computing
electric-vehicle
electronic-warfare
elevator-system
embedded-software
environmental-sensor
ev-charging
event-management
fashion-tech
fire-safety-system
fitness-tracking
fleet-management
food-delivery
food-tech
```

---

## 작업 명령어

```bash
# 브랜치 생성
git checkout -b claude/ebook-session-5-[SESSION_ID]

# 작업할 표준 목록 확인
for std in 3d-image-sensor 3d-printing-construction 5g-6g-spectrum 6g-communication adas additive-manufacturing anti-gravity api-gateway artificial-organ augmentation-ethics augmentation-safety autonomous-ship autonomous-weapon-ethics battery-management-system beauty-tech bio-banking bio-ethics bio-integration bio-manufacturing bio-safety biodiversity-index bioinformatics biomarker-data bionic-ear bionic-limb biopharma biosensor building-energy-management cdn cellular-therapy ci-cd circular-economy clinical-trial-data cloud-computing code-quality cognitive-enhancement connected-car container-technology cosmetics-data crispr-protocol cyber-defense cyber-weapon cybernetic-implant dark-energy-research dark-matter-detection data-center deep-sea-exploration delivery-drone devops digital-factory dimension-portal distributed-computing drought-monitoring drug-discovery e-waste-management ecosystem-monitoring edge-computing electric-vehicle electronic-warfare elevator-system embedded-software environmental-sensor ev-charging event-management fashion-tech fire-safety-system fitness-tracking fleet-management food-delivery food-tech; do
  echo "standards/$std"
done

# Spec 파일 읽기 (예시)
cat standards/bio-ethics/spec/*.md

# 품질 확인
for std in 3d-image-sensor 3d-printing-construction bio-ethics biosensor; do
  find standards/$std/ebook -name "*.html" -exec wc -c {} \;
done | awk '{if($1 < 15000) print "FAIL:", $2}'

# 커밋 및 푸시
git add -A
git commit -m "feat: Expand ebook content for Technology standards (Session 5)"
git push -u origin claude/ebook-session-5-[SESSION_ID]
```

---

## 콘텐츠 가이드 (카테고리별)

### 바이오/의료
- bio-banking, bio-ethics, bio-integration, bio-manufacturing, bio-safety
- bioinformatics, biomarker-data, bionic-ear, bionic-limb, biopharma, biosensor
- cellular-therapy, clinical-trial-data, crispr-protocol, drug-discovery
- artificial-organ, cognitive-enhancement

### 통신/네트워크
- 5g-6g-spectrum, 6g-communication, cdn, cloud-computing
- container-technology, data-center, distributed-computing, edge-computing

### 차량/운송
- adas, autonomous-ship, connected-car, delivery-drone
- electric-vehicle, ev-charging, fleet-management

### 제조/에너지
- 3d-printing-construction, additive-manufacturing, battery-management-system
- building-energy-management, digital-factory, elevator-system

### 보안/국방
- cyber-defense, cyber-weapon, electronic-warfare, autonomous-weapon-ethics

### 기타
- anti-gravity, dark-energy-research, dark-matter-detection, dimension-portal

---

**참조:** docs/EBOOK_DISTRIBUTION_GUIDE.md, docs/EBOOK_TEMPLATE.html
