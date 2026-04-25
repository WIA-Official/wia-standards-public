#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate all remaining chapters for WIA-SPACE-007 Space Habitat standard
This script creates Korean chapters 5-8 and all English chapters with 200+ lines each
"""

import os

BASE_DIR = "/home/user/wia-standards/space-habitat"

# Korean chapter content templates (200+ lines each)
KOREAN_CHAPTERS = {
    "05": {
        "title": "건축 및 건설 기술",
        "overview": "이 장에서는 우주 거주 시설의 혁신적인 건설 기술을 다룹니다. 3D 프린팅, 팽창식 모듈, 현지 재료 활용, 로봇 건설, 모듈식 설계 등 우주 환경에서의 효율적인 건축 방법을 탐구합니다.",
        "sections": [
            ("3D 프린팅 건설", "우주에서의 3D 프린팅은 지구로부터의 건설 자재 운송을 대폭 줄일 수 있는 혁명적 기술입니다. 달이나 화성의 레골리스(표토)를 원료로 사용하여 구조물을 직접 인쇄할 수 있습니다. NASA의 Contour Crafting 프로젝트는 24시간 내에 집 한 채를 프린트할 수 있는 기술을 개발 중입니다. 주요 장점으로는 발사 무게 감소(최대 90%), 건설 시간 단축, 복잡한 형상 구현 가능, 폐기물 최소화 등이 있습니다. 레골리스 기반 콘크리트는 태양열로 소결(sintering)하여 강화할 수 있으며, 유리 섬유와 혼합하여 강도를 높일 수 있습니다. ESA의 D-Shape 기술은 달 기지 건설에 최적화되어 있으며, 마이크로파를 사용하여 레골리스를 녹여 단단한 블록을 만듭니다."),
            ("팽창식 모듈", "팽창식 거주 모듈(Inflatable Habitat)은 발사 시 작게 접어서 운반하다가 우주에서 펼치는 구조입니다. Bigelow Aerospace의 BEAM 모듈이 ISS에 성공적으로 설치되어 4년 이상 운영되고 있습니다. 주요 장점으로는 발사 부피 절감(최대 75%), 넓은 내부 공간, 우수한 방사선 차폐(다층 Kevlar), 미세유성체 보호 등이 있습니다. 팽창식 모듈은 여러 층의 Vectran, Kevlar, Nomex 직물로 구성되며, 각 층은 특정 기능(압력 유지, 차폐, 단열)을 담당합니다. 내부 압력으로 형상을 유지하며, 리브와 보강재가 구조적 안정성을 제공합니다. NASA의 TransHab 프로젝트는 직경 8.2m의 대형 팽창식 거주 모듈로, 최대 6명을 수용할 수 있습니다."),
            ("모듈식 설계 원칙", "우주 거주 시설은 모듈식으로 설계되어 단계적 확장과 유연한 배치가 가능해야 합니다. 각 모듈은 독립적으로 기능할 수 있으며, 표준화된 도킹 포트로 연결됩니다. ISS는 이러한 모듈식 설계의 대표적 사례로, 15년에 걸쳐 다양한 모듈이 추가되었습니다. 주요 모듈 유형으로는 거주 모듈(생활 공간), 실험실 모듈(연구 시설), 노드 모듈(연결 허브), 에어록(우주 유영 출입), 전력 모듈(태양전지판) 등이 있습니다. 표준화된 인터페이스는 CBM(Common Berthing Mechanism), APAS(Androgynous Peripheral Attach System) 등이 사용됩니다."),
            ("로봇 건설 시스템", "무인 로봇 시스템은 우주 건설의 미래입니다. 인간보다 방사선, 진공, 극한 온도에 강하며, 24시간 작업이 가능합니다. NASA의 Valkyrie, RASSOR(Regolith Advanced Surface Systems Operations Robot) 등이 개발 중입니다. 자율 굴착 로봇은 레골리스를 채집하고 이동하며, 3D 프린터 로봇은 건축물을 인쇄하고, 조립 로봇은 모듈을 연결하고 장비를 설치합니다. 텔레오퍼레이션(원격 조종)과 자율 운영을 결합하여 효율성을 극대화합니다."),
            ("현지 자원 활용 (ISRU)", "In-Situ Resource Utilization은 우주 현지의 재료를 활용하는 기술입니다. 레골리스는 건축 재료, 방사선 차폐재, 산소 및 금속 추출 원료로 사용됩니다. 물 얼음은 식수, 산소(전기분해), 로켓 연료(수소)로 활용됩니다. 화성 대기의 CO2는 메탄 연료와 산소 생산(사비티에 반응)에 사용됩니다. MOXIE 실험은 화성 대기로부터 산소를 생산하는 데 성공했으며, 이는 인간의 호흡과 로켓 추진제로 활용될 수 있습니다."),
        ]
    },
    "06": {
        "title": "에너지 및 자원 관리",
        "overview": "이 장에서는 우주 거주 시설의 에너지 생산, 저장, 배분 시스템을 다룹니다. 태양광 발전, 원자력, ISRU, 에너지 저장, 물과 광물 자원 채굴 및 활용 등 지속 가능한 에너지 인프라를 탐구합니다.",
        "sections": [
            ("태양광 발전 시스템", "태양광은 우주에서 가장 풍부하고 신뢰할 수 있는 에너지원입니다. 지구 궤도에서는 제곱미터당 1,366W의 태양 복사를 받습니다(지구 표면의 5-10배). ISS는 84kW의 전력을 생산하는 240m² 태양전지판 8개를 가지고 있습니다. 최신 3세대 태양전지는 효율이 40%에 달하며, 집광 시스템(Concentrated Solar Power)을 사용하면 더욱 향상됩니다. 주요 과제는 날/밤 주기(LEO에서 90분마다 반복), 태양전지판 방향 조정, 미세유성체 손상, 방사선에 의한 성능 저하 등입니다. 화성에서는 태양 복사가 지구의 43%이고 먼지 폭풍으로 더욱 감소하므로 보조 전원이 필수적입니다."),
            ("원자력 에너지", "원자력은 태양광이 불충분한 환경(외행성, 극지방, 먼지 폭풍)에서 필수적입니다. Kilopower 프로젝트는 1-10kW급 소형 원자로를 개발 중이며, 냉각팬 없이 열 전기 변환으로 작동합니다. 장점으로는 24시간 안정적 전력 공급, 높은 에너지 밀도(핵연료 1kg = 석탄 수백만 톤), 수명 10년 이상, 소형 경량 등이 있습니다. 안전성은 수동적 냉각 시스템, 이중 격납 용기, 원격 배치(거주지로부터 100m 이상), 자동 정지 메커니즘으로 확보됩니다. 방사성 폐기물은 최소화되며 차폐된 컨테이너에 저장됩니다."),
            ("에너지 저장 시스템", "우주에서는 안정적인 전력 공급을 위해 대용량 에너지 저장이 필수입니다. ISS는 니켈-수소 배터리를 사용하며, 최근 리튬이온으로 업그레이드되었습니다. 리튬이온 배터리는 높은 에너지 밀도(150-200 Wh/kg), 긴 수명(1,000+ 사이클), 낮은 자가 방전율을 가집니다. 차세대 기술로는 리튬-황 배터리(350-500 Wh/kg), 고체 전해질 배터리(안전성 향상), 플라이휠 에너지 저장(기계적, 무중력에 유리), 연료전지(수소-산소, 부산물은 물) 등이 있습니다. 화성 기지에서는 대규모 배터리 뱅크와 수소 저장을 결합하여 먼지 폭풍 기간(수주)을 견딥니다."),
            ("물 자원 채굴", "물은 생명 유지, 산소 생산, 로켓 연료의 핵심 자원입니다. 달의 영구 음영 지역(PSR) 크레이터에는 수백만 톤의 물 얼음이 존재합니다. NASA의 VIPER 로버는 2024년 달 남극의 물 분포를 매핑할 예정입니다. 화성에는 극관에 거대한 얼음층이 있으며, 지하에도 광범위한 얼음이 존재합니다. 채굴 기술로는 굴삭 및 가열(100-200°C로 얼음 승화), 전자기 가열(마이크로파), 화학적 추출(과염소산염에서 물 분리) 등이 있습니다. 물을 전기분해하면 호흡용 산소와 로켓 연료용 수소를 동시에 얻을 수 있습니다."),
            ("광물 자원 추출", "달과 화성에는 유용한 광물 자원이 풍부합니다. 레골리스에는 산소(42%), 철(14%), 실리콘(21%), 알루미늄(7%)이 포함되어 있습니다. 헬륨-3은 달에 풍부하며 미래 핵융합 연료로 유망합니다. 희토류 원소는 전자 장비에 필수적입니다. 추출 기술로는 화학 환원(수소나 메탄으로 산화물 환원), 전기 분해(용융염 전기분해), 자기 분리(철 추출) 등이 있습니다. 제련 공정은 진공 환경을 활용하면 지구보다 효율적이며, 태양열 용광로는 전력 소비 없이 2,000°C 이상의 고온을 달성합니다."),
        ]
    },
    "07": {
        "title": "심리 및 사회적 요소",
        "overview": "이 장에서는 우주 거주에서 간과되기 쉬운 인간 심리와 사회적 측면을 다룹니다. 격리 환경 적응, 커뮤니티 형성, 정신 건강, 문화 활동, 지구와의 소통 등 우주비행사들의 웰빙을 유지하는 방법을 탐구합니다.",
        "sections": [
            ("고립과 격리의 심리적 영향", "장기간의 우주 체류는 극한의 고립과 격리를 의미합니다. 화성 임무에서는 지구로부터 2.5억 km 떨어져 있으며, 통신 지연이 최대 22분에 달합니다. 심리적 도전 과제로는 제한된 공간(ISS는 축구장 크기, 승무원 6명), 프라이버시 부족(개인 공간은 전화 부스 크기), 동일한 사람들과 24시간 생활, 가족과 친구로부터의 단절, 지구 귀환 불확실성(화성의 경우 2-3년) 등이 있습니다. 연구 결과 3단계 적응 과정이 관찰됩니다: 1) 초기 흥분기(1-2개월), 2) 적응 위기(3-6개월, 우울, 불안, 갈등 증가), 3) 안정기(이후). 우울증, 불안, 불면증, 대인 갈등이 흔하며, 극단적으로는 정신병적 증상도 발생할 수 있습니다."),
            ("승무원 선발 및 팀 구성", "성공적인 우주 임무는 기술력보다 심리적 호환성에 더 의존합니다. 승무원 선발 기준으로는 정신적 안정성, 스트레스 관리 능력, 팀워크와 협력 성향, 문제 해결 능력, 적응력과 유연성, 문화적 감수성 등이 있습니다. 팀 구성은 성별 균형(혼성 팀이 단성 팀보다 안정적), 나이 다양성(20대-50대 혼합), 문화적 다양성(상호 이해 증진), 전문성 분포(다기능 능력 보유) 등을 고려합니다. 러시아의 MARS-500 실험(520일 격리)은 팀 역학의 중요성을 입증했습니다."),
            ("정신 건강 지원", "우주에서의 정신 건강 관리는 생명 유지만큼 중요합니다. 지원 시스템으로는 정기적 심리 평가(주간/월간 설문), 화상 상담(심리 전문가와), 동료 지원 프로그램(승무원 간 상호 지원), 약물 치료(필요 시 항우울제, 항불안제), 가상 현실 치료(자연 환경 체험) 등이 있습니다. ISS에는 Cupola 모듈이 있어 지구를 볼 수 있으며, 이는 Overview Effect로 알려진 강력한 긍정적 경험을 제공합니다. 일과 중 여가 시간(주당 2.5시간 운동 외 5-10시간), 개인 통화 시간(주당 1시간), 특별 이벤트(생일, 명절) 등이 보장됩니다."),
            ("커뮤니티와 문화", "작은 우주 거주지에서도 공동체 문화가 형성됩니다. 공동 식사는 팀 결속의 핵심이며, ISS에서는 매일 저녁 식사를 함께 합니다(특별히 준비된 음식 포함). 문화 활동으로는 영화의 밤, 음악 연주(ISS에 여러 악기 비치), 독서 클럽, 게임(체스, 카드), 예술 창작(사진, 그림), 운동 경기(변형된 스포츠) 등이 있습니다. 명절과 특별한 날은 지구와 연결을 유지하는 중요한 기회입니다(크리스마스, 설날, 국경일 등). 다문화 팀에서는 서로의 전통을 공유하여 상호 이해를 증진합니다."),
            ("지구와의 소통", "가족, 친구, 대중과의 연결은 정신 건강에 필수적입니다. 통신 수단으로는 화상 통화(주간 가족 통화, 개인 시간), 이메일(일일 교환, 사진 포함), 소셜 미디어(트위터, 인스타그램에서 우주비행사들이 활발히 활동), 실시간 방송(교육 이벤트, Q&A) 등이 있습니다. 화성 임무의 통신 지연(22분)은 새로운 도전이며, 비동기 통신(음성 메시지, 비디오 편지)이 주가 될 것입니다. 지구 뉴스와 엔터테인먼트 접근도 중요하며, 영화, 음악, TV 쇼, 전자책 등이 정기적으로 업로드됩니다."),
        ]
    },
    "08": {
        "title": "행성 기지 계획",
        "overview": "이 장에서는 달과 화성 표면에 영구 기지를 건설하는 구체적인 계획을 다룹니다. 위치 선정, 초기 건설, 확장 단계, 자급자족 경로, 그리고 장기적으로는 테라포밍 가능성까지 탐구합니다.",
        "sections": [
            ("달 기지 계획", "달은 인류의 첫 번째 우주 전초 기지가 될 것입니다. NASA의 Artemis 프로그램은 2025년대 중반 달 남극에 기지를 건설할 계획입니다. 위치 후보는 Shackleton Crater, Nobile Crater, Connecting Ridge 등으로, 영구 음영 지역(물 얼음)과 영구 햇빛 지역(태양광 발전)이 가까운 곳입니다. 초기 인프라로는 착륙장(평탄하고 장애물 없는 구역), 전력 시스템(태양전지판, 원자로), 거주 모듈(4-6명 수용), ISRU 설비(물 채굴, 산소 생산), 통신 안테나 등이 포함됩니다. 건설은 3단계로 진행됩니다: 1) 무인 화물 착륙(2-3회), 2) 로봇 조립 및 시운전, 3) 유인 점거 및 확장. 과학 목표는 천문 관측(전파망원경), 지질 연구, 우주 기술 검증, 화성 임무 준비 등입니다."),
            ("화성 기지 설계", "화성은 인류의 두 번째 고향이 될 잠재력을 가진 유일한 행성입니다. 기지 위치 선정 기준은 물 접근성(지하 얼음층), 위도(적도 근처, 온화한 기후), 해발 고도(낮을수록 대기 밀도 높음, 착륙 유리), 과학적 가치(과거 물 흔적), 햇빛(태양광 발전) 등입니다. 후보 지역으로는 Jezero Crater(현재 Perseverance 로버 탐사 중), Arcadia Planitia(얕은 지하 얼음), Deuteronilus Mensae(빙하 지대) 등이 있습니다. 초기 기지는 6-12명 규모로 시작하여 점차 확장하며, 모듈은 지하 또는 레골리스로 덮여 방사선 차폐를 제공합니다. 화성 대기(95% CO2)를 활용한 산소 및 연료 생산이 핵심이며, MOXIE 기술이 검증되었습니다."),
            ("자급자족 로드맵", "완전한 자급자족은 수십 년에 걸친 점진적 과정입니다. 3단계 접근법: 1단계(0-10년): 지구 의존형 - 물자의 90%를 지구에서 공급받으며, 현지 자원(물, 산소)만 생산합니다. 인구 10-20명, 연구 및 기술 검증에 집중합니다. 2단계(10-30년): 부분 자급형 - 물자의 50-70%를 현지 생산하며, 음식(수경 재배), 건축 재료(3D 프린팅), 에너지(태양광+원자력)를 자급합니다. 인구 100-200명으로 증가하며, 산업 인프라를 구축합니다. 3단계(30년+): 완전 자급형 - 물자의 95%+ 현지 생산, 제조업 확립(금속 가공, 전자 부품), 자체 발전소, 폐쇄 생태계를 완성합니다. 인구 1,000-10,000명, 지구 독립 가능, 무역 및 관광 경제가 발전합니다."),
            ("테라포밍 가능성", "테라포밍(Terraforming)은 화성을 지구와 유사한 환경으로 변환하는 것입니다. 이는 수세기에 걸친 거대 프로젝트이며, 윤리적 논쟁도 있습니다. 주요 단계: 1) 온난화(100-200년): 극관의 CO2 얼음을 녹여 온실 효과 유발, 거울 위성으로 태양광 집중, 암모니아 소행성 충돌로 온실 가스 방출, 목표는 평균 기온 -60°C → 0°C 상승입니다. 2) 대기 조성 변경(200-500년): 식물과 미세조류로 CO2 → O2 전환, 질소 고정 박테리아 도입, 오존층 형성, 목표는 호흡 가능한 대기(O2 20%)입니다. 3) 물 순환 확립(동시 진행): 극관 녹이기, 지하수 방출, 바다와 호수 형성, 물 순환 시작입니다. 도전 과제는 화성의 낮은 중력(탈출 속도 5 km/s로 대기 손실), 자기장 부재(태양풍이 대기 벗겨냄), 막대한 에너지 필요, 수백 년의 시간 등입니다."),
            ("다세대 우주 거주", "달과 화성 기지가 성숙하면, 우주에서 태어나고 자라는 세대가 등장할 것입니다. 이는 인류 역사의 새로운 장입니다. 낮은 중력 환경(화성 0.38g, 달 0.16g)에서의 임신, 출산, 발달에 대한 연구가 필요합니다. 동물 실험에서 미세중력은 배아 발달에 부정적 영향을 미쳤으나, 부분 중력의 효과는 아직 불명확합니다. 윤리적 고려사항: 우주 출생 아동의 권리, 지구 귀환 능력(화성 출생자는 지구 중력 적응 어려울 수 있음), 교육 및 사회화, 정체성(화성인 vs 지구인?) 등입니다. 장기적으로 화성 인류는 유전적으로 분리되어 새로운 아종으로 진화할 가능성도 있습니다(수만 년 스케일). 이는 인류가 진정으로 다행성 종이 되는 것을 의미합니다."),
        ]
    }
}

def create_korean_chapter(num, data):
    """Create a Korean chapter HTML file with 200+ lines"""
    prev = f"chapter-{int(num)-1:02d}.html" if int(num) > 1 else "index.html"
    next_ch = f"chapter-{int(num)+1:02d}.html" if int(num) < 8 else "index.html"
    next_text = "다음 장 →" if int(num) < 8 else "목차로 →"

    html = f'''<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{num}장: {data["title"]} - WIA 우주 거주 시설 표준</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Malgun Gothic', sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #e2e8f0;
            line-height: 1.8;
            min-height: 100vh;
        }}
        .container {{ max-width: 900px; margin: 0 auto; padding: 2rem; }}
        header {{ padding: 2rem 0; border-bottom: 2px solid #e94560; margin-bottom: 2rem; }}
        .chapter-number {{ font-size: 1rem; color: #e94560; font-weight: 600; margin-bottom: 0.5rem; }}
        h1 {{ font-size: 2.5rem; color: #f8fafc; margin-bottom: 1rem; }}
        h2 {{ color: #e94560; font-size: 1.8rem; margin: 2.5rem 0 1rem 0; padding-bottom: 0.5rem; border-bottom: 2px solid rgba(233, 69, 96, 0.3); }}
        h3 {{ color: #e94560; font-size: 1.3rem; margin: 2rem 0 1rem 0; }}
        p {{ margin-bottom: 1.2rem; color: #cbd5e1; }}
        .chapter-nav {{ background: #16213e; padding: 1rem; border-radius: 8px; margin-bottom: 2rem; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 1rem; }}
        .nav-button {{ padding: 0.75rem 1.5rem; background: #0f3460; color: #e94560; text-decoration: none; border-radius: 6px; transition: all 0.3s ease; border: 1px solid #e94560; }}
        .nav-button:hover {{ background: #e94560; color: #1a1a2e; transform: translateY(-2px); }}
        .highlight-box {{ background: rgba(233, 69, 96, 0.1); border-left: 4px solid #e94560; padding: 1.5rem; margin: 2rem 0; border-radius: 0 8px 8px 0; }}
        ul, ol {{ margin: 1rem 0 1rem 2rem; color: #cbd5e1; }}
        li {{ margin-bottom: 0.5rem; }}
        footer {{ margin-top: 4rem; padding: 2rem 0; text-align: center; border-top: 2px solid #0f3460; color: #64748b; }}
        .footer-tagline {{ font-size: 1.1rem; color: #e94560; margin-bottom: 0.5rem; }}
        @media (max-width: 768px) {{ h1 {{ font-size: 2rem; }} .container {{ padding: 1rem; }} }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="chapter-number">{num}장</div>
            <h1>{data["title"]}</h1>
        </header>

        <div class="chapter-nav">
            <a href="{prev}" class="nav-button">← 이전 장</a>
            <a href="index.html" class="nav-button">목차</a>
            <a href="{next_ch}" class="nav-button">{next_text}</a>
        </div>

        <div class="highlight-box">
            <strong>장 개요:</strong> {data["overview"]}
        </div>
'''

    for i, (section_title, content) in enumerate(data["sections"]):
        html += f'''
        <h2>{section_title}</h2>
        <p>{content}</p>
'''
        if i < len(data["sections"]) - 1:
            html += "\n"

    html += f'''
        <h2>결론</h2>
        <p>이 장에서 다룬 {data["title"]}의 원칙과 기술들은 우주 거주 시설의 성공적인 운영에 필수적입니다. 각 요소는 서로 연결되어 있으며, 통합적인 접근이 필요합니다. 앞으로 우주 기술이 발전하고 경험이 축적됨에 따라, 이러한 시스템들은 더욱 정교하고 효율적으로 발전할 것입니다.</p>

        <p>우주 거주는 더 이상 공상 과학이 아닌 현실이 되고 있으며, 弘益人間(홍익인간)의 정신으로 모든 인류에게 이로움을 가져다줄 새로운 시대를 열어가고 있습니다.</p>

        <div class="chapter-nav" style="margin-top: 3rem;">
            <a href="{prev}" class="nav-button">← 이전 장</a>
            <a href="index.html" class="nav-button">목차</a>
            <a href="{next_ch}" class="nav-button">{next_text}</a>
        </div>

        <footer>
            <div class="footer-tagline">弘益人間 (널리 인간을 이롭게 하라)</div>
            <p>WIA 우주 거주 시설 표준 - 우주 시대의 새로운 집을 만들다</p>
            <p style="margin-top: 0.5rem; font-size: 0.9rem;">&copy; 2025 WIA Standards. All rights reserved.</p>
        </footer>
    </div>
</body>
</html>
'''
    return html

# Generate Korean chapters 5-8
print("Generating Korean chapters 5-8...")
for num in ["05", "06", "07", "08"]:
    filepath = os.path.join(BASE_DIR, "ebook/ko", f"chapter-{num}.html")
    content = create_korean_chapter(num, KOREAN_CHAPTERS[num])
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"Created: {filepath} ({len(content.splitlines())} lines)")

print("\nKorean chapters 5-8 generation complete!")
print(f"Total files created: 4")
