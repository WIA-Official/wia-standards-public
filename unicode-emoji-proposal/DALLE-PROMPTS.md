# DALL-E 3 API 프롬프트 - WIA 접근성 이모지 25개

## API 설정

```python
import openai
from openai import OpenAI

client = OpenAI(api_key="YOUR_API_KEY")

def generate_emoji(prompt, filename):
    response = client.images.generate(
        model="dall-e-3",
        prompt=prompt,
        size="1024x1024",  # 나중에 72x72로 리사이즈
        quality="standard",
        n=1,
    )
    image_url = response.data[0].url
    print(f"{filename}: {image_url}")
    return image_url
```

---

## Category A: 청각 보조기기 (5개)

### A1. Cochlear Implant (인공와우)
```
Simple flat emoji icon of a cochlear implant device. Show a small processor unit behind an ear with a round magnetic coil. Minimalist design, solid colors, white background, Apple emoji style, no text, no shadows, centered composition.
```

### A2. Bone Conduction Device (골전도 기기)
```
Simple flat emoji icon of a bone conduction hearing device. Show a headband-style device worn around the head with small transducers on each side. Minimalist design, solid colors, white background, Apple emoji style, no text, no shadows.
```

### A3. Hearing Loop Symbol (히어링루프)
```
Simple flat emoji icon of hearing loop symbol. Show a stylized ear with a "T" letter inside, indicating T-coil compatibility. Minimalist design, blue color, white background, Apple emoji style, no text, centered.
```

### A4. Person Signing KSL (한국수어)
```
Simple flat emoji icon of a person making Korean Sign Language gesture for "love" or "hello". Yellow skin tone, simple face, hands in signing position. Minimalist design, Apple emoji style, white background, no text.
```

### A5. Person Signing BSL (영국수어)
```
Simple flat emoji icon of a person making British Sign Language gesture. Yellow skin tone, simple face, both hands visible in signing position. Minimalist design, Apple emoji style, white background, no text.
```

---

## Category B: 시각 보조기기 (5개)

### B1. Braille Display (점자 디스플레이)
```
Simple flat emoji icon of a refreshable braille display device. Show a rectangular device with raised braille dots on top and control buttons below. Minimalist design, gray/silver color, white background, Apple emoji style, no text.
```

### B2. Screen Reader (스크린리더)
```
Simple flat emoji icon representing screen reader software. Show a computer monitor with sound waves emanating from it. Minimalist design, solid colors, white background, Apple emoji style, no text.
```

### B3. Braille Keyboard (점자 키보드)
```
Simple flat emoji icon of a Perkins-style braille keyboard. Show 6 keys in two rows of 3, with a space bar below. Minimalist design, gray color, white background, Apple emoji style, no text.
```

### B4. Audio Description (음성해설)
```
Simple flat emoji icon of audio description symbol. Show "AD" letters inside a speech bubble or with sound waves. Minimalist design, solid colors, white background, Apple emoji style, centered.
```

### B5. Magnifier App (확대 앱)
```
Simple flat emoji icon of a smartphone with magnifying glass overlay showing enlarged text. Minimalist design, solid colors, white background, Apple emoji style, no text.
```

---

## Category C: 이동/신체 (5개)

### C1. Exoskeleton (외골격)
```
Simple flat emoji icon of a powered exoskeleton for legs. Show a standing human figure with mechanical leg supports. Minimalist design, silver/blue colors, white background, Apple emoji style, no text.
```

### C2. Sports Wheelchair (스포츠 휠체어)
```
Simple flat emoji icon of a sports wheelchair with angled wheels. Show dynamic racing-style wheelchair from side view. Minimalist design, solid colors, white background, Apple emoji style, no text.
```

### C3. Running Blade (러닝 블레이드)
```
Simple flat emoji icon of a prosthetic running blade. Show curved carbon fiber blade prosthetic leg. Minimalist design, black/silver colors, white background, Apple emoji style, no text.
```

### C4. Accessible Bathroom (장애인 화장실)
```
Simple flat emoji icon of accessible bathroom symbol. Show wheelchair user symbol next to restroom icon. Minimalist design, blue color, white background, Apple emoji style, no text.
```

### C5. Service Dog Vest (안내견 조끼)
```
Simple flat emoji icon of a guide dog wearing a service vest. Show a Labrador-style dog with harness/vest. Minimalist design, golden/blue colors, white background, Apple emoji style, no text.
```

---

## Category D: AAC/의사소통 (5개)

### D1. AAC Device (AAC 기기)
```
Simple flat emoji icon of an AAC communication device. Show a tablet-like device with picture symbol grid (4-6 squares with simple icons). Minimalist design, solid colors, white background, Apple emoji style, no text.
```

### D2. Eye Gaze System (시선추적)
```
Simple flat emoji icon of eye gaze communication system. Show a computer screen with an eye symbol and tracking indicator. Minimalist design, solid colors, white background, Apple emoji style, no text.
```

### D3. Symbol Board (그림판)
```
Simple flat emoji icon of a picture communication board. Show a board with 6-9 simple picture symbols arranged in grid. Minimalist design, colorful squares, white background, Apple emoji style, no text.
```

### D4. Brain-Computer Interface (BCI)
```
Simple flat emoji icon of a BCI headset. Show a head silhouette wearing a futuristic headband with sensors. Minimalist design, silver/blue colors, white background, Apple emoji style, no text.
```

### D5. Communication Partner (의사소통 파트너)
```
Simple flat emoji icon of two people communicating with AAC. Show two simple figures, one holding a device, facing each other. Minimalist design, yellow skin tones, white background, Apple emoji style, no text.
```

---

## Category E: 신경다양성/인지 (5개)

### E1. Gold Infinity (자폐 수용)
```
Simple flat emoji icon of a gold infinity symbol. Horizontal figure-8 shape in golden/yellow color. Minimalist design, solid gold color, white background, Apple emoji style, no text, centered.
```

### E2. Sensory Room (감각실)
```
Simple flat emoji icon of a sensory room. Show a calm room with soft lighting elements like bubble tube or fiber optics. Minimalist design, purple/blue colors, white background, Apple emoji style, no text.
```

### E3. Fidget Tool (피젯 도구)
```
Simple flat emoji icon of a fidget spinner or fidget cube. Show a simple 3-pronged spinner or cube with buttons. Minimalist design, colorful, white background, Apple emoji style, no text.
```

### E4. Easy Read Symbol (쉬운 글)
```
Simple flat emoji icon of easy read symbol. Show an open book with a checkmark or simplified text lines. Minimalist design, green/blue colors, white background, Apple emoji style, no text.
```

### E5. Quiet Hour (조용한 시간)
```
Simple flat emoji icon representing quiet hour. Show a clock with sound-off symbol or ear with "mute" indicator. Minimalist design, solid colors, white background, Apple emoji style, no text.
```

---

## 전체 생성 스크립트

```python
import openai
from openai import OpenAI
import requests
import os

client = OpenAI(api_key="YOUR_API_KEY")

emojis = {
    "A1_cochlear_implant": "Simple flat emoji icon of a cochlear implant device. Show a small processor unit behind an ear with a round magnetic coil. Minimalist design, solid colors, white background, Apple emoji style, no text, no shadows, centered composition.",
    "A2_bone_conduction": "Simple flat emoji icon of a bone conduction hearing device. Show a headband-style device worn around the head with small transducers on each side. Minimalist design, solid colors, white background, Apple emoji style, no text, no shadows.",
    "A3_hearing_loop": "Simple flat emoji icon of hearing loop symbol. Show a stylized ear with a T letter inside, indicating T-coil compatibility. Minimalist design, blue color, white background, Apple emoji style, no text, centered.",
    "A4_sign_ksl": "Simple flat emoji icon of a person making Korean Sign Language gesture for love or hello. Yellow skin tone, simple face, hands in signing position. Minimalist design, Apple emoji style, white background, no text.",
    "A5_sign_bsl": "Simple flat emoji icon of a person making British Sign Language gesture. Yellow skin tone, simple face, both hands visible in signing position. Minimalist design, Apple emoji style, white background, no text.",
    "B1_braille_display": "Simple flat emoji icon of a refreshable braille display device. Show a rectangular device with raised braille dots on top and control buttons below. Minimalist design, gray silver color, white background, Apple emoji style, no text.",
    "B2_screen_reader": "Simple flat emoji icon representing screen reader software. Show a computer monitor with sound waves emanating from it. Minimalist design, solid colors, white background, Apple emoji style, no text.",
    "B3_braille_keyboard": "Simple flat emoji icon of a Perkins-style braille keyboard. Show 6 keys in two rows of 3, with a space bar below. Minimalist design, gray color, white background, Apple emoji style, no text.",
    "B4_audio_description": "Simple flat emoji icon of audio description symbol. Show AD letters inside a speech bubble or with sound waves. Minimalist design, solid colors, white background, Apple emoji style, centered.",
    "B5_magnifier_app": "Simple flat emoji icon of a smartphone with magnifying glass overlay showing enlarged text. Minimalist design, solid colors, white background, Apple emoji style, no text.",
    "C1_exoskeleton": "Simple flat emoji icon of a powered exoskeleton for legs. Show a standing human figure with mechanical leg supports. Minimalist design, silver blue colors, white background, Apple emoji style, no text.",
    "C2_sports_wheelchair": "Simple flat emoji icon of a sports wheelchair with angled wheels. Show dynamic racing-style wheelchair from side view. Minimalist design, solid colors, white background, Apple emoji style, no text.",
    "C3_running_blade": "Simple flat emoji icon of a prosthetic running blade. Show curved carbon fiber blade prosthetic leg. Minimalist design, black silver colors, white background, Apple emoji style, no text.",
    "C4_accessible_bathroom": "Simple flat emoji icon of accessible bathroom symbol. Show wheelchair user symbol next to restroom icon. Minimalist design, blue color, white background, Apple emoji style, no text.",
    "C5_service_dog": "Simple flat emoji icon of a guide dog wearing a service vest. Show a Labrador-style dog with harness vest. Minimalist design, golden blue colors, white background, Apple emoji style, no text.",
    "D1_aac_device": "Simple flat emoji icon of an AAC communication device. Show a tablet-like device with picture symbol grid 4-6 squares with simple icons. Minimalist design, solid colors, white background, Apple emoji style, no text.",
    "D2_eye_gaze": "Simple flat emoji icon of eye gaze communication system. Show a computer screen with an eye symbol and tracking indicator. Minimalist design, solid colors, white background, Apple emoji style, no text.",
    "D3_symbol_board": "Simple flat emoji icon of a picture communication board. Show a board with 6-9 simple picture symbols arranged in grid. Minimalist design, colorful squares, white background, Apple emoji style, no text.",
    "D4_bci_headset": "Simple flat emoji icon of a BCI headset. Show a head silhouette wearing a futuristic headband with sensors. Minimalist design, silver blue colors, white background, Apple emoji style, no text.",
    "D5_communication_partner": "Simple flat emoji icon of two people communicating with AAC. Show two simple figures, one holding a device, facing each other. Minimalist design, yellow skin tones, white background, Apple emoji style, no text.",
    "E1_gold_infinity": "Simple flat emoji icon of a gold infinity symbol. Horizontal figure-8 shape in golden yellow color. Minimalist design, solid gold color, white background, Apple emoji style, no text, centered.",
    "E2_sensory_room": "Simple flat emoji icon of a sensory room. Show a calm room with soft lighting elements like bubble tube or fiber optics. Minimalist design, purple blue colors, white background, Apple emoji style, no text.",
    "E3_fidget_tool": "Simple flat emoji icon of a fidget spinner or fidget cube. Show a simple 3-pronged spinner or cube with buttons. Minimalist design, colorful, white background, Apple emoji style, no text.",
    "E4_easy_read": "Simple flat emoji icon of easy read symbol. Show an open book with a checkmark or simplified text lines. Minimalist design, green blue colors, white background, Apple emoji style, no text.",
    "E5_quiet_hour": "Simple flat emoji icon representing quiet hour. Show a clock with sound-off symbol or ear with mute indicator. Minimalist design, solid colors, white background, Apple emoji style, no text.",
}

# 이미지 저장 폴더
os.makedirs("emoji_images", exist_ok=True)

def generate_and_save(name, prompt):
    try:
        response = client.images.generate(
            model="dall-e-3",
            prompt=prompt,
            size="1024x1024",
            quality="standard",
            n=1,
        )
        image_url = response.data[0].url

        # 이미지 다운로드
        img_data = requests.get(image_url).content
        with open(f"emoji_images/{name}.png", "wb") as f:
            f.write(img_data)

        print(f"✅ {name} 생성 완료")
        return True
    except Exception as e:
        print(f"❌ {name} 실패: {e}")
        return False

# 전체 생성
for name, prompt in emojis.items():
    generate_and_save(name, prompt)

print("\n=== 완료! ===")
print("emoji_images/ 폴더에 25개 이미지 저장됨")
print("다음 단계: 72x72 및 18x18로 리사이즈")
```

---

## 리사이즈 스크립트

```python
from PIL import Image
import os

input_dir = "emoji_images"
output_72 = "emoji_72x72"
output_18 = "emoji_18x18"

os.makedirs(output_72, exist_ok=True)
os.makedirs(output_18, exist_ok=True)

for filename in os.listdir(input_dir):
    if filename.endswith(".png"):
        img = Image.open(f"{input_dir}/{filename}")

        # 72x72
        img_72 = img.resize((72, 72), Image.LANCZOS)
        img_72.save(f"{output_72}/{filename}")

        # 18x18
        img_18 = img.resize((18, 18), Image.LANCZOS)
        img_18.save(f"{output_18}/{filename}")

print("리사이즈 완료!")
```

---

## 예상 비용

| 항목 | 수량 | 단가 | 합계 |
|-----|-----|-----|-----|
| DALL-E 3 이미지 | 25개 | $0.040 | **$1.00** |
| 재생성 (필요시) | ~10개 | $0.040 | $0.40 |
| **총 예상** | | | **~$1.50** |

---

## 사용법

```bash
# 1. OpenAI API 키 설정
export OPENAI_API_KEY="sk-..."

# 2. 필요한 패키지 설치
pip install openai requests pillow

# 3. 스크립트 실행
python generate_emojis.py

# 4. 결과 확인
ls emoji_images/
ls emoji_72x72/
ls emoji_18x18/
```

---

**WIA Accessibility Emoji Project**
**홍익인간 🤟**
