"""
WIA Emotion AI - Basic Python Example

This example demonstrates basic usage of the WIA Emotion AI SDK.

弘益人間 (홍익인간) - Benefit All Humanity
"""

from wia_emotion_ai import EmotionClient, EmotionCategory

# Initialize client
client = EmotionClient(api_key="your-api-key")


# Example 1: Analyze facial expression in an image
def analyze_image():
    result = client.analyze_facial_image(
        image_path="photo.jpg",
        detect_action_units=True,
        cultural_context="ko-KR"
    )

    print(f"Request ID: {result.request_id}")
    print(f"Processing time: {result.processing_time_ms}ms")

    for emotion in result.emotions:
        print(f"  {emotion.category}: {emotion.intensity:.2f} (confidence: {emotion.confidence:.2f})")

    if result.dimensional:
        print(f"Valence: {result.dimensional.valence:.2f}")
        print(f"Arousal: {result.dimensional.arousal:.2f}")


# Example 2: Analyze voice emotion
def analyze_voice():
    result = client.analyze_voice(
        audio_path="speech.wav",
        language="ko-KR",
        extract_prosody=True
    )

    print(f"Dominant emotion: {result.emotions[0].category}")
    print(f"Segments analyzed: {len(result.segments)}")


# Example 3: Analyze text sentiment
def analyze_text():
    result = client.analyze_text(
        text="오늘 정말 좋은 하루였어요!",
        language="ko",
        detect_sarcasm=True
    )

    print(f"Emotion: {result.emotions[0].category}")
    print(f"Valence: {result.dimensional.valence:.2f}")


# Example 4: Multimodal analysis
def analyze_multimodal():
    result = client.analyze_multimodal(
        modalities={
            "facial": {"image": "photo.jpg"},
            "voice": {"audio": "speech.wav"},
            "text": {"content": "I'm very happy today!"}
        },
        fusion_strategy="late_fusion",
        modality_weights={
            "facial": 0.4,
            "voice": 0.35,
            "text": 0.25
        }
    )

    print(f"Fused emotion: {result.emotions[0].category}")
    print(f"Agreement score: {result.aggregate.get('agreement_score', 'N/A')}")


if __name__ == "__main__":
    print("=== Image Analysis ===")
    # analyze_image()

    print("\n=== Voice Analysis ===")
    # analyze_voice()

    print("\n=== Text Analysis ===")
    # analyze_text()

    print("\n=== Multimodal Analysis ===")
    # analyze_multimodal()

    print("\nNote: Uncomment the function calls and provide valid API key and files.")
