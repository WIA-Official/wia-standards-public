"""
WIA Emotion AI - Real-time Streaming Example (Python)

This example demonstrates real-time emotion streaming using WebSocket.

弘益人間 (홍익인간) - Benefit All Humanity
"""

import asyncio
from wia_emotion_ai import EmotionClient

# Initialize client
client = EmotionClient(api_key="your-api-key")


async def main():
    # Create streaming session
    stream = client.create_stream(
        modalities=["facial", "voice"],
        frame_rate=15
    )

    # Register event handlers
    @stream.on("emotion")
    def handle_emotion(event):
        """Handle individual emotion events."""
        emotion = event["emotions"][0]
        print(f"[{event['timestamp']}] "
              f"Emotion: {emotion['category']} "
              f"({emotion['intensity']:.2f})")

        # Check for specific emotions
        if emotion["category"] == "frustration" and emotion["intensity"] > 0.7:
            print("⚠️  High frustration detected!")

    @stream.on("aggregate")
    def handle_aggregate(summary):
        """Handle periodic aggregate summaries."""
        agg = summary["aggregate"]
        print(f"\n📊 Summary ({summary['frame_count']} frames):")
        print(f"   Dominant: {agg['dominant_emotion']}")
        print(f"   Valence: {agg['average_valence']:.2f}")
        print(f"   Arousal: {agg['average_arousal']:.2f}")
        print(f"   Engagement: {agg['engagement_score']:.2f}\n")

    @stream.on("error")
    def handle_error(error):
        """Handle errors."""
        print(f"❌ Error: {error}")

    @stream.on("connected")
    def handle_connected():
        """Handle connection established."""
        print("✅ Connected to emotion stream")

    @stream.on("disconnected")
    def handle_disconnected():
        """Handle disconnection."""
        print("🔌 Disconnected from emotion stream")

    # Start streaming
    print("Starting emotion stream...")
    await stream.start()

    # Simulate sending frames (in real app, capture from camera/mic)
    try:
        while True:
            # In a real application:
            # frame = capture_video_frame()
            # stream.send_frame(frame)
            #
            # audio = capture_audio_chunk()
            # stream.send_audio(audio)

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping stream...")

    # Stop and get summary
    summary = await stream.stop(request_summary=True)
    if summary:
        print(f"\n📈 Session Summary:")
        print(f"   Total frames: {summary.get('total_frames_processed', 'N/A')}")
        print(f"   Duration: {summary.get('duration_seconds', 'N/A')}s")
        print(f"   Dominant emotion: {summary.get('dominant_emotion', 'N/A')}")


if __name__ == "__main__":
    print("WIA Emotion AI - Real-time Streaming Example")
    print("=" * 50)
    print("\nNote: This is a demonstration. Provide valid API key")
    print("and implement actual video/audio capture.\n")

    # asyncio.run(main())
