"""
WIA AAC Full Demo
Phase 4: WIA Ecosystem Integration

Demonstrates the complete AAC pipeline:
Sensor Input → Text Generation → Multi-Output
"""

import asyncio
import json
from wia_aac import (
    # Core (Phase 2)
    WiaAac,
    MockAdapter,
    SensorType,
    # Output (Phase 4)
    OutputManager,
    MockTTSAdapter,
    MockSignLanguageAdapter,
    MockBrailleAdapter,
    OutputType,
    OutputOptions
)


async def main():
    print("=== WIA AAC Full Demo ===\n")

    # 1. Initialize AAC Core (Phase 1-3)
    print("1. Initializing AAC Core...")
    aac = WiaAac(sensor_type=SensorType.EYE_TRACKER)

    # 2. Initialize Output Manager (Phase 4)
    print("2. Initializing Output Manager...")
    output = OutputManager()

    # 3. Initialize and register output adapters
    print("3. Setting up output adapters...")

    # TTS Adapter
    tts = MockTTSAdapter()
    await tts.initialize()
    output.register(tts)
    print("   - TTS adapter registered")

    # Sign Language Adapter
    sign_language = MockSignLanguageAdapter()
    await sign_language.initialize()
    output.register(sign_language)
    print("   - Sign Language adapter registered")

    # Braille Adapter
    braille = MockBrailleAdapter()
    await braille.initialize()
    output.register(braille)
    print("   - Braille adapter registered")

    # 4. Set up event handlers
    print("4. Setting up event handlers...\n")

    def on_output_start(data):
        print(f"   [Output Started] {json.dumps(data)}")

    def on_output_end(data):
        print(f"   [Output Completed] {json.dumps(data)}")

    def on_error(data):
        print(f"   [Output Error] {json.dumps(data)}")

    output.on("output_start", on_output_start)
    output.on("output_end", on_output_end)
    output.on("error", on_error)

    # 5. Demo: Output to each adapter individually
    print("5. Demo: Individual Adapter Output\n")

    test_text = "안녕하세요"
    print(f'   Test text: "{test_text}"\n')

    # TTS Output
    print("   --- TTS Output ---")
    await output.output_to(OutputType.TTS, test_text, OutputOptions(speed=1.0))
    print()

    # Sign Language Output
    print("   --- Sign Language Output ---")
    isp_codes = await sign_language.text_to_isp(test_text)
    for code in isp_codes:
        print(f"   ISP Code: {code.code} (meaning: {code.meaning})")
    await output.output_to(OutputType.SIGN_LANGUAGE, test_text)
    print()

    # Braille Output
    print("   --- Braille Output ---")
    braille_output = await braille.text_to_braille(test_text)
    print(f"   IPA: {braille_output.ipa}")
    print(f"   Braille: {braille_output.braille}")
    print(f"   Unicode: {', '.join(braille_output.unicode)}")
    await output.output_to(OutputType.BRAILLE, test_text)
    print()

    # 6. Demo: Broadcast to all adapters
    print("6. Demo: Broadcast to All Adapters\n")

    broadcast_text = "감사합니다"
    print(f'   Broadcasting: "{broadcast_text}"\n')
    await output.broadcast(broadcast_text, OutputOptions(speed=0.8))
    print()

    # 7. Cleanup
    print("7. Cleaning up...")
    await output.dispose()
    await aac.disconnect()

    print("\n=== Demo Complete ===")


if __name__ == "__main__":
    asyncio.run(main())
