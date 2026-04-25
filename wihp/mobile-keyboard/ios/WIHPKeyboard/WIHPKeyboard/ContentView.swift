import SwiftUI

/**
 * WIHP Keyboard - Main App View
 * Setup and demo interface
 */

struct ContentView: View {
    @State private var inputText = ""
    @State private var convertedText = "한글"
    @State private var brailleText = "⠚⠏⠈⠪⠇"

    var body: some View {
        ScrollView {
            VStack(spacing: 24) {
                // Header
                VStack(spacing: 8) {
                    Text("WIHP Keyboard")
                        .font(.system(size: 32, weight: .bold))
                        .foregroundColor(.white)

                    Text("Universal Hangul Phonology")
                        .font(.system(size: 16))
                        .foregroundColor(.gray)
                }
                .padding(.top, 40)

                // Enable Instructions
                VStack(alignment: .leading, spacing: 12) {
                    Text("📌 키보드 활성화 방법")
                        .font(.headline)
                        .foregroundColor(.white)

                    InstructionRow(number: "1", text: "설정 앱 열기")
                    InstructionRow(number: "2", text: "일반 → 키보드 → 키보드")
                    InstructionRow(number: "3", text: "새 키보드 추가")
                    InstructionRow(number: "4", text: "WIHP Keyboard 선택")
                    InstructionRow(number: "5", text: "전체 접근 허용 (옵션)")
                }
                .padding()
                .background(Color(red: 0.15, green: 0.18, blue: 0.25))
                .cornerRadius(12)

                // Demo Section
                VStack(spacing: 16) {
                    Text("테스트")
                        .font(.headline)
                        .foregroundColor(.gray)
                        .frame(maxWidth: .infinity, alignment: .leading)

                    TextField("hello, bonjour, konnichiwa...", text: $inputText)
                        .textFieldStyle(RoundedBorderTextFieldStyle())
                        .font(.system(size: 18))
                        .onChange(of: inputText) { newValue in
                            if !newValue.isEmpty {
                                convertedText = WIHPEngine.shared.convert(newValue)
                                brailleText = WIHPEngine.shared.toBraille(convertedText)
                            } else {
                                convertedText = "한글"
                                brailleText = "⠚⠏⠈⠪⠇"
                            }
                        }

                    // Result
                    VStack(spacing: 8) {
                        Text(convertedText)
                            .font(.system(size: 48, weight: .bold))
                            .foregroundColor(Color(red: 0.98, green: 0.75, blue: 0.15))

                        Text(brailleText)
                            .font(.system(size: 24))
                            .foregroundColor(Color(red: 0.02, green: 0.59, blue: 0.41))
                    }
                    .padding()
                    .frame(maxWidth: .infinity)
                    .background(Color.white.opacity(0.1))
                    .cornerRadius(12)
                }
                .padding()
                .background(Color(red: 0.15, green: 0.18, blue: 0.25))
                .cornerRadius(12)

                // Usage
                VStack(alignment: .leading, spacing: 12) {
                    Text("⌨️ 사용 방법")
                        .font(.headline)
                        .foregroundColor(.white)

                    Text("• 영어/외국어 입력 후 '한글' 버튼 탭")
                        .foregroundColor(.gray)
                    Text("• 자동으로 한글 발음 변환")
                        .foregroundColor(.gray)
                    Text("• 후보 영역에서 직접 선택 가능")
                        .foregroundColor(.gray)
                }
                .padding()
                .background(Color(red: 0.15, green: 0.18, blue: 0.25))
                .cornerRadius(12)

                // Philosophy
                VStack(spacing: 8) {
                    Text("홍익인간 (弘益人間)")
                        .font(.system(size: 16, weight: .medium))
                        .foregroundColor(.gray)

                    Text("Benefit All Humanity")
                        .font(.system(size: 14))
                        .foregroundColor(.gray.opacity(0.7))
                }
                .padding(.top, 24)

                // Footer
                Text("© 2025 SmileStory Inc. / WIA")
                    .font(.system(size: 12))
                    .foregroundColor(.gray.opacity(0.5))
                    .padding(.bottom, 32)
            }
            .padding(.horizontal, 24)
        }
        .background(Color(red: 0.12, green: 0.16, blue: 0.22))
    }
}

struct InstructionRow: View {
    let number: String
    let text: String

    var body: some View {
        HStack(spacing: 12) {
            Text(number)
                .font(.system(size: 14, weight: .bold))
                .foregroundColor(.white)
                .frame(width: 24, height: 24)
                .background(Color(red: 0.12, green: 0.25, blue: 0.69))
                .cornerRadius(12)

            Text(text)
                .foregroundColor(.gray)
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
