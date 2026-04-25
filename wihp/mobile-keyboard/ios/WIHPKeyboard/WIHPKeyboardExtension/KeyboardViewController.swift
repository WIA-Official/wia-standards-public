import UIKit

/**
 * WIHP Keyboard - iOS Custom Keyboard Extension
 * Universal Hangul Phonology Keyboard
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 * © 2025 SmileStory Inc. / WIA
 */

class KeyboardViewController: UIInputViewController {

    // MARK: - Properties

    private var inputBuffer = ""
    private var isShifted = false
    private var isNumeric = false

    private var candidateScrollView: UIScrollView!
    private var candidateStackView: UIStackView!
    private var keyboardStackView: UIStackView!

    private let qwertyRows = [
        ["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"],
        ["a", "s", "d", "f", "g", "h", "j", "k", "l"],
        ["⇧", "z", "x", "c", "v", "b", "n", "m", "⌫"],
        ["123", "🌐", " ", "한글", "↵"]
    ]

    private let numericRows = [
        ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0"],
        ["@", "#", "$", "%", "&", "*", "-", "+", "(", ")"],
        ["!", "\"", "'", ":", ";", "/", "?", ",", ".", "⌫"],
        ["ABC", "🌐", " ", "한글", "↵"]
    ]

    // MARK: - Colors

    private let colors = (
        background: UIColor(red: 0.12, green: 0.16, blue: 0.22, alpha: 1.0),
        key: UIColor(red: 0.26, green: 0.30, blue: 0.35, alpha: 1.0),
        keyPressed: UIColor(red: 0.29, green: 0.33, blue: 0.39, alpha: 1.0),
        keySpecial: UIColor(red: 0.12, green: 0.25, blue: 0.69, alpha: 1.0),
        keyConvert: UIColor(red: 0.02, green: 0.59, blue: 0.41, alpha: 1.0),
        text: UIColor.white,
        textHangul: UIColor(red: 0.98, green: 0.75, blue: 0.15, alpha: 1.0),
        candidate: UIColor(red: 0.07, green: 0.09, blue: 0.15, alpha: 1.0)
    )

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        setupKeyboard()
    }

    override func viewWillLayoutSubviews() {
        super.viewWillLayoutSubviews()
        // Adjust for different screen sizes
    }

    // MARK: - Setup

    private func setupKeyboard() {
        view.backgroundColor = colors.background

        // Main container
        let mainStack = UIStackView()
        mainStack.axis = .vertical
        mainStack.spacing = 4
        mainStack.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(mainStack)

        NSLayoutConstraint.activate([
            mainStack.topAnchor.constraint(equalTo: view.topAnchor, constant: 4),
            mainStack.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 4),
            mainStack.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -4),
            mainStack.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -4)
        ])

        // Candidate bar
        candidateScrollView = UIScrollView()
        candidateScrollView.showsHorizontalScrollIndicator = false
        candidateScrollView.backgroundColor = colors.candidate
        candidateScrollView.layer.cornerRadius = 8
        candidateScrollView.heightAnchor.constraint(equalToConstant: 40).isActive = true
        mainStack.addArrangedSubview(candidateScrollView)

        candidateStackView = UIStackView()
        candidateStackView.axis = .horizontal
        candidateStackView.spacing = 8
        candidateStackView.alignment = .center
        candidateStackView.translatesAutoresizingMaskIntoConstraints = false
        candidateScrollView.addSubview(candidateStackView)

        NSLayoutConstraint.activate([
            candidateStackView.topAnchor.constraint(equalTo: candidateScrollView.topAnchor),
            candidateStackView.leadingAnchor.constraint(equalTo: candidateScrollView.leadingAnchor, constant: 8),
            candidateStackView.trailingAnchor.constraint(equalTo: candidateScrollView.trailingAnchor, constant: -8),
            candidateStackView.bottomAnchor.constraint(equalTo: candidateScrollView.bottomAnchor),
            candidateStackView.heightAnchor.constraint(equalTo: candidateScrollView.heightAnchor)
        ])

        // Keyboard rows container
        keyboardStackView = UIStackView()
        keyboardStackView.axis = .vertical
        keyboardStackView.spacing = 6
        keyboardStackView.distribution = .fillEqually
        mainStack.addArrangedSubview(keyboardStackView)

        buildKeyboard()
    }

    private func buildKeyboard() {
        // Remove existing rows
        keyboardStackView.arrangedSubviews.forEach { $0.removeFromSuperview() }

        let rows = isNumeric ? numericRows : qwertyRows

        for row in rows {
            let rowStack = UIStackView()
            rowStack.axis = .horizontal
            rowStack.spacing = 4
            rowStack.distribution = .fill

            for key in row {
                let button = createKeyButton(key)
                rowStack.addArrangedSubview(button)
            }

            keyboardStackView.addArrangedSubview(rowStack)
        }
    }

    private func createKeyButton(_ key: String) -> UIButton {
        let button = UIButton(type: .system)

        // Display text
        let displayText: String
        if isShifted && key.count == 1 && key.first?.isLetter == true {
            displayText = key.uppercased()
        } else if key == " " {
            displayText = "WIHP"
        } else {
            displayText = key
        }

        button.setTitle(displayText, for: .normal)
        button.titleLabel?.font = key.count > 2 ? .systemFont(ofSize: 14, weight: .medium) : .systemFont(ofSize: 18, weight: .medium)
        button.setTitleColor(colors.text, for: .normal)
        button.layer.cornerRadius = 6

        // Background color
        switch key {
        case "⇧":
            button.backgroundColor = isShifted ? colors.keyConvert : colors.keySpecial
        case "⌫", "↵":
            button.backgroundColor = colors.keySpecial
        case "123", "ABC":
            button.backgroundColor = colors.key
        case "한글":
            button.backgroundColor = colors.keyConvert
        case "🌐":
            button.backgroundColor = colors.key
        case " ":
            button.backgroundColor = colors.key
        default:
            button.backgroundColor = colors.key
        }

        // Width constraints
        button.translatesAutoresizingMaskIntoConstraints = false
        switch key {
        case " ":
            button.widthAnchor.constraint(greaterThanOrEqualToConstant: 120).isActive = true
        case "한글":
            button.widthAnchor.constraint(greaterThanOrEqualToConstant: 60).isActive = true
        case "⇧", "⌫":
            button.widthAnchor.constraint(greaterThanOrEqualToConstant: 45).isActive = true
        default:
            break
        }

        button.heightAnchor.constraint(equalToConstant: 44).isActive = true

        // Action
        button.addTarget(self, action: #selector(keyPressed(_:)), for: .touchUpInside)
        button.accessibilityLabel = key

        return button
    }

    // MARK: - Key Actions

    @objc private func keyPressed(_ sender: UIButton) {
        guard let key = sender.accessibilityLabel else { return }

        // Haptic feedback
        let generator = UIImpactFeedbackGenerator(style: .light)
        generator.impactOccurred()

        switch key {
        case "⇧":
            isShifted.toggle()
            buildKeyboard()

        case "⌫":
            if !inputBuffer.isEmpty {
                inputBuffer.removeLast()
                updateCandidates()
            } else {
                textDocumentProxy.deleteBackward()
            }

        case "↵":
            commitBuffer()
            textDocumentProxy.insertText("\n")

        case "123":
            isNumeric = true
            buildKeyboard()

        case "ABC":
            isNumeric = false
            buildKeyboard()

        case "🌐":
            advanceToNextInputMode()

        case "한글":
            convertAndCommit()

        case " ":
            commitBuffer()
            textDocumentProxy.insertText(" ")

        default:
            let char = isShifted && key.count == 1 ? key.uppercased() : key
            inputBuffer += char
            updateCandidates()

            if isShifted && key.count == 1 && key.first?.isLetter == true {
                isShifted = false
                buildKeyboard()
            }
        }
    }

    // MARK: - Candidates

    private func updateCandidates() {
        candidateStackView.arrangedSubviews.forEach { $0.removeFromSuperview() }

        guard !inputBuffer.isEmpty else { return }

        // Original text
        addCandidate(inputBuffer, isHangul: false)

        // WIHP Hangul conversion
        let hangul = WIHPEngine.shared.convert(inputBuffer)
        if !hangul.isEmpty && hangul != inputBuffer {
            addCandidate(hangul, isHangul: true)

            // Braille
            let braille = WIHPEngine.shared.toBraille(hangul)
            if !braille.isEmpty {
                addCandidate(braille, isHangul: false)
            }
        }
    }

    private func addCandidate(_ text: String, isHangul: Bool) {
        let button = UIButton(type: .system)
        button.setTitle(text, for: .normal)
        button.titleLabel?.font = isHangul ? .systemFont(ofSize: 20, weight: .bold) : .systemFont(ofSize: 16)
        button.setTitleColor(isHangul ? colors.textHangul : colors.text, for: .normal)
        button.contentEdgeInsets = UIEdgeInsets(top: 4, left: 12, bottom: 4, right: 12)

        button.addAction(UIAction { [weak self] _ in
            self?.inputBuffer = ""
            self?.textDocumentProxy.insertText(text)
            self?.updateCandidates()
        }, for: .touchUpInside)

        candidateStackView.addArrangedSubview(button)
    }

    // MARK: - Helpers

    private func convertAndCommit() {
        guard !inputBuffer.isEmpty else { return }

        let hangul = WIHPEngine.shared.convert(inputBuffer)
        textDocumentProxy.insertText(hangul)
        inputBuffer = ""
        updateCandidates()
    }

    private func commitBuffer() {
        guard !inputBuffer.isEmpty else { return }

        textDocumentProxy.insertText(inputBuffer)
        inputBuffer = ""
        updateCandidates()
    }
}
