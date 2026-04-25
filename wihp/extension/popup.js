/**
 * WIHP Chrome Extension - Popup Script
 */

document.addEventListener('DOMContentLoaded', () => {
  const inputText = document.getElementById('inputText');
  const langSelect = document.getElementById('langSelect');
  const resultHangul = document.getElementById('resultHangul');
  const resultBraille = document.getElementById('resultBraille');
  const inputDisplay = document.getElementById('inputDisplay');
  const syllableCount = document.getElementById('syllableCount');
  const copyBtn = document.getElementById('copyBtn');

  // Convert on input
  function convert() {
    const text = inputText.value.trim();
    const lang = langSelect.value;

    if (!text) {
      resultHangul.textContent = '...';
      resultBraille.textContent = '...';
      inputDisplay.textContent = '-';
      syllableCount.textContent = '0';
      return;
    }

    const hangul = WIHPEngine.convert(text, lang);
    const braille = WIHPEngine.toBraille(hangul);

    resultHangul.textContent = hangul || '(변환 중...)';
    resultBraille.textContent = braille || '...';
    inputDisplay.textContent = text;

    // Count syllables
    const count = hangul.replace(/[^가-힣]/g, '').length;
    syllableCount.textContent = count || hangul.length;
  }

  // Event listeners
  inputText.addEventListener('input', convert);
  langSelect.addEventListener('change', convert);

  // Copy button
  copyBtn.addEventListener('click', () => {
    const hangul = resultHangul.textContent;
    if (hangul && hangul !== '...' && hangul !== '(변환 중...)') {
      navigator.clipboard.writeText(hangul).then(() => {
        copyBtn.textContent = '✓ Copied!';
        copyBtn.classList.add('copied');
        setTimeout(() => {
          copyBtn.textContent = '📋 Copy Hangul';
          copyBtn.classList.remove('copied');
        }, 1500);
      });
    }
  });

  // Initial conversion
  inputText.value = 'hello';
  convert();
});

// Try example function (called from HTML)
function tryExample(word) {
  document.getElementById('inputText').value = word;
  document.getElementById('inputText').dispatchEvent(new Event('input'));
}
