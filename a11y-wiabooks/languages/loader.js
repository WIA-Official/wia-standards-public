/**
 * WIA A11Y Translation Loader
 * 동적으로 언어 파일을 로드하는 시스템
 */

const TranslationLoader = {
    currentLang: "ko",
    translations: {},
    loadedLanguages: new Set(),
    
    async loadLanguage(langCode) {
        if (this.loadedLanguages.has(langCode)) {
            this.currentLang = langCode;
            return this.translations[langCode];
        }
        
        try {
            const scriptUrl = `/languages/a11y-${langCode}.js`;
            await this.loadScript(scriptUrl);
            const translationData = window[`translations_a11y_${langCode.replace(/-/g, "_")}`];
            
            if (translationData) {
                this.translations[langCode] = translationData;
                this.loadedLanguages.add(langCode);
                this.currentLang = langCode;
                console.log(`✅ Loaded language: ${langCode}`);
                return translationData;
            } else {
                throw new Error(`Translation data not found for ${langCode}`);
            }
        } catch (error) {
            console.warn(`⚠️ Failed to load ${langCode}, falling back to English`);
            if (langCode !== "en") {
                return await this.loadLanguage("en");
            }
            return null;
        }
    },
    
    loadScript(url) {
        return new Promise((resolve, reject) => {
            if (document.querySelector(`script[src="${url}"]`)) {
                resolve();
                return;
            }
            const script = document.createElement("script");
            script.src = url;
            script.onload = () => resolve();
            script.onerror = () => reject(new Error(`Failed to load: ${url}`));
            document.head.appendChild(script);
        });
    },
    
    async translatePage(langCode) {
        const translations = await this.loadLanguage(langCode);
        if (!translations) return;
        
        document.querySelectorAll("[data-i18n]").forEach(el => {
            const key = el.dataset.i18n;
            if (translations[key]) el.innerHTML = translations[key];
        });
        
        document.querySelectorAll("[data-i18n-placeholder]").forEach(el => {
            const key = el.dataset.i18nPlaceholder;
            if (translations[key]) el.placeholder = translations[key];
        });
        
        const display = document.getElementById("currentLangDisplay");
        if (display) display.textContent = langCode.toUpperCase().substring(0, 2);
        
        const rtlLangs = ["ar", "he", "fa", "ur"];
        document.documentElement.setAttribute("dir", rtlLangs.includes(langCode) ? "rtl" : "ltr");
        
        localStorage.setItem("wiabooks_language", langCode);
        console.log(`🌍 Page translated to: ${langCode}`);
    },
    
    async init() {
        const savedLang = localStorage.getItem("wiabooks_language") || "ko";
        await this.translatePage(savedLang);
        
        window.addEventListener("wia-language-changed", async (event) => {
            await this.translatePage(event.detail.language);
        });
        
        console.log("✅ Translation Loader initialized");
    }
};

window.TranslationLoader = TranslationLoader;

if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", () => TranslationLoader.init());
} else {
    TranslationLoader.init();
}
