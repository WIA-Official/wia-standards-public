#!/usr/bin/env node
/**
 * Add network, categories, and drilldown translations to all i18n files
 *
 * Rules:
 * - Chinese characters (人, 技, 界, 産, 弘益人間) are kept in ALL languages
 * - Each language gets natural translations for descriptive text
 */

const fs = require('fs');
const path = require('path');

const i18nDir = path.join(__dirname, '../assets/i18n');

// Translation data for each language
// Format: [subtitle, philosophy_desc, total, domains, live, coming, reset, condense, links,
//          humanity_name, humanity_full, humanity_desc,
//          technology_name, technology_full, technology_desc,
//          world_name, world_full, world_desc,
//          industry_name, industry_full, industry_desc,
//          drilldown_total, drilldown_live, drilldown_coming, drilldown_view_simulator, drilldown_close]

const translations = {
  // Already done - skip
  en: null,
  ko: null,

  // Major languages with full translations
  ja: {
    network: {
      title: "765 Standards",
      subtitle: "人(ひと) 技(わざ) 界(せかい) 産(さん)",
      philosophy: "弘益人間 (こういきにんげん)",
      philosophy_desc: "すべての人類に利益をもたらす",
      total: "合計",
      domains: "ドメイン",
      live: "稼働中",
      coming: "準備中",
      reset: "リセット",
      condense: "4分類",
      links: "リンク"
    },
    categories: {
      humanity: { display: "人 ヒューマニティ", name: "人 (ひと)", full: "ヒューマニティ", desc: "生命、福祉、アクセシビリティ" },
      technology: { display: "技 テクノロジー", name: "技 (わざ)", full: "テクノロジー", desc: "接続、ツール、インターフェース" },
      world: { display: "界 ワールド", name: "界 (せかい)", full: "ワールド", desc: "宇宙、自然、環境" },
      industry: { display: "産 インダストリー", name: "産 (さん)", full: "インダストリー", desc: "経済、現実、生産" }
    },
    drilldown: {
      total: "合計",
      live: "稼働中",
      coming: "近日公開",
      view_simulator: "シミュレーターを見る",
      close: "閉じる"
    }
  },

  "zh-CN": {
    network: {
      title: "765 Standards",
      subtitle: "人 技 界 産",
      philosophy: "弘益人间 (弘益人間)",
      philosophy_desc: "造福全人类",
      total: "总计",
      domains: "领域",
      live: "运行中",
      coming: "即将推出",
      reset: "重置",
      condense: "4分类",
      links: "链接"
    },
    categories: {
      humanity: { display: "人 人文", name: "人", full: "人文", desc: "生命、福利、无障碍" },
      technology: { display: "技 科技", name: "技", full: "科技", desc: "连接、工具、接口" },
      world: { display: "界 世界", name: "界", full: "世界", desc: "宇宙、自然、环境" },
      industry: { display: "産 产业", name: "産", full: "产业", desc: "经济、现实、生产" }
    },
    drilldown: {
      total: "总计",
      live: "运行中",
      coming: "即将推出",
      view_simulator: "查看模拟器",
      close: "关闭"
    }
  },

  "zh-TW": {
    network: {
      title: "765 Standards",
      subtitle: "人 技 界 産",
      philosophy: "弘益人間",
      philosophy_desc: "造福全人類",
      total: "總計",
      domains: "領域",
      live: "運行中",
      coming: "即將推出",
      reset: "重置",
      condense: "4分類",
      links: "連結"
    },
    categories: {
      humanity: { display: "人 人文", name: "人", full: "人文", desc: "生命、福利、無障礙" },
      technology: { display: "技 科技", name: "技", full: "科技", desc: "連接、工具、介面" },
      world: { display: "界 世界", name: "界", full: "世界", desc: "宇宙、自然、環境" },
      industry: { display: "産 產業", name: "産", full: "產業", desc: "經濟、現實、生產" }
    },
    drilldown: {
      total: "總計",
      live: "運行中",
      coming: "即將推出",
      view_simulator: "查看模擬器",
      close: "關閉"
    }
  },

  "zh-HK": {
    network: {
      title: "765 Standards",
      subtitle: "人 技 界 産",
      philosophy: "弘益人間",
      philosophy_desc: "造福全人類",
      total: "總計",
      domains: "領域",
      live: "運行中",
      coming: "即將推出",
      reset: "重置",
      condense: "4分類",
      links: "連結"
    },
    categories: {
      humanity: { display: "人 人文", name: "人", full: "人文", desc: "生命、福利、無障礙" },
      technology: { display: "技 科技", name: "技", full: "科技", desc: "連接、工具、介面" },
      world: { display: "界 世界", name: "界", full: "世界", desc: "宇宙、自然、環境" },
      industry: { display: "産 產業", name: "産", full: "產業", desc: "經濟、現實、生產" }
    },
    drilldown: {
      total: "總計",
      live: "運行中",
      coming: "即將推出",
      view_simulator: "查看模擬器",
      close: "關閉"
    }
  },

  de: {
    network: {
      title: "765 Standards",
      subtitle: "Mensch(人) Technik(技) Welt(界) Industrie(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Zum Wohle der gesamten Menschheit",
      total: "Gesamt",
      domains: "Bereiche",
      live: "Aktiv",
      coming: "Demnächst",
      reset: "Zurücksetzen",
      condense: "4 Kategorien",
      links: "Links"
    },
    categories: {
      humanity: { display: "人 MENSCHHEIT", name: "Mensch (人)", full: "Menschheit", desc: "Leben, Wohlfahrt, Barrierefreiheit" },
      technology: { display: "技 TECHNOLOGIE", name: "Technik (技)", full: "Technologie", desc: "Verbindung, Werkzeuge, Schnittstelle" },
      world: { display: "界 WELT", name: "Welt (界)", full: "Welt", desc: "Weltraum, Natur, Umwelt" },
      industry: { display: "産 INDUSTRIE", name: "Industrie (産)", full: "Industrie", desc: "Wirtschaft, Realität, Produktion" }
    },
    drilldown: {
      total: "Gesamt",
      live: "Aktiv",
      coming: "Demnächst",
      view_simulator: "Simulator anzeigen",
      close: "Schließen"
    }
  },

  fr: {
    network: {
      title: "765 Standards",
      subtitle: "Personne(人) Technologie(技) Monde(界) Industrie(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Au bénéfice de toute l'humanité",
      total: "Total",
      domains: "Domaines",
      live: "Actif",
      coming: "À venir",
      reset: "Réinitialiser",
      condense: "4 Catégories",
      links: "Liens"
    },
    categories: {
      humanity: { display: "人 HUMANITÉ", name: "Personne (人)", full: "Humanité", desc: "Vie, Bien-être, Accessibilité" },
      technology: { display: "技 TECHNOLOGIE", name: "Technologie (技)", full: "Technologie", desc: "Connexion, Outils, Interface" },
      world: { display: "界 MONDE", name: "Monde (界)", full: "Monde", desc: "Espace, Nature, Environnement" },
      industry: { display: "産 INDUSTRIE", name: "Industrie (産)", full: "Industrie", desc: "Économie, Réalité, Production" }
    },
    drilldown: {
      total: "Total",
      live: "Actif",
      coming: "À venir",
      view_simulator: "Voir le simulateur",
      close: "Fermer"
    }
  },

  "fr-CA": {
    network: {
      title: "765 Standards",
      subtitle: "Personne(人) Technologie(技) Monde(界) Industrie(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Au bénéfice de toute l'humanité",
      total: "Total",
      domains: "Domaines",
      live: "Actif",
      coming: "À venir",
      reset: "Réinitialiser",
      condense: "4 Catégories",
      links: "Liens"
    },
    categories: {
      humanity: { display: "人 HUMANITÉ", name: "Personne (人)", full: "Humanité", desc: "Vie, Bien-être, Accessibilité" },
      technology: { display: "技 TECHNOLOGIE", name: "Technologie (技)", full: "Technologie", desc: "Connexion, Outils, Interface" },
      world: { display: "界 MONDE", name: "Monde (界)", full: "Monde", desc: "Espace, Nature, Environnement" },
      industry: { display: "産 INDUSTRIE", name: "Industrie (産)", full: "Industrie", desc: "Économie, Réalité, Production" }
    },
    drilldown: {
      total: "Total",
      live: "Actif",
      coming: "À venir",
      view_simulator: "Voir le simulateur",
      close: "Fermer"
    }
  },

  es: {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  "es-MX": {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  "es-AR": {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  "es-CL": {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  "es-CO": {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  "es-PE": {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnología(技) Mundo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "En beneficio de toda la humanidad",
      total: "Total",
      domains: "Dominios",
      live: "Activo",
      coming: "Próximamente",
      reset: "Restablecer",
      condense: "4 Categorías",
      links: "Enlaces"
    },
    categories: {
      humanity: { display: "人 HUMANIDAD", name: "Persona (人)", full: "Humanidad", desc: "Vida, Bienestar, Accesibilidad" },
      technology: { display: "技 TECNOLOGÍA", name: "Tecnología (技)", full: "Tecnología", desc: "Conexión, Herramientas, Interfaz" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espacio, Naturaleza, Medio Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economía, Realidad, Producción" }
    },
    drilldown: {
      total: "Total",
      live: "Activo",
      coming: "Próximamente",
      view_simulator: "Ver simulador",
      close: "Cerrar"
    }
  },

  pt: {
    network: {
      title: "765 Standards",
      subtitle: "Pessoa(人) Tecnologia(技) Mundo(界) Indústria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Em benefício de toda a humanidade",
      total: "Total",
      domains: "Domínios",
      live: "Ativo",
      coming: "Em breve",
      reset: "Redefinir",
      condense: "4 Categorias",
      links: "Links"
    },
    categories: {
      humanity: { display: "人 HUMANIDADE", name: "Pessoa (人)", full: "Humanidade", desc: "Vida, Bem-estar, Acessibilidade" },
      technology: { display: "技 TECNOLOGIA", name: "Tecnologia (技)", full: "Tecnologia", desc: "Conexão, Ferramentas, Interface" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espaço, Natureza, Meio Ambiente" },
      industry: { display: "産 INDÚSTRIA", name: "Indústria (産)", full: "Indústria", desc: "Economia, Realidade, Produção" }
    },
    drilldown: {
      total: "Total",
      live: "Ativo",
      coming: "Em breve",
      view_simulator: "Ver simulador",
      close: "Fechar"
    }
  },

  "pt-BR": {
    network: {
      title: "765 Standards",
      subtitle: "Pessoa(人) Tecnologia(技) Mundo(界) Indústria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Em benefício de toda a humanidade",
      total: "Total",
      domains: "Domínios",
      live: "Ativo",
      coming: "Em breve",
      reset: "Redefinir",
      condense: "4 Categorias",
      links: "Links"
    },
    categories: {
      humanity: { display: "人 HUMANIDADE", name: "Pessoa (人)", full: "Humanidade", desc: "Vida, Bem-estar, Acessibilidade" },
      technology: { display: "技 TECNOLOGIA", name: "Tecnologia (技)", full: "Tecnologia", desc: "Conexão, Ferramentas, Interface" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Espaço, Natureza, Meio Ambiente" },
      industry: { display: "産 INDÚSTRIA", name: "Indústria (産)", full: "Indústria", desc: "Economia, Realidade, Produção" }
    },
    drilldown: {
      total: "Total",
      live: "Ativo",
      coming: "Em breve",
      view_simulator: "Ver simulador",
      close: "Fechar"
    }
  },

  it: {
    network: {
      title: "765 Standards",
      subtitle: "Persona(人) Tecnologia(技) Mondo(界) Industria(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "A beneficio di tutta l'umanità",
      total: "Totale",
      domains: "Domini",
      live: "Attivo",
      coming: "In arrivo",
      reset: "Ripristina",
      condense: "4 Categorie",
      links: "Link"
    },
    categories: {
      humanity: { display: "人 UMANITÀ", name: "Persona (人)", full: "Umanità", desc: "Vita, Benessere, Accessibilità" },
      technology: { display: "技 TECNOLOGIA", name: "Tecnologia (技)", full: "Tecnologia", desc: "Connessione, Strumenti, Interfaccia" },
      world: { display: "界 MONDO", name: "Mondo (界)", full: "Mondo", desc: "Spazio, Natura, Ambiente" },
      industry: { display: "産 INDUSTRIA", name: "Industria (産)", full: "Industria", desc: "Economia, Realtà, Produzione" }
    },
    drilldown: {
      total: "Totale",
      live: "Attivo",
      coming: "In arrivo",
      view_simulator: "Vedi simulatore",
      close: "Chiudi"
    }
  },

  nl: {
    network: {
      title: "765 Standards",
      subtitle: "Persoon(人) Technologie(技) Wereld(界) Industrie(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Ten behoeve van de hele mensheid",
      total: "Totaal",
      domains: "Domeinen",
      live: "Actief",
      coming: "Binnenkort",
      reset: "Reset",
      condense: "4 Categorieën",
      links: "Links"
    },
    categories: {
      humanity: { display: "人 MENSHEID", name: "Persoon (人)", full: "Mensheid", desc: "Leven, Welzijn, Toegankelijkheid" },
      technology: { display: "技 TECHNOLOGIE", name: "Technologie (技)", full: "Technologie", desc: "Verbinding, Gereedschap, Interface" },
      world: { display: "界 WERELD", name: "Wereld (界)", full: "Wereld", desc: "Ruimte, Natuur, Milieu" },
      industry: { display: "産 INDUSTRIE", name: "Industrie (産)", full: "Industrie", desc: "Economie, Realiteit, Productie" }
    },
    drilldown: {
      total: "Totaal",
      live: "Actief",
      coming: "Binnenkort",
      view_simulator: "Bekijk simulator",
      close: "Sluiten"
    }
  },

  ru: {
    network: {
      title: "765 Standards",
      subtitle: "Человек(人) Технология(技) Мир(界) Индустрия(産)",
      philosophy: "Хонъик Инган (弘益人間)",
      philosophy_desc: "На благо всего человечества",
      total: "Всего",
      domains: "Области",
      live: "Активно",
      coming: "Скоро",
      reset: "Сброс",
      condense: "4 Категории",
      links: "Ссылки"
    },
    categories: {
      humanity: { display: "人 ЧЕЛОВЕЧЕСТВО", name: "Человек (人)", full: "Человечество", desc: "Жизнь, Благополучие, Доступность" },
      technology: { display: "技 ТЕХНОЛОГИЯ", name: "Технология (技)", full: "Технология", desc: "Связь, Инструменты, Интерфейс" },
      world: { display: "界 МИР", name: "Мир (界)", full: "Мир", desc: "Космос, Природа, Окружающая среда" },
      industry: { display: "産 ИНДУСТРИЯ", name: "Индустрия (産)", full: "Индустрия", desc: "Экономика, Реальность, Производство" }
    },
    drilldown: {
      total: "Всего",
      live: "Активно",
      coming: "Скоро",
      view_simulator: "Смотреть симулятор",
      close: "Закрыть"
    }
  },

  ar: {
    network: {
      title: "765 Standards",
      subtitle: "الإنسان(人) التقنية(技) العالم(界) الصناعة(産)",
      philosophy: "هونغيك إنغان (弘益人間)",
      philosophy_desc: "لصالح البشرية جمعاء",
      total: "المجموع",
      domains: "المجالات",
      live: "نشط",
      coming: "قريباً",
      reset: "إعادة تعيين",
      condense: "4 فئات",
      links: "روابط"
    },
    categories: {
      humanity: { display: "人 الإنسانية", name: "الإنسان (人)", full: "الإنسانية", desc: "الحياة، الرفاهية، إمكانية الوصول" },
      technology: { display: "技 التقنية", name: "التقنية (技)", full: "التقنية", desc: "الاتصال، الأدوات، الواجهة" },
      world: { display: "界 العالم", name: "العالم (界)", full: "العالم", desc: "الفضاء، الطبيعة، البيئة" },
      industry: { display: "産 الصناعة", name: "الصناعة (産)", full: "الصناعة", desc: "الاقتصاد، الواقع، الإنتاج" }
    },
    drilldown: {
      total: "المجموع",
      live: "نشط",
      coming: "قريباً",
      view_simulator: "عرض المحاكي",
      close: "إغلاق"
    }
  },

  hi: {
    network: {
      title: "765 Standards",
      subtitle: "व्यक्ति(人) प्रौद्योगिकी(技) विश्व(界) उद्योग(産)",
      philosophy: "होंगिक इंगान (弘益人間)",
      philosophy_desc: "समस्त मानवता के लाभ के लिए",
      total: "कुल",
      domains: "डोमेन",
      live: "सक्रिय",
      coming: "जल्द आ रहा है",
      reset: "रीसेट",
      condense: "4 श्रेणियाँ",
      links: "लिंक"
    },
    categories: {
      humanity: { display: "人 मानवता", name: "व्यक्ति (人)", full: "मानवता", desc: "जीवन, कल्याण, सुगम्यता" },
      technology: { display: "技 प्रौद्योगिकी", name: "प्रौद्योगिकी (技)", full: "प्रौद्योगिकी", desc: "कनेक्शन, उपकरण, इंटरफेस" },
      world: { display: "界 विश्व", name: "विश्व (界)", full: "विश्व", desc: "अंतरिक्ष, प्रकृति, पर्यावरण" },
      industry: { display: "産 उद्योग", name: "उद्योग (産)", full: "उद्योग", desc: "अर्थव्यवस्था, वास्तविकता, उत्पादन" }
    },
    drilldown: {
      total: "कुल",
      live: "सक्रिय",
      coming: "जल्द आ रहा है",
      view_simulator: "सिम्युलेटर देखें",
      close: "बंद करें"
    }
  },

  bn: {
    network: {
      title: "765 Standards",
      subtitle: "ব্যক্তি(人) প্রযুক্তি(技) বিশ্ব(界) শিল্প(産)",
      philosophy: "হংইক ইংগান (弘益人間)",
      philosophy_desc: "সমগ্র মানবতার কল্যাণে",
      total: "মোট",
      domains: "ডোমেইন",
      live: "সক্রিয়",
      coming: "শীঘ্রই আসছে",
      reset: "রিসেট",
      condense: "4 বিভাগ",
      links: "লিংক"
    },
    categories: {
      humanity: { display: "人 মানবতা", name: "ব্যক্তি (人)", full: "মানবতা", desc: "জীবন, কল্যাণ, প্রবেশযোগ্যতা" },
      technology: { display: "技 প্রযুক্তি", name: "প্রযুক্তি (技)", full: "প্রযুক্তি", desc: "সংযোগ, সরঞ্জাম, ইন্টারফেস" },
      world: { display: "界 বিশ্ব", name: "বিশ্ব (界)", full: "বিশ্ব", desc: "মহাকাশ, প্রকৃতি, পরিবেশ" },
      industry: { display: "産 শিল্প", name: "শিল্প (産)", full: "শিল্প", desc: "অর্থনীতি, বাস্তবতা, উৎপাদন" }
    },
    drilldown: {
      total: "মোট",
      live: "সক্রিয়",
      coming: "শীঘ্রই আসছে",
      view_simulator: "সিমুলেটর দেখুন",
      close: "বন্ধ করুন"
    }
  },

  th: {
    network: {
      title: "765 Standards",
      subtitle: "บุคคล(人) เทคโนโลยี(技) โลก(界) อุตสาหกรรม(産)",
      philosophy: "ฮงอิก อินกัน (弘益人間)",
      philosophy_desc: "เพื่อประโยชน์ของมนุษยชาติทั้งมวล",
      total: "รวม",
      domains: "โดเมน",
      live: "ใช้งานอยู่",
      coming: "เร็วๆ นี้",
      reset: "รีเซ็ต",
      condense: "4 หมวดหมู่",
      links: "ลิงก์"
    },
    categories: {
      humanity: { display: "人 มนุษยชาติ", name: "บุคคล (人)", full: "มนุษยชาติ", desc: "ชีวิต, สวัสดิการ, การเข้าถึง" },
      technology: { display: "技 เทคโนโลยี", name: "เทคโนโลยี (技)", full: "เทคโนโลยี", desc: "การเชื่อมต่อ, เครื่องมือ, อินเทอร์เฟซ" },
      world: { display: "界 โลก", name: "โลก (界)", full: "โลก", desc: "อวกาศ, ธรรมชาติ, สิ่งแวดล้อม" },
      industry: { display: "産 อุตสาหกรรม", name: "อุตสาหกรรม (産)", full: "อุตสาหกรรม", desc: "เศรษฐกิจ, ความเป็นจริง, การผลิต" }
    },
    drilldown: {
      total: "รวม",
      live: "ใช้งานอยู่",
      coming: "เร็วๆ นี้",
      view_simulator: "ดูตัวจำลอง",
      close: "ปิด"
    }
  },

  vi: {
    network: {
      title: "765 Standards",
      subtitle: "Con người(人) Công nghệ(技) Thế giới(界) Công nghiệp(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Vì lợi ích của toàn nhân loại",
      total: "Tổng",
      domains: "Lĩnh vực",
      live: "Hoạt động",
      coming: "Sắp ra mắt",
      reset: "Đặt lại",
      condense: "4 Danh mục",
      links: "Liên kết"
    },
    categories: {
      humanity: { display: "人 NHÂN LOẠI", name: "Con người (人)", full: "Nhân loại", desc: "Cuộc sống, Phúc lợi, Khả năng tiếp cận" },
      technology: { display: "技 CÔNG NGHỆ", name: "Công nghệ (技)", full: "Công nghệ", desc: "Kết nối, Công cụ, Giao diện" },
      world: { display: "界 THẾ GIỚI", name: "Thế giới (界)", full: "Thế giới", desc: "Không gian, Thiên nhiên, Môi trường" },
      industry: { display: "産 CÔNG NGHIỆP", name: "Công nghiệp (産)", full: "Công nghiệp", desc: "Kinh tế, Thực tế, Sản xuất" }
    },
    drilldown: {
      total: "Tổng",
      live: "Hoạt động",
      coming: "Sắp ra mắt",
      view_simulator: "Xem trình mô phỏng",
      close: "Đóng"
    }
  },

  id: {
    network: {
      title: "765 Standards",
      subtitle: "Orang(人) Teknologi(技) Dunia(界) Industri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Demi kebaikan seluruh umat manusia",
      total: "Total",
      domains: "Domain",
      live: "Aktif",
      coming: "Segera",
      reset: "Atur Ulang",
      condense: "4 Kategori",
      links: "Tautan"
    },
    categories: {
      humanity: { display: "人 KEMANUSIAAN", name: "Orang (人)", full: "Kemanusiaan", desc: "Kehidupan, Kesejahteraan, Aksesibilitas" },
      technology: { display: "技 TEKNOLOGI", name: "Teknologi (技)", full: "Teknologi", desc: "Koneksi, Alat, Antarmuka" },
      world: { display: "界 DUNIA", name: "Dunia (界)", full: "Dunia", desc: "Ruang Angkasa, Alam, Lingkungan" },
      industry: { display: "産 INDUSTRI", name: "Industri (産)", full: "Industri", desc: "Ekonomi, Realitas, Produksi" }
    },
    drilldown: {
      total: "Total",
      live: "Aktif",
      coming: "Segera",
      view_simulator: "Lihat Simulator",
      close: "Tutup"
    }
  },

  ms: {
    network: {
      title: "765 Standards",
      subtitle: "Orang(人) Teknologi(技) Dunia(界) Industri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Untuk kebaikan seluruh umat manusia",
      total: "Jumlah",
      domains: "Domain",
      live: "Aktif",
      coming: "Akan datang",
      reset: "Set semula",
      condense: "4 Kategori",
      links: "Pautan"
    },
    categories: {
      humanity: { display: "人 KEMANUSIAAN", name: "Orang (人)", full: "Kemanusiaan", desc: "Kehidupan, Kebajikan, Kebolehcapaian" },
      technology: { display: "技 TEKNOLOGI", name: "Teknologi (技)", full: "Teknologi", desc: "Sambungan, Alat, Antara muka" },
      world: { display: "界 DUNIA", name: "Dunia (界)", full: "Dunia", desc: "Angkasa, Alam, Persekitaran" },
      industry: { display: "産 INDUSTRI", name: "Industri (産)", full: "Industri", desc: "Ekonomi, Realiti, Pengeluaran" }
    },
    drilldown: {
      total: "Jumlah",
      live: "Aktif",
      coming: "Akan datang",
      view_simulator: "Lihat Simulator",
      close: "Tutup"
    }
  },

  fil: {
    network: {
      title: "765 Standards",
      subtitle: "Tao(人) Teknolohiya(技) Mundo(界) Industriya(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Para sa kabutihan ng buong sangkatauhan",
      total: "Kabuuan",
      domains: "Mga Domain",
      live: "Aktibo",
      coming: "Malapit na",
      reset: "I-reset",
      condense: "4 na Kategorya",
      links: "Mga Link"
    },
    categories: {
      humanity: { display: "人 SANGKATAUHAN", name: "Tao (人)", full: "Sangkatauhan", desc: "Buhay, Kapakanan, Aksesibilidad" },
      technology: { display: "技 TEKNOLOHIYA", name: "Teknolohiya (技)", full: "Teknolohiya", desc: "Koneksyon, Mga Kagamitan, Interface" },
      world: { display: "界 MUNDO", name: "Mundo (界)", full: "Mundo", desc: "Kalawakan, Kalikasan, Kapaligiran" },
      industry: { display: "産 INDUSTRIYA", name: "Industriya (産)", full: "Industriya", desc: "Ekonomiya, Realidad, Produksyon" }
    },
    drilldown: {
      total: "Kabuuan",
      live: "Aktibo",
      coming: "Malapit na",
      view_simulator: "Tingnan ang Simulator",
      close: "Isara"
    }
  },

  tr: {
    network: {
      title: "765 Standards",
      subtitle: "İnsan(人) Teknoloji(技) Dünya(界) Endüstri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Tüm insanlığın yararına",
      total: "Toplam",
      domains: "Alanlar",
      live: "Aktif",
      coming: "Yakında",
      reset: "Sıfırla",
      condense: "4 Kategori",
      links: "Bağlantılar"
    },
    categories: {
      humanity: { display: "人 İNSANLIK", name: "İnsan (人)", full: "İnsanlık", desc: "Yaşam, Refah, Erişilebilirlik" },
      technology: { display: "技 TEKNOLOJİ", name: "Teknoloji (技)", full: "Teknoloji", desc: "Bağlantı, Araçlar, Arayüz" },
      world: { display: "界 DÜNYA", name: "Dünya (界)", full: "Dünya", desc: "Uzay, Doğa, Çevre" },
      industry: { display: "産 ENDÜSTRİ", name: "Endüstri (産)", full: "Endüstri", desc: "Ekonomi, Gerçeklik, Üretim" }
    },
    drilldown: {
      total: "Toplam",
      live: "Aktif",
      coming: "Yakında",
      view_simulator: "Simülatörü Görüntüle",
      close: "Kapat"
    }
  },

  pl: {
    network: {
      title: "765 Standards",
      subtitle: "Osoba(人) Technologia(技) Świat(界) Przemysł(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Dla dobra całej ludzkości",
      total: "Razem",
      domains: "Domeny",
      live: "Aktywne",
      coming: "Wkrótce",
      reset: "Resetuj",
      condense: "4 Kategorie",
      links: "Linki"
    },
    categories: {
      humanity: { display: "人 LUDZKOŚĆ", name: "Osoba (人)", full: "Ludzkość", desc: "Życie, Dobrobyt, Dostępność" },
      technology: { display: "技 TECHNOLOGIA", name: "Technologia (技)", full: "Technologia", desc: "Połączenie, Narzędzia, Interfejs" },
      world: { display: "界 ŚWIAT", name: "Świat (界)", full: "Świat", desc: "Przestrzeń, Natura, Środowisko" },
      industry: { display: "産 PRZEMYSŁ", name: "Przemysł (産)", full: "Przemysł", desc: "Ekonomia, Rzeczywistość, Produkcja" }
    },
    drilldown: {
      total: "Razem",
      live: "Aktywne",
      coming: "Wkrótce",
      view_simulator: "Zobacz symulator",
      close: "Zamknij"
    }
  },

  uk: {
    network: {
      title: "765 Standards",
      subtitle: "Людина(人) Технологія(技) Світ(界) Індустрія(産)",
      philosophy: "Хонік Інган (弘益人間)",
      philosophy_desc: "На благо всього людства",
      total: "Всього",
      domains: "Домени",
      live: "Активно",
      coming: "Незабаром",
      reset: "Скинути",
      condense: "4 Категорії",
      links: "Посилання"
    },
    categories: {
      humanity: { display: "人 ЛЮДСТВО", name: "Людина (人)", full: "Людство", desc: "Життя, Добробут, Доступність" },
      technology: { display: "技 ТЕХНОЛОГІЯ", name: "Технологія (技)", full: "Технологія", desc: "Зв'язок, Інструменти, Інтерфейс" },
      world: { display: "界 СВІТ", name: "Світ (界)", full: "Світ", desc: "Космос, Природа, Довкілля" },
      industry: { display: "産 ІНДУСТРІЯ", name: "Індустрія (産)", full: "Індустрія", desc: "Економіка, Реальність, Виробництво" }
    },
    drilldown: {
      total: "Всього",
      live: "Активно",
      coming: "Незабаром",
      view_simulator: "Переглянути симулятор",
      close: "Закрити"
    }
  },

  cs: {
    network: {
      title: "765 Standards",
      subtitle: "Osoba(人) Technologie(技) Svět(界) Průmysl(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Pro dobro celého lidstva",
      total: "Celkem",
      domains: "Domény",
      live: "Aktivní",
      coming: "Brzy",
      reset: "Obnovit",
      condense: "4 Kategorie",
      links: "Odkazy"
    },
    categories: {
      humanity: { display: "人 LIDSTVO", name: "Osoba (人)", full: "Lidstvo", desc: "Život, Blahobyt, Přístupnost" },
      technology: { display: "技 TECHNOLOGIE", name: "Technologie (技)", full: "Technologie", desc: "Připojení, Nástroje, Rozhraní" },
      world: { display: "界 SVĚT", name: "Svět (界)", full: "Svět", desc: "Vesmír, Příroda, Prostředí" },
      industry: { display: "産 PRŮMYSL", name: "Průmysl (産)", full: "Průmysl", desc: "Ekonomika, Realita, Výroba" }
    },
    drilldown: {
      total: "Celkem",
      live: "Aktivní",
      coming: "Brzy",
      view_simulator: "Zobrazit simulátor",
      close: "Zavřít"
    }
  },

  ro: {
    network: {
      title: "765 Standards",
      subtitle: "Persoană(人) Tehnologie(技) Lume(界) Industrie(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "În beneficiul întregii umanități",
      total: "Total",
      domains: "Domenii",
      live: "Activ",
      coming: "În curând",
      reset: "Resetare",
      condense: "4 Categorii",
      links: "Linkuri"
    },
    categories: {
      humanity: { display: "人 UMANITATE", name: "Persoană (人)", full: "Umanitate", desc: "Viață, Bunăstare, Accesibilitate" },
      technology: { display: "技 TEHNOLOGIE", name: "Tehnologie (技)", full: "Tehnologie", desc: "Conexiune, Instrumente, Interfață" },
      world: { display: "界 LUME", name: "Lume (界)", full: "Lume", desc: "Spațiu, Natură, Mediu" },
      industry: { display: "産 INDUSTRIE", name: "Industrie (産)", full: "Industrie", desc: "Economie, Realitate, Producție" }
    },
    drilldown: {
      total: "Total",
      live: "Activ",
      coming: "În curând",
      view_simulator: "Vezi simulatorul",
      close: "Închide"
    }
  },

  hu: {
    network: {
      title: "765 Standards",
      subtitle: "Személy(人) Technológia(技) Világ(界) Ipar(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Az egész emberiség javára",
      total: "Összesen",
      domains: "Tartományok",
      live: "Aktív",
      coming: "Hamarosan",
      reset: "Visszaállítás",
      condense: "4 Kategória",
      links: "Linkek"
    },
    categories: {
      humanity: { display: "人 EMBERISÉG", name: "Személy (人)", full: "Emberiség", desc: "Élet, Jólét, Hozzáférhetőség" },
      technology: { display: "技 TECHNOLÓGIA", name: "Technológia (技)", full: "Technológia", desc: "Kapcsolat, Eszközök, Felület" },
      world: { display: "界 VILÁG", name: "Világ (界)", full: "Világ", desc: "Űr, Természet, Környezet" },
      industry: { display: "産 IPAR", name: "Ipar (産)", full: "Ipar", desc: "Gazdaság, Valóság, Termelés" }
    },
    drilldown: {
      total: "Összesen",
      live: "Aktív",
      coming: "Hamarosan",
      view_simulator: "Szimulátor megtekintése",
      close: "Bezárás"
    }
  },

  el: {
    network: {
      title: "765 Standards",
      subtitle: "Άνθρωπος(人) Τεχνολογία(技) Κόσμος(界) Βιομηχανία(産)",
      philosophy: "Χόνγκικ Ίνγκαν (弘益人間)",
      philosophy_desc: "Προς όφελος ολόκληρης της ανθρωπότητας",
      total: "Σύνολο",
      domains: "Τομείς",
      live: "Ενεργό",
      coming: "Σύντομα",
      reset: "Επαναφορά",
      condense: "4 Κατηγορίες",
      links: "Σύνδεσμοι"
    },
    categories: {
      humanity: { display: "人 ΑΝΘΡΩΠΟΤΗΤΑ", name: "Άνθρωπος (人)", full: "Ανθρωπότητα", desc: "Ζωή, Ευημερία, Προσβασιμότητα" },
      technology: { display: "技 ΤΕΧΝΟΛΟΓΙΑ", name: "Τεχνολογία (技)", full: "Τεχνολογία", desc: "Σύνδεση, Εργαλεία, Διεπαφή" },
      world: { display: "界 ΚΟΣΜΟΣ", name: "Κόσμος (界)", full: "Κόσμος", desc: "Διάστημα, Φύση, Περιβάλλον" },
      industry: { display: "産 ΒΙΟΜΗΧΑΝΙΑ", name: "Βιομηχανία (産)", full: "Βιομηχανία", desc: "Οικονομία, Πραγματικότητα, Παραγωγή" }
    },
    drilldown: {
      total: "Σύνολο",
      live: "Ενεργό",
      coming: "Σύντομα",
      view_simulator: "Προβολή προσομοιωτή",
      close: "Κλείσιμο"
    }
  },

  he: {
    network: {
      title: "765 Standards",
      subtitle: "אדם(人) טכנולוגיה(技) עולם(界) תעשייה(産)",
      philosophy: "הונגיק אינגאן (弘益人間)",
      philosophy_desc: "לטובת כל האנושות",
      total: "סה״כ",
      domains: "תחומים",
      live: "פעיל",
      coming: "בקרוב",
      reset: "איפוס",
      condense: "4 קטגוריות",
      links: "קישורים"
    },
    categories: {
      humanity: { display: "人 אנושות", name: "אדם (人)", full: "אנושות", desc: "חיים, רווחה, נגישות" },
      technology: { display: "技 טכנולוגיה", name: "טכנולוגיה (技)", full: "טכנולוגיה", desc: "חיבור, כלים, ממשק" },
      world: { display: "界 עולם", name: "עולם (界)", full: "עולם", desc: "חלל, טבע, סביבה" },
      industry: { display: "産 תעשייה", name: "תעשייה (産)", full: "תעשייה", desc: "כלכלה, מציאות, ייצור" }
    },
    drilldown: {
      total: "סה״כ",
      live: "פעיל",
      coming: "בקרוב",
      view_simulator: "צפה בסימולטור",
      close: "סגור"
    }
  },

  sv: {
    network: {
      title: "765 Standards",
      subtitle: "Person(人) Teknik(技) Värld(界) Industri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Till gagn för hela mänskligheten",
      total: "Totalt",
      domains: "Domäner",
      live: "Aktiv",
      coming: "Kommer snart",
      reset: "Återställ",
      condense: "4 Kategorier",
      links: "Länkar"
    },
    categories: {
      humanity: { display: "人 MÄNSKLIGHET", name: "Person (人)", full: "Mänsklighet", desc: "Liv, Välfärd, Tillgänglighet" },
      technology: { display: "技 TEKNIK", name: "Teknik (技)", full: "Teknik", desc: "Anslutning, Verktyg, Gränssnitt" },
      world: { display: "界 VÄRLD", name: "Värld (界)", full: "Värld", desc: "Rymd, Natur, Miljö" },
      industry: { display: "産 INDUSTRI", name: "Industri (産)", full: "Industri", desc: "Ekonomi, Verklighet, Produktion" }
    },
    drilldown: {
      total: "Totalt",
      live: "Aktiv",
      coming: "Kommer snart",
      view_simulator: "Visa simulator",
      close: "Stäng"
    }
  },

  da: {
    network: {
      title: "765 Standards",
      subtitle: "Person(人) Teknologi(技) Verden(界) Industri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Til gavn for hele menneskeheden",
      total: "I alt",
      domains: "Domæner",
      live: "Aktiv",
      coming: "Kommer snart",
      reset: "Nulstil",
      condense: "4 Kategorier",
      links: "Links"
    },
    categories: {
      humanity: { display: "人 MENNESKEHED", name: "Person (人)", full: "Menneskehed", desc: "Liv, Velfærd, Tilgængelighed" },
      technology: { display: "技 TEKNOLOGI", name: "Teknologi (技)", full: "Teknologi", desc: "Forbindelse, Værktøjer, Grænseflade" },
      world: { display: "界 VERDEN", name: "Verden (界)", full: "Verden", desc: "Rum, Natur, Miljø" },
      industry: { display: "産 INDUSTRI", name: "Industri (産)", full: "Industri", desc: "Økonomi, Virkelighed, Produktion" }
    },
    drilldown: {
      total: "I alt",
      live: "Aktiv",
      coming: "Kommer snart",
      view_simulator: "Se simulator",
      close: "Luk"
    }
  },

  no: {
    network: {
      title: "765 Standards",
      subtitle: "Person(人) Teknologi(技) Verden(界) Industri(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Til fordel for hele menneskeheten",
      total: "Totalt",
      domains: "Domener",
      live: "Aktiv",
      coming: "Kommer snart",
      reset: "Tilbakestill",
      condense: "4 Kategorier",
      links: "Lenker"
    },
    categories: {
      humanity: { display: "人 MENNESKEHET", name: "Person (人)", full: "Menneskehet", desc: "Liv, Velferd, Tilgjengelighet" },
      technology: { display: "技 TEKNOLOGI", name: "Teknologi (技)", full: "Teknologi", desc: "Tilkobling, Verktøy, Grensesnitt" },
      world: { display: "界 VERDEN", name: "Verden (界)", full: "Verden", desc: "Rom, Natur, Miljø" },
      industry: { display: "産 INDUSTRI", name: "Industri (産)", full: "Industri", desc: "Økonomi, Virkelighet, Produksjon" }
    },
    drilldown: {
      total: "Totalt",
      live: "Aktiv",
      coming: "Kommer snart",
      view_simulator: "Se simulator",
      close: "Lukk"
    }
  },

  fi: {
    network: {
      title: "765 Standards",
      subtitle: "Henkilö(人) Teknologia(技) Maailma(界) Teollisuus(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Koko ihmiskunnan hyväksi",
      total: "Yhteensä",
      domains: "Alueet",
      live: "Aktiivinen",
      coming: "Tulossa pian",
      reset: "Nollaa",
      condense: "4 Kategoriaa",
      links: "Linkit"
    },
    categories: {
      humanity: { display: "人 IHMISKUNTA", name: "Henkilö (人)", full: "Ihmiskunta", desc: "Elämä, Hyvinvointi, Saavutettavuus" },
      technology: { display: "技 TEKNOLOGIA", name: "Teknologia (技)", full: "Teknologia", desc: "Yhteys, Työkalut, Käyttöliittymä" },
      world: { display: "界 MAAILMA", name: "Maailma (界)", full: "Maailma", desc: "Avaruus, Luonto, Ympäristö" },
      industry: { display: "産 TEOLLISUUS", name: "Teollisuus (産)", full: "Teollisuus", desc: "Talous, Todellisuus, Tuotanto" }
    },
    drilldown: {
      total: "Yhteensä",
      live: "Aktiivinen",
      coming: "Tulossa pian",
      view_simulator: "Näytä simulaattori",
      close: "Sulje"
    }
  },

  sk: {
    network: {
      title: "765 Standards",
      subtitle: "Osoba(人) Technológia(技) Svet(界) Priemysel(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Pre dobro celého ľudstva",
      total: "Celkom",
      domains: "Domény",
      live: "Aktívne",
      coming: "Čoskoro",
      reset: "Obnoviť",
      condense: "4 Kategórie",
      links: "Odkazy"
    },
    categories: {
      humanity: { display: "人 ĽUDSTVO", name: "Osoba (人)", full: "Ľudstvo", desc: "Život, Blahobyt, Prístupnosť" },
      technology: { display: "技 TECHNOLÓGIA", name: "Technológia (技)", full: "Technológia", desc: "Pripojenie, Nástroje, Rozhranie" },
      world: { display: "界 SVET", name: "Svet (界)", full: "Svet", desc: "Vesmír, Príroda, Prostredie" },
      industry: { display: "産 PRIEMYSEL", name: "Priemysel (産)", full: "Priemysel", desc: "Ekonomika, Realita, Výroba" }
    },
    drilldown: {
      total: "Celkom",
      live: "Aktívne",
      coming: "Čoskoro",
      view_simulator: "Zobraziť simulátor",
      close: "Zavrieť"
    }
  },

  sl: {
    network: {
      title: "765 Standards",
      subtitle: "Oseba(人) Tehnologija(技) Svet(界) Industrija(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "V korist celotnega človeštva",
      total: "Skupaj",
      domains: "Domene",
      live: "Aktivno",
      coming: "Kmalu",
      reset: "Ponastavi",
      condense: "4 Kategorije",
      links: "Povezave"
    },
    categories: {
      humanity: { display: "人 ČLOVEŠTVO", name: "Oseba (人)", full: "Človeštvo", desc: "Življenje, Blaginja, Dostopnost" },
      technology: { display: "技 TEHNOLOGIJA", name: "Tehnologija (技)", full: "Tehnologija", desc: "Povezava, Orodja, Vmesnik" },
      world: { display: "界 SVET", name: "Svet (界)", full: "Svet", desc: "Vesolje, Narava, Okolje" },
      industry: { display: "産 INDUSTRIJA", name: "Industrija (産)", full: "Industrija", desc: "Gospodarstvo, Realnost, Proizvodnja" }
    },
    drilldown: {
      total: "Skupaj",
      live: "Aktivno",
      coming: "Kmalu",
      view_simulator: "Ogled simulatorja",
      close: "Zapri"
    }
  },

  hr: {
    network: {
      title: "765 Standards",
      subtitle: "Osoba(人) Tehnologija(技) Svijet(界) Industrija(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Za dobrobit cijelog čovječanstva",
      total: "Ukupno",
      domains: "Domene",
      live: "Aktivno",
      coming: "Uskoro",
      reset: "Resetiraj",
      condense: "4 Kategorije",
      links: "Poveznice"
    },
    categories: {
      humanity: { display: "人 ČOVJEČANSTVO", name: "Osoba (人)", full: "Čovječanstvo", desc: "Život, Blagostanje, Pristupačnost" },
      technology: { display: "技 TEHNOLOGIJA", name: "Tehnologija (技)", full: "Tehnologija", desc: "Povezivanje, Alati, Sučelje" },
      world: { display: "界 SVIJET", name: "Svijet (界)", full: "Svijet", desc: "Svemir, Priroda, Okoliš" },
      industry: { display: "産 INDUSTRIJA", name: "Industrija (産)", full: "Industrija", desc: "Ekonomija, Stvarnost, Proizvodnja" }
    },
    drilldown: {
      total: "Ukupno",
      live: "Aktivno",
      coming: "Uskoro",
      view_simulator: "Pogledaj simulator",
      close: "Zatvori"
    }
  },

  sr: {
    network: {
      title: "765 Standards",
      subtitle: "Особа(人) Технологија(技) Свет(界) Индустрија(産)",
      philosophy: "Хонгик Инган (弘益人間)",
      philosophy_desc: "За добробит целог човечанства",
      total: "Укупно",
      domains: "Домени",
      live: "Активно",
      coming: "Ускоро",
      reset: "Ресетуј",
      condense: "4 Категорије",
      links: "Линкови"
    },
    categories: {
      humanity: { display: "人 ЧОВЕЧАНСТВО", name: "Особа (人)", full: "Човечанство", desc: "Живот, Благостање, Приступачност" },
      technology: { display: "技 ТЕХНОЛОГИЈА", name: "Технологија (技)", full: "Технологија", desc: "Повезивање, Алати, Интерфејс" },
      world: { display: "界 СВЕТ", name: "Свет (界)", full: "Свет", desc: "Свемир, Природа, Околина" },
      industry: { display: "産 ИНДУСТРИЈА", name: "Индустрија (産)", full: "Индустрија", desc: "Економија, Стварност, Производња" }
    },
    drilldown: {
      total: "Укупно",
      live: "Активно",
      coming: "Ускоро",
      view_simulator: "Погледај симулатор",
      close: "Затвори"
    }
  },

  bg: {
    network: {
      title: "765 Standards",
      subtitle: "Човек(人) Технология(技) Свят(界) Индустрия(産)",
      philosophy: "Хонгик Инган (弘益人間)",
      philosophy_desc: "В полза на цялото човечество",
      total: "Общо",
      domains: "Домейни",
      live: "Активен",
      coming: "Очаквайте скоро",
      reset: "Нулиране",
      condense: "4 Категории",
      links: "Връзки"
    },
    categories: {
      humanity: { display: "人 ЧОВЕЧЕСТВО", name: "Човек (人)", full: "Човечество", desc: "Живот, Благосъстояние, Достъпност" },
      technology: { display: "技 ТЕХНОЛОГИЯ", name: "Технология (技)", full: "Технология", desc: "Свързаност, Инструменти, Интерфейс" },
      world: { display: "界 СВЯТ", name: "Свят (界)", full: "Свят", desc: "Космос, Природа, Околна среда" },
      industry: { display: "産 ИНДУСТРИЯ", name: "Индустрия (産)", full: "Индустрия", desc: "Икономика, Реалност, Производство" }
    },
    drilldown: {
      total: "Общо",
      live: "Активен",
      coming: "Очаквайте скоро",
      view_simulator: "Виж симулатора",
      close: "Затвори"
    }
  },

  fa: {
    network: {
      title: "765 Standards",
      subtitle: "انسان(人) فناوری(技) جهان(界) صنعت(産)",
      philosophy: "هونگیک اینگان (弘益人間)",
      philosophy_desc: "به نفع تمام بشریت",
      total: "مجموع",
      domains: "حوزه‌ها",
      live: "فعال",
      coming: "به زودی",
      reset: "بازنشانی",
      condense: "4 دسته",
      links: "پیوندها"
    },
    categories: {
      humanity: { display: "人 بشریت", name: "انسان (人)", full: "بشریت", desc: "زندگی، رفاه، دسترسی" },
      technology: { display: "技 فناوری", name: "فناوری (技)", full: "فناوری", desc: "اتصال، ابزار، رابط" },
      world: { display: "界 جهان", name: "جهان (界)", full: "جهان", desc: "فضا، طبیعت، محیط زیست" },
      industry: { display: "産 صنعت", name: "صنعت (産)", full: "صنعت", desc: "اقتصاد، واقعیت، تولید" }
    },
    drilldown: {
      total: "مجموع",
      live: "فعال",
      coming: "به زودی",
      view_simulator: "مشاهده شبیه‌ساز",
      close: "بستن"
    }
  },

  sw: {
    network: {
      title: "765 Standards",
      subtitle: "Mtu(人) Teknolojia(技) Ulimwengu(界) Viwanda(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Kwa manufaa ya ubinadamu wote",
      total: "Jumla",
      domains: "Nyanja",
      live: "Hai",
      coming: "Inakuja hivi karibuni",
      reset: "Weka upya",
      condense: "Kategoria 4",
      links: "Viungo"
    },
    categories: {
      humanity: { display: "人 UBINADAMU", name: "Mtu (人)", full: "Ubinadamu", desc: "Maisha, Ustawi, Upatikanaji" },
      technology: { display: "技 TEKNOLOJIA", name: "Teknolojia (技)", full: "Teknolojia", desc: "Muunganisho, Zana, Kiolesura" },
      world: { display: "界 ULIMWENGU", name: "Ulimwengu (界)", full: "Ulimwengu", desc: "Anga, Asili, Mazingira" },
      industry: { display: "産 VIWANDA", name: "Viwanda (産)", full: "Viwanda", desc: "Uchumi, Uhalisia, Uzalishaji" }
    },
    drilldown: {
      total: "Jumla",
      live: "Hai",
      coming: "Inakuja hivi karibuni",
      view_simulator: "Tazama Kiigaji",
      close: "Funga"
    }
  },

  af: {
    network: {
      title: "765 Standards",
      subtitle: "Persoon(人) Tegnologie(技) Wêreld(界) Nywerheid(産)",
      philosophy: "Hongik Ingan (弘益人間)",
      philosophy_desc: "Tot voordeel van die hele mensdom",
      total: "Totaal",
      domains: "Domeine",
      live: "Aktief",
      coming: "Binnekort",
      reset: "Herstel",
      condense: "4 Kategorieë",
      links: "Skakels"
    },
    categories: {
      humanity: { display: "人 MENSDOM", name: "Persoon (人)", full: "Mensdom", desc: "Lewe, Welstand, Toeganklikheid" },
      technology: { display: "技 TEGNOLOGIE", name: "Tegnologie (技)", full: "Tegnologie", desc: "Verbinding, Gereedskap, Koppelvlak" },
      world: { display: "界 WÊRELD", name: "Wêreld (界)", full: "Wêreld", desc: "Ruimte, Natuur, Omgewing" },
      industry: { display: "産 NYWERHEID", name: "Nywerheid (産)", full: "Nywerheid", desc: "Ekonomie, Werklikheid, Produksie" }
    },
    drilldown: {
      total: "Totaal",
      live: "Aktief",
      coming: "Binnekort",
      view_simulator: "Bekyk Simulator",
      close: "Sluit"
    }
  }
};

// Default English template for languages without specific translations
const defaultTemplate = {
  network: {
    title: "765 Standards",
    subtitle: "Person(人) Tech(技) World(界) Industry(産)",
    philosophy: "Hongik Ingan (弘益人間)",
    philosophy_desc: "Benefit All Humanity",
    total: "Total",
    domains: "Domains",
    live: "Live",
    coming: "Coming",
    reset: "Reset",
    condense: "4 Categories",
    links: "Links"
  },
  categories: {
    humanity: { display: "人 HUMANITY", name: "Person (人)", full: "Humanity", desc: "Life, Welfare, Accessibility" },
    technology: { display: "技 TECHNOLOGY", name: "Tech (技)", full: "Technology", desc: "Connection, Tools, Interface" },
    world: { display: "界 WORLD", name: "World (界)", full: "World", desc: "Space, Nature, Environment" },
    industry: { display: "産 INDUSTRY", name: "Industry (産)", full: "Industry", desc: "Economy, Reality, Production" }
  },
  drilldown: {
    total: "Total",
    live: "Live",
    coming: "Coming Soon",
    view_simulator: "View Simulator",
    close: "Close"
  }
};

// Process each JSON file
const files = fs.readdirSync(i18nDir).filter(f => f.endsWith('.json'));
let processed = 0;
let skipped = 0;

for (const file of files) {
  const langCode = file.replace('.json', '');
  const filePath = path.join(i18nDir, file);

  // Skip en and ko (already done)
  if (langCode === 'en' || langCode === 'ko') {
    skipped++;
    continue;
  }

  try {
    const content = fs.readFileSync(filePath, 'utf8');
    const json = JSON.parse(content);

    // Check if already has the sections
    if (json.network && json.categories && json.drilldown) {
      console.log(`[SKIP] ${file} - already has translations`);
      skipped++;
      continue;
    }

    // Get translations for this language or use default
    const trans = translations[langCode] || defaultTemplate;

    // Add the three new sections
    json.network = trans.network;
    json.categories = trans.categories;
    json.drilldown = trans.drilldown;

    // Write back
    fs.writeFileSync(filePath, JSON.stringify(json, null, 2) + '\n');
    console.log(`[OK] ${file}`);
    processed++;
  } catch (err) {
    console.error(`[ERROR] ${file}: ${err.message}`);
  }
}

console.log(`\nDone! Processed: ${processed}, Skipped: ${skipped}`);
