import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

const normalize = (lng?: string | null) => {
  if (!lng) return "en-US";
  return lng.replace("_", "-");
};

function useHeaderHook() {
  const { t, i18n } = useTranslation("header");
  const initialLang = normalize(i18n.resolvedLanguage || i18n.language);
  const [currentLang, setCurrentLang] = useState(initialLang);

  const handleClick = () => (window.location.href = "/");

  useEffect(() => {
    function handleLanguageChanged(lang: string) {
      const normalized = normalize(lang);
      setCurrentLang(normalized);
      if (typeof window !== "undefined") {
        localStorage.setItem("app.language", normalized);
      }
    }

    i18n.on("languageChanged", handleLanguageChanged);
    return () => {
      i18n.off("languageChanged", handleLanguageChanged);
    };
  }, [i18n]);

  const toggleLanguage = useCallback(() => {
    const nextLang = currentLang === "pt-BR" ? "en-US" : "pt-BR";
    i18n.changeLanguage(nextLang);
  }, [currentLang, i18n]);

  const languageLabel = useMemo(() => {
    return currentLang === "pt-BR" ? "Português" : "English";
  }, [currentLang]);

  return {
    t,
    state: {
      version: __APP_VERSION__,
      currentLang,
      languageLabel,
    },
    handlers: { onClick: handleClick, toggleLanguage },
  };
}

export default useHeaderHook;
