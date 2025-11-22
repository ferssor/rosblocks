import i18n from "i18next";
import { initReactI18next } from "react-i18next";

const STORAGE_KEY = "app.language";

const normalize = (lng?: string | null) => {
  if (!lng) return undefined;
  return lng.replace("_", "-");
};

const saved =
  typeof window !== "undefined" ? localStorage.getItem(STORAGE_KEY) : null;

i18n.use(initReactI18next).init({
  lng: normalize(saved) || "en-US",
  fallbackLng: ["en-US", "en_US", "pt-BR", "pt_BR"],
  supportedLngs: ["en-US", "en_US", "pt-BR", "pt_BR"],
  interpolation: { escapeValue: false },
  react: { useSuspense: false },
});

i18n.on("languageChanged", (lng) => {
  if (typeof window === "undefined") return;
  const normalized = normalize(lng);
  if (normalized) {
    localStorage.setItem(STORAGE_KEY, normalized);
  }
});

export default i18n;
