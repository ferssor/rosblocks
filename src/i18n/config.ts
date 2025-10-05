import i18n from "i18next";
import { initReactI18next } from "react-i18next";

const saved =
  typeof window !== "undefined" ? localStorage.getItem("app.language") : null;
const normalize = (lng?: string | null) => {
  if (!lng) return undefined;
  return lng.replace("_", "-");
};

i18n.use(initReactI18next).init({
  lng: normalize(saved) || "en-US",
  fallbackLng: ["en-US", "en_US"],
  interpolation: { escapeValue: false },
  react: { useSuspense: false },
});

export default i18n;
