import i18n from "./config";

const normalizeLanguage = (lng: string) => lng.replace("_", "-");

export function registerBundles(
  bundles: Record<string, Record<string, unknown>>
) {
  Object.entries(bundles).forEach(([lng, resourcesByNs]) => {
    const normalized = normalizeLanguage(lng);
    Object.entries(resourcesByNs).forEach(([ns, data]) => {
      i18n.addResourceBundle(lng, ns, data, true, true);
      if (normalized !== lng) {
        i18n.addResourceBundle(normalized, ns, data, true, true);
      }
    });
  });
}
