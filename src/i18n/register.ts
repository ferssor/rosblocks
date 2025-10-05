import i18n from "./config";

export function registerBundles(
  bundles: Record<string, Record<string, unknown>>
) {
  Object.entries(bundles).forEach(([lng, resourcesByNs]) => {
    Object.entries(resourcesByNs).forEach(([ns, data]) => {
      i18n.addResourceBundle(lng, ns, data, true, true);
    });
  });
}
