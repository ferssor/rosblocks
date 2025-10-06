import { useCallback, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

function usePackageDialogHook() {
  const { t } = useTranslation("package_dialog");
  const [showContent, setShowContent] = useState(true);
  const handleClick = useCallback(function () {
    setShowContent(function (p) {
      return !p;
    });
  }, []);
  return {
    t: t,
    state: { showContent: showContent },
    handlers: { onClick: handleClick },
  };
}

export default usePackageDialogHook;
