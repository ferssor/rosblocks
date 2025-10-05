import { useTranslation } from "react-i18next";
import "./i18n";

function useHeaderHook() {
  const { t } = useTranslation("header");
  const handleClick = () => (window.location.href = "/");

  return {
    t: t,
    state: {},
    handlers: { onClick: handleClick },
  };
}

export default useHeaderHook;
