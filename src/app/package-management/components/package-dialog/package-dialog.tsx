import { memo } from "react";

import usePackageDialogHook from "./package-dialog.hook";
import styles from "./package-dialog.styles.module.css";

import type { PackageDialogProps } from "./types";

function PackageDialog(props: PackageDialogProps) {
  const { className = "", style, children } = props;
  const { t, state, handlers } = usePackageDialogHook();

  return (
    <div
      id={props.id}
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
      onClick={handlers.onClick}
    >
      <h2 className={styles.title}>{t("title")}</h2>
      {state.showContent && children}
    </div>
  );
}

export default memo(PackageDialog);
