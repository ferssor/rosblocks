import { RobotOutlined } from "@ant-design/icons";
import { Layout, Switch } from "antd";
import { memo } from "react";

import useHeaderHook from "./header.hook";
import styles from "./header.styles.module.css";

import type { HeaderProps } from "./types";

function Header(props: HeaderProps) {
  const { className = "", style } = props;
  const { handlers, state, t } = useHeaderHook();
  const LOGO_NAME = "ROSBLOCKS";

  const { Header } = Layout;

  return (
    <div
      id={props.id}
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      <Header className={styles.headerBar}>
        <div className={styles.logo} onClick={handlers.onClick}>
          <RobotOutlined className={styles.icon} />
          <h1 className={styles.title}>{LOGO_NAME}</h1>
        </div>
        <div className={styles.controls}>
          <span className={styles.version}>
            {t("version")}: {state.version}
          </span>
          <div className={styles.languageToggle}>
            <span>
              {t("languageLabel")}: {state.languageLabel}
            </span>
            <Switch
              checked={state.currentLang === "en-US"}
              checkedChildren="EN"
              unCheckedChildren="PT"
              onChange={handlers.toggleLanguage}
              aria-label={t("languageSwitch")}
              className={styles.languageSwitch}
              size="small"
            />
          </div>
        </div>
      </Header>
    </div>
  );
}

export default memo(Header);
