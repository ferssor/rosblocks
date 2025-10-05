import { memo } from "react";
import { HeaderProps } from "./types";
import useHeaderHook from "./header.hook";
import styles from "./header.styles.module.css";
import { Layout } from "antd";
import { RobotOutlined } from "@ant-design/icons";

function Header(props: HeaderProps) {
  const { className = "", style } = props;
  const { handlers } = useHeaderHook();
  const LOGO_NAME = "ROSBLOCKS";

  const { Header } = Layout;

  return (
    <div
      id={props.id}
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      <Header>
        <div className={styles.logo} onClick={handlers.onClick}>
          <RobotOutlined className={styles.icon} />
          <h1 className={styles.title}>{LOGO_NAME}</h1>
        </div>
      </Header>
    </div>
  );
}

export default memo(Header);
