import { RobotOutlined } from "@ant-design/icons";
import "./styles.css";

const LOGO_NAME = "RoBlocks";
const ICON_COLOR = "#FFF";

function Logo() {
  return (
    <>
      <section className="logo">
        <RobotOutlined color={ICON_COLOR} />
        <h1 className="title">{LOGO_NAME}</h1>
      </section>
    </>
  );
}

export default Logo;
