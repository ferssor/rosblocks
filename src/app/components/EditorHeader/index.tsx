import { RobotOutlined } from "@ant-design/icons";
import "./styles.css";
import { Layout } from "antd";

const LOGO_NAME = "RoBlocks";
const ICON_COLOR = "#FFF";
const { Header } = Layout;

function EditorHeader() {
  return (
    <>
      <Header>
        <div className="logo">
          <RobotOutlined color={ICON_COLOR} />
          <h1 className="title">{LOGO_NAME}</h1>
        </div>
      </Header>
    </>
  );
}

export default EditorHeader;
