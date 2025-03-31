import { Button, Layout, Tooltip } from "antd";
import "./styles.css";
import { Content, Footer, Header } from "antd/es/layout/layout";
import Sider from "antd/es/layout/Sider";
import {
  ApartmentOutlined,
  CodeOutlined,
  FileTextOutlined,
} from "@ant-design/icons";

interface Props {
  selectedNode: ROSNode;
}

function NodeEditor(props: Props) {
  const { selectedNode } = props;
  console.log("🚀 ~ NodeEditor ~ selectedNode:", selectedNode);

  return (
    <>
      <Layout className="layout-container">
        <Header className="header-container">
          <div className="editor-actions">
            <Tooltip title="Mostrar editor de blocos">
              <ApartmentOutlined />
            </Tooltip>
            <Tooltip title="Mostrar terminal">
              <CodeOutlined />
            </Tooltip>
            <Tooltip>
              <FileTextOutlined title="Mostrar editor de texto" />
            </Tooltip>
          </div>
          <Button type="primary">Salvar</Button>
        </Header>
        <Layout>
          <Content className="content-container">Blockly</Content>
          <Sider className="sider-container" width="50%">
            {selectedNode.content}
          </Sider>
        </Layout>
        <Footer className="footer-container">Terminal</Footer>
      </Layout>
    </>
  );
}

export default NodeEditor;
