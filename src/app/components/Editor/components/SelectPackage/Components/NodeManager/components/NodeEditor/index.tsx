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
              <Button type="default" icon={<ApartmentOutlined />} />
            </Tooltip>
            <Tooltip title="Mostrar terminal">
              <Button type="default" icon={<CodeOutlined />} />
            </Tooltip>
            <Tooltip>
              <Button type="default" icon={<FileTextOutlined />} />
            </Tooltip>
          </div>
          <h3>{selectedNode.name}</h3>
          <Button type="primary" color="green" variant="solid">
            Salvar
          </Button>
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
