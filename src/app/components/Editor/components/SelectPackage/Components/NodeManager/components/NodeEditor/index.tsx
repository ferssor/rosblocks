import { Layout } from "antd";
import "./styles.css";
import { Content, Footer, Header } from "antd/es/layout/layout";
import Sider from "antd/es/layout/Sider";

interface Props {
  selectedNode: ROSNode;
}

function NodeEditor(props: Props) {
  const { selectedNode } = props;
  console.log("🚀 ~ NodeEditor ~ selectedNode:", selectedNode);

  return (
    <>
      <Layout className="layout-container">
        <Header className="header-container">{selectedNode.name}</Header>
        <Layout>
          <Content className="content-container">Blockly</Content>
          <Sider className="sider-container" width="50%">
            Monaco Editor
          </Sider>
        </Layout>
        <Footer className="footer-container">Terminal</Footer>
      </Layout>
    </>
  );
}

export default NodeEditor;
