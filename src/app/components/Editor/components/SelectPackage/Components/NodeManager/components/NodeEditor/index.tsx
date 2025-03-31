import { Button, Layout, Tooltip } from "antd";
import "./styles.css";
import { Content, Footer, Header } from "antd/es/layout/layout";
import Sider from "antd/es/layout/Sider";
import {
  ApartmentOutlined,
  CodeOutlined,
  FileTextOutlined,
} from "@ant-design/icons";
import { useEffect, useState } from "react";

interface Props {
  selectedNode: ROSNode;
}

function NodeEditor(props: Props) {
  const { selectedNode } = props;
  const [showBlockEditor, setShowBlockEditor] = useState(false);
  const [showTextEditor, setShowTextEditor] = useState(false);
  const [showTerminal, setShowTerminal] = useState(false);

  useEffect(() => {
    console.log({ showBlockEditor });
  }, [showBlockEditor]);

  return (
    <>
      <Layout className="layout-container">
        <Header className="header-container">
          <div className="editor-actions">
            <Tooltip
              title={`${!showBlockEditor ? "Mostrar" : "Ocultar"} editor de blocos`}
            >
              <Button
                className={showBlockEditor ? "active-button" : ""}
                type="default"
                icon={<ApartmentOutlined />}
                onClick={() => {
                  setShowBlockEditor(!showBlockEditor);
                }}
                aria-checked={showBlockEditor}
              />
            </Tooltip>
            <Tooltip
              title={`${!showTerminal ? "Mostrar" : "Ocultar"} terminal`}
            >
              <Button
                className={showTerminal ? "active-button" : ""}
                type="default"
                icon={<CodeOutlined />}
                onClick={() => {
                  setShowTerminal(!showTerminal);
                }}
                aria-checked={showTerminal}
              />
            </Tooltip>
            <Tooltip
              title={`${!showTextEditor ? "Mostrar" : "Ocultar"} editor de texto`}
            >
              <Button
                type="default"
                icon={<FileTextOutlined />}
                className={showTextEditor ? "active-button" : ""}
                onClick={() => {
                  setShowTextEditor(!showTextEditor);
                }}
                aria-checked={showTextEditor}
              />
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
