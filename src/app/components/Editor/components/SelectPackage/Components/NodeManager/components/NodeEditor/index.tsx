import { useEffect, useRef, useState } from "react";
import { Button, Empty, Layout, message, Tooltip } from "antd";
import "./styles.css";
import { Content, Footer, Header } from "antd/es/layout/layout";
import Sider from "antd/es/layout/Sider";
import { BlocklyWorkspace, ToolboxDefinition } from "react-blockly";
import { pythonGenerator } from "blockly/python";
import Blockly from "blockly";
import Editor from "@monaco-editor/react";
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
  const [showBlockEditor, setShowBlockEditor] = useState(true);
  const [showTextEditor, setShowTextEditor] = useState(false);
  const [showTerminal, setShowTerminal] = useState(false);
  const [xml, setXml] = useState("");
  const [generatedCode, setGeneratedCode] = useState("");
  const workspaceRef = useRef<Blockly.WorkspaceSvg | null>(null);

  const MY_TOOLBOX: ToolboxDefinition = {
    kind: "categoryToolbox",
    contents: [
      {
        kind: "category",
        name: "ROS2",
        colour: "#5C81A6",
        contents: [
          { kind: "block", type: "ros_connect" },
          { kind: "block", type: "ros_disconnect" },
          { kind: "block", type: "ros_log_info" },
          { kind: "block", type: "ros_log_warn" },
          { kind: "block", type: "ros_log_error" },
        ],
      },
      {
        kind: "category",
        name: "Functions",
        colour: "#A65C81",
        contents: [
          { kind: "block", type: "controls_for" },
          { kind: "block", type: "controls_whileUntil" },
          { kind: "block", type: "lists_create_with" },
          { kind: "block", type: "lists_getIndex" },
          { kind: "block", type: "procedures_defnoreturn" },
          { kind: "block", type: "procedures_defreturn" },
        ],
      },
      {
        kind: "category",
        name: "Logic",
        colour: "#5CA65C",
        contents: [
          { kind: "block", type: "controls_if" },
          { kind: "block", type: "logic_compare" },
          { kind: "block", type: "logic_operation" },
          { kind: "block", type: "logic_negate" },
          { kind: "block", type: "logic_boolean" },
        ],
      },
      {
        kind: "category",
        name: "Math",
        colour: "#5C68A6",
        contents: [
          { kind: "block", type: "math_number" },
          { kind: "block", type: "math_arithmetic" },
          { kind: "block", type: "math_single" },
          { kind: "block", type: "math_trig" },
          { kind: "block", type: "math_constant" },
        ],
      },
      {
        kind: "category",
        name: "Text",
        colour: "#A65C5C",
        contents: [
          { kind: "block", type: "text" },
          { kind: "block", type: "text_print" },
          { kind: "block", type: "text_join" },
        ],
      },
    ],
  };

  const WORKSPACE_CONFIG: Blockly.BlocklyOptions = {
    grid: {
      spacing: 20,
      length: 3,
      colour: "#ccc",
      snap: true,
    },
    zoom: {
      controls: true,
      wheel: true,
      startScale: 1.0,
      maxScale: 2.0,
      minScale: 0.5,
      scaleSpeed: 1.0,
    },
    trashcan: true,
    move: {
      scrollbars: true,
      drag: true,
      wheel: true,
    },
  };

  const handleGenerateCode = () => {
    if (workspaceRef.current) {
      const code = pythonGenerator.workspaceToCode(workspaceRef.current);
      setGeneratedCode(code);
    } else {
      message.error("Workspace is not available!");
    }
  };

  const handleWorkspaceChange = (workspace: Blockly.WorkspaceSvg) => {
    workspaceRef.current = workspace;
  };

  useEffect(() => {
    if (xml) {
      handleGenerateCode();
    }
  }, [xml]);

  return (
    <>
      <Layout className="layout-container">
        <Header className="header-container">
          <div className="editor-actions">
            <Tooltip
              title={`${
                !showBlockEditor ? "Mostrar" : "Ocultar"
              } editor de blocos`}
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
              title={`${
                !showTextEditor ? "Mostrar" : "Ocultar"
              } editor de texto`}
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
        <Layout
          className={
            !showBlockEditor && !showTextEditor
              ? "empty-container"
              : "editor-container"
          }
        >
          {!showBlockEditor && !showTextEditor ? (
            <Empty
              className="empty-node-editor"
              description="Não há editor de código selecionado"
            />
          ) : (
            <>
              <Content className="content-container" hidden={!showBlockEditor}>
                <BlocklyWorkspace
                  className="width-100"
                  toolboxConfiguration={MY_TOOLBOX}
                  initialXml={xml}
                  onXmlChange={(e) => setXml(e)}
                  workspaceConfiguration={WORKSPACE_CONFIG}
                  onWorkspaceChange={handleWorkspaceChange}
                />
              </Content>
              <Sider
                className="sider-container"
                hidden={!showTextEditor}
                width={!showBlockEditor ? "100%" : "50%"}
              >
                <Editor
                  height="100%"
                  defaultLanguage="python"
                  value={generatedCode ?? selectedNode.content}
                  options={{ readOnly: false, minimap: { enabled: false } }}
                />
              </Sider>
            </>
          )}
        </Layout>
        <Footer className="footer-container" hidden={!showTerminal}>
          Terminal
        </Footer>
      </Layout>
    </>
  );
}

export default NodeEditor;
