import { useCallback, useEffect, useRef, useState } from "react";
import { Button, Empty, Layout, message, Tooltip } from "antd";
import "./styles.css";
import { Content, Footer, Header } from "antd/es/layout/layout";
import Sider from "antd/es/layout/Sider";
import { BlocklyWorkspace } from "react-blockly";
import { pythonGenerator } from "blockly/python";
import Blockly from "blockly";
import Editor from "@monaco-editor/react";
import {
  ApartmentOutlined,
  CodeOutlined,
  FileTextOutlined,
} from "@ant-design/icons";
import { toolbox } from "./toolbox";
import "./blocks/customBlocks";
import { registerCustomBlocksToPython } from "./generators/blocksToPython";
import { defineCustomBlocks } from "./blocks/customBlocks";

interface Props {
  selectedNode: ROSNode;
}

registerCustomBlocksToPython();
defineCustomBlocks();

function NodeEditor(props: Props) {
  const { selectedNode } = props;
  const [showBlockEditor, setShowBlockEditor] = useState(true);
  const [showTextEditor, setShowTextEditor] = useState(false);
  const [showTerminal, setShowTerminal] = useState(false);
  const [xml, setXml] = useState("");
  const [json, setJson] = useState(new Object());
  const [dependency, setDependency] = useState("");
  const [generatedCode, setGeneratedCode] = useState("");
  const workspaceRef = useRef<Blockly.WorkspaceSvg | null>(null);

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

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const findValueByKey = useCallback((obj: any, key: string): string | null => {
    if (!obj || typeof obj !== "object") return null;

    if (Object.prototype.hasOwnProperty.call(obj, key)) {
      return obj[key];
    }

    for (const k in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, k)) {
        const value = findValueByKey(obj[k], key);
        if (value !== null) {
          return value;
        }
      }
    }

    return null;
  }, []);

  const getInterfaceDependency = useCallback(
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (json: any) => {
      const dependencyName = findValueByKey(json, "INTERFACE");
      if (dependencyName) {
        const match = dependencyName.match(/from\s+(\w+)\./);
        if (match && match[1]) {
          setDependency(match[1]);
        } else {
          setDependency("");
        }
      }
    },
    [findValueByKey]
  );

  const handleSaveCode = async () => {
    if (selectedNode && dependency && generatedCode) {
      try {
        const result = await window.electronAPI.createBlocks(
          dependency,
          selectedNode.name,
          selectedNode.relativePath,
          selectedNode.fullPath,
          generatedCode
        );

        if (result.created) {
          message.success("Blocos criados com sucesso!");
        } else {
          message.error(result.error);
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(`Ocorreu um erro ao criar o bloco!`);
        }
      }
    }
  };

  useEffect(() => {
    if (xml || json) {
      handleGenerateCode();
      getInterfaceDependency(json);
    }
  }, [getInterfaceDependency, json, xml]);

  useEffect(() => {
    if (workspaceRef.current) {
      workspaceRef.current.clear();
    }
    setXml(selectedNode.content || "");
  }, [selectedNode]);

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
          <div className="editor-action-buttons">
            <Button variant="solid" danger>
              Deletar
            </Button>
            <Button
              type="primary"
              color="green"
              variant="solid"
              onClick={handleSaveCode}
            >
              Salvar
            </Button>
          </div>
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
                  key={selectedNode.fullPath}
                  className="width-100"
                  initialJson={new Object(selectedNode.content)}
                  initialXml={selectedNode.content}
                  onXmlChange={(e) => setXml(e)}
                  onJsonChange={(e) => setJson(e)}
                  toolboxConfiguration={toolbox}
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
