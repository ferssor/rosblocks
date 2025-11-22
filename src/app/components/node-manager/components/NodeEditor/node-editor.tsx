import {
  ApartmentOutlined,
  CodeOutlined,
  FileTextOutlined,
} from "@ant-design/icons";
import Editor from "@monaco-editor/react";
import { Button, Empty, Layout, Tooltip } from "antd";
import Sider from "antd/es/layout/Sider";
import { memo } from "react";
import { BlocklyWorkspace } from "react-blockly";

import { defineCustomBlocks } from "./blocks/customBlocks";
import { registerCustomBlocksToPython } from "./generators/blocksToPython";
import useNodeEditorHook from "./node-editor.hook";
import styles from "./node-editor.styles.module.css";
import { toolbox } from "./toolbox";

import type { NodeEditorProps } from "./types";

registerCustomBlocksToPython();
defineCustomBlocks();

const { Header, Content, Footer } = Layout;

function NodeEditor(props: NodeEditorProps) {
  const { className = "", style } = props;
  const { state, handlers, workspaceConfig, text } = useNodeEditorHook(props);

  return (
    <Layout
      className={(styles.layoutContainer + " " + (className || "")).trim()}
      style={style}
    >
      <Header className={styles.headerContainer}>
        <div className={styles.editorActions}>
          <Tooltip
            title={
              state.showBlockEditor ? text.hideBlockEditor : text.showBlockEditor
            }
          >
            <Button
              className={
                state.showBlockEditor ? styles.activeButton : undefined
              }
              type="default"
              icon={<ApartmentOutlined />}
              onClick={handlers.toggleBlockEditor}
              aria-checked={state.showBlockEditor}
            />
          </Tooltip>
          <Tooltip title={state.showTerminal ? text.hideTerminal : text.showTerminal}>
            <Button
              style={{ display: "none" }}
              className={
                state.showTerminal ? styles.activeButton : undefined
              }
              type="default"
              icon={<CodeOutlined />}
              onClick={handlers.toggleTerminal}
              aria-checked={state.showTerminal}
            />
          </Tooltip>
          <Tooltip
            title={
              state.showTextEditor ? text.hideTextEditor : text.showTextEditor
            }
          >
            <Button
              type="default"
              icon={<FileTextOutlined />}
              className={
                state.showTextEditor ? styles.activeButton : undefined
              }
              onClick={handlers.toggleTextEditor}
              aria-checked={state.showTextEditor}
            />
          </Tooltip>
        </div>
        <h3 className={styles.nodeTitle}>{props.selectedNode.name}</h3>
        <div className={styles.editorActionButtons}>
          <Button variant="solid" danger onClick={handlers.handleDeleteNode}>
            {text.deleteButton}
          </Button>
          <Button type="primary" onClick={handlers.handleExecuteCode}>
            {text.executeButton}
          </Button>
          <Button
            type="primary"
            color="green"
            variant="solid"
            onClick={handlers.handleSaveCode}
            disabled={!state.wasEdited}
          >
            {text.saveButton}
          </Button>
        </div>
      </Header>
      <Layout
        className={
          state.showBlockEditor || state.showTextEditor
            ? styles.editorContainer
            : styles.emptyContainer
        }
      >
        {state.showBlockEditor || state.showTextEditor ? (
          <>
            <Content
              className={styles.contentContainer}
              hidden={!state.showBlockEditor}
            >
              <BlocklyWorkspace
                key={props.selectedNode.fullPath}
                className={styles.workspace}
                initialJson={state.initialWorkspace}
                onJsonChange={handlers.handleJsonChange}
                toolboxConfiguration={toolbox}
                workspaceConfiguration={workspaceConfig}
                onWorkspaceChange={handlers.handleWorkspaceChange}
              />
            </Content>
            <Sider
              className={styles.siderContainer}
              hidden={!state.showTextEditor}
              width={!state.showBlockEditor ? "100%" : "50%"}
            >
              <Editor
                height="100%"
                defaultLanguage="python"
                language="python"
                value={state.generatedCode || props.selectedNode.content}
                options={{
                  readOnly: true,
                  minimap: { enabled: false },
                  mouseWheelZoom: true,
                }}
              />
            </Sider>
          </>
        ) : (
          <Empty
            className={styles.emptyNodeEditor}
            description={text.noEditorSelected}
          />
        )}
      </Layout>
      <Footer className={styles.footerContainer} hidden={!state.showTerminal}>
        {text.terminalPlaceholder}
      </Footer>
    </Layout>
  );
}

export default memo(NodeEditor);
