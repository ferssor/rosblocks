import { message } from "antd";
import { pythonGenerator } from "blockly/python";
import { useCallback, useEffect, useMemo, useReducer, useRef } from "react";
import { useTranslation } from "react-i18next";

import "../../i18n";

import { toolbox } from "./toolbox";

import type { NodeEditorProps } from "./types";
import type Blockly from "blockly";

const WORKSPACE_CONFIG: Blockly.BlocklyOptions = {
  grid: {
    spacing: 20,
    length: 3,
    colour: "#ccc",
    snap: true,
  },
  scrollbars: true,
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
  toolbox,
};

type NodeEditorState = {
  showBlockEditor: boolean;
  showTextEditor: boolean;
  showTerminal: boolean;
  json: Record<string, unknown>;
  dependency: string;
  generatedCode: string | undefined;
  wasEdited: boolean;
};

type NodeEditorAction =
  | { type: "TOGGLE_BLOCK_EDITOR" }
  | { type: "TOGGLE_TEXT_EDITOR" }
  | { type: "TOGGLE_TERMINAL" }
  | { type: "SET_JSON"; payload: Record<string, unknown> }
  | { type: "SET_DEPENDENCY"; payload: string }
  | { type: "SET_GENERATED_CODE"; payload: string | undefined }
  | { type: "SET_WAS_EDITED"; payload: boolean };

const initialState: NodeEditorState = {
  showBlockEditor: true,
  showTextEditor: false,
  showTerminal: false,
  json: {},
  dependency: "",
  generatedCode: undefined,
  wasEdited: false,
};

function nodeEditorReducer(
  state: NodeEditorState,
  action: NodeEditorAction,
): NodeEditorState {
  switch (action.type) {
    case "TOGGLE_BLOCK_EDITOR":
      return { ...state, showBlockEditor: !state.showBlockEditor };
    case "TOGGLE_TEXT_EDITOR":
      return { ...state, showTextEditor: !state.showTextEditor };
    case "TOGGLE_TERMINAL":
      return { ...state, showTerminal: !state.showTerminal };
    case "SET_JSON": {
      const newState = { ...state, json: action.payload };
      if (!action.payload || Object.keys(action.payload).length === 0) {
        newState.generatedCode = undefined;
      }
      return newState;
    }
    case "SET_DEPENDENCY":
      return { ...state, dependency: action.payload };
    case "SET_GENERATED_CODE":
      return { ...state, generatedCode: action.payload };
    case "SET_WAS_EDITED":
      return { ...state, wasEdited: action.payload };
    default:
      return state;
  }
}

function useNodeEditorHook(props: NodeEditorProps) {
  const { selectedNode, pkgLocation, pkgName, fetchNodes, setSelectedNode } =
    props;

  const [state, dispatch] = useReducer(nodeEditorReducer, initialState);
  const {
    showBlockEditor,
    showTextEditor,
    showTerminal,
    json,
    dependency,
    generatedCode,
    wasEdited,
  } = state;

  const workspaceRef = useRef<Blockly.WorkspaceSvg | null>(null);
  const { t } = useTranslation("node_manager");

  const initialWorkspace = useMemo(() => {
    if (selectedNode.content) {
      try {
        return JSON.parse(selectedNode.content);
      } catch {
        return {};
      }
    }

    return {};
  }, [selectedNode.content]);

  const handleWorkspaceChange = useCallback(
    (workspace: Blockly.WorkspaceSvg) => {
      workspaceRef.current = workspace;
    },
    [],
  );

  const handleGenerateCode = useCallback(() => {
    if (workspaceRef.current) {
      const code = pythonGenerator.workspaceToCode(workspaceRef.current);
      dispatch({
        type: "SET_GENERATED_CODE",
        payload: code.trim().length > 0 ? code : undefined,
      });
      return;
    }

    dispatch({ type: "SET_GENERATED_CODE", payload: undefined });
  }, []);

  const findValueByKey = useCallback(
    (obj: unknown, key: string): string | null => {
      if (!obj || typeof obj !== "object") {
        return null;
      }

      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        const value = (obj as Record<string, string>)[key];
        return typeof value === "string" ? value : null;
      }

      for (const propKey in obj as Record<string, unknown>) {
        if (Object.prototype.hasOwnProperty.call(obj, propKey)) {
          const value = findValueByKey(
            (obj as Record<string, unknown>)[propKey],
            key,
          );
          if (value !== null) {
            return value;
          }
        }
      }

      return null;
    },
    [],
  );

  const getInterfaceDependency = useCallback(
    (jsonValue: Record<string, unknown>) => {
      const dependencyName = findValueByKey(jsonValue, "INTERFACE");
      if (dependencyName) {
        const match = dependencyName.match(/from\s+(\w+)\./);
        if (match && match[1]) {
          dispatch({ type: "SET_DEPENDENCY", payload: match[1] });
        } else {
          dispatch({ type: "SET_DEPENDENCY", payload: "" });
        }
      } else {
        dispatch({ type: "SET_DEPENDENCY", payload: "" });
      }
    },
    [findValueByKey],
  );

  const handleJsonChange = useCallback((value: object) => {
    const parsedValue = value as Record<string, unknown>;
    dispatch({ type: "SET_JSON", payload: parsedValue });
  }, []);

  const handleAddDependency = useCallback(
    async (relativePath: string, nodePath: string, interfaceName: string) => {
      if (relativePath && nodePath && interfaceName) {
        try {
          const result = await window.electronAPI.addDependency(
            relativePath,
            nodePath,
            interfaceName,
          );

          if (result.wasAdded) {
            message.success(t("editor.messages.dependencyAdded"));
          } else if (result.error) {
            message.error(
              t("editor.messages.dependencyError", { error: result.error }),
            );
          }
        } catch (error) {
          if (error instanceof Error) {
            message.error(
              t("editor.messages.dependencyError", { error: error.message }),
            );
          }
        }
      }
    },
    [t],
  );

  const handleAddScript = useCallback(
    async (relativePath: string, nodePath: string, scriptName: string) => {
      try {
        const result = await window.electronAPI.addScript(
          relativePath,
          nodePath,
          scriptName,
        );
        if (result.wasAdded) {
          message.success(t("editor.messages.scriptAdded"));
        } else if (result.error) {
          message.error(
            t("editor.messages.scriptError", { error: result.error }),
          );
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("editor.messages.scriptError", { error: error.message }),
          );
        }
      }
    },
    [t],
  );

  const handleBuildPackage = useCallback(
    async (packageName: string, packageLocation: string) => {
      try {
        const result = await window.electronAPI.buildPackage(
          packageLocation,
          packageName,
        );
        if (result.wasBuilded) {
          message.success(t("editor.messages.buildSuccess"));
        } else if (result.error) {
          message.error(
            t("editor.messages.buildError", { error: result.error }),
          );
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("editor.messages.buildError", { error: error.message }),
          );
        }
      }
    },
    [t],
  );

  const handleSaveCode = useCallback(async () => {
    if (!selectedNode || !json || !generatedCode) {
      return;
    }

    try {
      const result = await window.electronAPI.createBlocks(
        selectedNode.fullPath,
        JSON.stringify(json),
        generatedCode,
      );

      if (result.created) {
        message.success(t("editor.messages.blocksSuccess"));
        await fetchNodes(pkgLocation, pkgName);
        setSelectedNode(selectedNode);

        if (dependency) {
          await handleAddDependency(
            selectedNode.relativePath,
            selectedNode.fullPath,
            dependency,
          );
        }

        await handleAddScript(
          selectedNode.relativePath,
          selectedNode.fullPath,
          selectedNode.name,
        );
        await handleBuildPackage(pkgName, pkgLocation);
      } else if (result.error) {
        message.error(
          t("editor.messages.blocksError", { error: result.error }),
        );
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.blocksError", { error: error.message }),
        );
      }
    }
  }, [
    dependency,
    fetchNodes,
    generatedCode,
    handleAddDependency,
    handleAddScript,
    handleBuildPackage,
    json,
    pkgLocation,
    pkgName,
    selectedNode,
    setSelectedNode,
    t,
  ]);

  const handleDeleteNode = useCallback(async () => {
    if (!selectedNode) {
      return;
    }

    try {
      const result = await window.electronAPI.deleteNode(
        selectedNode.name,
        selectedNode.fullPath,
        pkgName,
      );
      if (result.wasDeleted) {
        message.success(t("editor.messages.deleteSuccess"));
        await fetchNodes(pkgLocation, pkgName);
        setSelectedNode(undefined);

        if (dependency) {
          const removeDependencyResult =
            await window.electronAPI.removeDependency(
              selectedNode.relativePath,
              selectedNode.fullPath,
              selectedNode.name,
              dependency,
            );
          if (removeDependencyResult.wasRemoved) {
            message.success(t("editor.messages.dependencyRemovedSuccess"));
          } else if (removeDependencyResult.error) {
            message.error(
              t("editor.messages.dependencyRemovedError", {
                error: removeDependencyResult.error,
              }),
            );
          }
        }
      } else if (result.error) {
        message.error(
          t("editor.messages.deleteError", { error: result.error }),
        );
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.deleteError", { error: error.message }),
        );
      }
    }
  }, [
    dependency,
    fetchNodes,
    pkgLocation,
    pkgName,
    selectedNode,
    setSelectedNode,
    t,
  ]);

  const handleExecuteCode = useCallback(async () => {
    if (!selectedNode) {
      return;
    }

    await handleSaveCode();

    try {
      const result = await window.electronAPI.executeNode(
        pkgName,
        selectedNode.name,
        pkgLocation,
      );
      if (result.executed) {
        message.success(t("editor.messages.executeSuccess"));
      } else if (result.error) {
        message.error(
          t("editor.messages.executeError", { error: result.error }),
        );
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.executeError", { error: error.message }),
        );
      }
    }
  }, [handleSaveCode, pkgLocation, pkgName, selectedNode, t]);

  useEffect(() => {
    if (json && Object.keys(json).length > 0) {
      handleGenerateCode();
      getInterfaceDependency(json);
    }

    const parsedContent = selectedNode.content
      ? (() => {
          try {
            return JSON.parse(selectedNode.content);
          } catch {
            return {};
          }
        })()
      : {};

    dispatch({
      type: "SET_WAS_EDITED",
      payload: JSON.stringify(json) !== JSON.stringify(parsedContent),
    });
  }, [getInterfaceDependency, handleGenerateCode, json, selectedNode.content]);

  useEffect(() => {
    if (workspaceRef.current) {
      workspaceRef.current.clear();
    }
    dispatch({ type: "SET_JSON", payload: initialWorkspace });
  }, [initialWorkspace]);

  const text = useMemo(
    () => ({
      hideBlockEditor: t("editor.hideBlockEditor"),
      showBlockEditor: t("editor.showBlockEditor"),
      hideTerminal: t("editor.hideTerminal"),
      showTerminal: t("editor.showTerminal"),
      hideTextEditor: t("editor.hideTextEditor"),
      showTextEditor: t("editor.showTextEditor"),
      deleteButton: t("editor.deleteButton"),
      executeButton: t("editor.executeButton"),
      saveButton: t("editor.saveButton"),
      noEditorSelected: t("editor.noEditorSelected"),
      terminalPlaceholder: t("editor.terminalPlaceholder"),
    }),
    [t],
  );

  return {
    state: {
      showBlockEditor,
      showTextEditor,
      showTerminal,
      generatedCode,
      wasEdited,
      initialWorkspace,
    },
    handlers: {
      toggleBlockEditor: () => dispatch({ type: "TOGGLE_BLOCK_EDITOR" }),
      toggleTextEditor: () => dispatch({ type: "TOGGLE_TEXT_EDITOR" }),
      toggleTerminal: () => dispatch({ type: "TOGGLE_TERMINAL" }),
      handleWorkspaceChange,
      handleJsonChange,
      handleSaveCode,
      handleDeleteNode,
      handleExecuteCode,
    },
    workspaceConfig: WORKSPACE_CONFIG,
    text,
  };
}

export default useNodeEditorHook;
