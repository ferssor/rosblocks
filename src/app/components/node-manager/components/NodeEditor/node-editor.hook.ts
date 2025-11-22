import { message } from "antd";
import { pythonGenerator } from "blockly/python";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";

import "../../i18n";

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
};

function useNodeEditorHook(props: NodeEditorProps) {
  const { selectedNode, pkgLocation, pkgName, fetchNodes, setSelectedNode } =
    props;
  const [showBlockEditor, setShowBlockEditor] = useState(true);
  const [showTextEditor, setShowTextEditor] = useState(false);
  const [showTerminal, setShowTerminal] = useState(false);
  const [json, setJson] = useState<Record<string, unknown>>({});
  const [dependency, setDependency] = useState("");
  const [generatedCode, setGeneratedCode] = useState<string | undefined>(
    undefined
  );
  const [wasEdited, setWasEdited] = useState(false);
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

  const handleWorkspaceChange = useCallback((workspace: Blockly.WorkspaceSvg) => {
    workspaceRef.current = workspace;
  }, []);

  const handleGenerateCode = useCallback(() => {
    if (workspaceRef.current) {
      const code = pythonGenerator.workspaceToCode(workspaceRef.current);
      setGeneratedCode(code.trim().length > 0 ? code : undefined);
      return;
    }

    setGeneratedCode(undefined);
  }, []);

  const findValueByKey = useCallback((obj: unknown, key: string): string | null => {
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
          key
        );
        if (value !== null) {
          return value;
        }
      }
    }

    return null;
  }, []);

  const getInterfaceDependency = useCallback(
    (jsonValue: Record<string, unknown>) => {
      const dependencyName = findValueByKey(jsonValue, "INTERFACE");
      if (dependencyName) {
        const match = dependencyName.match(/from\s+(\w+)\./);
        if (match && match[1]) {
          setDependency(match[1]);
        } else {
          setDependency("");
        }
      } else {
        setDependency("");
      }
    },
    [findValueByKey]
  );

  const handleJsonChange = useCallback((value: object) => {
    const parsedValue = value as Record<string, unknown>;
    setJson(parsedValue);

    if (!parsedValue || Object.keys(parsedValue).length === 0) {
      setGeneratedCode(undefined);
    }
  }, []);

  const handleAddDependency = useCallback(
    async (relativePath: string, nodePath: string, interfaceName: string) => {
      if (relativePath && nodePath && interfaceName) {
        try {
          const result = await window.electronAPI.addDependency(
            relativePath,
            nodePath,
            interfaceName
          );

          if (result.wasAdded) {
            message.success(t("editor.messages.dependencyAdded"));
          } else if (result.error) {
            message.error(
              t("editor.messages.dependencyError", { error: result.error })
            );
          }
        } catch (error) {
          if (error instanceof Error) {
            message.error(
              t("editor.messages.dependencyError", { error: error.message })
            );
          }
        }
      }
    },
    [t]
  );

  const handleAddScript = useCallback(
    async (relativePath: string, nodePath: string, scriptName: string) => {
      try {
        const result = await window.electronAPI.addScript(
          relativePath,
          nodePath,
          scriptName
        );
        if (result.wasAdded) {
          message.success(t("editor.messages.scriptAdded"));
        } else if (result.error) {
          message.error(
            t("editor.messages.scriptError", { error: result.error })
          );
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("editor.messages.scriptError", { error: error.message })
          );
        }
      }
    },
    [t]
  );

  const handleBuildPackage = useCallback(
    async (packageName: string, packageLocation: string) => {
      try {
        const result = await window.electronAPI.buildPackage(
          packageLocation,
          packageName
        );
        if (result.wasBuilded) {
          message.success(t("editor.messages.buildSuccess"));
        } else if (result.error) {
          message.error(
            t("editor.messages.buildError", { error: result.error })
          );
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("editor.messages.buildError", { error: error.message })
          );
        }
      }
    },
    [t]
  );

  const handleSaveCode = useCallback(async () => {
    if (!selectedNode || !json || !generatedCode) {
      return;
    }

    try {
      const result = await window.electronAPI.createBlocks(
        selectedNode.fullPath,
        JSON.stringify(json),
        generatedCode
      );

      if (result.created) {
        message.success(t("editor.messages.blocksSuccess"));
        await fetchNodes(pkgLocation, pkgName);
        setSelectedNode(selectedNode);

        if (dependency) {
          await handleAddDependency(
            selectedNode.relativePath,
            selectedNode.fullPath,
            dependency
          );
        }

        await handleAddScript(
          selectedNode.relativePath,
          selectedNode.fullPath,
          selectedNode.name
        );
        await handleBuildPackage(pkgName, pkgLocation);
      } else if (result.error) {
        message.error(t("editor.messages.blocksError", { error: result.error }));
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.blocksError", { error: error.message })
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
        pkgName
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
              dependency
            );
          if (removeDependencyResult.wasRemoved) {
            message.success(t("editor.messages.dependencyRemovedSuccess"));
          } else if (removeDependencyResult.error) {
            message.error(
              t("editor.messages.dependencyRemovedError", {
                error: removeDependencyResult.error,
              })
            );
          }
        }
      } else if (result.error) {
        message.error(t("editor.messages.deleteError", { error: result.error }));
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.deleteError", { error: error.message })
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
        pkgLocation
      );
      if (result.executed) {
        message.success(t("editor.messages.executeSuccess"));
      } else if (result.error) {
        message.error(t("editor.messages.executeError", { error: result.error }));
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("editor.messages.executeError", { error: error.message })
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

    setWasEdited(JSON.stringify(json) !== JSON.stringify(parsedContent));
  }, [
    getInterfaceDependency,
    handleGenerateCode,
    json,
    selectedNode.content,
  ]);

  useEffect(() => {
    if (workspaceRef.current) {
      workspaceRef.current.clear();
    }
    setJson(initialWorkspace);
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
    [t]
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
      toggleBlockEditor: () => setShowBlockEditor((prev) => !prev),
      toggleTextEditor: () => setShowTextEditor((prev) => !prev),
      toggleTerminal: () => setShowTerminal((prev) => !prev),
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
