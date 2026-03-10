import { message } from "antd";
import { useCallback, useEffect, useState } from "react";
import { useTranslation } from "react-i18next";

import { useWorkspace } from "../components/workspace-provider/workspace-provider";

import "./i18n";

import type { RecentWorkspace } from "./types";

function useWorkspaceManagementHook() {
  const { t } = useTranslation("workspace_management");
  const { setWorkspacePath, setIsValidWorkspace } = useWorkspace();
  const [openModal, setOpenModal] = useState(false);
  const [recentWorkspaces, setRecentWorkspaces] = useState<RecentWorkspace[]>(
    [],
  );

  useEffect(() => {
    const stored = localStorage.getItem("recentWorkspaces");
    if (stored) {
      try {
        setRecentWorkspaces(JSON.parse(stored));
      } catch (error) {
        console.error("Failed to parse recent workspaces", error);
      }
    }
  }, []);

  const addToRecent = useCallback((path: string) => {
    setRecentWorkspaces((prev) => {
      const name = path.split(/[/\\]/).pop() || path;
      const newItem: RecentWorkspace = { name, path };
      const filtered = prev.filter((item) => item.path !== path);
      const updated = [newItem, ...filtered].slice(0, 5);
      localStorage.setItem("recentWorkspaces", JSON.stringify(updated));
      return updated;
    });
  }, []);

  const handleOpenWorkspace = useCallback(async () => {
    const result = await window.electronAPI.openWorkspaceLocation();

    if (result.canceled) return;

    const validateWorkspace = await window.electronAPI.validateWorkspace(
      result.workspaceLocation,
    );

    if (!validateWorkspace.valid) {
      message.error(t("invalidWorkspace"));
    } else {
      addToRecent(result.workspaceLocation);
    }

    setWorkspacePath(result.workspaceLocation || "");
    setIsValidWorkspace(Boolean(validateWorkspace.valid));
  }, [setIsValidWorkspace, setWorkspacePath, t, addToRecent]);

  const handleOpenRecentWorkspace = useCallback(
    async (path: string) => {
      const validateWorkspace =
        await window.electronAPI.validateWorkspace(path);

      if (!validateWorkspace.valid) {
        message.error(t("invalidWorkspace"));
      } else {
        addToRecent(path);
      }

      setWorkspacePath(path);
      setIsValidWorkspace(Boolean(validateWorkspace.valid));
    },
    [setIsValidWorkspace, setWorkspacePath, t, addToRecent],
  );

  return {
    t,
    state: { openModal, recentWorkspaces },
    handlers: {
      openCreateModal: () => setOpenModal(true),
      closeCreateModal: () => setOpenModal(false),
      handleOpenWorkspace,
      handleOpenRecentWorkspace,
      setOpenModal,
    },
  };
}

export default useWorkspaceManagementHook;
