import { message } from "antd";
import { useCallback, useState } from "react";
import { useTranslation } from "react-i18next";

import { useWorkspace } from "../components/workspace-provider/workspace-provider";

import "./i18n";

function useWorkspaceManagementHook() {
  const { t } = useTranslation("workspace_management");
  const { setWorkspacePath, setIsValidWorkspace } = useWorkspace();
  const [openModal, setOpenModal] = useState(false);

  const handleOpenWorkspace = useCallback(async () => {
    const result = await window.electronAPI.openWorkspaceLocation();
    const validateWorkspace = await window.electronAPI.validateWorkspace(
      result.workspaceLocation
    );

    if (!validateWorkspace.valid && !result.canceled) {
      message.error(t("invalidWorkspace"));
    }

    setWorkspacePath(result.workspaceLocation || "");
    setIsValidWorkspace(Boolean(validateWorkspace.valid));
  }, [setIsValidWorkspace, setWorkspacePath, t]);

  return {
    t,
    state: { openModal },
    handlers: {
      openCreateModal: () => setOpenModal(true),
      closeCreateModal: () => setOpenModal(false),
      handleOpenWorkspace,
      setOpenModal,
    },
  };
}

export default useWorkspaceManagementHook;
