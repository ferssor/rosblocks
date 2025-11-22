import { message } from "antd";
import { useCallback, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { SelectWorkspaceProps } from "./types";

function useSelectWorkspaceHook(props: SelectWorkspaceProps) {
  const { setWorkspacePath, setValidWorkspace } = props;
  const [isModalOpen, setIsModalOpen] = useState(false);
  const { t } = useTranslation("select_workspace");

  const handleOpenWorkspace = useCallback(async () => {
    const result = await window.electronAPI.openWorkspaceLocation();
    const validation = await window.electronAPI.validateWorkspace(
      result.workspaceLocation
    );

    if (!validation.valid && !result.canceled) {
      message.error(t("cta.invalidSelection"));
    }

    setWorkspacePath(result.workspaceLocation);
    setValidWorkspace(validation.valid);
  }, [setValidWorkspace, setWorkspacePath, t]);

  const openCreateModal = useCallback(() => {
    setIsModalOpen(true);
  }, []);

  const text = useMemo(
    () => ({
      title: t("cta.title"),
      subtitle: t("cta.subtitle"),
      primaryButtonTitle: t("cta.primaryButton"),
      secondaryButtonTitle: t("cta.secondaryButton"),
    }),
    [t]
  );

  return {
    text,
    state: {
      isModalOpen,
    },
    handlers: {
      setModalOpen: setIsModalOpen,
      setWorkspaceLocation: setWorkspacePath,
      setValidWorkspace,
      openCreateModal,
      handleOpenWorkspace,
    },
  };
}

export default useSelectWorkspaceHook;
