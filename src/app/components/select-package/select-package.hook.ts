import { useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { SelectPackageProps } from "./types";

function getWorkspaceName(fullPath: string) {
  const match = fullPath.match(/([^/]+_ws)$/);
  return match ? match[1] : "";
}

function useSelectPackageHook(props: SelectPackageProps) {
  const { workspaceLocation, setPackages } = props;
  const [isModalOpen, setIsModalOpen] = useState(false);
  const { t } = useTranslation("select_package");

  const selectedWorkspaceName = useMemo(() => {
    if (!workspaceLocation) {
      return "";
    }

    return getWorkspaceName(workspaceLocation);
  }, [workspaceLocation]);

  useEffect(() => {
    async function fetchPackages() {
      if (workspaceLocation && !isModalOpen) {
        const result = await window.electronAPI.getPackages(workspaceLocation);
        setPackages(result);
      }
    }

    fetchPackages();
  }, [isModalOpen, setPackages, workspaceLocation]);

  const text = useMemo(
    () => ({
      title: t("empty.title", { workspace: selectedWorkspaceName }),
      subtitle: t("empty.subtitle"),
      primaryButtonTitle: t("empty.primaryButton"),
    }),
    [selectedWorkspaceName, t]
  );

  return {
    text,
    state: {
      isModalOpen,
      selectedWorkspaceName,
    },
    handlers: {
      setModalOpen: setIsModalOpen,
    },
  };
}

export default useSelectPackageHook;
