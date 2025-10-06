import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import { useWorkspace } from "../components/workspace-provider/workspace-provider";

import "./i18n";

import type { PackageItem } from "./types";

function getWorkspaceName(fullPath: string) {
  const match = fullPath.match(/([^/\\]+_ws)$/);
  return match ? match[1] : "";
}

function usePackageManagementHook() {
  const { workspacePath } = useWorkspace();
  const { t } = useTranslation("package_management");

  const [packages, setPackages] = useState<PackageItem[]>([]);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const selectedWorkspaceName = useMemo(
    () => getWorkspaceName(workspacePath || ""),
    [workspacePath]
  );

  const fetchPackages = useCallback(async () => {
    if (!workspacePath) return;
    const result = await window.electronAPI.getPackages(workspacePath);

    if (Array.isArray(result)) {
      setPackages(result as PackageItem[]);
    } else if (result && typeof result === "object" && "error" in result) {
      console.error("[get-packages]");
      setPackages([]);
    } else {
      setPackages([]);
    }
  }, [workspacePath]);

  useEffect(() => {
    if (!isModalOpen && workspacePath) fetchPackages();
  }, [isModalOpen, workspacePath, fetchPackages]);

  const title = useMemo(() => {
    return t("title", { ws: selectedWorkspaceName });
  }, [t, selectedWorkspaceName]);

  const subtitle = t("subtitle");
  const primaryButtonTitle = t("primaryButton");

  return {
    t,
    state: {
      packages,
      isModalOpen,
      selectedWorkspaceName,
      workspacePath,
    },
    text: {
      title,
      subtitle,
      primaryButtonTitle,
    },
    handlers: {
      openModal: () => setIsModalOpen(true),
      setIsModalOpen,
      setPackages,
      refetch: fetchPackages,
    },
  };
}

export default usePackageManagementHook;
