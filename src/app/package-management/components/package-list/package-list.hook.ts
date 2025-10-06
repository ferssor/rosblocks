import { message } from "antd";
import { ptBR } from "date-fns/locale";
import { useCallback, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import type { PackageListProps } from "./types";
import type { PackageItem } from "../../types";

function usePackageListHook(props: PackageListProps) {
  const { t, i18n } = useTranslation("package_list"); // ⬅️ use o i18n daqui

  const {
    packages,
    selectedWorkspaceName,
    selectedWorkspaceLocation,
    setPackages,
  } = props;

  const [pkgName, setPkgName] = useState("");
  const [pkgLocation, setPkgLocation] = useState("");
  const [pkgType, setPkgType] = useState("");
  const [isModalOpen, setIsModalOpen] = useState(false);

  const locale = useMemo(
    () => (i18n.language?.toLowerCase().startsWith("pt") ? ptBR : undefined),
    [i18n.language]
  );

  const fetchPackages = useCallback(async () => {
    const result = await window.electronAPI.getPackages(
      selectedWorkspaceLocation
    );
    if (Array.isArray(result)) {
      setPackages(result as PackageItem[]);
    } else if (result && typeof result === "object" && "error" in result) {
      message.error(t("errors.loadFailed"));
      setPackages([]);
    }
  }, [selectedWorkspaceLocation, setPackages, t]);

  const handleDeletePackage = useCallback(
    async (path: string) => {
      try {
        const result = await window.electronAPI.deletePackage(path);
        if (result?.deleted) {
          message.success(t("messages.deleted"));
          fetchPackages();
        } else if (result?.error) {
          const errorMessage =
            typeof result.error === "string"
              ? result.error
              : JSON.stringify(result.error);
          message.error(t("errors.deleteFailed", { msg: errorMessage }));
        }
      } catch {
        message.error(t("errors.deleteUnexpected"));
      }
    },
    [fetchPackages, t]
  );

  const handleBuildPackage = useCallback(
    async (path: string, name: string) => {
      try {
        const result = await window.electronAPI.buildPackage(path, name);
        if (result?.wasBuilded) {
          message.success(t("messages.built"));
          fetchPackages();
        } else if (result?.error) {
          const errorMessage =
            typeof result.error === "string"
              ? result.error
              : JSON.stringify(result.error);
          message.error(t("errors.buildFailed", { msg: errorMessage }));
        }
      } catch {
        message.error(t("errors.buildUnexpected"));
      }
    },
    [fetchPackages, t]
  );

  return {
    t,
    locale,
    state: {
      packages,
      isModalOpen,
      pkgName,
      pkgLocation,
      pkgType,
      selectedWorkspaceName,
      selectedWorkspaceLocation,
    },
    handlers: {
      handleDeletePackage,
      handleBuildPackage,
      setIsModalOpen,
      setPackages,
      fetchPackages,
      setPkgName,
      setPkgLocation,
      setPkgType,
    },
  };
}

export default usePackageListHook;
