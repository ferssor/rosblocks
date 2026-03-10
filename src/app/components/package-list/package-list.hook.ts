import { message } from "antd";
import { useCallback, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { PackageListProps } from "./types";

function usePackageListHook(props: PackageListProps) {
  const { selectedWorkspaceLocation, setPackages } = props;
  const [packageName, setPackageName] = useState("");
  const [packageLocation, setPackageLocation] = useState("");
  const [packageType, setPackageType] = useState("");
  const [isModalOpen, setIsModalOpen] = useState(false);

  const { t } = useTranslation("package_list");

  const fetchPackages = useCallback(async () => {
    const result = await window.electronAPI.getPackages(
      selectedWorkspaceLocation
    );
    setPackages(result);
  }, [selectedWorkspaceLocation, setPackages]);

  const handleDeletePackage = useCallback(
    async (path: string) => {
      try {
        const result = await window.electronAPI.deletePackage(path);

        if (result.deleted) {
          message.success(t("messages.deleteSuccess"));
          fetchPackages();
        } else if (result.error) {
          const errorMessage =
            typeof result.error === "string"
              ? result.error
              : JSON.stringify(result.error);
          message.error(t("messages.deleteError", { error: errorMessage }));
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("messages.deleteGenericError", { error: error.message })
          );
        }
      }
    },
    [fetchPackages, t]
  );

  const handleBuildPackage = useCallback(
    async (path: string, name: string) => {
      try {
        const result = await window.electronAPI.buildPackage(path, name);
        if (result.wasBuilded) {
          message.success(t("messages.buildSuccess"));
          fetchPackages();
        } else if (result.error) {
          const errorMessage =
            typeof result.error === "string"
              ? result.error
              : JSON.stringify(result.error);
          message.error(t("messages.buildError", { error: errorMessage }));
        }
      } catch (error) {
        if (error instanceof Error) {
          message.error(
            t("messages.buildGenericError", { error: error.message })
          );
        }
      }
    },
    [fetchPackages, t]
  );

  const handleSelectPackage = useCallback((pkg: Package) => {
    setPackageName(pkg.name);
    setPackageLocation(pkg.fullPath);
    setPackageType(pkg.packageType);
  }, []);

  const text = useMemo(
    () => ({
      packageName: t("table.packageName"),
      items: t("table.items"),
      storage: t("table.storage"),
      packageType: t("table.packageType"),
      createdAt: t("table.createdAt"),
      lastUpdated: t("table.lastUpdated"),
      editButton: t("actions.edit"),
      buildButton: t("actions.build"),
      deleteButton: t("actions.delete"),
      addPackageButton: t("actions.addPackage"),
    }),
    [t]
  );

  return {
    t,
    text,
    state: {
      packageName,
      packageLocation,
      packageType,
      isModalOpen,
    },
    handlers: {
      handleDeletePackage,
      handleBuildPackage,
      handleSelectPackage,
      setIsModalOpen,
      setPackageName,
      setPackageLocation,
    },
  };
}

export default usePackageListHook;
