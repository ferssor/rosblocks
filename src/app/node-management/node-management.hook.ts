import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import { useWorkspace } from "../components/workspace-provider/workspace-provider";

import "./i18n";

import type { Dispatch, SetStateAction } from "react";

function getWorkspaceName(fullPath: string) {
  const match = fullPath.match(/([^/\\]+_ws)$/);
  return match ? match[1] : "";
}

function useNodeManagementHook() {
  const { workspacePath } = useWorkspace();
  const { t } = useTranslation("node_management");

  const [packages, setPackages] = useState<Package[]>([]);
  const [selectedPackage, setSelectedPackage] = useState<Package | null>(null);
  const [isModalOpen, setIsModalOpen] = useState(false);

  const setPackagesHandler = useCallback(
    (updater: React.SetStateAction<Package[]>) => {
      setPackages((prev) => {
        const next =
          typeof updater === "function"
            ? (updater as (state: Package[]) => Package[])(prev)
            : updater;
        setSelectedPackage((current) => {
          if (!next.length) {
            return null;
          }
          if (current) {
            const found = next.find(
              (pkg) => pkg.fullPath === current.fullPath
            );
            if (found) {
              return found;
            }
          }
          return next[0];
        });
        return next;
      });
    },
    []
  );

  const fetchPackages = useCallback(async () => {
    if (!workspacePath) {
      setPackagesHandler([]);
      return;
    }

    const result = await window.electronAPI.getPackages(workspacePath);
    if (Array.isArray(result)) {
      setPackagesHandler(result as Package[]);
    } else {
      setPackagesHandler([]);
    }
  }, [setPackagesHandler, workspacePath]);

  useEffect(() => {
    if (!isModalOpen) {
      fetchPackages();
    }
  }, [fetchPackages, isModalOpen]);

  const handleSetPackageName = useCallback<Dispatch<SetStateAction<string>>>(
    (value) => {
      setSelectedPackage((prev) => {
        if (!prev) {
          return prev;
        }
        const current = prev.name;
        const nextValue =
          typeof value === "function" ? value(current) : value ?? "";
        return { ...prev, name: nextValue };
      });
    },
    []
  );

  const handleSetPackageLocation =
    useCallback<Dispatch<SetStateAction<string>>>(
      (value) => {
        setSelectedPackage((prev) => {
          if (!prev) {
            return prev;
          }
          const current = prev.fullPath;
          const nextValue =
            typeof value === "function" ? value(current) : value ?? "";
          const found = packages.find((pkg) => pkg.fullPath === nextValue);
          if (found) {
            return found;
          }
          return { ...prev, fullPath: nextValue };
        });
      },
      [packages]
    );

  const selectedWorkspaceName = useMemo(
    () => getWorkspaceName(workspacePath || ""),
    [workspacePath]
  );

  const text = useMemo(
    () => ({
      title: t("title"),
      subtitle: t("subtitle"),
      primaryButtonTitle: t("primaryButton"),
      emptyTitle: t("emptyTitle", { workspace: selectedWorkspaceName }),
      emptySubtitle: t("emptySubtitle"),
    }),
    [selectedWorkspaceName, t]
  );

  return {
    text,
    state: {
      workspacePath,
      packages,
      selectedPackage,
      isModalOpen,
    },
    handlers: {
      openModal: () => setIsModalOpen(true),
      setIsModalOpen,
      setPackages: setPackagesHandler,
      setPackageName: handleSetPackageName,
      setPackageLocation: handleSetPackageLocation,
    },
  };
}

export default useNodeManagementHook;
