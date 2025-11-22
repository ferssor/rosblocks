import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { NodeManagerProps } from "./types";

function useNodeManagerHook(props: NodeManagerProps) {
  const {
    packageLocation,
    packageName,
    selectedWorkspaceLocation,
    setPackageLocation,
    setPackageName,
  } = props;
  const [nodes, setNodes] = useState<ROSNode[]>([]);
  const [selectedNode, setSelectedNode] = useState<ROSNode | undefined>(
    undefined
  );
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isPackageModalOpen, setIsPackageModalOpen] = useState(false);
  const [packages, setPackages] = useState<Package[]>([]);
  const { t } = useTranslation("node_manager");

  const fetchNodes = useCallback(
    async (pkgLocation: string, pkgName: string) => {
      const result = await window.electronAPI.getNodes(pkgLocation, pkgName);
      setNodes(result);
    },
    []
  );

  const fetchPackages = useCallback(async () => {
    if (!packageLocation) {
      return;
    }

    const result = await window.electronAPI.getPackages(
      selectedWorkspaceLocation
    );
    setPackages(result);
  }, [packageLocation, selectedWorkspaceLocation]);

  useEffect(() => {
    fetchNodes(packageLocation, packageName);
  }, [fetchNodes, packageLocation, packageName]);

  const handlePackageSelect = useCallback(
    async (option: { value: string; label: string }) => {
      await fetchNodes(option.value, option.label);
      setPackageName(option.label);
      setPackageLocation(option.value);
      setSelectedNode(undefined);
    },
    [fetchNodes, setPackageLocation, setPackageName]
  );

  const text = useMemo(
    () => ({
      emptyTitle: (pkg: string) => t("manager.emptyTitle", { package: pkg }),
      emptySubtitle: t("manager.emptySubtitle"),
      createNodeButton: t("manager.createNode"),
      createPackageButton: t("manager.createPackage"),
      nodeSelectionPlaceholder: t("manager.selectPlaceholder"),
    }),
    [t]
  );

  return {
    text,
    state: {
      nodes,
      selectedNode,
      isModalOpen,
      isPackageModalOpen,
      packages,
    },
    handlers: {
      setSelectedNode,
      setIsModalOpen,
      setIsPackageModalOpen,
      setNodes,
      setPackages,
      fetchNodes,
      fetchPackages,
      handlePackageSelect,
    },
  };
}

export default useNodeManagerHook;
