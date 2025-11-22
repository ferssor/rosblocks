import { Button, Result } from "antd";
import { memo } from "react";

import { NodeManager } from "../components/node-manager";
import { PackageDialog } from "../components/package-dialog";

import useNodeManagementHook from "./node-management.hook";
import styles from "./node-management.styles.module.css";

import type { NodeManagementProps } from "./types";

function NodeManagement(props: NodeManagementProps) {
  const { className = "", style } = props;
  const { state, text, handlers } = useNodeManagementHook();

  if (!state.workspacePath) {
    return null;
  }

  const selectedPackage = state.selectedPackage;

  if (!selectedPackage) {
    return (
      <div className={styles.fullCenter}>
        <Result
          className={(styles.fullCenter + " " + (className || "")).trim()}
          style={style}
          status="404"
          title={text.emptyTitle}
          subTitle={text.emptySubtitle}
          extra={[
            <div key="actions" className={styles.actions}>
              <Button type="primary" onClick={handlers.openModal}>
                {text.primaryButtonTitle}
              </Button>
              <PackageDialog
                packageLocation={state.workspacePath}
                isModalOpen={state.isModalOpen}
                setIsModalOpen={handlers.setIsModalOpen}
                setPackages={handlers.setPackages}
              />
            </div>,
          ]}
        />
      </div>
    );
  }

  return (
    <div
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      <NodeManager
        packageLocation={selectedPackage.fullPath}
        packageName={selectedPackage.name}
        packageType={selectedPackage.packageType}
        selectedWorkspaceLocation={state.workspacePath}
        setPackageName={handlers.setPackageName}
        setPackageLocation={handlers.setPackageLocation}
      />
      <PackageDialog
        packageLocation={state.workspacePath}
        isModalOpen={state.isModalOpen}
        setIsModalOpen={handlers.setIsModalOpen}
        setPackages={handlers.setPackages}
      />
    </div>
  );
}

export default memo(NodeManagement);
