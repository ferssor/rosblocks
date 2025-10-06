import { Button, Result } from "antd";
import { memo } from "react";

import { PackageDialog } from "./components/package-dialog";
import { PackageList } from "./components/package-list";
import usePackageManagementHook from "./package-management.hook";
import styles from "./package-management.styles.module.css";

import type { PackageManagementProps } from "./types";

function PackageManagement(props: PackageManagementProps) {
  const { className = "", style } = props;
  const { text, state, handlers } = usePackageManagementHook();

  if (!state.workspacePath) {
    return null;
  }

  return (
    <>
      {state.packages.length > 0 ? (
        <div
          className={(styles.selectContainer + " " + (className || "")).trim()}
          style={style}
        >
          <PackageList
            packages={state.packages}
            selectedWorkspaceName={state.selectedWorkspaceName}
            selectedWorkspaceLocation={state.workspacePath}
            setPackages={handlers.setPackages}
          />
        </div>
      ) : (
        <div className={styles.fullCenter}>
          <Result
            className={(
              styles.packageContainer +
              " " +
              (className || "")
            ).trim()}
            style={style}
            status="404"
            title={text.title}
            subTitle={text.subtitle}
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
      )}
    </>
  );
}

export default memo(PackageManagement);
