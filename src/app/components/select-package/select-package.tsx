import { Button, Result } from "antd";
import { memo } from "react";

import { PackageDialog } from "../package-dialog";
import { PackageList } from "../package-list";

import useSelectPackageHook from "./select-package.hook";
import styles from "./select-package.styles.module.css";

import type { SelectPackageProps } from "./types";

function SelectPackage(props: SelectPackageProps) {
  const { className = "", style, id, packages } = props;
  const { text, state, handlers } = useSelectPackageHook(props);

  if (!props.workspaceLocation) {
    return null;
  }

  return (
    <>
      {packages.length > 0 ? (
        <PackageList
          packages={packages}
          selectedWorkspaceName={state.selectedWorkspaceName}
          selectedWorkspaceLocation={props.workspaceLocation}
          setPackages={props.setPackages}
        />
      ) : (
        <div
          id={id}
          className={(styles.packageContainer + " " + (className || "")).trim()}
          style={style}
        >
          <Result
            className={styles.result}
            status="404"
            title={text.title}
            subTitle={text.subtitle}
            extra={[
              <div key="actions" className={styles.actions}>
                <Button type="primary" onClick={() => handlers.setModalOpen(true)}>
                  {text.primaryButtonTitle}
                </Button>
                <PackageDialog
                  packageLocation={props.workspaceLocation}
                  isModalOpen={state.isModalOpen}
                  setIsModalOpen={handlers.setModalOpen}
                  setPackages={props.setPackages}
                />
              </div>,
            ]}
          />
        </div>
      )}
    </>
  );
}

export default memo(SelectPackage);
