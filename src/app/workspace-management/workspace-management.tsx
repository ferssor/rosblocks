import { Button, Result } from "antd";
import { memo } from "react";

import { WorkspaceDialog } from "./components/workspace-dialog";
import useWorkspaceManagementHook from "./workspace-management.hook";
import styles from "./workspace-management.styles.module.css";

import type { WorkspaceManagementProps } from "./types";

function WorkspaceManagement(props: WorkspaceManagementProps) {
  const { className = "", style } = props;
  const { t, state, handlers } = useWorkspaceManagementHook();

  return (
    <div className={styles.fullCenter}>
      <Result
        className={(styles.welcomeContainer + " " + (className || "")).trim()}
        style={style}
        status="info"
        title={t("title")}
        subTitle={t("subtitle")}
        extra={[
          <div key="actions" className={styles.actions}>
            <Button type="primary" onClick={handlers.openCreateModal}>
              {t("primaryButton")}
            </Button>
            <Button onClick={handlers.handleOpenWorkspace}>
              {t("secondaryButton")}
            </Button>

            <WorkspaceDialog
              isModalOpen={state.openModal}
              setIsModalOpen={handlers.setOpenModal}
            />
          </div>,
        ]}
      />
    </div>
  );
}

export default memo(WorkspaceManagement);
