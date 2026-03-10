import { Button, Result, Table } from "antd";
import { memo } from "react";

import { WorkspaceDialog } from "./components/workspace-dialog";
import useWorkspaceManagementHook from "./workspace-management.hook";
import styles from "./workspace-management.styles.module.css";

import type { RecentWorkspace, WorkspaceManagementProps } from "./types";
import type { TableProps } from "antd";

function WorkspaceManagement(props: WorkspaceManagementProps) {
  const { className = "", style } = props;
  const { t, state, handlers } = useWorkspaceManagementHook();

  const hasRecentWorkspaces =
    state.recentWorkspaces && state.recentWorkspaces.length > 0;

  const columns: TableProps<RecentWorkspace>["columns"] = [
    {
      title: t("name"),
      dataIndex: "name",
      key: "name",
    },
    {
      title: t("path"),
      dataIndex: "path",
      key: "path",
    },
  ];

  return (
    <>
      <div
        className={hasRecentWorkspaces ? styles.container : styles.fullCenter}
      >
        {hasRecentWorkspaces ? (
          <div
            className={[styles.recentListContainer, className]
              .filter(Boolean)
              .join(" ")}
            style={style}
          >
            <div className={styles.header}>
              <h1 className={styles.title}>{t("recentWorkspacesTitle")}</h1>
              <div className={styles.actions}>
                <Button type="primary" onClick={handlers.openCreateModal}>
                  {t("primaryButton")}
                </Button>
                <Button onClick={handlers.handleOpenWorkspace}>
                  {t("secondaryButton")}
                </Button>
              </div>
            </div>
            <Table
              dataSource={state.recentWorkspaces}
              columns={columns}
              rowKey="path"
              onRow={(record) => ({
                onClick: () => handlers.handleOpenRecentWorkspace(record.path),
                style: { cursor: "pointer" },
              })}
              pagination={false}
            />
          </div>
        ) : (
          <Result
            className={[styles.welcomeScreenContainer, className]
              .filter(Boolean)
              .join(" ")}
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
              </div>,
            ]}
          />
        )}
      </div>
      <WorkspaceDialog
        isModalOpen={state.openModal}
        setIsModalOpen={handlers.setOpenModal}
      />
    </>
  );
}

export default memo(WorkspaceManagement);
