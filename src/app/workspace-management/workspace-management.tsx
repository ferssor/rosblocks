import { Button, Result, Table } from "antd";
import {
  differenceInDays,
  differenceInMonths,
  differenceInYears,
  format,
} from "date-fns";
import { ptBR } from "date-fns/locale";
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
    {
      title: t("packageCount"),
      dataIndex: "packageCount",
      key: "packageCount",
      render: (value?: number) => value ?? 0,
    },
    {
      title: t("nodeCount"),
      dataIndex: "nodeCount",
      key: "nodeCount",
      render: (value?: number) => value ?? 0,
    },
    {
      title: t("lastModified"),
      dataIndex: "lastModified",
      key: "lastModified",
      render: (lastModified?: string) => {
        if (!lastModified) return "-";
        const modifiedAtDate = new Date(lastModified);
        const now = new Date();
        const diffInDays = differenceInDays(now, modifiedAtDate);
        const diffInWeeks = Math.floor(diffInDays / 7);
        const diffInMonths = differenceInMonths(now, modifiedAtDate);
        const diffInYears = differenceInYears(now, modifiedAtDate);

        if (diffInDays < 7) {
          return format(modifiedAtDate, "EEEE, HH:mm", { locale: ptBR });
        } else if (diffInDays < 30) {
          return `${diffInWeeks} semanas atrás`;
        } else if (diffInYears === 0) {
          return `${diffInMonths} meses atrás`;
        }
        return format(modifiedAtDate, "dd/MM/yyyy", { locale: ptBR });
      },
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
