import { Button, Card, List, Result, Typography } from "antd";
import { memo } from "react";

import { WorkspaceDialog } from "./components/workspace-dialog";
import useWorkspaceManagementHook from "./workspace-management.hook";
import styles from "./workspace-management.styles.module.css";

import type { RecentWorkspace, WorkspaceManagementProps } from "./types";

const { Title } = Typography;

function WorkspaceManagement(props: WorkspaceManagementProps) {
  const { className = "", style } = props;
  const { t, state, handlers } = useWorkspaceManagementHook();

  const hasRecentWorkspaces =
    state.recentWorkspaces && state.recentWorkspaces.length > 0;

  return (
    <>
      <div className={styles.fullCenter}>
        {hasRecentWorkspaces ? (
          <Card
            className={(
              styles.welcomeContainer +
              " " +
              (className || "")
            ).trim()}
            style={style}
          >
            <div className={styles.header}>
              <Title level={4}>{t("recentWorkspacesTitle")}</Title>
              <div className={styles.actions}>
                <Button type="primary" onClick={handlers.openCreateModal}>
                  {t("primaryButton")}
                </Button>
                <Button onClick={handlers.handleOpenWorkspace}>
                  {t("secondaryButton")}
                </Button>
              </div>
            </div>
            <List
              itemLayout="horizontal"
              dataSource={state.recentWorkspaces}
              renderItem={(item: RecentWorkspace) => (
                <List.Item
                  className={styles.listItem}
                  onClick={() => handlers.handleOpenRecentWorkspace(item.path)}
                >
                  <List.Item.Meta title={item.name} description={item.path} />
                </List.Item>
              )}
            />
          </Card>
        ) : (
          <Result
            className={(
              styles.welcomeContainer +
              " " +
              (className || "")
            ).trim()}
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
