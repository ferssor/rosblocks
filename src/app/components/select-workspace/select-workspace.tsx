import { Button, Result } from "antd";
import { memo } from "react";

import { WorkspaceDialog } from "../workspace-dialog";

import useSelectWorkspaceHook from "./select-workspace.hook";
import styles from "./select-workspace.styles.module.css";

import type { SelectWorkspaceProps } from "./types";

function SelectWorkspace(props: SelectWorkspaceProps) {
  const { className = "", style } = props;
  const { text, state, handlers } = useSelectWorkspaceHook(props);

  return (
    <Result
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
      status="info"
      title={text.title}
      subTitle={text.subtitle}
      extra={[
        <div key="actions" className={styles.actions}>
          <Button type="primary" onClick={handlers.openCreateModal}>
            {text.primaryButtonTitle}
          </Button>
          <Button onClick={handlers.handleOpenWorkspace}>
            {text.secondaryButtonTitle}
          </Button>
          <WorkspaceDialog
            isModalOpen={state.isModalOpen}
            setIsModalOpen={handlers.setModalOpen}
            setWorkspaceLocationFromCreationDialog={
              handlers.setWorkspaceLocation
            }
            setValidWorkspace={handlers.setValidWorkspace}
          />
        </div>,
      ]}
    />
  );
}

export default memo(SelectWorkspace);
