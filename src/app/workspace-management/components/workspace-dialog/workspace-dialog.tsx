import { Button, Form, Input, Modal } from "antd";
import { memo } from "react";

import useWorkspaceDialogHook from "./workspace-dialog.hook";
import styles from "./workspace-dialog.styles.module.css";

import type { WorkspaceDialogProps } from "./types";

function WorkspaceDialog(props: WorkspaceDialogProps) {
  const { t, form, state, handlers, rules } = useWorkspaceDialogHook(props);

  return (
    <Modal
      className={styles.dialog}
      title={t("title")}
      open={state.isModalOpen}
      centered
      maskClosable={false}
      onCancel={handlers.close}
      footer={[
        <Button
          key="save"
          type="primary"
          htmlType="submit"
          form="workspace-form"
          disabled={!state.isInputChanged}
          onClick={handlers.handleCreate}
        >
          {t("actions.create")}
        </Button>,
        <Button key="cancel" type="default" onClick={handlers.close}>
          {t("actions.cancel")}
        </Button>,
      ]}
      destroyOnClose
    >
      <Form
        layout="vertical"
        form={form}
        id="workspace-form"
        onFieldsChange={handlers.handleFieldsChange}
      >
        <Form.Item
          label={t("labels.location")}
          name="location"
          rules={rules.location}
        >
          <Input
            readOnly
            onClick={handlers.handleOpenDialog}
            placeholder={t("placeholders.location")}
          />
        </Form.Item>

        <Form.Item label={t("labels.name")} name="name" rules={rules.name}>
          <Input addonAfter="_ws" placeholder={t("placeholders.name")} />
        </Form.Item>
      </Form>
    </Modal>
  );
}

export default memo(WorkspaceDialog);
