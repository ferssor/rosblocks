import { Button, Form, Input, Modal } from "antd";
import { memo } from "react";

import useWorkspaceDialogHook from "./workspace-dialog.hook";
import styles from "./workspace-dialog.styles.module.css";

import type { WorkspaceDialogProps } from "./types";

function WorkspaceDialog(props: WorkspaceDialogProps) {
  const { className = "" } = props;
  const { text, state, handlers, form } = useWorkspaceDialogHook(props);

  return (
    <Modal
      className={(styles.container + " " + (className || "")).trim()}
      title={text.title}
      open={props.isModalOpen}
      centered
      maskClosable={false}
      onCancel={handlers.closeDialog}
      footer={[
        <Button
          key="save"
          type="primary"
          htmlType="submit"
          form="workspace"
          disabled={!state.isInputChanged}
          onClick={handlers.handleCreateWorkspace}
        >
          {text.primaryButtonTitle}
        </Button>,
        <Button key="cancel" type="default" onClick={handlers.closeDialog}>
          {text.secondaryButtonTitle}
        </Button>,
      ]}
    >
      <Form
        layout="vertical"
        form={form}
        id="workspace"
        onFieldsChange={handlers.handleFieldsChange}
      >
        <Form.Item
          label={text.locationLabel}
          name="location"
          rules={[{ required: true, message: text.locationRequiredMessage }]}
        >
          <Input
            readOnly
            onClick={handlers.handleOpenDialog}
            placeholder={text.locationPlaceholder}
          />
        </Form.Item>
        <Form.Item
          label={text.nameLabel}
          name="name"
          rules={[{ required: true, message: text.nameRequiredMessage }]}
        >
          <Input addonAfter="_ws" placeholder={text.namePlaceholder} />
        </Form.Item>
      </Form>
    </Modal>
  );
}

export default memo(WorkspaceDialog);
