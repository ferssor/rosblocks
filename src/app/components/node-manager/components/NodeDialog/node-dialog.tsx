import { Button, Form, Input, Modal } from "antd";
import { memo } from "react";

import useNodeDialogHook from "./node-dialog.hook";
import styles from "./node-dialog.styles.module.css";

import type { NodeDialogProps } from "./types";

function NodeDialog(props: NodeDialogProps) {
  const { text, state, handlers, form } = useNodeDialogHook(props);

  return (
    <Modal
      className={styles.container}
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
          form="node"
          disabled={!state.isInputChanged}
          onClick={handlers.handleCreateNode}
        >
          {text.primaryButton}
        </Button>,
        <Button key="cancel" type="default" onClick={handlers.closeDialog}>
          {text.secondaryButton}
        </Button>,
      ]}
    >
      <Form
        layout="vertical"
        form={form}
        id="node"
        onFieldsChange={handlers.handleFieldsChange}
        initialValues={state.initialValues}
      >
        <Form.Item
          label={text.nodeNameLabel}
          name="name"
          rules={[{ required: true, message: text.nodeNameRequired }]}
        >
          <Input placeholder={text.nodeNamePlaceholder} />
        </Form.Item>
        <Form.Item
          label={text.nodeLocationLabel}
          name="location"
          rules={[{ required: true, message: text.nodeLocationRequired }]}
        >
          <Input readOnly placeholder={text.nodeLocationPlaceholder} />
        </Form.Item>
        <Form.Item
          label={text.nodeTypeLabel}
          name="nodeType"
          rules={[{ required: true, message: text.nodeTypeRequired }]}
        >
          <Input readOnly placeholder={text.nodeTypePlaceholder} />
        </Form.Item>
      </Form>
    </Modal>
  );
}

export default memo(NodeDialog);
