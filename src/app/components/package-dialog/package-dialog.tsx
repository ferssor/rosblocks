import { Button, Form, Input, Modal, Radio } from "antd";
import { memo } from "react";

import usePackageDialogHook, {
  PACKAGE_DEPENDENCY_OPTIONS,
} from "./package-dialog.hook";
import styles from "./package-dialog.styles.module.css";

import type { PackageDialogProps } from "./types";

function PackageDialog(props: PackageDialogProps) {
  const { state, handlers, text } = usePackageDialogHook(props);

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
          form="packageCreate"
          disabled={!state.isInputChanged}
          onClick={handlers.handleCreatePackage}
        >
          {text.createButton}
        </Button>,
        <Button key="cancel" type="default" onClick={handlers.closeDialog}>
          {text.cancelButton}
        </Button>,
      ]}
    >
      <Form
        layout="vertical"
        form={handlers.createForm}
        id="packageCreate"
        onFieldsChange={handlers.handleFieldsChange}
        initialValues={state.initialValues}
      >
        <Form.Item
          label={text.packageLocationLabel}
          name="location"
          rules={[
            {
              required: true,
              message: text.packageLocationRequiredMessage,
            },
          ]}
        >
          <Input readOnly placeholder={text.packageLocationPlaceholder} />
        </Form.Item>
        <Form.Item
          label={text.packageNameLabel}
          name="name"
          rules={[
            {
              required: true,
              message: text.packageNameRequiredMessage,
            },
          ]}
        >
          <Input placeholder={text.packageNamePlaceholder} />
        </Form.Item>
        <Form.Item
          label={text.packageTypeLabel}
          name="type"
          rules={[{ required: true, message: text.packageTypeRequiredMessage }]}
        >
          <Radio.Group
            block
            options={PACKAGE_DEPENDENCY_OPTIONS}
            optionType="button"
            buttonStyle="solid"
            onChange={handlers.handleDependencyChange}
          />
        </Form.Item>
        <Form.Item label={text.packageDependencyLabel} name="dependency">
          <Input readOnly placeholder={text.packageDependencyPlaceholder} />
        </Form.Item>
      </Form>
    </Modal>
  );
}

export default memo(PackageDialog);
