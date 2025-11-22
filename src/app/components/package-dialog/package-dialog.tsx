import { Button, Form, Input, Modal, Radio, Result } from "antd";
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
      footer={
        state.showCreateForm || state.showImportForm
          ? [
              <Button
                key="save"
                type="primary"
                htmlType="submit"
                form={state.showCreateForm ? "packageCreate" : "packageImport"}
                disabled={!state.isInputChanged}
                onClick={
                  state.showCreateForm
                    ? handlers.handleCreatePackage
                    : handlers.handleImportPackage
                }
              >
                {state.showCreateForm ? text.createButton : text.importButton}
              </Button>,
              <Button key="cancel" type="default" onClick={handlers.closeDialog}>
                {text.cancelButton}
              </Button>,
            ]
          : null
      }
    >
      {!state.showCreateForm && !state.showImportForm ? (
        <Result
          status="info"
          title={text.subtitle}
          subTitle={text.description}
          extra={[
            <div key="actions" className={styles.actions}>
              <Button type="primary" onClick={handlers.toggleImportForm}>
                {text.importButton}
              </Button>
              <Button
                type="primary"
                color="green"
                variant="solid"
                onClick={handlers.toggleCreateForm}
              >
                {text.createButton}
              </Button>
            </div>,
          ]}
        />
      ) : null}
      <Form
        layout="vertical"
        form={handlers.createForm}
        id="packageCreate"
        onFieldsChange={handlers.handleFieldsChange}
        initialValues={state.initialValues}
        hidden={!state.showCreateForm}
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
      <Form
        layout="vertical"
        form={handlers.importForm}
        id="packageImport"
        onFieldsChange={handlers.handleFieldsChangeOnImport}
        hidden={!state.showImportForm}
      >
        <Form.Item
          label={text.packageUrlLabel}
          name="url"
          rules={[
            {
              required: true,
              message: text.packageUrlRequiredMessage,
            },
          ]}
        >
          <Input placeholder={text.packageUrlPlaceholder} />
        </Form.Item>
      </Form>
    </Modal>
  );
}

export default memo(PackageDialog);
