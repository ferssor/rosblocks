import { Button, Form, Input, message, Modal } from "antd";
import { ReactNode, useEffect, useState } from "react";
import "./styles.css";
import path from "path-browserify";

interface Props {
  children?: ReactNode;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

function WorkspaceDialog(props: Props) {
  const { isModalOpen, setIsModalOpen } = props;
  const [form] = Form.useForm();
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [initialValues, setInitialValues] = useState({
    location: undefined,
    name: undefined,
  });

  const closeDialog = () => {
    setIsModalOpen(false);
    form.setFieldsValue({ location: undefined, name: undefined });
    form.resetFields();
    setIsInputChanged(false);
  };

  const handleOpenDialog = async () => {
    const selectedPath = await window.electronAPI.openWorkspaceLocation();
    if (selectedPath) {
      form.setFieldsValue({ location: selectedPath });
    }
  };

  const handleCreateWorkspace = async () => {
    try {
      await form.validateFields();

      const values = form.getFieldsValue();
      const workspacePath = path.join(values.location, `${values.name}_ws`);
      const result = await window.electronAPI.createWorkspace(workspacePath);

      if (result.created) {
        message.success("Workspace criado e build concluído!");
        closeDialog();
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(`Ocorreu um erro ao criar o workspace! ${error.message}`);
      }
    }
  };

  const handleFieldsChange = () => {
    const currentValues = form.getFieldsValue();
    const nameValue = String(form.getFieldValue("name"));
    const isChanged =
      JSON.stringify(currentValues) !== JSON.stringify(initialValues);
    setIsInputChanged(isChanged && nameValue.length > 0);
  };

  useEffect(() => {
    if (isModalOpen) {
      const currentValues = form.getFieldsValue();
      setInitialValues(currentValues);
    }
  }, [isModalOpen, form]);

  return (
    <>
      <Modal
        title="Criar um novo workspace"
        open={isModalOpen}
        centered
        maskClosable={false}
        onCancel={closeDialog}
        footer={[
          <Button
            key="save"
            type="primary"
            htmlType="submit"
            form="workspace"
            disabled={!isInputChanged}
            onClick={handleCreateWorkspace}
          >
            Criar
          </Button>,
          <Button key="cancel" type="default" onClick={closeDialog}>
            Cancelar
          </Button>,
        ]}
      >
        <Form
          layout="vertical"
          form={form}
          id="workspace"
          onFieldsChange={handleFieldsChange}
        >
          <Form.Item
            label="Localização da pasta"
            name="location"
            rules={[{ required: true, message: "Escolha o local da pasta" }]}
          >
            <Input
              readOnly
              onClick={handleOpenDialog}
              placeholder="Escolha o local do Workspace"
            />
          </Form.Item>
          <Form.Item
            label="Nome do workspace"
            name="name"
            rules={[{ required: true, message: "Escolha o nome da pasta" }]}
          >
            <Input addonAfter="_ws" placeholder="Defina o nome do Workspace" />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default WorkspaceDialog;
