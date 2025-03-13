import { Button, Form, Input, message, Modal } from "antd";
import { ReactNode } from "react";
import "./styles.css";

interface Props {
  children?: ReactNode;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

function WorkspaceDialog(props: Props) {
  const { isModalOpen, setIsModalOpen } = props;
  const [form] = Form.useForm();

  const closeDialog = () => {
    setIsModalOpen(false);
    form.setFieldsValue({ location: "", name: "" });
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
      const workspacePath = `${values.location}/${values.name}_ws`; //TODO add a rule to agnostic OS

      const result = await window.electronAPI.createWorkspace(workspacePath);
      message.success("Workspace criado e build concluído!");
      closeDialog();
      console.log(result);
    } catch (error) {
      message.error("Erro ao criar workspace!");
      console.log(error);
    }
  };

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
            onClick={handleCreateWorkspace}
          >
            Criar
          </Button>,
          <Button key="cancel" type="default" onClick={closeDialog}>
            Cancelar
          </Button>,
        ]}
      >
        <Form layout="vertical" form={form} id="workspace">
          <Form.Item
            label="Localização da pasta"
            name="location"
            required
            rules={[{ message: "Escolha o local da pasta" }]}
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
            required
            rules={[{ message: "Escolha o nome da pasta" }]}
          >
            <Input
              required
              addonAfter="_ws"
              placeholder="Defina o nome do Workspace"
            />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default WorkspaceDialog;
