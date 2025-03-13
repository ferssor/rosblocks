import { Form, Input, Modal } from "antd";
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
    form.setFieldsValue({ location: "" });
  };

  const handleOpenDialog = async () => {
    const selectedPath = await window.electronAPI.openWorkspaceLocation();
    if (selectedPath) {
      form.setFieldsValue({ location: selectedPath });
    }
  };

  return (
    <>
      <Modal
        title="Criar um novo workspace"
        open={isModalOpen}
        cancelText="Cancelar"
        okText="Criar"
        onClose={() => closeDialog()}
        onCancel={() => closeDialog()}
        maskClosable={false}
        centered
      >
        <Form layout="vertical" form={form}>
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
            <Input addonAfter="_ws" placeholder="Defina o nome do Workspace" />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default WorkspaceDialog;
