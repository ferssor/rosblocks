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
  const closeDialog = () => {
    setIsModalOpen(false);
  };

  const layout = {
    labelCol: { span: 8 },
    wrapperCol: { span: 16 },
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
        <Form {...layout}>
          <Form.Item
            label="Localização da pasta"
            name="location"
            required
            rules={[{ message: "Escolha o local da pasta" }]}
          >
            <Input title="Nome" />
          </Form.Item>

          <Form.Item
            label="Nome do workspace"
            name="name"
            required
            rules={[{ message: "Escolha o nome da pasta" }]}
          >
            <Input title="Nome" addonAfter="_ws" />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default WorkspaceDialog;
