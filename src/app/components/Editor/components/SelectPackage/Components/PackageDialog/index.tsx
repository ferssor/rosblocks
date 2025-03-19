import { Button, Form, Input, Modal } from "antd";
import "./styles.css";
import { useState } from "react";

interface Props {
  packageLocation: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

function PackageDialog(props: Props) {
  const [form] = Form.useForm();
  const { packageLocation, isModalOpen, setIsModalOpen } = props;
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

  const handleCreatePackage = () => {};

  const handleFieldsChange = () => {};

  return (
    <>
      <Modal
        title="Criar um novo package"
        open={isModalOpen}
        centered
        maskClosable={false}
        onCancel={closeDialog}
        footer={[
          <Button
            key="save"
            type="primary"
            htmlType="submit"
            form="package"
            disabled={!isInputChanged}
            onClick={handleCreatePackage}
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
          id="package"
          onFieldsChange={handleFieldsChange}
        >
          <Form.Item
            label="Localização da pasta"
            name="location"
            rules={[{ required: true, message: "Escolha o local da pasta" }]}
          >
            <Input readOnly placeholder="Escolha o local do Workspace" />
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

export default PackageDialog;
