import { Button, Form, Input, message, Modal } from "antd";
import { useEffect, useState } from "react";

interface Props {
  packageLocation: string;
  packageName: string;
  packageType: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

function NodeDialog(props: Props) {
  const [form] = Form.useForm();
  const {
    isModalOpen,
    packageLocation,
    packageName,
    packageType,
    setIsModalOpen,
  } = props;
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [initialValues, setInitialValues] = useState({
    name: undefined,
    location: packageLocation,
    nodeType: packageType,
  });

  const closeDialog = () => {
    setIsModalOpen(false);
    form.resetFields();
    form.setFieldsValue({
      name: undefined,
      location: packageLocation,
      nodeType: packageType,
    });
    setIsInputChanged(false);
  };

  const handleCreateNode = async () => {
    try {
      await form.validateFields();
      const nodeName = form.getFieldValue("name");
      const result = await window.electronAPI.createNode(
        nodeName,
        packageType,
        packageLocation,
        packageName
      );
      console.log(result);
      if (result.created) {
        setIsModalOpen(false);
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(`Ocorreu um erro ao criar o nó!`);
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
  }, [form, isModalOpen, packageLocation]);

  return (
    <>
      <Modal
        title="Criar um novo nó"
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
            onClick={handleCreateNode}
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
          initialValues={initialValues}
        >
          <Form.Item
            label="Nome do nó"
            name="name"
            rules={[
              {
                required: true,
                message: "É obrigatório escolher o nome do nó",
              },
            ]}
          >
            <Input placeholder="Escolha o nome do nó" />
          </Form.Item>
          <Form.Item
            label="Localização do nó"
            name="location"
            rules={[
              {
                required: true,
                message: "É obrigatório escolher a localização do nó",
              },
            ]}
          >
            <Input
              readOnly
              placeholder="Escolha a localizaçao do nó"
              value={packageLocation}
            />
          </Form.Item>
          <Form.Item
            label="Tipo do nó"
            name="nodeType"
            rules={[
              {
                required: true,
                message: "É obrigatório escolher o tipo do nó",
              },
            ]}
          >
            <Input
              readOnly
              placeholder="Escolha o tipo do nó"
              value={packageType}
            />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default NodeDialog;
