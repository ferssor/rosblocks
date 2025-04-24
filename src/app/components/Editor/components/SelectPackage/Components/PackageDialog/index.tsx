import { Button, Form, Input, message, Modal, Radio } from "antd";
import "./styles.css";
import { useEffect, useState } from "react";
import { CheckboxGroupProps } from "antd/es/checkbox";

interface Props {
  packageLocation: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setPackages: React.Dispatch<React.SetStateAction<Package[]>>;
}
const options: CheckboxGroupProps<string>["options"] = [
  { label: "Python", value: "rclpy" },
  { label: "C++", value: "rclcpp", disabled: true },
];

function PackageDialog(props: Props) {
  const [form] = Form.useForm();
  const { packageLocation, isModalOpen, setIsModalOpen, setPackages } = props;
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [packageDependency, setPackageDependency] = useState("rclpy");
  const [initialValues, setInitialValues] = useState({
    location: packageLocation,
    name: undefined,
    type: packageDependency,
    dependency: packageDependency,
  });

  const closeDialog = () => {
    setIsModalOpen(false);
    form.resetFields();
    form.setFieldsValue({
      location: packageLocation,
      name: undefined,
      type: "rclpy",
      dependency: "rclpy",
    });
    setPackageDependency("rclpy");
    setIsInputChanged(false);
  };

  const fetchPackages = async () => {
    const result = await window.electronAPI.getPackages(packageLocation);
    setPackages(result);
  };

  const handleCreatePackage = async () => {
    try {
      await form.validateFields();

      const packageName: string = form.getFieldValue("name");
      const packageDeps: string = form.getFieldValue("dependency");
      const result = await window.electronAPI.createPackage(
        packageLocation,
        packageName.toLowerCase(),
        packageDeps.toLowerCase()
      );

      if (result.created) {
        message.success("Package criado com sucesso!");
        closeDialog();
        fetchPackages();
      } else if (result.error) {
        const errorMessage =
          typeof result.error === "string"
            ? result.error
            : JSON.stringify(result.error);
        message.error(`Erro ao criar o pacote: ${errorMessage}`);
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(`Ocorreu um erro ao criar o pacote!`);
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

    form.setFieldsValue({
      dependency: packageDependency,
    });
  }, [form, isModalOpen, packageDependency, packageLocation]);

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
          initialValues={initialValues}
        >
          <Form.Item
            label="Localizaçao do pacote"
            name="location"
            rules={[
              {
                required: true,
                message: "É obrigatório escolher a localização do pacote",
              },
            ]}
          >
            <Input
              readOnly
              placeholder="Escolha o nome do pacote"
              value={packageLocation}
            />
          </Form.Item>
          <Form.Item
            label="Nome do pacote"
            name="name"
            rules={[
              {
                required: true,
                message: "É obrigatório escolher o nome do pacote",
              },
            ]}
          >
            <Input placeholder="Escolha o nome do pacote" />
          </Form.Item>
          <Form.Item
            label="Tipo do pacote"
            name="type"
            rules={[{ required: true, message: "Escolha o tipo do pacote" }]}
          >
            <Radio.Group
              block
              options={options}
              optionType="button"
              buttonStyle="solid"
              onChange={(e) => setPackageDependency(e.target.value)}
            />
          </Form.Item>
          <Form.Item label="Dependência do pacote" name="dependency">
            <Input
              readOnly
              placeholder="Escolha a dependência do pacote"
              value={packageDependency}
            />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default PackageDialog;
