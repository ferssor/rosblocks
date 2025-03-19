import { Button, Form, Input, Modal, Radio } from "antd";
import "./styles.css";
import { useEffect, useState } from "react";
import { CheckboxGroupProps } from "antd/es/checkbox";

interface Props {
  packageLocation: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}
const options: CheckboxGroupProps<string>["options"] = [
  { label: "Python", value: "rclpy" },
  { label: "C++", value: "rclcpp" },
];

function PackageDialog(props: Props) {
  const [form] = Form.useForm();
  const { packageLocation, isModalOpen, setIsModalOpen } = props;
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
    form.setFieldsValue({ location: undefined, name: undefined });
    form.resetFields();
    setIsInputChanged(false);
  };

  const handleCreatePackage = () => {};

  const handleFieldsChange = () => {};

  useEffect(() => {
    form.setFieldsValue({
      dependency: packageDependency,
    });

    if (isModalOpen) {
      const currentValues = form.getFieldsValue();
      setInitialValues(currentValues);
    }
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
