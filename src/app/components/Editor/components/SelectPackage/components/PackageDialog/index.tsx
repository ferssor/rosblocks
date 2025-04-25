import { Button, Form, Input, message, Modal, Radio, Result } from "antd";
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
  const [createForm] = Form.useForm();
  const [importForm] = Form.useForm();
  const { packageLocation, isModalOpen, setIsModalOpen, setPackages } = props;
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [showCreateForm, setShowCreateForm] = useState(false);
  const [showImportForm, setShowImportForm] = useState(false);
  const [packageDependency, setPackageDependency] = useState("rclpy");
  const [initialValues, setInitialValues] = useState({
    location: packageLocation,
    name: undefined,
    type: packageDependency,
    dependency: packageDependency,
  });

  const ADD_PKG_TITLE = "Importar ou adicionar um novo pacote";
  const ADD_PKG_SUBTITLE =
    "Você pode criar um novo pacote ou importar um pacote ROS2 via URL do GitHub";
  const IMPORT_BUTTON_TITLE = "Importar";
  const CREATE_BUTTON_TITLE = "Criar";

  const closeDialog = () => {
    setIsModalOpen(false);
    createForm.resetFields();
    createForm.setFieldsValue({
      location: packageLocation,
      name: undefined,
      type: "rclpy",
      dependency: "rclpy",
    });
    setPackageDependency("rclpy");
    setIsInputChanged(false);
    setShowCreateForm(false);
    setShowImportForm(false);
  };

  const fetchPackages = async () => {
    const result = await window.electronAPI.getPackages(packageLocation);
    setPackages(result);
  };

  const handleCreatePackage = async () => {
    try {
      await createForm.validateFields();

      const packageName: string = createForm.getFieldValue("name");
      const packageDeps: string = createForm.getFieldValue("dependency");
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
    const currentValues = createForm.getFieldsValue();
    const nameValue = String(createForm.getFieldValue("name"));
    const isChanged =
      JSON.stringify(currentValues) !== JSON.stringify(initialValues);
    setIsInputChanged(isChanged && nameValue.length > 0);
  };

  const handleFieldsChangeOnImport = () => {
    const urlValue = String(importForm.getFieldValue("url"));
    setIsInputChanged(urlValue.length > 0 ? true : false);
  };

  const handleImportPackage = () => {};

  useEffect(() => {
    if (isModalOpen) {
      const currentValues = createForm.getFieldsValue();
      setInitialValues(currentValues);
    }

    createForm.setFieldsValue({
      dependency: packageDependency,
    });
  }, [createForm, isModalOpen, packageDependency, packageLocation]);

  return (
    <>
      <Modal
        title="Adicionar um novo package"
        open={isModalOpen}
        centered
        maskClosable={false}
        onCancel={closeDialog}
        footer={
          showCreateForm || showImportForm
            ? [
                <Button
                  key="save"
                  type="primary"
                  htmlType="submit"
                  form={showCreateForm ? "packageCreate" : "packageImport"}
                  disabled={!isInputChanged}
                  onClick={
                    showCreateForm ? handleCreatePackage : handleImportPackage
                  }
                >
                  {showCreateForm ? "Criar" : "Importar"}
                </Button>,
                <Button key="cancel" type="default" onClick={closeDialog}>
                  Cancelar
                </Button>,
              ]
            : null
        }
      >
        {!showCreateForm && !showImportForm ? (
          <Result
            status="info"
            title={ADD_PKG_TITLE}
            subTitle={ADD_PKG_SUBTITLE}
            extra={[
              <>
                <Button
                  type="primary"
                  onClick={() => {
                    setShowImportForm(!showImportForm);
                  }}
                >
                  {IMPORT_BUTTON_TITLE}
                </Button>
                <Button
                  type="primary"
                  color="green"
                  variant="solid"
                  onClick={() => setShowCreateForm(!showCreateForm)}
                >
                  {CREATE_BUTTON_TITLE}
                </Button>
              </>,
            ]}
          />
        ) : null}
        <Form
          layout="vertical"
          form={createForm}
          id="packageCreate"
          onFieldsChange={handleFieldsChange}
          initialValues={initialValues}
          hidden={!showCreateForm}
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
        <Form
          layout="vertical"
          form={importForm}
          id="packageImport"
          onFieldsChange={handleFieldsChangeOnImport}
          hidden={!showImportForm}
        >
          <Form.Item
            label="URL do pacote"
            name="url"
            rules={[
              {
                required: true,
                message: "É obrigatório preencher a URL do pacote!",
              },
            ]}
          >
            <Input placeholder="https://github/user/repository-name" />
          </Form.Item>
        </Form>
      </Modal>
    </>
  );
}

export default PackageDialog;
