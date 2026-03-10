import { Form, message } from "antd";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { PackageDialogProps } from "./types";
import type { RadioChangeEvent } from "antd";
import type { CheckboxGroupProps } from "antd/es/checkbox";

export const PACKAGE_DEPENDENCY_OPTIONS: CheckboxGroupProps<string>["options"] =
  [
    { label: "Python", value: "rclpy" },
    { label: "C++", value: "rclcpp", disabled: true },
  ];

function usePackageDialogHook(props: PackageDialogProps) {
  const { packageLocation, setPackages, setIsModalOpen } = props;
  const [createForm] = Form.useForm();
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [packageDependency, setPackageDependency] = useState("rclpy");
  const [initialValues, setInitialValues] = useState({
    location: packageLocation,
    name: undefined,
    type: "rclpy",
    dependency: "rclpy",
  });
  const { t } = useTranslation("component_package_dialog");

  const closeDialog = useCallback(() => {
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
  }, [createForm, packageLocation, setIsModalOpen]);

  const fetchPackages = useCallback(async () => {
    const result = await window.electronAPI.getPackages(packageLocation);
    setPackages(result);
  }, [packageLocation, setPackages]);

  const handleCreatePackage = useCallback(async () => {
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
        message.success(t("messages.createSuccess"));
        closeDialog();
        fetchPackages();
      } else if (result.error) {
        message.error(
          t("messages.createError", {
            error:
              typeof result.error === "string"
                ? result.error
                : JSON.stringify(result.error),
          })
        );
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(
          t("messages.createGenericError", { error: error.message })
        );
      }
    }
  }, [closeDialog, createForm, fetchPackages, packageLocation, t]);

  const handleFieldsChange = useCallback(() => {
    const currentValues = createForm.getFieldsValue();
    const nameValue = String(createForm.getFieldValue("name") ?? "");
    const isChanged =
      JSON.stringify(currentValues) !== JSON.stringify(initialValues);
    setIsInputChanged(isChanged && nameValue.length > 0);
  }, [createForm, initialValues]);

  const handleDependencyChange = useCallback(
    (event: RadioChangeEvent) => {
      if (event.target?.value) {
        setPackageDependency(String(event.target.value));
      }
    },
    []
  );

  useEffect(() => {
    if (props.isModalOpen) {
      const currentValues = createForm.getFieldsValue();
      setInitialValues(currentValues);
    }
  }, [createForm, props.isModalOpen]);

  useEffect(() => {
    createForm.setFieldsValue({
      dependency: packageDependency,
      location: packageLocation,
    });
  }, [createForm, packageDependency, packageLocation]);

  const text = useMemo(
    () => ({
      title: t("modal.title"),
      createButton: t("modal.createButton"),
      cancelButton: t("modal.cancelButton"),
      packageLocationLabel: t("form.locationLabel"),
      packageLocationPlaceholder: t("form.locationPlaceholder"),
      packageLocationRequiredMessage: t("form.locationRequired"),
      packageNameLabel: t("form.nameLabel"),
      packageNamePlaceholder: t("form.namePlaceholder"),
      packageNameRequiredMessage: t("form.nameRequired"),
      packageTypeLabel: t("form.typeLabel"),
      packageTypeRequiredMessage: t("form.typeRequired"),
      packageDependencyLabel: t("form.dependencyLabel"),
      packageDependencyPlaceholder: t("form.dependencyPlaceholder"),
    }),
    [t]
  );

  return {
    text,
    state: {
      isInputChanged,
      initialValues,
    },
    handlers: {
      createForm,
      closeDialog,
      handleCreatePackage,
      handleFieldsChange,
      handleDependencyChange,
      setModalOpen: setIsModalOpen,
    },
  };
}

export default usePackageDialogHook;

