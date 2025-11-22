import { Form, message } from "antd";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { PackageDialogProps } from "./types";
import type { CheckboxGroupProps } from "antd/es/checkbox";

export const PACKAGE_DEPENDENCY_OPTIONS: CheckboxGroupProps<string>["options"] =
  [
    { label: "Python", value: "rclpy" },
    { label: "C++", value: "rclcpp", disabled: true },
  ];

function usePackageDialogHook(props: PackageDialogProps) {
  const { packageLocation, setPackages, setIsModalOpen } = props;
  const [createForm] = Form.useForm();
  const [importForm] = Form.useForm();
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [showCreateForm, setShowCreateForm] = useState(false);
  const [showImportForm, setShowImportForm] = useState(false);
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
    importForm.resetFields();
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
  }, [createForm, importForm, packageLocation, setIsModalOpen]);

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
        message.error(t("messages.createGenericError", { error: error.message }));
      }
    }
  }, [closeDialog, createForm, fetchPackages, packageLocation, t]);

  const handleImportPackage = useCallback(async () => {
    try {
      await importForm.validateFields();
      const packageUrl: string = importForm.getFieldValue("url");
      const result = await window.electronAPI.importPackage(
        packageUrl,
        packageLocation
      );

      if (result.imported) {
        message.success(t("messages.importSuccess"));
        closeDialog();
        fetchPackages();
      } else if (result.error) {
        message.error(
          t("messages.importError", {
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
          t("messages.importGenericError", { error: error.message })
        );
      }
    }
  }, [closeDialog, fetchPackages, importForm, packageLocation, t]);

  const handleFieldsChange = useCallback(() => {
    const currentValues = createForm.getFieldsValue();
    const nameValue = String(createForm.getFieldValue("name") ?? "");
    const isChanged =
      JSON.stringify(currentValues) !== JSON.stringify(initialValues);
    setIsInputChanged(isChanged && nameValue.length > 0);
  }, [createForm, initialValues]);

  const handleFieldsChangeOnImport = useCallback(() => {
    const urlValue = String(importForm.getFieldValue("url") ?? "");
    setIsInputChanged(urlValue.length > 0);
  }, [importForm]);

  const handleDependencyChange = useCallback((event: {
    target: { value: string };
  }) => {
    setPackageDependency(event.target.value);
  }, []);

  const toggleCreateForm = useCallback(() => {
    setShowCreateForm((prev) => !prev);
    setShowImportForm(false);
    setIsInputChanged(false);
  }, []);

  const toggleImportForm = useCallback(() => {
    setShowImportForm((prev) => !prev);
    setShowCreateForm(false);
    setIsInputChanged(false);
  }, []);

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
      subtitle: t("modal.subtitle"),
      description: t("modal.description"),
      importButton: t("modal.importButton"),
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
      packageUrlLabel: t("form.urlLabel"),
      packageUrlPlaceholder: t("form.urlPlaceholder"),
      packageUrlRequiredMessage: t("form.urlRequired"),
    }),
    [t]
  );

  return {
    text,
    state: {
      isInputChanged,
      showCreateForm,
      showImportForm,
      initialValues,
    },
    handlers: {
      createForm,
      importForm,
      closeDialog,
      handleCreatePackage,
      handleImportPackage,
      handleFieldsChange,
      handleFieldsChangeOnImport,
      handleDependencyChange,
      toggleCreateForm,
      toggleImportForm,
      setModalOpen: setIsModalOpen,
    },
  };
}

export default usePackageDialogHook;
