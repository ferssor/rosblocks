import { Form, message } from "antd";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "../../i18n";

import type { NodeDialogProps } from "./types";

function useNodeDialogHook(props: NodeDialogProps) {
  const { setIsModalOpen, packageLocation, packageName, packageType, setNodes } =
    props;
  const [form] = Form.useForm();
  const [isInputChanged, setIsInputChanged] = useState(false);
  const [initialValues, setInitialValues] = useState({
    name: undefined,
    location: packageLocation,
    nodeType: packageType,
  });
  const { t } = useTranslation("node_manager");

  const closeDialog = useCallback(() => {
    setIsModalOpen(false);
    form.resetFields();
    form.setFieldsValue({
      name: undefined,
      location: packageLocation,
      nodeType: packageType,
    });
    setIsInputChanged(false);
  }, [form, packageLocation, packageType, setIsModalOpen]);

  const fetchNodes = useCallback(async () => {
    const result = await window.electronAPI.getNodes(
      packageLocation,
      packageName
    );
    setNodes(result);
  }, [packageLocation, packageName, setNodes]);

  const handleCreateNode = useCallback(async () => {
    try {
      await form.validateFields();
      const nodeName: string = form.getFieldValue("name");
      const result = await window.electronAPI.createNode(
        nodeName.toLowerCase(),
        packageType,
        packageLocation,
        packageName
      );

      if (result.created) {
        closeDialog();
        fetchNodes();
      } else if (result.error) {
        message.error(result.error);
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(t("dialog.messages.createError", { error: error.message }));
      }
    }
  }, [
    closeDialog,
    fetchNodes,
    form,
    packageLocation,
    packageName,
    packageType,
    t,
  ]);

  const handleFieldsChange = useCallback(() => {
    const currentValues = form.getFieldsValue();
    const nameValue = String(form.getFieldValue("name") ?? "");
    const isChanged =
      JSON.stringify(currentValues) !== JSON.stringify(initialValues);
    setIsInputChanged(isChanged && nameValue.length > 0);
  }, [form, initialValues]);

  useEffect(() => {
    if (props.isModalOpen) {
      const currentValues = form.getFieldsValue();
      setInitialValues(currentValues);
    }
  }, [form, props.isModalOpen]);

  useEffect(() => {
    form.setFieldsValue({
      location: packageLocation,
      nodeType: packageType,
    });
  }, [form, packageLocation, packageType]);

  const text = useMemo(
    () => ({
      title: t("dialog.title"),
      primaryButton: t("dialog.primaryButton"),
      secondaryButton: t("dialog.secondaryButton"),
      nodeNameLabel: t("dialog.nameLabel"),
      nodeNamePlaceholder: t("dialog.namePlaceholder"),
      nodeNameRequired: t("dialog.nameRequired"),
      nodeLocationLabel: t("dialog.locationLabel"),
      nodeLocationPlaceholder: t("dialog.locationPlaceholder"),
      nodeLocationRequired: t("dialog.locationRequired"),
      nodeTypeLabel: t("dialog.typeLabel"),
      nodeTypePlaceholder: t("dialog.typePlaceholder"),
      nodeTypeRequired: t("dialog.typeRequired"),
    }),
    [t]
  );

  return {
    text,
    state: { isInputChanged, initialValues },
    handlers: {
      closeDialog,
      handleCreateNode,
      handleFieldsChange,
    },
    form,
  };
}

export default useNodeDialogHook;
