import { Form, message } from "antd";
import path from "path-browserify";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import "./i18n";

import type { WorkspaceDialogProps } from "./types";

function useWorkspaceDialogHook(props: WorkspaceDialogProps) {
  const {
    setIsModalOpen,
    setWorkspaceLocationFromCreationDialog,
    setValidWorkspace,
  } = props;
  const [form] = Form.useForm();
  const [isInputChanged, setIsInputChanged] = useState(false);
  const { t } = useTranslation("component_workspace_dialog");
  const [initialValues, setInitialValues] = useState({
    location: undefined,
    name: undefined,
  });

  const closeDialog = useCallback(() => {
    setIsModalOpen(false);
    form.resetFields();
    form.setFieldsValue({
      location: undefined,
      name: undefined,
    });
    setIsInputChanged(false);
  }, [form, setIsModalOpen]);

  const handleOpenDialog = useCallback(async () => {
    const selectedPath = await window.electronAPI.openWorkspaceLocation();
    if (selectedPath.workspaceLocation) {
      form.setFieldsValue({ location: selectedPath.workspaceLocation });
    }
  }, [form]);

  const handleCreateWorkspace = useCallback(async () => {
    try {
      await form.validateFields();

      const values = form.getFieldsValue();
      const workspacePath = path.join(
        values.location,
        `${String(values.name).replace(" ", "_")}_ws`
      );
      const result = await window.electronAPI.createWorkspace(
        workspacePath.toLowerCase()
      );

      if (result.wasCreated) {
        const validation = await window.electronAPI.validateWorkspace(
          result.workspacePath
        );
        setWorkspaceLocationFromCreationDialog(result.workspacePath);
        setValidWorkspace(validation.valid);

        message.success(t("messages.createSuccess"));
        closeDialog();
      } else if (result.error) {
        message.error(t("messages.genericError", { error: String(result.error) }));
      }
    } catch (error) {
      if (error instanceof Error) {
        message.error(t("messages.createError", { error: error.message }));
      }
    }
  }, [
    closeDialog,
    form,
    setValidWorkspace,
    setWorkspaceLocationFromCreationDialog,
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

  const text = useMemo(
    () => ({
      title: t("modal.title"),
      primaryButtonTitle: t("modal.primaryButton"),
      secondaryButtonTitle: t("modal.secondaryButton"),
      locationLabel: t("form.locationLabel"),
      locationPlaceholder: t("form.locationPlaceholder"),
      locationRequiredMessage: t("form.locationRequired"),
      nameLabel: t("form.nameLabel"),
      namePlaceholder: t("form.namePlaceholder"),
      nameRequiredMessage: t("form.nameRequired"),
    }),
    [t]
  );

  return {
    text,
    state: { isInputChanged },
    handlers: {
      closeDialog,
      handleOpenDialog,
      handleCreateWorkspace,
      handleFieldsChange,
    },
    form,
  };
}

export default useWorkspaceDialogHook;
