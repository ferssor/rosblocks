import { message } from "antd";
import { useForm } from "antd/es/form/Form";
import path from "path-browserify";
import { useCallback, useEffect, useMemo, useState } from "react";
import { useTranslation } from "react-i18next";

import { useWorkspace } from "../../../components/workspace-provider/workspace-provider";

import "./i18n";

import type { WorkspaceDialogProps } from "./types";

function sanitizeName(raw?: string) {
  const s = String(raw || "");
  return s.trim().replace(/\s+/g, "_");
}

function useWorkspaceDialogHook(props: WorkspaceDialogProps) {
  const { isModalOpen, setIsModalOpen } = props;
  const { setWorkspacePath, setIsValidWorkspace } = useWorkspace();
  const [form] = useForm();
  const { t } = useTranslation("workspace_dialog");

  const [isInputChanged, setIsInputChanged] = useState(false);
  const [initialValues, setInitialValues] = useState<{
    location?: string;
    name?: string;
  }>({});

  const rules = useMemo(
    () => ({
      location: [{ required: true, message: t("validation.locationRequired") }],
      name: [{ required: true, message: t("validation.nameRequired") }],
    }),
    [t]
  );
  const close = useCallback(() => {
    setIsModalOpen(false);
    form.setFieldsValue({ location: undefined, name: undefined });
    form.resetFields();
    setIsInputChanged(false);
  }, [form, setIsModalOpen]);

  const handleOpenDialog = useCallback(async () => {
    const selected = await window.electronAPI.openWorkspaceLocation();
    if (selected?.workspaceLocation) {
      form.setFieldsValue({ location: selected.workspaceLocation });
    }
  }, [form]);

  const handleFieldsChange = useCallback(() => {
    const current = form.getFieldsValue();
    const changed = JSON.stringify(current) !== JSON.stringify(initialValues);
    const hasName = Boolean(
      String(form.getFieldValue("name") || "").trim().length
    );
    setIsInputChanged(changed && hasName);
  }, [form, initialValues]);

  const handleCreate = useCallback(async () => {
    try {
      await form.validateFields();
      const vals = form.getFieldsValue();
      const folderName = sanitizeName(vals.name) + "_ws";
      const workspacePath = path.join(vals.location, folderName).toLowerCase();

      const result = await window.electronAPI.createWorkspace(workspacePath);
      if (result.wasCanceled) return;

      if (result.wasCreated) {
        const validate = await window.electronAPI.validateWorkspace(
          result.workspacePath
        );
        setWorkspacePath(result.workspacePath);
        setIsValidWorkspace(validate.valid);
        message.success(t("messages.createdAndBuilt"));
        close();
      } else {
        message.error(result.error || t("errors.createFailed"));
      }
    } catch (err: unknown) {
      if (err) return;
      message.error(t("errors.unexpected"));
    }
  }, [form, close, setIsValidWorkspace, setWorkspacePath, t]);

  useEffect(() => {
    if (isModalOpen) {
      const current = form.getFieldsValue();
      setInitialValues(current);
    }
  }, [isModalOpen, form]);

  return {
    t,
    form,
    state: {
      isModalOpen,
      isInputChanged,
    },
    handlers: {
      close,
      handleOpenDialog,
      handleFieldsChange,
      handleCreate,
    },
    rules,
  };
}

export default useWorkspaceDialogHook;
