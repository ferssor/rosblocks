import { App, ConfigProvider } from "antd";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import "../i18n/config";
import { registerBundles } from "../i18n/register";

import { Header } from "./components/header";
import { WorkspaceProvider } from "./components/workspace-provider";
import "./index.css";
import { WorkspaceManagement } from "./workspace-management";
import wmd from "./workspace-management/components/workspace-dialog/i18n";
import wm from "./workspace-management/i18n";
registerBundles(wm);
registerBundles(wmd);

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <WorkspaceProvider>
        <App>
          <Header />
          <WorkspaceManagement />
        </App>
      </WorkspaceProvider>
    </ConfigProvider>
  </StrictMode>
);
