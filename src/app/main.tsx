import { App, ConfigProvider } from "antd";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import "../i18n/config";
import { registerBundles } from "../i18n/register";

import { Header } from "./components/header";
import { WorkspaceProvider } from "./components/workspace-provider";
import { useWorkspace } from "./components/workspace-provider/workspace-provider";
import "./index.css";
import { PackageManagement } from "./package-management";
import pl from "./package-management/components/package-list/i18n";
import pm from "./package-management/i18n";
import { WorkspaceManagement } from "./workspace-management";
import wmd from "./workspace-management/components/workspace-dialog/i18n";
import wm from "./workspace-management/i18n";
registerBundles(wm);
registerBundles(wmd);
registerBundles(pm);
registerBundles(pl);

export function AppContent() {
  const { isValidWorkspace } = useWorkspace();
  return isValidWorkspace ? <PackageManagement /> : <WorkspaceManagement />;
}

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <WorkspaceProvider>
        <App>
          <Header />
          <AppContent />
        </App>
      </WorkspaceProvider>
    </ConfigProvider>
  </StrictMode>
);
