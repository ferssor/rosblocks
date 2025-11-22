import { App, ConfigProvider } from "antd";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import "../i18n/config";
import { registerBundles } from "../i18n/register";

import { Header } from "./components/header";
import hd from "./components/header/i18n";
import nd from "./components/node-manager/i18n";
import pd from "./components/package-dialog/i18n";
import pl from "./components/package-list/i18n";
import sp from "./components/select-package/i18n";
import sw from "./components/select-workspace/i18n";
import wd from "./components/workspace-dialog/i18n";
import { WorkspaceProvider } from "./components/workspace-provider";
import { useWorkspace } from "./components/workspace-provider/workspace-provider";
import "./index.css";
import nm from "./node-management/i18n";
import { PackageManagement } from "./package-management";
import pm from "./package-management/i18n";
import { WorkspaceManagement } from "./workspace-management";
import wmd from "./workspace-management/components/workspace-dialog/i18n";
import wm from "./workspace-management/i18n";
registerBundles(wm);
registerBundles(wmd);
registerBundles(pm);
registerBundles(pl);
registerBundles(sp);
registerBundles(pd);
registerBundles(nd);
registerBundles(sw);
registerBundles(wd);
registerBundles(hd);
registerBundles(nm);

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
