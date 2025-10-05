import { App, ConfigProvider } from "antd";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { Header } from "./components/header";
import { WorkspaceProvider } from "./components/workspace-provider";
import "./index.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <WorkspaceProvider>
        <App>
          <Header />
        </App>
      </WorkspaceProvider>
    </ConfigProvider>
  </StrictMode>
);
