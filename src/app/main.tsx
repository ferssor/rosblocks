import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { App, ConfigProvider } from "antd";
import EditorHeader from "./components/EditorHeader";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <App>
        <EditorHeader />
      </App>
    </ConfigProvider>
  </StrictMode>
);
