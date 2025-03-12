import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { App, ConfigProvider } from "antd";
import EditorHeader from "./components/EditorHeader";
import Editor from "./components/Editor";
import "@ant-design/v5-patch-for-react-19";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <App>
        <EditorHeader />
        <Editor />
      </App>
    </ConfigProvider>
  </StrictMode>
);
