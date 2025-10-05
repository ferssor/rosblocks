import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { App, ConfigProvider } from "antd";
import { Header } from "./components/header";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <App>
        <Header />
      </App>
    </ConfigProvider>
  </StrictMode>
);
