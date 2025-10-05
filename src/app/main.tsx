import { App, ConfigProvider } from "antd";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { Header } from "./components/header";
import "./index.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <App>
        <Header />
      </App>
    </ConfigProvider>
  </StrictMode>
);
