import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { App, ConfigProvider, Layout } from "antd";
import Logo from "./components/Logo";

const { Header, Sider, Content } = Layout;
createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ConfigProvider>
      <App>
        <Header>
          <Logo />
        </Header>
        <Layout>
          <Sider />
          <Content />
        </Layout>
      </App>
    </ConfigProvider>
  </StrictMode>
);
