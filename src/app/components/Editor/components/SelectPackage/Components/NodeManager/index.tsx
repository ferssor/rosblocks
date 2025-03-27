import { useEffect, useState } from "react";
import "./styles.css";
import { Button, Layout, Menu, Result, Tag } from "antd";
import { ApartmentOutlined } from "@ant-design/icons";
import Sider from "antd/es/layout/Sider";
import { Content } from "antd/es/layout/layout";
import { ItemType } from "antd/es/menu/interface";
import NodeDialog from "./components/NodeDialog";

interface Props {
  packageName: string;
  packageLocation: string;
  packageType: string;
}

function NodeManager(props: Props) {
  const { packageLocation, packageName, packageType } = props;
  const [nodes, setNodes] = useState(Array<ROSNode>);
  const [menuItems, setMenuItems] = useState<ItemType[]>([]);
  const [isModalOpen, setIsModalOpen] = useState(false);

  const TITLE = `Parece que o package ${packageName} não possui nós ROS`;
  const SUBTITLE =
    "Nós são programas executáveis que permitem que o robô posso atuar.";
  const PRIMARY_BUTTON_TITLE = "Criar um novo nó";

  useEffect(() => {
    const fetchNodes = async () => {
      const result = await window.electronAPI.getNodes(
        packageLocation,
        packageName
      );
      setNodes(result);
    };

    fetchNodes();
  }, [packageLocation, packageName, setNodes]);

  useEffect(() => {
    const mapNodesToMenuItems = (nodes: ROSNode[]): ItemType[] => {
      return nodes.map((node) => ({
        key: node.fullPath,
        icon: <ApartmentOutlined />,
        label: node.name,
      }));
    };

    setMenuItems(mapNodesToMenuItems(nodes));
  }, [nodes]);

  return (
    <>
      {nodes.length > 0 ? (
        <Layout className="node-management-container">
          <Sider theme="light" className="sider-container">
            <div className="package-list-header">
              <h3>{packageName}</h3>
              <Tag
                color={
                  packageType === "cpp"
                    ? "blue"
                    : packageType === "python"
                      ? "green"
                      : "default"
                }
              >
                {packageType}
              </Tag>
            </div>
            <Menu mode="inline" items={menuItems} />
          </Sider>
          <Content style={{ background: "#fff" }}>Content</Content>
        </Layout>
      ) : (
        <>
          <Result
            className="new-node-container"
            status="404"
            title={TITLE}
            subTitle={SUBTITLE}
            extra={[
              <>
                <Button
                  type="primary"
                  onClick={() => {
                    setIsModalOpen(true);
                  }}
                >
                  {PRIMARY_BUTTON_TITLE}
                </Button>
                <NodeDialog
                  packageLocation={packageLocation}
                  packageName={packageName}
                  isModalOpen={isModalOpen}
                  packageType={packageType}
                  setIsModalOpen={setIsModalOpen}
                  setNodes={setNodes}
                />
              </>,
            ]}
          />
        </>
      )}
    </>
  );
}

export default NodeManager;
