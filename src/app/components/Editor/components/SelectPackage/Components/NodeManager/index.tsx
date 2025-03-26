import { useEffect, useState } from "react";
import "./styles.css";
import { Layout, Menu } from "antd";
import { ApartmentOutlined } from "@ant-design/icons";
import Sider from "antd/es/layout/Sider";
import { Content } from "antd/es/layout/layout";
import { ItemType } from "antd/es/menu/interface";

interface Props {
  packageName: string;
  packageLocation: string;
}

function NodeManager(props: Props) {
  const { packageLocation, packageName } = props;
  const [nodes, setNodes] = useState(Array<ROSNode>);
  const [menuItems, setMenuItems] = useState<ItemType[]>([]);

  useEffect(() => {
    const fetchPackages = async () => {
      const result = await window.electronAPI.getNodes(
        packageLocation,
        packageName
      );
      setNodes(result);
    };

    fetchPackages();
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
      <Layout className="node-management-container">
        <Sider theme="light" className="sider-container">
          <div className="package-list-header">
            <h3>{packageName}</h3>
          </div>
          <Menu mode="inline" items={menuItems} />
        </Sider>
        <Content style={{ background: "#fff" }}>Content</Content>
      </Layout>
    </>
  );
}

export default NodeManager;
