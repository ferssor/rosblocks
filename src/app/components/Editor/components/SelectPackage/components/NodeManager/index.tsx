import { useEffect, useState } from "react";
import "./styles.css";
import { Button, Empty, Layout, Menu, Result, Select } from "antd";
import { ApartmentOutlined } from "@ant-design/icons";
import Sider from "antd/es/layout/Sider";
import { Content } from "antd/es/layout/layout";
import { ItemType } from "antd/es/menu/interface";
import NodeDialog from "./components/NodeDialog";
import NodeEditor from "./components/NodeEditor";
import PackageDialog from "../PackageDialog";

interface Props {
  packageName: string;
  packageLocation: string;
  packageType: string;
  selectedWorkspaceLocation: string;
  setPackageName: React.Dispatch<React.SetStateAction<string>>;
  setPackageLocation: React.Dispatch<React.SetStateAction<string>>;
}

function NodeManager(props: Props) {
  const {
    packageLocation,
    packageName,
    packageType,
    selectedWorkspaceLocation,
    setPackageName,
    setPackageLocation,
  } = props;
  const [nodes, setNodes] = useState(Array<ROSNode>);
  const [menuItems, setMenuItems] = useState<ItemType[]>([]);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isPackageModalOpen, setIsPackageModalOpen] = useState(false);
  const [selectedNode, setSelectedNode] = useState<ROSNode>();
  const [packages, setPackages] = useState(Array<Package>);

  const TITLE = `Parece que o package ${packageName} não possui nós ROS`;
  const SUBTITLE =
    "Nós são programas executáveis que permitem que o robô posso atuar.";
  const PRIMARY_BUTTON_TITLE = "Criar um novo nó";

  const fetchPackages = async () => {
    if (packageLocation) {
      const result = await window.electronAPI.getPackages(
        selectedWorkspaceLocation
      );
      setPackages(result);
    }
  };

  const fetchNodes = async (pkgLocation: string, pkgName: string) => {
    const result = await window.electronAPI.getNodes(pkgLocation, pkgName);
    setNodes(result);
  };

  useEffect(() => {
    fetchNodes(packageLocation, packageName);
  }, [packageLocation, packageName]);

  useEffect(() => {
    const mapNodesToMenuItems = (nodes: ROSNode[]): ItemType[] => {
      return nodes.map((node) => ({
        key: node.fullPath,
        icon: <ApartmentOutlined />,
        label: node.name,
        onClick: () => {
          setSelectedNode(node);
        },
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
              <Select
                variant="underlined"
                labelRender={() => <h3>{packageName}</h3>}
                value={packageName}
                options={packages.map((pkg) => {
                  return {
                    value: pkg.fullPath,
                    label: pkg.name,
                  };
                })}
                onDropdownVisibleChange={fetchPackages}
                onSelect={async (_, label) => {
                  const result = await window.electronAPI.getNodes(
                    label.value,
                    label.label
                  );
                  setNodes(result);
                  setPackageName(label.label);
                  setSelectedNode(undefined);
                  setPackageLocation(label.value);
                }}
              />
            </div>
            <div className="action-buttons-header">
              <Button
                type="default"
                color="primary"
                variant="outlined"
                onClick={() => setIsModalOpen(true)}
              >
                Criar novo nó
              </Button>

              <Button
                type="default"
                color="primary"
                variant="outlined"
                onClick={() => setIsPackageModalOpen(true)}
              >
                Criar novo pacote
              </Button>
            </div>
            <Menu mode="inline" items={menuItems} />
          </Sider>
          <Content
            className={selectedNode ? "content-container" : "empty-container"}
          >
            {selectedNode ? (
              <NodeEditor
                pkgLocation={packageLocation}
                pkgName={packageName}
                selectedNode={selectedNode}
                fetchNodes={fetchNodes}
                setSelectedNode={setSelectedNode}
              />
            ) : (
              <Empty
                className="empty-message"
                description="Nenhum nó foi selecionado, para abrir o editor é necessário selecionar um!"
              />
            )}
          </Content>
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
              </>,
            ]}
          />
        </>
      )}
      <NodeDialog
        packageLocation={packageLocation}
        packageName={packageName}
        isModalOpen={isModalOpen}
        packageType={packageType}
        setIsModalOpen={setIsModalOpen}
        setNodes={setNodes}
      />
      <PackageDialog
        isModalOpen={isPackageModalOpen}
        setIsModalOpen={setIsPackageModalOpen}
        packageLocation={selectedWorkspaceLocation}
        setPackages={setPackages}
      />
    </>
  );
}

export default NodeManager;
