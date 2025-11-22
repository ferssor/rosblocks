import { ApartmentOutlined } from "@ant-design/icons";
import { Button, Empty, Layout, Menu, Result, Select } from "antd";
import Sider from "antd/es/layout/Sider";
import { memo } from "react";

import { PackageDialog } from "../package-dialog";

import { NodeDialog } from "./components/NodeDialog";
import { NodeEditor } from "./components/NodeEditor";
import useNodeManagerHook from "./node-manager.hook";
import styles from "./node-manager.styles.module.css";

import type { NodeManagerProps } from "./types";

const { Content } = Layout;

function NodeManager(props: NodeManagerProps) {
  const { className = "", style } = props;
  const { state, handlers, text } = useNodeManagerHook(props);

  const menuItems = state.nodes.map((node) => ({
    key: node.fullPath,
    icon: <ApartmentOutlined />,
    label: node.name,
    onClick: () => handlers.setSelectedNode(node),
  }));

  return (
    <>
      {state.nodes.length > 0 ? (
        <Layout
          className={(styles.container + " " + (className || "")).trim()}
          style={style}
        >
          <Sider theme="light" className={styles.sider}>
            <div className={styles.packageHeader}>
              <Select
                variant="underlined"
                labelRender={() => <h3>{props.packageName}</h3>}
                value={props.packageName}
                options={state.packages.map((pkg) => ({
                  value: pkg.fullPath,
                  label: pkg.name,
                }))}
                onDropdownVisibleChange={handlers.fetchPackages}
                onSelect={(_, option) =>
                  handlers.handlePackageSelect(
                    option as { value: string; label: string }
                  )
                }
              />
            </div>
            <div className={styles.actionButtons}>
              <Button
                type="default"
                color="primary"
                variant="outlined"
                onClick={() => handlers.setIsModalOpen(true)}
              >
                {text.createNodeButton}
              </Button>

              <Button
                type="default"
                color="primary"
                variant="outlined"
                onClick={() => handlers.setIsPackageModalOpen(true)}
              >
                {text.createPackageButton}
              </Button>
            </div>
            <Menu mode="inline" items={menuItems} className={styles.menu} />
          </Sider>
          <Content
            className={
              state.selectedNode ? styles.contentContainer : styles.emptyContent
            }
          >
            {state.selectedNode ? (
              <NodeEditor
                pkgLocation={props.packageLocation}
                pkgName={props.packageName}
                selectedNode={state.selectedNode}
                fetchNodes={handlers.fetchNodes}
                setSelectedNode={handlers.setSelectedNode}
              />
            ) : (
              <Empty
                className={styles.emptyMessage}
                description={text.nodeSelectionPlaceholder}
              />
            )}
          </Content>
        </Layout>
      ) : (
        <Result
          className={styles.newNodeContainer}
          status="404"
          title={text.emptyTitle(props.packageName)}
          subTitle={text.emptySubtitle}
          extra={[
            <Button
              key="create"
              type="primary"
              onClick={() => handlers.setIsModalOpen(true)}
            >
              {text.createNodeButton}
            </Button>,
          ]}
        />
      )}

      <NodeDialog
        packageLocation={props.packageLocation}
        packageName={props.packageName}
        packageType={props.packageType}
        isModalOpen={state.isModalOpen}
        setIsModalOpen={handlers.setIsModalOpen}
        setNodes={handlers.setNodes}
      />
      <PackageDialog
        isModalOpen={state.isPackageModalOpen}
        setIsModalOpen={handlers.setIsPackageModalOpen}
        packageLocation={props.selectedWorkspaceLocation}
        setPackages={handlers.setPackages}
      />
    </>
  );
}

export default memo(NodeManager);
