import { Button, Table, Tag } from "antd";
import {
  differenceInDays,
  differenceInMonths,
  differenceInYears,
  format,
} from "date-fns";
import { ptBR } from "date-fns/locale";
import { memo } from "react";

import { NodeManager } from "../node-manager";
import { PackageDialog } from "../package-dialog";

import usePackageListHook from "./package-list.hook";
import styles from "./package-list.styles.module.css";

import type { PackageListProps } from "./types";
import type { TableProps } from "antd";

function PackageList(props: PackageListProps) {
  const { className = "", style } = props;
  const { state, handlers, text } = usePackageListHook(props);

  const columns: TableProps<Package>["columns"] = [
    {
      title: text.packageName,
      dataIndex: "name",
      key: "name",
    },
    {
      title: text.items,
      dataIndex: "numberOfItems",
      key: "numberOfItems",
    },
    {
      title: text.storage,
      dataIndex: "totalSize",
      key: "totalSize",
    },
    {
      title: text.packageType,
      dataIndex: "packageType",
      key: "packageType",
      render: (packageType: string) => (
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
      ),
    },
    {
      title: text.createdAt,
      dataIndex: "createdAt",
      key: "createdAt",
      render: (createdAt: string) =>
        format(new Date(createdAt), "dd/MM/yyyy", { locale: ptBR }),
    },
    {
      title: text.lastUpdated,
      dataIndex: "modifiedAt",
      key: "modifiedAt",
      render: (modifiedAt: string) => {
        const modifiedAtDate = new Date(modifiedAt);
        const now = new Date();
        const diffInDays = differenceInDays(now, modifiedAtDate);
        const diffInWeeks = Math.floor(diffInDays / 7);
        const diffInMonths = differenceInMonths(now, modifiedAtDate);
        const diffInYears = differenceInYears(now, modifiedAtDate);

        if (diffInDays < 7) {
          return format(modifiedAtDate, "EEEE, HH:mm", { locale: ptBR });
        } else if (diffInDays < 30) {
          return `${diffInWeeks} semanas atrás, ${format(
            modifiedAtDate,
            "HH:mm",
            { locale: ptBR }
          )}`;
        } else if (diffInYears === 0) {
          return `${diffInMonths} meses atrás`;
        }

        return format(modifiedAtDate, "dd/MM/yyyy", { locale: ptBR });
      },
    },
    {
      title: "",
      dataIndex: "actionButtons",
      key: "actionButtons",
      width: 180,
      render: (_: unknown, pkg: Package) => (
        <div className={styles.actionButtons}>
          <Button
            type="default"
            color="blue"
            variant="outlined"
            onClick={() => handlers.handleSelectPackage(pkg)}
          >
            {text.editButton}
          </Button>
          <Button
            type="default"
            color="green"
            variant="outlined"
            onClick={() => handlers.handleBuildPackage(pkg.fullPath, pkg.name)}
          >
            {text.buildButton}
          </Button>
          <Button
            type="default"
            color="danger"
            variant="outlined"
            onClick={() => handlers.handleDeletePackage(pkg.fullPath)}
          >
            {text.deleteButton}
          </Button>
        </div>
      ),
    },
  ];

  if (
    state.packageName &&
    state.packageLocation &&
    state.packageType.length > 0
  ) {
    return (
      <NodeManager
        packageLocation={state.packageLocation}
        packageName={state.packageName}
        packageType={state.packageType}
        selectedWorkspaceLocation={props.selectedWorkspaceLocation}
        setPackageName={handlers.setPackageName}
        setPackageLocation={handlers.setPackageLocation}
      />
    );
  }

  return (
    <div
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      <div className={styles.header}>
        <h1 className={styles.packageName}>{props.selectedWorkspaceName}</h1>
        <Button
          type="primary"
          color="geekblue"
          variant="solid"
          onClick={() => handlers.setIsModalOpen(true)}
        >
          {text.addPackageButton}
        </Button>
      </div>
      <div className={styles.tableWrapper}>
        <Table
          dataSource={props.packages}
          columns={columns}
          rowKey={(record) => record.fullPath}
        />
      </div>
      <PackageDialog
        packageLocation={props.selectedWorkspaceLocation}
        isModalOpen={state.isModalOpen}
        setIsModalOpen={handlers.setIsModalOpen}
        setPackages={props.setPackages}
      />
    </div>
  );
}

export default memo(PackageList);
