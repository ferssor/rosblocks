import { Button, Table, Tag } from "antd";
import {
  differenceInDays,
  differenceInHours,
  differenceInMinutes,
  differenceInMonths,
  differenceInYears,
  format,
} from "date-fns";
import { ptBR } from "date-fns/locale";
import { memo } from "react";
import { useTranslation } from "react-i18next";

import { NodeManager } from "../node-manager";
import { PackageDialog } from "../package-dialog";

import usePackageListHook from "./package-list.hook";
import styles from "./package-list.styles.module.css";

import type { PackageListProps } from "./types";
import type { TableProps } from "antd";

function PackageList(props: PackageListProps) {
  const { className = "", style } = props;
  const { state, handlers, text, t } = usePackageListHook(props);
  const { i18n } = useTranslation();

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

        if (modifiedAtDate > now) {
          return format(modifiedAtDate, "P p", {
            locale: i18n.language === "pt" ? ptBR : undefined,
          });
        }

        const diffInMinutes = differenceInMinutes(now, modifiedAtDate);
        if (diffInMinutes < 1) return t("time.justNow");
        if (diffInMinutes < 60)
          return t("time.minutesAgo", { count: diffInMinutes });

        const diffInHours = differenceInHours(now, modifiedAtDate);
        if (diffInHours < 24)
          return t("time.hoursAgo", { count: diffInHours });

        const diffInDays = differenceInDays(now, modifiedAtDate);
        if (diffInDays < 7) return t("time.daysAgo", { count: diffInDays });

        const diffInMonths = differenceInMonths(now, modifiedAtDate);
        if (diffInMonths < 1) {
          const diffInWeeks = Math.floor(diffInDays / 7);
          return t("time.weeksAgo", { count: diffInWeeks });
        }
        if (diffInMonths < 12)
          return t("time.monthsAgo", { count: diffInMonths });

        const diffInYears = differenceInYears(now, modifiedAtDate);
        return t("time.yearsAgo", { count: diffInYears });
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
