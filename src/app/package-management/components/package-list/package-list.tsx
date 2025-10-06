import { Button, Space, Table, Tag, Tooltip } from "antd";
import {
  differenceInDays,
  differenceInMonths,
  differenceInYears,
} from "date-fns";
import { memo, useCallback, useMemo } from "react";

import usePackageListHook from "./package-list.hook";
import styles from "./package-list.styles.module.css";

import type { PackageListProps } from "./types";
import type { PackageItem } from "../../types";
import type { ColumnsType } from "antd/es/table";
function PackageList(props: PackageListProps) {
  const { className = "", style } = props;
  const { t, state, handlers } = usePackageListHook(props);

  const toDate = (v: unknown): Date | null => {
    if (!v) return null;
    if (v instanceof Date) return v;
    const n = typeof v === "number" ? v : Date.parse(String(v));
    return Number.isFinite(n) ? new Date(n) : null;
  };

  const humanDiff = useCallback(
    (d: Date) => {
      const now = new Date();
      const y = differenceInYears(now, d);
      if (y > 0) return t("columns.updatedAt.years", { count: y });
      const m = differenceInMonths(now, d);
      if (m > 0) return t("columns.updatedAt.months", { count: m });
      const dd = differenceInDays(now, d);
      if (dd > 0) return t("columns.updatedAt.days", { count: dd });
      return t("columns.updatedAt.today");
    },
    [t]
  );

  const typeFilters = Array.from(
    new Set((state.packages ?? []).map((p) => p.packageType).filter(Boolean))
  ).map((v) => ({ text: String(v), value: v }));

  const columns: ColumnsType<PackageItem> = useMemo(
    () => [
      {
        title: t("columns.name"),
        dataIndex: "name",
        key: "name",
        sorter: (a, b) =>
          String(a.name ?? "").localeCompare(String(b.name ?? "")),
        render: (_, record) => (
          <Tooltip title={record.name}>
            <span style={{ fontWeight: 500 }}>{record.name}</span>
          </Tooltip>
        ),
      },
      {
        title: t("columns.location"),
        dataIndex: "fullPath",
        key: "fullPath",
        ellipsis: true,
        sorter: (a, b) =>
          String(a.fullPath ?? "").localeCompare(String(b.fullPath ?? "")),
        render: (text: unknown) => {
          const v = String(text ?? "");
          return (
            <Tooltip title={v}>
              <span style={{ fontFamily: "mono" }}>{v}</span>
            </Tooltip>
          );
        },
      },
      {
        title: t("columns.type"),
        dataIndex: "packageType",
        key: "packageType",
        width: 140,
        filters: typeFilters,
        onFilter: (value, record) =>
          String(record.packageType) === String(value),
        render: (type: unknown) => (
          <Tag>{String(type ?? t("columns.type.unknown"))}</Tag>
        ),
      },
      {
        title: t("columns.updatedAt.title"),
        dataIndex: "modifiedAt",
        key: "modifiedAt",
        width: 230,
        sorter: (a, b) => {
          const da = toDate(a.modifiedAt) ?? toDate(a.createdAt) ?? new Date(0);
          const db = toDate(b.modifiedAt) ?? toDate(b.createdAt) ?? new Date(0);
          return da.getTime() - db.getTime();
        },
        render: (_: unknown, record) => {
          const d = toDate(record.modifiedAt) ?? toDate(record.createdAt);
          if (!d) return <span>—</span>;
          return (
            <Space direction="vertical" size={0}>
              <span>{d.toLocaleString()}</span>
              <span style={{ fontSize: 12, opacity: 0.7 }}>{humanDiff(d)}</span>
            </Space>
          );
        },
      },
      {
        title: t("columns.actions"),
        key: "actions",
        fixed: "right",
        width: 220,
        render: (_: unknown, record) => (
          <Space>
            <Button
              type="primary"
              onClick={() =>
                handlers.handleBuildPackage?.(record.fullPath, record.name)
              }
            >
              {t("actions.build")}
            </Button>
            <Button
              danger
              onClick={() => handlers.handleDeletePackage?.(record.fullPath)}
            >
              {t("actions.delete")}
            </Button>
          </Space>
        ),
      },
    ],
    [t, typeFilters, humanDiff, handlers]
  );

  return (
    <div
      id={props.id}
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      <h2 className={styles.title}>{state.selectedWorkspaceName}</h2>

      <>
        <Table
          columns={columns}
          dataSource={state.packages}
          rowKey={(r) => r.name ?? r.fullPath}
          size="middle"
          pagination={{ pageSize: 10 }}
          style={{ marginTop: 16 }}
        />
      </>
    </div>
  );
}

export default memo(PackageList);
