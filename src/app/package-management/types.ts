export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageManagementProps extends BaseProps {
  children?: React.ReactNode;
}

export type PackageItem = {
  name: string;
  fullPath: string;
  modifiedAt: string;
  createdAt: string;
  numberOfItems: number;
  totalSize: number;
  packageType: "python" | "cpp" | "unknown";
};
