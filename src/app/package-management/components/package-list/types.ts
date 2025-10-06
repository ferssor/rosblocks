import type { PackageItem } from "../../types";

export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageListProps extends BaseProps {
  children?: React.ReactNode;
  packages: PackageItem[];
  selectedWorkspaceName: string;
  selectedWorkspaceLocation: string;
  setPackages: React.Dispatch<React.SetStateAction<PackageItem[]>>;
}
