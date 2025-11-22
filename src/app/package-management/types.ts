export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageManagementProps extends BaseProps {
  children?: React.ReactNode;
}

export type PackageItem = Package;
