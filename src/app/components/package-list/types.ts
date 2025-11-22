export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageListProps extends BaseProps {
  packages: Package[];
  selectedWorkspaceName: string;
  selectedWorkspaceLocation: string;
  setPackages: React.Dispatch<React.SetStateAction<Package[]>>;
}
