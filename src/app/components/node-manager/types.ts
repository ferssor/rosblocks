export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface NodeManagerProps extends BaseProps {
  packageName: string;
  packageLocation: string;
  packageType: string;
  selectedWorkspaceLocation: string;
  setPackageName: React.Dispatch<React.SetStateAction<string>>;
  setPackageLocation: React.Dispatch<React.SetStateAction<string>>;
}
