export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface SelectPackageProps extends BaseProps {
  workspaceLocation: string;
  packages: Package[];
  setPackages: React.Dispatch<React.SetStateAction<Package[]>>;
}
