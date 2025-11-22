export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageDialogProps extends BaseProps {
  packageLocation: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setPackages: React.Dispatch<React.SetStateAction<Package[]>>;
}
