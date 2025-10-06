export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface PackageDialogProps extends BaseProps {
  children?: React.ReactNode;
}
