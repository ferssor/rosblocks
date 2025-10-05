export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface WorkspaceManagementProps extends BaseProps {
  children?: React.ReactNode;
}
