export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface SelectWorkspaceProps extends BaseProps {
  setValidWorkspace: React.Dispatch<React.SetStateAction<boolean>>;
  setWorkspacePath: React.Dispatch<React.SetStateAction<string>>;
}
