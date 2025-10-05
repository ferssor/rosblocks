export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface WorkspaceProviderProps extends BaseProps {
  children?: React.ReactNode;
}

export type WorkspaceContextValue = {
  workspacePath: string;
  isValidWorkspace: boolean;
  setWorkspacePath: (path: string) => void;
  setIsValidWorkspace: (valid: boolean) => void;
};
