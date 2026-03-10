export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface WorkspaceManagementProps extends BaseProps {
  children?: React.ReactNode;
}
export interface RecentWorkspace {
  name: string;
  path: string;
}

export interface WorkspaceManagementHookState {
  recentWorkspaces: RecentWorkspace[];
  openModal: boolean;
}
