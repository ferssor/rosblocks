export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface WorkspaceDialogProps extends BaseProps {
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setWorkspaceLocationFromCreationDialog: React.Dispatch<
    React.SetStateAction<string>
  >;
  setValidWorkspace: React.Dispatch<React.SetStateAction<boolean>>;
}
