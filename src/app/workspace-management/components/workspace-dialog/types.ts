export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface WorkspaceDialogProps extends BaseProps {
  children?: React.ReactNode;
  isModalOpen: boolean;
  setIsModalOpen: (open: boolean) => void;
}
