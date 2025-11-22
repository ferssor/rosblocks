export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface NodeDialogProps extends BaseProps {
  packageLocation: string;
  packageName: string;
  packageType: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
  setNodes: React.Dispatch<React.SetStateAction<ROSNode[]>>;
}
