export interface BaseProps {
  className?: string;
  style?: React.CSSProperties;
  id?: string;
}

export interface NodeEditorProps extends BaseProps {
  selectedNode: ROSNode;
  pkgLocation: string;
  pkgName: string;
  setSelectedNode: React.Dispatch<React.SetStateAction<ROSNode | undefined>>;
  fetchNodes: (pkgLocation: string, pkgName: string) => Promise<void>;
}
