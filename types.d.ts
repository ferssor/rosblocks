type Package = {
  name: string;
  fullPath: string;
  modifiedAt: string;
  createdAt: string;
  numberOfItems: number;
  totalSize: number;
  packageType: string;
};

type ROSNode = {
  name: string;
  fullPath: string;
  relativePath: string;
  content: string;
};
interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<{
      workspaceLocation: string;
      canceled: boolean;
    }>;
    createWorkspace: (
      path: string
    ) => Promise<{ created: boolean; workspacePath: string }>;
    validateWorkspace: (path: string) => Promise<{ valid: boolean }>;
    getPackages: (path: string) => Promise<Array<Package>>;
    createPackage: (
      path: string,
      name: string,
      dependency: string
    ) => Promise<{ created: boolean; packagePath: string; error?: string }>;
    getNodes: (
      packagePath: string,
      packageName: string
    ) => Promise<Array<ROSNode>>;
    createNode: (
      nodeName: string,
      nodeType: string,
      packagePath: string,
      packageName: string
    ) => Promise<{ created: boolean; nodePath?: string; error?: string }>;
  };
}
