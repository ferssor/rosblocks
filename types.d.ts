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

type ROSInterface = {
  name: string;
  location: string;
};

type MessageProperty = {
  name: string;
  property: string;
};

interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<{
      workspaceLocation: string;
      canceled: boolean;
    }>;
    createWorkspace: (
      path: string
    ) => Promise<{ created: boolean; workspacePath: string; error?: boolean }>;
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
    importPackage: (
      url: string,
      workspacePath: string
    ) => Promise<{ imported: boolean; error?: string }>;
    deletePackage: (
      path: string
    ) => Promise<{ deleted: boolean; error?: string }>;
    getInterfaces: () => Promise<Array<ROSInterface>>;
    createBlocks: (
      nodePath: string,
      blocks: string,
      code: string
    ) => Promise<{ created: boolean; error?: string }>;
    buildPackage: (
      packagePath: string,
      packageName: string
    ) => Promise<{ wasBuilded: boolean; error?: string }>;
    addDependency: (
      relativePath: string,
      nodePath: string,
      interfaceName: string
    ) => Promise<{ wasAdded: boolean; error?: string }>;
    deleteNode: (
      nodeName: string,
      packagePath: string,
      packageName: string
    ) => Promise<{ wasDeleted: boolean; error?: string }>;
    removeDependency: (
      relativePath: string,
      nodePath: string,
      scriptName: string,
      interfaceName: string
    ) => Promise<{ wasRemoved: boolean; error?: string }>;
    executeNode: (
      packageName: string,
      nodeName: string,
      packagePath: string
    ) => Promise<{ executed: boolean; error?: string }>;
    addScript: (
      relativePath: string,
      nodePath: string,
      scriptName: string
    ) => Promise<{ wasAdded: boolean; error?: string }>;
    getMessageProperties: (
      interfaceName: string
    ) => Promise<Array<MessageProperty>>;
  };
}
