type Package = {
  name: string;
  fullPath: string;
  modified: Date;
  created: Date;
  numberOfItems: number;
  totalSize: number;
  packageType: string;
};
interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<string>;
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
  };
}
