type Package = {
  name: string;
  fullPath: string;
};
interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<string>;
    createWorkspace: (
      path: string
    ) => Promise<{ created: boolean; workspacePath: string }>;
    validateWorkspace: (path: string) => Promise<{ valid: boolean }>;
    getPackages: (path: string) => Promise<Array<Package>>;
  };
}
