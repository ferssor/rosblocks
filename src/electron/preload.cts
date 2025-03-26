import electron, { ipcRenderer } from "electron";

electron.contextBridge.exposeInMainWorld("electronAPI", {
  openWorkspaceLocation: () => ipcRenderer.invoke("open-dialog"),
  createWorkspace: (path: string) =>
    ipcRenderer.invoke("create-workspace", path),
  validateWorkspace: (path: string) =>
    ipcRenderer.invoke("validate-workspace", path),
  getPackages: (path: string) => ipcRenderer.invoke("get-packages", path),
  createPackage: (path: string, name: string, dependency: string) =>
    ipcRenderer.invoke("create-package", path, name, dependency),
  getNodes: (packagePath: string, packageName: string) =>
    ipcRenderer.invoke("get-nodes", packagePath, packageName),
  createNode: (
    nodeName: string,
    nodeType: string,
    packagePath: string,
    packageName: string
  ) =>
    ipcRenderer.invoke(
      "create-node",
      nodeName,
      nodeType,
      packagePath,
      packageName
    ),
});
