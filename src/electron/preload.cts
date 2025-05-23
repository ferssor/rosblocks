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
  importPackage: (url: string, workspacePath: string) =>
    ipcRenderer.invoke("import-package", url, workspacePath),
  deletePackage: (path: string) => ipcRenderer.invoke("delete-package", path),
  getInterfaces: () => ipcRenderer.invoke("get-interfaces"),
  createBlocks: (
    interfaceName: string,
    scriptName: string,
    relativePath: string,
    nodePath: string,
    blocks: string,
    code: string
  ) =>
    ipcRenderer.invoke(
      "create-blocks",
      interfaceName,
      scriptName,
      relativePath,
      nodePath,
      blocks,
      code
    ),
  buildPackage: (packagePath: string, packageName: string) =>
    ipcRenderer.invoke("build-package", packagePath, packageName),
});
