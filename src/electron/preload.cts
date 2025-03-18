import electron, { ipcRenderer } from 'electron';

electron.contextBridge.exposeInMainWorld('electronAPI', {
  openWorkspaceLocation: () => ipcRenderer.invoke('open-dialog'),
  createWorkspace: (path: string) => ipcRenderer.invoke('create-workspace', path),
  validateWorkspace: (path: string) => ipcRenderer.invoke('validate-workspace', path),
  getPackages: (path: string) => ipcRenderer.invoke('get-packages', path)
});
