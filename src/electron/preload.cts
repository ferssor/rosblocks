import electron, { ipcRenderer } from 'electron';

electron.contextBridge.exposeInMainWorld('electronAPI', {
  openWorkspaceLocation: () => ipcRenderer.invoke('open-dialog')
});
