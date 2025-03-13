interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<string>,
    createWorkspace: (path: string) => Promise<void>
  }
}
