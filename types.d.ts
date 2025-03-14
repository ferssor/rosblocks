interface Window {
  electronAPI: {
    openWorkspaceLocation: () => Promise<string>,
    createWorkspace: (path: string) => Promise<{created: boolean}>
  }
}
