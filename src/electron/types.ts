export type createWorkspaceResult = {
  wasCreated: boolean;
  wasCanceled: boolean;
  error?: string;
  workspacePath?: string;
};
