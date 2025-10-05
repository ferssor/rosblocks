import { useCallback, useMemo, useState } from "react";

import type { WorkspaceContextValue } from "./types";

function useWorkspaceProviderHook() {
  const [workspacePath, setWorkspacePath] = useState<string>("");
  const [isValidWorkspace, setIsValidWorkspace] = useState<boolean>(false);

  const setPath = useCallback((path: string) => setWorkspacePath(path), []);
  const setValid = useCallback(
    (valid: boolean) => setIsValidWorkspace(valid),
    []
  );

  const value: WorkspaceContextValue = useMemo(
    () => ({
      workspacePath,
      isValidWorkspace,
      setWorkspacePath: setPath,
      setIsValidWorkspace: setValid,
    }),
    [workspacePath, isValidWorkspace, setPath, setValid]
  );

  return { value };
}

export default useWorkspaceProviderHook;
