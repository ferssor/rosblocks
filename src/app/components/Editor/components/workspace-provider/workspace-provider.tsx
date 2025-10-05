import { createContext, useContext } from "react";

import useWorkspaceProviderHook from "./workspace-provider.hook";

import type { WorkspaceContextValue, WorkspaceProviderProps } from "./types";

const WorkspaceContext = createContext<WorkspaceContextValue | undefined>(
  undefined
);
WorkspaceContext.displayName = "WorkspaceContext";

function WorkspaceProvider(props: WorkspaceProviderProps) {
  const { children } = props;
  const { value } = useWorkspaceProviderHook(); // <- garanta que é memoizado no hook
  return (
    <WorkspaceContext.Provider value={value}>
      {children}
    </WorkspaceContext.Provider>
  );
}

// eslint-disable-next-line react-refresh/only-export-components
export function useWorkspace() {
  const ctx = useContext(WorkspaceContext);
  if (ctx === undefined) {
    throw new Error("useWorkspace must be used within a WorkspaceProvider");
  }
  return ctx;
}

export default WorkspaceProvider;
