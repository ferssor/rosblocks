import { useEffect, useMemo, useState } from "react";

function useEditorHook() {
  const [workspacePath, setWorkspacePath] = useState("");
  const [isValidWorkspace, setIsValidWorkspace] = useState(false);
  const [packages, setPackages] = useState<Package[]>([]);

  useEffect(() => {
    async function fetchPackages() {
      if (!workspacePath) {
        setPackages([]);
        return;
      }

      const result = await window.electronAPI.getPackages(workspacePath);
      setPackages(result);
    }

    fetchPackages();
  }, [workspacePath]);

  const shouldShowPackageSelection = useMemo(() => {
    if (!workspacePath) {
      return false;
    }

    return workspacePath.includes("_ws") && isValidWorkspace;
  }, [workspacePath, isValidWorkspace]);

  return {
    state: {
      workspacePath,
      packages,
      isValidWorkspace,
      shouldShowPackageSelection,
    },
    handlers: {
      setPackages,
      setWorkspacePath,
      setValidWorkspace: setIsValidWorkspace,
    },
  };
}

export default useEditorHook;
