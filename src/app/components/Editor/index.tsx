import { useCallback, useEffect, useState } from "react";
import SelectWorkspace from "./components/SelectWorkspace";
import "./styles.css";

export function Editor() {
  const [workspacePath, setWorkspacePath] = useState("");
  const [validWorkspace, setValidWorkspace] = useState(false);

  const getPackages = useCallback(async () => {
    if (workspacePath.includes("_ws")) {
      const result = await window.electronAPI.getPackages(workspacePath);
      console.log("🚀 ~ getPackages ~ result:", result);
    }
  }, [workspacePath]);

  useEffect(() => {
    console.log({ workspacePath, validWorkspace });
    getPackages();
  }, [getPackages, validWorkspace, workspacePath]);

  return (
    <>
      <div className="editor-container">
        {workspacePath.includes("_ws") && validWorkspace ? (
          <p>{workspacePath}</p>
        ) : (
          <SelectWorkspace
            setValidWorkspace={setValidWorkspace}
            setWorkspacePath={setWorkspacePath}
          />
        )}
      </div>
    </>
  );
}

export default Editor;
