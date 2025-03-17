import { useEffect, useState } from "react";
import SelectWorkspace from "./components/SelectWorkspace";
import "./styles.css";

export function Editor() {
  const [workspacePath, setWorkspacePath] = useState("");
  const [validWorkspace, setValidWorkspace] = useState(false);

  useEffect(() => {
    console.log({ workspacePath, validWorkspace });
  }, [validWorkspace, workspacePath]);

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
