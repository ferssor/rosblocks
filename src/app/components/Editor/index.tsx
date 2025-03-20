import { useEffect, useState } from "react";
import SelectWorkspace from "./components/SelectWorkspace";
import "./styles.css";
import SelectPackage from "./components/SelectPackage";

export function Editor() {
  const [workspacePath, setWorkspacePath] = useState("");
  const [validWorkspace, setValidWorkspace] = useState(false);
  const [packages, setPackages] = useState(Array<Package>);

  useEffect(() => {
    const fetchPackages = async () => {
      if (workspacePath) {
        const result = await window.electronAPI.getPackages(workspacePath);
        setPackages(result);
      }
    };

    fetchPackages();
  }, [workspacePath]);

  return (
    <>
      <div className="editor-container">
        {workspacePath.includes("_ws") && validWorkspace ? (
          <SelectPackage
            packages={packages}
            workspaceLocation={workspacePath}
            setPackages={setPackages}
          />
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
