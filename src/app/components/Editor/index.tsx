import SelectWorkspace from "./components/SelectWorkspace";
import "./styles.css";

export function Editor() {
  return (
    <>
      <div className="editor-container">
        <SelectWorkspace />
      </div>
    </>
  );
}

export default Editor;
