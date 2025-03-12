import WelcomeMessage from "./components/WelcomeMessage";
import "./styles.css";

export function Editor() {
  return (
    <>
      <div className="editor-container">
        <WelcomeMessage />
      </div>
    </>
  );
}

export default Editor;
