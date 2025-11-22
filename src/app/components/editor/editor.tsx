import { memo } from "react";

import { SelectPackage } from "../select-package";
import { SelectWorkspace } from "../select-workspace";

import useEditorHook from "./editor.hook";
import styles from "./editor.styles.module.css";

import type { EditorProps } from "./types";

function Editor(props: EditorProps) {
  const { className = "", style, id } = props;
  const { state, handlers } = useEditorHook();

  return (
    <div
      id={id}
      className={(styles.container + " " + (className || "")).trim()}
      style={style}
    >
      {state.shouldShowPackageSelection ? (
        <SelectPackage
          packages={state.packages}
          workspaceLocation={state.workspacePath}
          setPackages={handlers.setPackages}
        />
      ) : (
        <SelectWorkspace
          setValidWorkspace={handlers.setValidWorkspace}
          setWorkspacePath={handlers.setWorkspacePath}
        />
      )}
    </div>
  );
}

export default memo(Editor);
