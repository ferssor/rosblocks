import "./styles.css";

interface Props {
  selectedNode: ROSNode;
}

function NodeEditor(props: Props) {
  const { selectedNode } = props;

  return (
    <>
      <h1>{selectedNode.name}</h1>
    </>
  );
}

export default NodeEditor;
