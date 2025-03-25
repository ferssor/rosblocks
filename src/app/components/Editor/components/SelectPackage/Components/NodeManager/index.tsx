import { useEffect, useState } from "react";
import "./styles.css";

interface Props {
  packageName: string;
  packageLocation: string;
}

function NodeManager(props: Props) {
  const { packageLocation, packageName } = props;
  const [nodes, setNodes] = useState(Array<ROSNode>);

  useEffect(() => {
    const fetchPackages = async () => {
      const result = await window.electronAPI.getNodes(
        packageLocation,
        packageName
      );
      setNodes(result);
    };

    fetchPackages();
  }, [packageLocation, packageName, setNodes]);

  useEffect(() => {
    console.log(nodes);
  }, [nodes]);

  return (
    <>
      <h1>NodeManager Page</h1>
      <p>{packageLocation}</p>
      <p>{packageName}</p>
    </>
  );
}

export default NodeManager;
