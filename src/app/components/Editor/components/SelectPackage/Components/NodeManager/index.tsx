import "./styles.css";

interface Props {
  packageName: string;
  packageLocation: string;
}

function NodeManager(props: Props) {
  const { packageLocation, packageName } = props;

  return (
    <>
      <h1>NodeManager Page</h1>
      <p>{packageLocation}</p>
      <p>{packageName}</p>
    </>
  );
}

export default NodeManager;
