import "./styles.css";

interface Props {
  workspaceName: string;
  packages: Package[];
}

function getWorkspaceName(fullPath: string) {
  const match = fullPath.match(/([^/]+_ws)$/);
  return match ? match[1] : "";
}

function SelectPackage(props: Props) {
  const { packages, workspaceName } = props;
  const selectedWorkspaceName = getWorkspaceName(workspaceName);

  return (
    <>
      <h1>{selectedWorkspaceName}</h1>
      <div>
        {packages.map((pkg) => (
          <ul>
            <li>{pkg.name}</li>
          </ul>
        ))}
      </div>
    </>
  );
}

export default SelectPackage;
