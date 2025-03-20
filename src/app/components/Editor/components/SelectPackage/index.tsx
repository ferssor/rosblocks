import { useEffect, useState } from "react";
import PackageDialog from "./Components/PackageDialog";
import "./styles.css";
import { Button, Card, Result } from "antd";

interface Props {
  workspaceLocation: string;
  packages: Package[];
  setPackages: React.Dispatch<React.SetStateAction<Package[]>>;
}

function getWorkspaceName(fullPath: string) {
  const match = fullPath.match(/([^/]+_ws)$/);
  return match ? match[1] : "";
}

function SelectPackage(props: Props) {
  const { packages, workspaceLocation, setPackages } = props;
  const [isModalOpen, setIsModalOpen] = useState(false);
  const selectedWorkspaceName = getWorkspaceName(workspaceLocation);
  const TITLE = `Parece que o workspace ${getWorkspaceName(
    workspaceLocation
  )} não possui package, para prosseguir é necessário criar um!`;
  const SUBTITLE =
    "Packages são um conjunto de programas ROS ou Nodes, que juntos exercem função específica.";
  const PRIMARY_BUTTON_TITLE = "Criar um novo package";

  useEffect(() => {
    const fetchPackages = async () => {
      if (workspaceLocation && !isModalOpen) {
        const result = await window.electronAPI.getPackages(workspaceLocation);
        setPackages(result);
      }
    };

    fetchPackages();
  }, [isModalOpen, workspaceLocation, setPackages]);

  return (
    <>
      {packages.length > 0 ? (
        <>
          <div className="package-list-container">
            <h1 className="package-name">{selectedWorkspaceName}</h1>
            <div className="package-list">
              {packages.map((pkg) => (
                <Card key={pkg.fullPath} title={pkg.name} variant="borderless">
                  {pkg.fullPath}
                </Card>
              ))}
            </div>
          </div>
        </>
      ) : (
        <>
          <Result
            className="package-container"
            status="404"
            title={TITLE}
            subTitle={SUBTITLE}
            extra={[
              <>
                <Button
                  type="primary"
                  onClick={() => {
                    setIsModalOpen(true);
                  }}
                >
                  {PRIMARY_BUTTON_TITLE}
                </Button>
                <PackageDialog
                  packageLocation={workspaceLocation}
                  isModalOpen={isModalOpen}
                  setIsModalOpen={setIsModalOpen}
                />
              </>,
            ]}
          />
        </>
      )}
    </>
  );
}

export default SelectPackage;
