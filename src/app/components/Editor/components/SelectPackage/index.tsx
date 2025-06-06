import { useEffect, useState } from "react";
import PackageDialog from "./components/PackageDialog";
import "./styles.css";
import { Button, Result } from "antd";
import PackageList from "./components/PackageList";

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
        <PackageList
          packages={packages}
          selectedWorkspaceName={selectedWorkspaceName}
          selectedWorkspaceLocation={workspaceLocation}
          setPackages={setPackages}
        />
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
                  setPackages={setPackages}
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
