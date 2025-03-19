import { useState } from "react";
import PackageDialog from "./Components/PackageDialog";
import "./styles.css";
import { Button, Result } from "antd";

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
  const [isModalOpen, setIsModalOpen] = useState(false);
  const selectedWorkspaceName = getWorkspaceName(workspaceName);
  const TITLE = `Parece que o workspace ${getWorkspaceName(
    workspaceName
  )} não possui package, para prosseguir é necessário criar um!`;
  const SUBTITLE =
    "Packages são um conjunto de programas ROS ou Nodes, que juntos exercem função específica.";
  const PRIMARY_BUTTON_TITLE = "Criar um novo package";

  return (
    <>
      {packages.length > 0 ? (
        <>
          <h1>{selectedWorkspaceName}</h1>
          <div>
            {packages.map((pkg) => (
              <ul>
                <li key={pkg.fullPath}>{pkg.name}</li>
              </ul>
            ))}
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
                  packageLocation={workspaceName}
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
