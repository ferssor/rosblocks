import { Button, Result } from "antd";
import "./styles.css";
import WorkspaceDialog from "./components/WorkspaceDialog";
import { useEffect, useState } from "react";

const TITLE =
  "É necessário criar ou abrir um workspace para prosseguir com a programação!";
const SUBTITLE =
  "Workspace é o local onde programas ROS ficam armazenados, por isso é necessário ter um workspace para armazenar-los";
const PRIMARY_BUTTON_TITLE = "Criar um novo workspace";
const SECONDARY_BUTTON_TITLE = "Abrir um workspace existente";

function WelcomeMessage() {
  const [openModal, setOpenModal] = useState(false);
  const [workspaceLocation, setWorkspaceLocation] = useState("");
  const [isValidWorkspace, setIsValidWorkspace] = useState(false);

  const handleOpenWorkspace = async () => {
    const result = await window.electronAPI.openWorkspaceLocation();
    const validateWorkspace = await window.electronAPI.validateWorkspace(
      result
    );
    setWorkspaceLocation(result);
    setIsValidWorkspace(validateWorkspace.valid);
  };

  useEffect(() => {
    console.log(workspaceLocation);
    console.log(isValidWorkspace);
  }, [isValidWorkspace, workspaceLocation]);

  return (
    <>
      <Result
        className="welcome-container"
        status="info"
        title={TITLE}
        subTitle={SUBTITLE}
        extra={[
          <>
            <Button
              type="primary"
              onClick={() => {
                setOpenModal(true);
              }}
            >
              {PRIMARY_BUTTON_TITLE}
            </Button>
            <Button onClick={handleOpenWorkspace}>
              {SECONDARY_BUTTON_TITLE}
            </Button>
            <WorkspaceDialog
              isModalOpen={openModal}
              setIsModalOpen={setOpenModal}
              setWorkspaceLocationFromCreationDialog={setWorkspaceLocation}
            />
          </>,
        ]}
      />
    </>
  );
}

export default WelcomeMessage;
