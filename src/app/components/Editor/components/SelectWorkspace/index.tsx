import { Button, message, Result } from "antd";
import "./styles.css";
import WorkspaceDialog from "./components/WorkspaceDialog";
import { useState } from "react";

interface Props {
  setValidWorkspace: React.Dispatch<React.SetStateAction<boolean>>;
  setWorkspacePath: React.Dispatch<React.SetStateAction<string>>;
}

const TITLE =
  "É necessário criar ou abrir um workspace para prosseguir com a programação!";
const SUBTITLE =
  "Workspace é o local onde programas ROS ficam armazenados, por isso é necessário ter um workspace para armazenar-los";
const PRIMARY_BUTTON_TITLE = "Criar um novo workspace";
const SECONDARY_BUTTON_TITLE = "Abrir um workspace existente";

function SelectWorkspace(props: Props) {
  const { setValidWorkspace, setWorkspacePath } = props;
  const [openModal, setOpenModal] = useState(false);

  const handleOpenWorkspace = async () => {
    const result = await window.electronAPI.openWorkspaceLocation();
    const validateWorkspace = await window.electronAPI.validateWorkspace(
      result
    );

    if (!validateWorkspace.valid) {
      message.error(
        "Ocorreu um erro ao selecionar o workspace, a pasta não é compatível!"
      );
    }
    setWorkspacePath(result);
    setValidWorkspace(validateWorkspace.valid);
  };

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
              setWorkspaceLocationFromCreationDialog={setWorkspacePath}
              setValidWorkspace={setValidWorkspace}
            />
          </>,
        ]}
      />
    </>
  );
}

export default SelectWorkspace;
