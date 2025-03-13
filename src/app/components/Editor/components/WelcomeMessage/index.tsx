import { Button, Result } from "antd";
import "./styles.css";
import WorkspaceDialog from "./components/WorkspaceDialog";
import { useState } from "react";

const TITLE =
  "É necessário criar ou abrir um workspace para prosseguir com a programação!";
const SUBTITLE =
  "Workspace é o local onde programas ROS ficam armazenados, por isso é necessário ter um workspace para armazenar-los";
const PRIMARY_BUTTON_TITLE = "Criar um novo workspace";
const SECONDARY_BUTTON_TITLE = "Abrir um workspace existente";

function WelcomeMessage() {
  const [openModal, setOpenModal] = useState(false);

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
            <Button>{SECONDARY_BUTTON_TITLE}</Button>
            <WorkspaceDialog
              isModalOpen={openModal}
              setIsModalOpen={setOpenModal}
            />
          </>,
        ]}
      />
    </>
  );
}

export default WelcomeMessage;
