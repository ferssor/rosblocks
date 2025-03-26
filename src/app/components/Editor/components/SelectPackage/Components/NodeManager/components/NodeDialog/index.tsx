interface Props {
  packageLocation: string;
  packageName: string;
  isModalOpen: boolean;
  setIsModalOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

function NodeDialog(props: Props) {
  const { isModalOpen, packageLocation, setIsModalOpen } = props;
  console.log("🚀 ~ NodeDialog ~ isModalOpen:", isModalOpen);
  console.log("🚀 ~ NodeDialog ~ setIsModalOpen:", setIsModalOpen);
  console.log("🚀 ~ NodeDialog ~ packageLocation:", packageLocation);

  return <></>;
}

export default NodeDialog;
