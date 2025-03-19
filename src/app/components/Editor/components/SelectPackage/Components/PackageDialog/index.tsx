import "./styles.css";
interface Props {
  packageLocation: string;
}

function PackageDialog(props: Props) {
  const { packageLocation } = props;
  console.log("🚀 ~ PackageDialog ~ packageLocation:", packageLocation);

  return (
    <>
      <h1>PackageDialog page</h1>
    </>
  );
}

export default PackageDialog;
