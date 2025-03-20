import { Card } from "antd";

interface Props {
  packages: Package[];
  selectedWorkspaceName: string;
}
function PackageList(props: Props) {
  const { packages, selectedWorkspaceName } = props;

  return (
    <>
      <div className="package-list-container">
        <h1 className="package-name">{selectedWorkspaceName}</h1>
        <div className="package-list">
          {packages.map((pkg) => (
            <Card key={pkg.fullPath} title={pkg.name} variant="borderless">
              <p>{pkg.created.toISOString()}</p>
              <p>{pkg.modified.toISOString()}</p>
              <p>{pkg.numberOfItems}</p>
              <p>{pkg.packageType}</p>
              <p>{pkg.totalSize}</p>
            </Card>
          ))}
        </div>
      </div>
    </>
  );
}

export default PackageList;
