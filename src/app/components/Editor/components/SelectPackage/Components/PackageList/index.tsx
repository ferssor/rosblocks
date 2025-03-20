import { Table } from "antd";
import "./style.css";

interface Props {
  packages: Package[];
  selectedWorkspaceName: string;
}

function PackageList(props: Props) {
  const { packages, selectedWorkspaceName } = props;
  const columns = [
    {
      title: "Nome",
      dataIndex: "name",
      key: "name",
    },
    {
      title: "Número de Items",
      dataIndex: "numberOfItems",
      key: "numberOfItems",
    },
    {
      title: "Armazenamento (mb)",
      dataIndex: "totalSize",
      key: "totalSize",
    },
    {
      title: "Tipo do pacote",
      dataIndex: "packageType",
      key: "packageType",
    },
    {
      title: "Criado em",
      dataIndex: "createdAt",
      key: "createdAt",
    },
    {
      title: "Modificado em",
      dataIndex: "modifiedAt",
      key: "modifiedAt",
    },
  ];

  return (
    <>
      <div className="package-list-container">
        <h1 className="package-name">{selectedWorkspaceName}</h1>
        <Table dataSource={packages} columns={columns} />
      </div>
    </>
  );
}

export default PackageList;
