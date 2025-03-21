import { Table, Tag } from "antd";
import "./style.css";
import { format } from "date-fns";

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
      render: (packageType: string) => (
        <Tag
          color={
            packageType === "cpp"
              ? "blue"
              : packageType === "python"
              ? "green"
              : "default"
          }
        >
          {packageType}
        </Tag>
      ),
    },
    {
      title: "Criado em",
      dataIndex: "createdAt",
      key: "createdAt",
      render: (createdAt: string) => format(new Date(createdAt), "dd/MM/yyyy"),
    },
    {
      title: "Modificado em",
      dataIndex: "modifiedAt",
      key: "modifiedAt",
      render: (createdAt: string) => format(new Date(createdAt), "dd/MM/yyyy"),
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
