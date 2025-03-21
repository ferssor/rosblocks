import { Table, Tag } from "antd";
import "./style.css";
import {
  differenceInDays,
  differenceInMonths,
  differenceInYears,
  format,
} from "date-fns";
import { ptBR } from "date-fns/locale";

interface Props {
  packages: Package[];
  selectedWorkspaceName: string;
}

function PackageList(props: Props) {
  const { packages, selectedWorkspaceName } = props;
  const columns = [
    {
      title: "Nome do pacote",
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
      title: "Última modificação",
      dataIndex: "modifiedAt",
      key: "modifiedAt",
      render: (modifiedAt: string) => {
        const modifiedAtDate = new Date(modifiedAt);
        const now = new Date();
        const diffInDays = differenceInDays(now, modifiedAtDate);
        const diffInWeeks = Math.floor(diffInDays / 7);
        const diffInMonths = differenceInMonths(now, modifiedAtDate);
        const diffInYears = differenceInYears(now, modifiedAtDate);

        if (diffInDays < 7) {
          return format(modifiedAtDate, "EEEE, HH:mm", { locale: ptBR });
        } else if (diffInDays < 30) {
          return `${diffInWeeks} semanas atrás, ${format(
            modifiedAtDate,
            "HH:mm",
            { locale: ptBR }
          )}`;
        } else if (diffInYears === 0) {
          return `${diffInMonths} meses atrás, `;
        } else {
          return format(modifiedAtDate, "dd/MM/yyyy", { locale: ptBR });
        }
      },
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
