export const addFromImport = {
  type: "add_from_import",
  message0: "A partir de %1 importe %2",
  args0: [
    {
      type: "field_input",
      name: "PACKAGE_NAME",
      text: "Defina o nome do pacote",
    },
    {
      type: "field_input",
      name: "METHOD_NAME",
      text: "Defina o nome do método",
    },
  ],
  colour: "#fe640b",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Importa um pacote Python",
  helpUrl: "",
};
