export const addImportAs = {
  type: "add_import_as",
  message0: "Importar %1 como %2",
  args0: [
    {
      type: "field_input",
      name: "IMPORT_NAME",
      text: "Defina o nome do pacote",
    },
    {
      type: "field_input",
      name: "ALIAS_NAME",
      text: "Defina o nome do apelido",
    },
  ],
  colour: "#fe640b",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Importa um pacote Python",
  helpUrl: "",
};
