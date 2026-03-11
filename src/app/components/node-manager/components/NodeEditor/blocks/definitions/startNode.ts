export const startNode = {
  type: "start_node",
  message0: "Inicialize o nó da classe %1 com a função %2",
  args0: [
    {
      type: "field_input",
      name: "CLASS_NAME",
      text: "Defina o nome da classe",
    },
    {
      type: "field_input",
      name: "FUNCTION_NAME",
      text: "Defina o nome da função",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Adicione o nó",
  helpUrl: "",
};
