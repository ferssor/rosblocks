export const nodeInit = {
  type: "node_init",
  message0: "Inicialize o nó %1",
  args0: [
    {
      type: "field_input",
      name: "NODE_NAME",
      text: "Defina o nome do nó",
    },
  ],
  message1: "%1",
  args1: [
    {
      type: "input_statement",
      name: "CONSTRUCTOR_BODY",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Inicialize o nó",
  helpUrl: "",
};
