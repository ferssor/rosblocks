const randomName = `node_${Math.random().toString(36).substring(2, 8)}`;

export const nodeInit = {
  type: "node_init",
  message0: "Adicione o nó %1",
  args0: [
    {
      type: "field_input",
      name: "NODE_NAME",
      text: randomName,
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
  colour: "#f89500",
  tooltip: "Inicialize o nó",
  helpUrl: "",
};
