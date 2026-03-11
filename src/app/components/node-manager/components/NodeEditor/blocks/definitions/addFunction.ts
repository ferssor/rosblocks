export const addFunction = {
  type: "add_function",
  message0: "Função %1",
  args0: [
    {
      type: "field_input",
      name: "FUNCTION_NAME",
      text: "Defina o nome da função",
    },
  ],
  message1: "%1",
  args1: [
    {
      type: "input_statement",
      name: "FUNCTION_BODY",
    },
  ],
  colour: "#8839ef",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Adicione uma função",
  helpUrl: "",
};
