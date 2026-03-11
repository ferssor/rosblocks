export const addClass = {
  type: "add_class",
  message0: "Iniciar classe %1",
  args0: [
    {
      type: "field_input",
      name: "CLASS_NAME",
      text: "Defina o nome da classe",
    },
  ],
  message1: "%1",
  args1: [
    {
      type: "input_statement",
      name: "CLASS_BODY",
    },
  ],
  colour: "#40a02b",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Define uma classe em Python.",
  helpUrl: "",
};
