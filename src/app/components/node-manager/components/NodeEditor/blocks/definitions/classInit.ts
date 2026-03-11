export const classInit = {
  type: "class_init",
  message0: "Inicializa o construtor da classe %1",
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
      name: "CONSTRUCTOR_BODY",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#40a02b",
  tooltip: "Adicione o construtor da classe",
  helpUrl: "",
};
