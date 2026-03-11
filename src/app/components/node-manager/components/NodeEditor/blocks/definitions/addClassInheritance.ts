export const addClassInheritance = {
  type: "add_class_inheritance",
  message0: "Iniciar classe %1 herdando da classe %2",
  args0: [
    {
      type: "field_input",
      name: "CLASS_NAME",
      text: "Defina o nome da classe",
    },
    {
      type: "field_input",
      name: "EXTENDS_NAME",
      text: "Defina o nome da classe a ser herdada",
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
