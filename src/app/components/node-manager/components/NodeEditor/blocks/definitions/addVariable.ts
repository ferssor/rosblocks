export const addVariable = {
  type: "add_variable",
  message0: "Adicione à variável %1 o valor %2",
  args0: [
    {
      type: "field_input",
      name: "VARIABLE",
      text: "Defina o nome da variável",
    },
    {
      type: "input_value",
      name: "VALUE",
      text: "Defina o valor da variável",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  inputsInline: true,
  colour: "#e64553",
  tooltip: "Adicione uma variável ao script",
  helpUrl: "",
};
