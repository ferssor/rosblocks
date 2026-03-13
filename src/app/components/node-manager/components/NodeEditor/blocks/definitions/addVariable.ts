const randomName = `var_${Math.random().toString(36).substring(2, 8)}`;

export const addVariable = {
  type: "add_variable",
  message0: "Adicione a variável %1 com o valor: %2",
  args0: [
    {
      type: "field_input",
      name: "VARIABLE",
      text: randomName,
    },
    {
      type: "input_value",
      name: "VALUE",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  inputsInline: true,
  colour: "#e64553",
  tooltip: "Adicione uma variável ao script",
  helpUrl: "",
};
