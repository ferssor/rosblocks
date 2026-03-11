export const addCallbackFunction = (interfaceOptions: string[][]) => ({
  type: "add_callback_function",
  message0: `Adicione a função de retorno %1 com a interface %2`,
  args0: [
    {
      type: "field_input",
      name: "FUNCTION_NAME",
      text: "Defina o nome da função",
    },
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
  ],
  message1: "%1",
  args1: [
    {
      type: "input_statement",
      name: "FUNCTION_BODY",
    },
  ],
  colour: "#1e66f5",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Adicione uma função de retorno de chamada",
  helpUrl: "",
});
