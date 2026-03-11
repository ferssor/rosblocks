export const executeFunctionTimer = {
  type: "execute_function_timer",
  message0: `Execute a função %1 por %2 segundos`,
  args0: [
    {
      type: "field_input",
      name: "FUNCTION_NAME",
      text: "Defina o nome da função",
    },
    {
      type: "field_number",
      name: "INTERVAL",
      value: 1,
      min: 1,
      precision: 1,
    },
  ],
  colour: "#1e66f5",
  previousStatement: null,
  nextStatement: null,
  tooltip: "Adiciona um contador com intervalo que chama uma função",
  helpUrl: "",
};
