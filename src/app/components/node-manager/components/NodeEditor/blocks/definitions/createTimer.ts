export const createTimer = {
  type: "create_timer",
  message0:
    "Criar um temporizador %1 com intervalo %2 e publique na função %3",
  args0: [
    {
      type: "field_input",
      name: "TEMP_NAME",
      text: "Defina o nome do temporizador",
    },
    {
      type: "field_number",
      name: "DURATION",
      value: 1.0,
      min: 1.0,
      precision: 1.0,
    },
    {
      type: "input_value",
      name: "PUBLISHER_COUNTER",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Adiciona uma função contadora e publicadora",
  helpUrl: "",
};
