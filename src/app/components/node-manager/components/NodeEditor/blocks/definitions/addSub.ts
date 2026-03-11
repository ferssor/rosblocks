export const addSub = (interfaceOptions: string[][]) => ({
  type: "add_sub",
  message0: `Adicionar assinante %1 
      no tópico %2 
      com interface %3, 
      taxa de atualização %4 Hz 
      e armazene os dados na %5`,
  args0: [
    {
      type: "field_input",
      name: "SUB_NAME",
      text: "Defina o nome do assinante",
    },
    {
      type: "field_input",
      name: "TOPIC_NAME",
      text: "Defina o nome do tópico",
    },
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
    {
      type: "field_number",
      name: "REFRESH_RATE",
      value: 10,
      min: 1,
      precision: 1,
    },
    {
      type: "field_input",
      name: "FUNCTION_NAME",
      text: "Defina o nome da função",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip:
    "Define um subscriber ROS2 com nome, interface e taxa de atualização.",
  helpUrl: "",
});
