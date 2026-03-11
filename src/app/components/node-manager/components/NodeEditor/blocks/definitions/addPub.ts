export const addPub = (interfaceOptions: string[][]) => ({
  type: "add_pub",
  message0:
    "Adicionar publicador %1 no tópico %2 com interface %3 e taxa de atualização %4 Hz",
  args0: [
    {
      type: "field_input",
      name: "PUB_NAME",
      text: "Defina o nome do publicador",
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
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip:
    "Define um publisher ROS com nome, interface e taxa de atualização.",
  helpUrl: "",
});
