export const publishMessage = {
  type: "publish_message",
  message0: "Publique a variável %1 no %2",
  args0: [
    {
      type: "field_input",
      name: "MESSAGE_VARIABLE",
      text: "msg",
    },
    {
      type: "field_input",
      name: "PUBLISHER_NAME",
      text: "Defina o nome do publicador",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Publica a variável instanciada utilizando um publicador",
  helpUrl: "",
};
