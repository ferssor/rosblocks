export const addMessage = (interfaceOptions: string[][]) => ({
  type: "add_message",
  message0:
    "Atribua à variável %1 com interface %2, propriedade %3 o valor %4",
  args0: [
    {
      type: "field_input",
      name: "MESSAGE_VARIABLE",
      text: "msg",
    },
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
    {
      type: "field_dropdown",
      name: "PROPERTY",
      options: [["Selecione interface primeiro", ""]],
    },
    {
      type: "input_value",
      name: "VALUE",
      text: "Defina o nome da variável",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Atribui valores às propriedades de uma mensagem ROS2",
  helpUrl: "",
});
