export const initMessageInterface = (interfaceOptions: string[][]) => ({
  type: "init_message_interface",
  message0: "Adicione à variável %1 os métodos da interface %2",
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
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Instancia uma interface ROS2 em uma variável",
  helpUrl: "",
});
