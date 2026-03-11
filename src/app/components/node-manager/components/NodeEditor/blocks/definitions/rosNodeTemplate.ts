export const rosNodeTemplate = (interfaceOptions: string[][]) => ({
  type: "ros_node_template",
  message0: "Importe a interface %1",
  args0: [
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
  ],
  message1: "Defina a classe %1",
  args1: [
    {
      type: "field_input",
      name: "CLASS_NAME",
      text: "Digite o nome da classe",
    },
  ],
  message2: "%1",
  args2: [
    {
      type: "input_statement",
      name: "NODE_BODY",
    },
  ],
  colour: "#5c5f77",
  tooltip:
    "Cria um nó ROS2 completo com importação de interface, classe e corpo.",
  helpUrl: "",
});
