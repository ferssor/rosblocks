export const addInterface = (interfaceOptions: string[][]) => ({
  type: "add_interface",
  message0: "Importe a interface %1",
  args0: [
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Adicione uma interface ROS2",
  helpUrl: "",
});
