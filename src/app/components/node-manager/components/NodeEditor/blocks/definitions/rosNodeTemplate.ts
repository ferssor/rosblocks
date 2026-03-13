export const rosNodeTemplate = () => {
  const randomName = `block_${Math.random().toString(36).substring(2, 8)}`;
  return {
    type: "ros_node_template",
    message0: "ROSBLOCKS %1:",
    args0: [
      {
        type: "field_input",
        name: "CLASS_NAME",
        text: randomName,
      },
    ],
    message1: "%1",
    args1: [
      {
        type: "input_statement",
        name: "NODE_BODY",
      },
    ],
    colour: "#1677FF",
    tooltip: "Slot superior: Importações. Slot inferior: Lógica do Nó.",
    helpUrl: "",
  };
};
