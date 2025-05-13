import * as Blockly from "blockly/core";

export async function defineCustomBlocks() {
  let interfaceOptions: string[][] = [["-", ""]];

  try {
    const result = await window.electronAPI.getInterfaces();
    const fetchedOptions = result.map((ROSInterface) => [
      ROSInterface.name,
      ROSInterface.location,
    ]);

    interfaceOptions = interfaceOptions.concat(fetchedOptions).sort();
  } catch (error) {
    console.error("Failed to fetch ROS2 interfaces:", error);
  }

  const blocks = [
    {
      type: "add_class",
      message0: "Iniciar %1 e o nó %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
        {
          type: "field_input",
          name: "NODE_NAME",
          text: "Defina o nome do nó",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CLASS_BODY",
        },
      ],
      colour: 120,
      tooltip: "Define uma classe em Python com nome e corpo opcional.",
      helpUrl: "",
    },
    {
      type: "add_pub",
      message0:
        "Adicionar publicador %1 com interface %2 e taxa de atualização %3 Hz",
      args0: [
        {
          type: "field_input",
          name: "PUB_NAME",
          text: "Defina o nome do publicador",
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
      colour: 230,
      tooltip:
        "Define um publisher ROS2 com nome, interface e taxa de atualização.",
      helpUrl: "",
    },
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);
}
