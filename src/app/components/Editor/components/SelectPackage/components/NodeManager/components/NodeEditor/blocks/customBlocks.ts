import * as Blockly from "blockly/core";

export function defineCustomBlocks() {
  // Define the blocks as JSON
  const blocks = [
    {
      type: "add_class",
      message0: "Classe %1 Node %2",
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
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);
}
