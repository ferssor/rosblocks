import * as Blockly from "blockly/core";

export function defineCustomBlocks() {
  // Define the blocks as JSON
  const blocks = [
    {
      type: "add_pub",
      message0: "Classe %1 herda de %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Minha Classe",
        },
        {
          type: "input_value",
          name: "PARENT_CLASS",
          text: "Classe",
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
