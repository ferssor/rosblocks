import { ToolboxDefinition } from "react-blockly";

export const toolbox: ToolboxDefinition = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Entrada",
      colour: "120",
      contents: [
        {
          kind: "block",
          type: "add_class",
        },
        {
          kind: "block",
          type: "add_pub",
        },
      ],
    },
    {
      kind: "sep",
    },
  ],
};
