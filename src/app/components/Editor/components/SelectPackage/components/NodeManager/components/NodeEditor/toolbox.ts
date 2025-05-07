import { ToolboxDefinition } from "react-blockly";

export const toolbox: ToolboxDefinition = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Class",
      colour: "120",
      contents: [
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
