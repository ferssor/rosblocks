import { ToolboxDefinition } from "react-blockly";

export const toolbox: ToolboxDefinition = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Ponto de início",
      colour: "120",
      contents: [
        {
          kind: "block",
          type: "add_class",
        },
      ],
    },
    {
      kind: "sep",
    },
  ],
};
