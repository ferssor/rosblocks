import { BLOCK_COLOR } from "../../constants/colors";

export const addLogger = {
  type: "add_logger",
  message0: "Registre %1: %2",
  args0: [
    {
      type: "field_dropdown",
      options: [
        ["a informação", "info"],
        ["o erro", "error"],
        ["o aviso", "warning"],
        ["o debug", "debug"],
      ],
      name: "LOG_LEVEL",
      checked: true,
    },
    {
      type: "input_value",
      name: "LOG_TEXT",
      text: "Defina o texto que será exibido",
    },
  ],
  colour: BLOCK_COLOR.ros,
  inputsInline: true,
  previousStatement: null,
  nextStatement: null,
  tooltip: "Adicione um registro ao nó",
  helpUrl: "",
};
