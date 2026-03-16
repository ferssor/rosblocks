import { BLOCK_COLOR } from "../../constants/colors";

export const createTimer = () => {
  const randomName = `temp_${Math.random().toString(36).substring(2, 8)}`;

  return {
    type: "create_timer",
    message0:
      "Adicione um temporizador %1 com intervalo de %2 segundos para executar a função %3",
    args0: [
      {
        type: "field_input",
        name: "TEMP_NAME",
        text: randomName,
      },
      {
        type: "field_number",
        name: "DURATION",
        value: 1.0,
        min: 1.0,
        precision: 1.0,
      },
      {
        type: "input_value",
        name: "PUBLISHER_COUNTER",
      },
    ],
    previousStatement: null,
    nextStatement: null,
    inputsInline: true,
    colour: BLOCK_COLOR.ros,
    tooltip: "Adiciona uma função contadora e publicadora",
    helpUrl: "",
  };
};
