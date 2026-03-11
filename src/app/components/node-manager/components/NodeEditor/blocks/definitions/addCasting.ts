export const addCasting = {
  type: "add_casting",
  message0: `como %1 %2`,
  args0: [
    {
      type: "field_dropdown",
      options: [
        ["texto", "str"],
        ["inteiro", "int"],
        ["flutuante", "float"],
        ["booleano", "bool"],
      ],
      name: "DATA_TYPE",
      checked: true,
    },
    {
      type: "input_value",
      name: "VALUE",
      text: "Defina o valor",
    },
  ],
  colour: "#df8e1d",
  inputsInline: true,
  output: null,
  tooltip: "Adicione uma variável ao script",
  helpUrl: "",
};
