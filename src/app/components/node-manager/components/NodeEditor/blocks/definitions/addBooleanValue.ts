export const addBooleanValue = {
  type: "add_boolean_value",
  message0: `%1`,
  args0: [
    {
      type: "field_dropdown",
      options: [
        ["Verdadeiro", "True"],
        ["Falso", "False"],
      ],
      name: "BOOL_VALUE",
      checked: true,
    },
  ],
  colour: "#df8e1d",
  output: null,
  tooltip: "Adicione uma variável ao script",
  helpUrl: "",
};
