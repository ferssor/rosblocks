export const counterFunction = (interfaceOptions: string[][]) => ({
  type: "counter_function",
  message0:
    "Adicione a função contadora com a interface %1 e inicie o contador com o valor %2 e publique em %3",
  args0: [
    {
      type: "field_dropdown",
      name: "COUNTER_INTERFACE",
      options: interfaceOptions,
    },
    {
      type: "field_number",
      name: "COUNTER",
      value: 0,
      min: 0,
      precision: 1,
    },
    {
      type: "field_input",
      name: "PUBLISHER",
      text: "Defina o nome do publicador",
    },
  ],
  output: null,
  colour: "#1e66f5",
  tooltip: "Adiciona uma função que publica o contador",
  helpUrl: "",
});
