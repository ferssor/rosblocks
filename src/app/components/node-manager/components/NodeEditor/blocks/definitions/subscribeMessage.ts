export const subscribeMessage = (interfaceOptions: string[][]) => ({
  type: "subscribe_message",
  message0: `Atribua o valor da mensagem com a interface %1
         e a propriedade %2 na variável %3`,
  args0: [
    {
      type: "field_dropdown",
      name: "MESSAGE_INTERFACE",
      options: interfaceOptions,
    },
    {
      type: "field_dropdown",
      name: "PROPERTY",
      options: [["Selecione interface primeiro", ""]],
    },
    {
      type: "field_input",
      name: "VARIABLE_NAME",
      text: "Defina o nome da variável",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip: "Armazena os dados da mensagem no assinante",
  helpUrl: "",
});
