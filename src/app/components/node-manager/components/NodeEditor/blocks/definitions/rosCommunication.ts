export const rosCommunication = (interfaceOptions: string[][]) => ({
  type: "ros_communication",
  message0: "Configure a variável %1 como %2",
  args0: [
    {
      type: "field_input",
      name: "COMM_NAME",
      text: "identificador",
    },
    {
      type: "field_dropdown",
      name: "COMM_TYPE",
      options: [
        ["Publicador", "publisher"],
        ["Assinante", "subscriber"],
      ],
    },
  ],
  message1: "Use a interface %1 para comunicar com o tópico %2",
  args1: [
    {
      type: "field_dropdown",
      name: "INTERFACE",
      options: interfaceOptions,
    },
    {
      type: "field_input",
      name: "TARGET_NAME",
      text: "/nome_do_topico",
    },
  ],
  message2: "%1 %2",
  args2: [
    {
      type: "field_label",
      name: "CALLBACK_LABEL",
      text: "Envie os dados para a função",
    },
    {
      type: "field_input",
      name: "CALLBACK_NAME",
      text: "nome_da_funcao",
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip:
    "Cria um bloco único para publishers ou subscribers com base no tipo selecionado.",
  helpUrl: "",
});
