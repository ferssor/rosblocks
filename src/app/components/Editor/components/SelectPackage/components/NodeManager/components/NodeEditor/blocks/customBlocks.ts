import * as Blockly from "blockly/core";

export async function defineCustomBlocks() {
  let interfaceOptions: string[][] = [["-", ""]];

  try {
    const result = await window.electronAPI.getInterfaces();
    const fetchedOptions = result.map((ROSInterface) => [
      ROSInterface.name,
      ROSInterface.location,
    ]);

    interfaceOptions = interfaceOptions.concat(fetchedOptions).sort();
  } catch (error) {
    console.error("Failed to fetch ROS2 interfaces:", error);
  }

  const blocks = [
    {
      type: "add_class",
      message0: "Iniciar classe %1",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CLASS_BODY",
        },
      ],
      colour: 13,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Define uma classe em Python.",
      helpUrl: "",
    },
    {
      type: "add_class_inheritance",
      message0: "Iniciar classe %1 herdando da classe %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
        {
          type: "field_input",
          name: "EXTENDS_NAME",
          text: "Defina o nome da classe a ser herdada",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CLASS_BODY",
        },
      ],
      colour: 13,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Define uma classe em Python.",
      helpUrl: "",
    },
    {
      type: "add_import",
      message0: "Importar %1",
      args0: [
        {
          type: "field_input",
          name: "IMPORT_NAME",
          text: "Defina o nome do pacote",
        },
      ],
      colour: 200,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_import_as",
      message0: "Importar %1 como %2",
      args0: [
        {
          type: "field_input",
          name: "IMPORT_NAME",
          text: "Defina o nome do pacote",
        },
        {
          type: "field_input",
          name: "ALIAS_NAME",
          text: "Defina o nome do apelido",
        },
      ],
      colour: 200,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_from_import",
      message0: "A partir de %1 importe %2",
      args0: [
        {
          type: "field_input",
          name: "PACKAGE_NAME",
          text: "Defina o nome do pacote",
        },
        {
          type: "field_input",
          name: "METHOD_NAME",
          text: "Defina o nome do método",
        },
      ],
      colour: 200,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_function",
      message0: "Função %1",
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "FUNCTION_BODY",
        },
      ],
      colour: 321,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione uma função",
      helpUrl: "",
    },
    {
      type: "class_init",
      message0: "Inicializa o construtor da classe %1",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CONSTRUCTOR_BODY",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Adicione o construtor da classe",
      helpUrl: "",
    },
    {
      type: "node_init",
      message0: "Configure o nó %1",
      args0: [
        {
          type: "field_input",
          name: "NODE_NAME",
          text: "Defina o nome do nó",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CONSTRUCTOR_BODY",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Adicione o nó",
      helpUrl: "",
    },
    {
      type: "start_node",
      message0: "Inicialize o nó da classe %1 com a função %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Adicione o nó",
      helpUrl: "",
    },
    {
      type: "init_statement",
      message0: "Inicializa a função %1 no script",
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Adicione o inicializador do script",
      helpUrl: "",
    },
    {
      type: "add_interface",
      message0: "Importe a interface %1",
      args0: [
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Adicione uma interface ROS2",
      helpUrl: "",
    },
    {
      type: "add_pub",
      message0:
        "Adicionar publicador %1 com interface %2 e taxa de atualização %3 Hz",
      args0: [
        {
          type: "field_input",
          name: "PUB_NAME",
          text: "Defina o nome do publicador",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_number",
          name: "REFRESH_RATE",
          value: 10,
          min: 1,
          precision: 1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 230,
      tooltip:
        "Define um publisher ROS com nome, interface e taxa de atualização.",
      helpUrl: "",
    },
    {
      type: "add_variable",
      message0: "Adicione uma variável %1 com o valor %2",
      args0: [
        {
          type: "field_input",
          name: "VARIABLE",
          text: "Defina o nome da variável",
        },
        {
          type: "input_value",
          name: "VALUE",
          text: "Defina o valor da variável",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 410,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "create_timer",
      message0:
        "Criar um temporizador %1 com intervalo %2 e publique na função %3",
      args0: [
        {
          type: "field_input",
          name: "TEMP_NAME",
          text: "Defina o nome do temporizador",
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
      colour: 50,
      tooltip: "Adiciona uma função contadora e publicadora",
      helpUrl: "",
    },
    {
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
      colour: 2,
      tooltip: "Adiciona uma função que publica o contador",
      helpUrl: "",
    },
    {
      type: "add_information",
      message0: "Adicione o texto de log %1",
      args0: [
        {
          type: "field_input",
          name: "LOG_INFO",
          text: "Defina o texto que será exibido",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 300,
      tooltip: "Adiciona um texto de log",
      helpUrl: "",
    },
    {
      type: "add_sub",
      message0:
        "Adicionar assinante no publicador %1 com interface %2, taxa de atualização %3 Hz e armazene na função %4",
      args0: [
        {
          type: "field_input",
          name: "PUB_NAME",
          text: "Defina o nome do publicador",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_number",
          name: "REFRESH_RATE",
          value: 10,
          min: 1,
          precision: 1,
        },
        {
          type: "input_value",
          name: "SUBSCRIBER_COUNTER",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 183,
      tooltip:
        "Define um subscriber ROS2 com nome, interface e taxa de atualização.",
      helpUrl: "",
    },
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);
}
