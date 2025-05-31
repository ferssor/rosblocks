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
      message0: "Adicione à variável %1 o valor %2",
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
      colour: "#2966ff",
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_numeric_value",
      message0: "%1",
      args0: [
        {
          type: "field_number",
          name: "NUMERIC_VALUE",
          text: "Defina o valor da variável",
        },
      ],
      colour: "#9966ff",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_string_value",
      message0: `" %1 "`,
      args0: [
        {
          type: "field_input",
          name: "STRING_VALUE",
        },
      ],
      colour: "#9966ff",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
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
      colour: "#9966ff",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_python_version",
      message0: "Defina a versão %1 do Python",
      args0: [
        {
          type: "field_dropdown",
          options: [
            ["3", "3"],
            ["3.6", "3.6"],
            ["3.7", "3.7"],
            ["3.8", "3.8"],
            ["3.9", "3.9"],
            ["3.10", "3.10"],
          ],
          name: "PYTHON_VERSION",
          checked: true,
        },
      ],
      colour: 200,
      nextStatement: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
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
          type: "field_input",
          name: "LOG_TEXT",
          text: "Defina o texto que será exibido",
        },
      ],
      colour: 200,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione um registro ao nó",
      helpUrl: "",
    },
    {
      type: "add_message",
      message0:
        "Adicione a mensagem com a interface %1 e os dados da variável %2",
      args0: [
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
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
      tooltip: "Adicione uma mensagem ROS2",
      helpUrl: "",
    },
    {
      type: "publish_message",
      message0: "Publique a mensagem no %1",
      args0: [
        {
          type: "field_input",
          name: "PUBLISHER_NAME",
          text: "Defina o nome do publicador",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Publica a mensagem no publicador",
      helpUrl: "",
    },
    {
      type: "subscribe_message",
      message0: "Inscreva a mensagem na %1",
      args0: [
        {
          type: "field_input",
          name: "VARIABLE_NAME",
          text: "Defina o nome da variável",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: 160,
      tooltip: "Armazena os dados da mensagem no assinante",
      helpUrl: "",
    },
    {
      type: "add_callback_function",
      message0: `Adicione a função de retorno %1 com a interface %2`,
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "FUNCTION_BODY",
        },
      ],
      colour: 200,
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione uma função de retorno de chamada",
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
      message0: `Adicionar assinante %1 
      no publicador %2 
      com interface %3, 
      taxa de atualização %4 Hz 
      e armazene os dados na %5`,
      args0: [
        {
          type: "field_input",
          name: "SUB_NAME",
          text: "Defina o nome do assinante",
        },
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
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
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
