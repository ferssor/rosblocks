export const launchExecutable = (executableOptions: string[][]) => ({
  type: "launch_executable",
  message0: "Execute o %1",
  args0: [
    {
      type: "field_dropdown",
      name: "EXECUTABLE_OPTION",
      options: executableOptions,
    },
  ],
  previousStatement: null,
  nextStatement: null,
  colour: "#1e66f5",
  tooltip:
    "Executa um comando externo (como o turtlesim) durante a execução do script.",
  helpUrl: "",
});
