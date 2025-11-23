export type RosCommunicationType =
  | "publisher"
  | "subscriber"
  | "service_client"
  | "service_server"
  | "action_client"
  | "action_server";

export type RosCommunicationConfig = {
  showCallback: boolean;
  showRate: boolean;
  rateLabel: string;
  callbackLabel: string;
  usesRateValue: boolean;
  requiresCallback: boolean;
  defaultRate: number;
  callbackParameters?: (interfaceClass: string) => string;
  callbackBodySuffix?: (interfaceClass: string) => string;
};

export const DEFAULT_ROS_COMMUNICATION_CONFIG: RosCommunicationConfig = {
  showCallback: false,
  showRate: true,
  rateLabel: "Taxa",
  callbackLabel: "Callback",
  usesRateValue: true,
  requiresCallback: false,
  defaultRate: 10,
};

export const ROS_COMMUNICATION_CONFIGS: Record<
  RosCommunicationType,
  RosCommunicationConfig
> = {
  publisher: {
    showCallback: false,
    showRate: true,
    rateLabel: "Taxa de publicação (Hz)",
    callbackLabel: "Callback",
    usesRateValue: true,
    requiresCallback: false,
    defaultRate: 10,
  },
  subscriber: {
    showCallback: true,
    showRate: true,
    rateLabel: "Taxa de atualização (Hz)",
    callbackLabel: "Callback do assinante",
    usesRateValue: true,
    requiresCallback: true,
    defaultRate: 10,
    callbackParameters: (interfaceClass: string) =>
      `self, msg: ${interfaceClass}`,
  },
  service_client: {
    showCallback: false,
    showRate: true,
    rateLabel: "Timeout de serviço (s)",
    callbackLabel: "Callback",
    usesRateValue: true,
    requiresCallback: false,
    defaultRate: 5,
  },
  service_server: {
    showCallback: true,
    showRate: false,
    rateLabel: "Timeout",
    callbackLabel: "Callback do serviço",
    usesRateValue: false,
    requiresCallback: true,
    defaultRate: 0,
    callbackParameters: (interfaceClass: string) =>
      `self, request: ${interfaceClass}.Request, response: ${interfaceClass}.Response`,
    callbackBodySuffix: () => "return response\n",
  },
  action_client: {
    showCallback: false,
    showRate: false,
    rateLabel: "Timeout",
    callbackLabel: "Callback",
    usesRateValue: false,
    requiresCallback: false,
    defaultRate: 0,
  },
  action_server: {
    showCallback: true,
    showRate: false,
    rateLabel: "Timeout",
    callbackLabel: "Callback da ação",
    usesRateValue: false,
    requiresCallback: true,
    defaultRate: 0,
    callbackParameters: () => "self, goal_handle",
    callbackBodySuffix: (interfaceClass: string) =>
      `result = ${interfaceClass}.Result()\ngoal_handle.succeed()\nreturn result\n`,
  },
};
