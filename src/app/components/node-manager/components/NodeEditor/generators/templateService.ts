import Handlebars from "handlebars";

import add_boolean_value from "../templates/add_boolean_value.hbs";
import add_callback_function from "../templates/add_callback_function.hbs";
import add_casting from "../templates/add_casting.hbs";
import add_class from "../templates/add_class.hbs";
import add_class_inheritance from "../templates/add_class_inheritance.hbs";
import add_from_import from "../templates/add_from_import.hbs";
import add_function from "../templates/add_function.hbs";
import add_import from "../templates/add_import.hbs";
import add_import_as from "../templates/add_import_as.hbs";
import add_information from "../templates/add_information.hbs";
import add_interface from "../templates/add_interface.hbs";
import add_literal from "../templates/add_literal.hbs";
import add_literal_value from "../templates/add_literal_value.hbs";
import add_logger from "../templates/add_logger.hbs";
import add_message from "../templates/add_message.hbs";
import add_numeric_value from "../templates/add_numeric_value.hbs";
import add_pub from "../templates/add_pub.hbs";
import add_python_version from "../templates/add_python_version.hbs";
import add_string_value from "../templates/add_string_value.hbs";
import add_sub from "../templates/add_sub.hbs";
import add_variable from "../templates/add_variable.hbs";
import add_velocity from "../templates/add_velocity.hbs";
import class_init from "../templates/class_init.hbs";
import counter_function from "../templates/counter_function.hbs";
import create_timer from "../templates/create_timer.hbs";
import execute_function_timer from "../templates/execute_function_timer.hbs";
import execute_literal from "../templates/execute_literal.hbs";
import init_message_interface from "../templates/init_message_interface.hbs";
import init_statement from "../templates/init_statement.hbs";
import launch_executable from "../templates/launch_executable.hbs";
import node_init from "../templates/node_init.hbs";
import publish_message from "../templates/publish_message.hbs";
import ros_communication from "../templates/ros_communication.hbs";
import ros_node_template from "../templates/ros_node_template.hbs";
import start_node from "../templates/start_node.hbs";
import subscribe_message from "../templates/subscribe_message.hbs";

Handlebars.registerHelper("eq", (a, b) => a === b);

const templates = {
  add_boolean_value,
  add_callback_function,
  add_casting,
  add_class_inheritance,
  add_class,
  add_from_import,
  add_function,
  add_import_as,
  add_import,
  add_information,
  add_interface,
  add_literal_value,
  add_literal,
  add_logger,
  add_message,
  add_numeric_value,
  add_pub,
  add_python_version,
  add_string_value,
  add_sub,
  add_variable,
  add_velocity,
  class_init,
  counter_function,
  create_timer,
  execute_function_timer,
  execute_literal,
  init_message_interface,
  init_statement,
  launch_executable,
  node_init,
  publish_message,
  ros_communication,
  ros_node_template,
  start_node,
  subscribe_message,
};

export function getTemplate(name: keyof typeof templates) {
  return Handlebars.compile(templates[name]);
}
