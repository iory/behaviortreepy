#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <memory>
#include <iostream>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

#ifdef ZMQ_FOUND
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif
#include <behaviortree_cpp/controls/sequence_node.h>
#include <behaviortree_cpp/controls/fallback_node.h>
#include <behaviortree_cpp/controls/parallel_node.h>
#include <behaviortree_cpp/controls/reactive_sequence.h>
#include <behaviortree_cpp/controls/reactive_fallback.h>
#include <behaviortree_cpp/decorators/inverter_node.h>
#include <behaviortree_cpp/decorators/retry_node.h>
#include <behaviortree_cpp/decorators/repeat_node.h>
#include <behaviortree_cpp/decorators/timeout_node.h>
#include <behaviortree_cpp/decorators/delay_node.h>
#include <behaviortree_cpp/decorators/force_success_node.h>
#include <behaviortree_cpp/decorators/force_failure_node.h>
#include <behaviortree_cpp/decorators/keep_running_until_failure_node.h>

namespace py = pybind11;
using namespace BT;

static NodeStatus pyresult_to_status(const py::object& result) {
    // Check if the result is already a NodeStatus enum
    try {
        return result.cast<NodeStatus>();
    } catch (py::cast_error&) {
        // Not a NodeStatus, try other types
    }

    if (py::isinstance<py::str>(result)) {
        std::string status = result.cast<std::string>();
        if (status == "SUCCESS") return NodeStatus::SUCCESS;
        if (status == "FAILURE") return NodeStatus::FAILURE;
        if (status == "RUNNING") return NodeStatus::RUNNING;
        if (status == "SKIPPED") return NodeStatus::SKIPPED;
    } else if (py::isinstance<py::bool_>(result)) {
        // Check bool before int since bool is a subclass of int in Python
        bool st = result.cast<bool>();
        return st ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    } else if (py::isinstance<py::int_>(result)) {
        int st = result.cast<int>();
        return static_cast<NodeStatus>(st);
    }
    return NodeStatus::FAILURE;
}

class PyTreeNode : public TreeNode {
public:
    using TreeNode::TreeNode;

    NodeStatus tick() override {
        PYBIND11_OVERRIDE_PURE(NodeStatus, TreeNode, tick);
    }

    void halt() override {
        PYBIND11_OVERRIDE_PURE(void, TreeNode, halt);
    }

    NodeType type() const override {
        PYBIND11_OVERRIDE_PURE(NodeType, TreeNode, type);
    }
};

class PySyncActionNode : public SyncActionNode {
public:
    PySyncActionNode(const std::string& name, const NodeConfig& config, py::function py_func)
        : SyncActionNode(name, config), py_func_(std::move(py_func)) {}

    NodeStatus tick() override {
        py::gil_scoped_acquire acquire;
        try {
            auto result = py_func_(py::cast(this, py::return_value_policy::reference));
            return pyresult_to_status(result);
        } catch (py::error_already_set& e) {
            std::cerr << "Python exception in action [" << name() << "]: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    static PortsList providedPorts() {
        return {};
    }

private:
    py::function py_func_;
};

class PyConditionNode : public ConditionNode {
public:
    PyConditionNode(const std::string& name, const NodeConfig& config, py::function py_func)
        : ConditionNode(name, config), py_func_(std::move(py_func)) {}

    NodeStatus tick() override {
        py::gil_scoped_acquire acquire;
        try {
            auto result = py_func_(py::cast(this, py::return_value_policy::reference));
            return pyresult_to_status(result);
        } catch (py::error_already_set& e) {
            std::cerr << "Python exception in condition [" << name() << "]: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    static PortsList providedPorts() {
        return {};
    }

private:
    py::function py_func_;
};

class PyStatefulActionNode : public StatefulActionNode {
public:
    PyStatefulActionNode(const std::string& name, const NodeConfig& config,
                         py::function on_start, py::function on_running, py::function on_halted)
        : StatefulActionNode(name, config),
          on_start_(std::move(on_start)),
          on_running_(std::move(on_running)),
          on_halted_(std::move(on_halted)) {}

    NodeStatus onStart() override {
        py::gil_scoped_acquire acquire;
        try {
            auto result = on_start_(py::cast(this, py::return_value_policy::reference));
            return pyresult_to_status(result);
        } catch (py::error_already_set& e) {
            std::cerr << "Python exception in onStart [" << name() << "]: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onRunning() override {
        py::gil_scoped_acquire acquire;
        try {
            auto result = on_running_(py::cast(this, py::return_value_policy::reference));
            return pyresult_to_status(result);
        } catch (py::error_already_set& e) {
            std::cerr << "Python exception in onRunning [" << name() << "]: " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }
    }

    void onHalted() override {
        py::gil_scoped_acquire acquire;
        try {
            on_halted_(py::cast(this, py::return_value_policy::reference));
        } catch (py::error_already_set& e) {
            std::cerr << "Python exception in onHalted [" << name() << "]: " << e.what() << std::endl;
        }
    }

    static PortsList providedPorts() {
        return {};
    }

private:
    py::function on_start_;
    py::function on_running_;
    py::function on_halted_;
};

PYBIND11_MODULE(_behaviortreepy, m) {
    m.doc() = "Python bindings for BehaviorTree.CPP v4";

    // NodeStatus enum
    py::enum_<NodeStatus>(m, "NodeStatus")
        .value("IDLE", NodeStatus::IDLE)
        .value("RUNNING", NodeStatus::RUNNING)
        .value("SUCCESS", NodeStatus::SUCCESS)
        .value("FAILURE", NodeStatus::FAILURE)
        .value("SKIPPED", NodeStatus::SKIPPED)
        .export_values();

    // NodeType enum
    py::enum_<NodeType>(m, "NodeType")
        .value("UNDEFINED", NodeType::UNDEFINED)
        .value("ACTION", NodeType::ACTION)
        .value("CONDITION", NodeType::CONDITION)
        .value("CONTROL", NodeType::CONTROL)
        .value("DECORATOR", NodeType::DECORATOR)
        .value("SUBTREE", NodeType::SUBTREE)
        .export_values();

    // PortDirection enum
    py::enum_<PortDirection>(m, "PortDirection")
        .value("INPUT", PortDirection::INPUT)
        .value("OUTPUT", PortDirection::OUTPUT)
        .value("INOUT", PortDirection::INOUT)
        .export_values();

    // Blackboard
    py::class_<Blackboard, Blackboard::Ptr>(m, "Blackboard")
        .def_static("create", [](py::object parent_bb) {
            if (parent_bb.is_none()) {
                return Blackboard::create();
            }
            return Blackboard::create(parent_bb.cast<Blackboard::Ptr>());
        }, py::arg("parent") = py::none())
        .def("set_string", [](Blackboard& self, const std::string& key, const std::string& value) {
            self.set<std::string>(key, value);
        })
        .def("set_int", [](Blackboard& self, const std::string& key, int value) {
            self.set<int>(key, value);
        })
        .def("set_double", [](Blackboard& self, const std::string& key, double value) {
            self.set<double>(key, value);
        })
        .def("set_bool", [](Blackboard& self, const std::string& key, bool value) {
            self.set<bool>(key, value);
        })
        .def("get_string", [](Blackboard& self, const std::string& key) {
            return self.get<std::string>(key);
        })
        .def("get_int", [](Blackboard& self, const std::string& key) {
            return self.get<int>(key);
        })
        .def("get_double", [](Blackboard& self, const std::string& key) {
            return self.get<double>(key);
        })
        .def("get_bool", [](Blackboard& self, const std::string& key) {
            return self.get<bool>(key);
        })
        .def("get_keys", [](Blackboard& self) {
            auto keys = self.getKeys();
            std::vector<std::string> result;
            for (const auto& k : keys) {
                result.push_back(std::string(k));
            }
            return result;
        })
        .def("unset", &Blackboard::unset);

    // NodeConfig
    py::class_<NodeConfig>(m, "NodeConfig")
        .def(py::init<>())
        .def_readwrite("blackboard", &NodeConfig::blackboard);

    // TreeNode base class
    py::class_<TreeNode, PyTreeNode, std::shared_ptr<TreeNode>>(m, "TreeNode")
        .def("name", &TreeNode::name)
        .def("full_path", &TreeNode::fullPath)
        .def("status", &TreeNode::status)
        .def("type", &TreeNode::type)
        .def("uid", &TreeNode::UID)
        .def("config", py::overload_cast<>(&TreeNode::config, py::const_))
        .def("halt_node", &TreeNode::haltNode)
        .def("is_halted", &TreeNode::isHalted)
        .def("execute_tick", &TreeNode::executeTick)
        .def("get_input_string", [](TreeNode& self, const std::string& key) -> py::object {
            std::string val;
            auto result = self.getInput(key, val);
            if (result) {
                return py::str(val);
            }
            return py::none();
        })
        .def("get_input_int", [](TreeNode& self, const std::string& key) -> py::object {
            int val;
            auto result = self.getInput(key, val);
            if (result) {
                return py::int_(val);
            }
            return py::none();
        })
        .def("get_input_double", [](TreeNode& self, const std::string& key) -> py::object {
            double val;
            auto result = self.getInput(key, val);
            if (result) {
                return py::float_(val);
            }
            return py::none();
        })
        .def("get_input_bool", [](TreeNode& self, const std::string& key) -> py::object {
            bool val;
            auto result = self.getInput(key, val);
            if (result) {
                return py::bool_(val);
            }
            return py::none();
        })
        .def("set_output_string", [](TreeNode& self, const std::string& key, const std::string& value) {
            return self.setOutput(key, value);
        })
        .def("set_output_int", [](TreeNode& self, const std::string& key, int value) {
            return self.setOutput(key, value);
        })
        .def("set_output_double", [](TreeNode& self, const std::string& key, double value) {
            return self.setOutput(key, value);
        })
        .def("set_output_bool", [](TreeNode& self, const std::string& key, bool value) {
            return self.setOutput(key, value);
        });

    // LeafNode
    py::class_<LeafNode, TreeNode, std::shared_ptr<LeafNode>>(m, "LeafNode");

    // ActionNodeBase
    py::class_<ActionNodeBase, LeafNode, std::shared_ptr<ActionNodeBase>>(m, "ActionNodeBase");

    // SyncActionNode
    py::class_<SyncActionNode, ActionNodeBase, std::shared_ptr<SyncActionNode>>(m, "SyncActionNode");

    // StatefulActionNode
    py::class_<StatefulActionNode, ActionNodeBase, std::shared_ptr<StatefulActionNode>>(m, "StatefulActionNode")
        .def("is_halt_requested", &StatefulActionNode::isHaltRequested);

    // ConditionNode
    py::class_<ConditionNode, LeafNode, std::shared_ptr<ConditionNode>>(m, "ConditionNode");

    // ControlNode
    py::class_<ControlNode, TreeNode, std::shared_ptr<ControlNode>>(m, "ControlNode");

    // DecoratorNode
    py::class_<DecoratorNode, TreeNode, std::shared_ptr<DecoratorNode>>(m, "DecoratorNode");

    // Tree
    py::class_<Tree>(m, "Tree")
        .def("tick_once", &Tree::tickOnce, "Tick the tree once (may wake up if a node signals)")
        .def("tick_exactly_once", &Tree::tickExactlyOnce, "Tick the tree exactly once")
        .def("tick_while_running", [](Tree& self, int sleep_ms) {
            return self.tickWhileRunning(std::chrono::milliseconds(sleep_ms));
        }, py::arg("sleep_ms") = 10, "Tick until the tree returns SUCCESS or FAILURE")
        .def("halt_tree", &Tree::haltTree, "Halt all nodes in the tree")
        .def("root_node", &Tree::rootNode, py::return_value_policy::reference, "Get the root node")
        .def("root_blackboard", &Tree::rootBlackboard, "Get the root blackboard")
        .def("sleep", [](Tree& self, int milliseconds) {
            return self.sleep(std::chrono::milliseconds(milliseconds));
        }, py::arg("milliseconds"), "Sleep for a duration (can be interrupted by wake up signal)");

    // BehaviorTreeFactory
    py::class_<BehaviorTreeFactory>(m, "BehaviorTreeFactory")
        .def(py::init<>())
        .def("register_behavior_tree_from_file", &BehaviorTreeFactory::registerBehaviorTreeFromFile,
             py::arg("filename"), "Register a behavior tree from an XML file")
        .def("register_behavior_tree_from_text", &BehaviorTreeFactory::registerBehaviorTreeFromText,
             py::arg("xml_text"), "Register a behavior tree from XML text")
        .def("registered_behavior_trees", &BehaviorTreeFactory::registeredBehaviorTrees,
             "Get list of registered tree IDs")
        .def("create_tree", [](BehaviorTreeFactory& self, const std::string& tree_name, py::object blackboard) {
            if (blackboard.is_none()) {
                return self.createTree(tree_name);
            }
            return self.createTree(tree_name, blackboard.cast<Blackboard::Ptr>());
        }, py::arg("tree_name"), py::arg("blackboard") = py::none(), "Create a tree by name")
        .def("create_tree_from_text", [](BehaviorTreeFactory& self, const std::string& text, py::object blackboard) {
            if (blackboard.is_none()) {
                return self.createTreeFromText(text);
            }
            return self.createTreeFromText(text, blackboard.cast<Blackboard::Ptr>());
        }, py::arg("text"), py::arg("blackboard") = py::none(), "Create a tree from XML text")
        .def("create_tree_from_file", [](BehaviorTreeFactory& self, const std::string& file_path, py::object blackboard) {
            if (blackboard.is_none()) {
                return self.createTreeFromFile(file_path);
            }
            return self.createTreeFromFile(file_path, blackboard.cast<Blackboard::Ptr>());
        }, py::arg("file_path"), py::arg("blackboard") = py::none(), "Create a tree from an XML file")
        .def("register_simple_action", [](BehaviorTreeFactory& factory, const std::string& name, py::function py_func) {
            auto tick_functor = [py_func](TreeNode& self) -> NodeStatus {
                py::gil_scoped_acquire acquire;
                try {
                    auto result = py_func(py::cast(&self, py::return_value_policy::reference));
                    return pyresult_to_status(result);
                } catch (py::error_already_set& e) {
                    std::cerr << "Python exception in action [" << self.name() << "]: " << e.what() << std::endl;
                    return NodeStatus::FAILURE;
                }
            };
            factory.registerSimpleAction(name, tick_functor);
        }, py::arg("name"), py::arg("tick_function"), "Register a simple action with a Python callback")
        .def("register_simple_condition", [](BehaviorTreeFactory& factory, const std::string& name, py::function py_func) {
            auto tick_functor = [py_func](TreeNode& self) -> NodeStatus {
                py::gil_scoped_acquire acquire;
                try {
                    auto result = py_func(py::cast(&self, py::return_value_policy::reference));
                    return pyresult_to_status(result);
                } catch (py::error_already_set& e) {
                    std::cerr << "Python exception in condition [" << self.name() << "]: " << e.what() << std::endl;
                    return NodeStatus::FAILURE;
                }
            };
            factory.registerSimpleCondition(name, tick_functor);
        }, py::arg("name"), py::arg("tick_function"), "Register a simple condition with a Python callback")
        .def("register_simple_decorator", [](BehaviorTreeFactory& factory, const std::string& name, py::function py_func) {
            auto tick_functor = [py_func](NodeStatus child_status, TreeNode& self) -> NodeStatus {
                py::gil_scoped_acquire acquire;
                try {
                    auto result = py_func(child_status, py::cast(&self, py::return_value_policy::reference));
                    return pyresult_to_status(result);
                } catch (py::error_already_set& e) {
                    std::cerr << "Python exception in decorator [" << self.name() << "]: " << e.what() << std::endl;
                    return NodeStatus::FAILURE;
                }
            };
            factory.registerSimpleDecorator(name, tick_functor);
        }, py::arg("name"), py::arg("tick_function"), "Register a simple decorator with a Python callback");

    // StdCoutLogger - must be stored via unique_ptr since it's non-copyable/movable
    py::class_<StdCoutLogger>(m, "StdCoutLogger")
        .def(py::init([](Tree& tree) {
            return new StdCoutLogger(tree);
        }), py::arg("tree"), "Create a console logger for the tree")
        .def("set_enabled", &StdCoutLogger::setEnabled)
        .def("flush", &StdCoutLogger::flush);

    // FileLogger2 - must be stored via unique_ptr since it's non-copyable/movable
    py::class_<FileLogger2>(m, "FileLogger")
        .def(py::init([](Tree& tree, const std::string& filepath) {
            return new FileLogger2(tree, filepath);
        }), py::arg("tree"), py::arg("filepath"), "Create a file logger for the tree")
        .def("set_enabled", &FileLogger2::setEnabled)
        .def("flush", &FileLogger2::flush);

    // Utility function
    m.def("to_string", [](NodeStatus status) {
        return BT::toStr(status);
    }, py::arg("status"), "Convert NodeStatus to string");

#ifdef ZMQ_FOUND
    // Groot2Publisher for live monitoring with Groot2
    py::class_<BT::Groot2Publisher>(m, "Groot2Publisher")
        .def(py::init([](const BT::Tree& tree, unsigned port) {
            return new BT::Groot2Publisher(tree, port);
        }), py::arg("tree"), py::arg("port") = 1667,
        "Create a Groot2 publisher for live monitoring. Default port is 1667.")
        .def("set_enabled", &BT::Groot2Publisher::setEnabled)
        .def("set_max_heartbeat_delay", [](BT::Groot2Publisher& self, int ms) {
            self.setMaxHeartbeatDelay(std::chrono::milliseconds(ms));
        }, py::arg("milliseconds"), "Set max heartbeat delay in milliseconds (default 5000)")
        .def("max_heartbeat_delay", [](BT::Groot2Publisher& self) {
            return self.maxHeartbeatDelay().count();
        }, "Get max heartbeat delay in milliseconds");

    m.attr("GROOT2_AVAILABLE") = true;
#else
    m.attr("GROOT2_AVAILABLE") = false;
#endif
}
