import importlib.metadata

from ._behaviortreepy import ActionNodeBase
from ._behaviortreepy import BehaviorTreeFactory
from ._behaviortreepy import Blackboard  # Core classes
from ._behaviortreepy import ConditionNode
from ._behaviortreepy import ControlNode
from ._behaviortreepy import DecoratorNode
from ._behaviortreepy import FileLogger
from ._behaviortreepy import GROOT2_AVAILABLE  # Groot2 availability flag
from ._behaviortreepy import LeafNode  # Node base classes
from ._behaviortreepy import NodeConfig
from ._behaviortreepy import NodeStatus  # Enums
from ._behaviortreepy import NodeType
from ._behaviortreepy import PortDirection
from ._behaviortreepy import StatefulActionNode
from ._behaviortreepy import StdCoutLogger  # Loggers
from ._behaviortreepy import SyncActionNode
from ._behaviortreepy import to_string  # Utility functions
from ._behaviortreepy import Tree
from ._behaviortreepy import TreeNode


__version__ = importlib.metadata.version('behaviortreepy')

__all__ = [
    # Enums
    "NodeStatus",
    "NodeType",
    "PortDirection",

    # Core classes
    "Blackboard",
    "NodeConfig",
    "TreeNode",
    "Tree",
    "BehaviorTreeFactory",

    # Node base classes
    "LeafNode",
    "ActionNodeBase",
    "SyncActionNode",
    "StatefulActionNode",
    "ConditionNode",
    "ControlNode",
    "DecoratorNode",

    # Loggers
    "StdCoutLogger",
    "FileLogger",

    # Utility functions
    "to_string",

    # Groot2 support
    "GROOT2_AVAILABLE",
]

# Conditionally import Groot2Publisher if available
if GROOT2_AVAILABLE:
    from ._behaviortreepy import Groot2Publisher  # noqa: F401
    __all__.append("Groot2Publisher")
