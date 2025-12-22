"""Basic tests for behaviortreepy."""

import pytest

import behaviortreepy as bt


class TestNodeStatus:
    """Test NodeStatus enum."""

    def test_node_status_values(self):
        assert bt.NodeStatus.IDLE is not None
        assert bt.NodeStatus.RUNNING is not None
        assert bt.NodeStatus.SUCCESS is not None
        assert bt.NodeStatus.FAILURE is not None
        assert bt.NodeStatus.SKIPPED is not None

    def test_to_string(self):
        assert bt.to_string(bt.NodeStatus.SUCCESS) == "SUCCESS"
        assert bt.to_string(bt.NodeStatus.FAILURE) == "FAILURE"
        assert bt.to_string(bt.NodeStatus.RUNNING) == "RUNNING"


class TestBlackboard:
    """Test Blackboard functionality."""

    def test_create_blackboard(self):
        bb = bt.Blackboard.create()
        assert bb is not None

    @pytest.mark.skip(reason="BehaviorTree.CPP Any type conversion issue with standalone Blackboard")
    def test_set_get_string(self):
        bb = bt.Blackboard.create()
        bb.set_string("key", "value")
        assert bb.get_string("key") == "value"

    def test_set_get_int(self):
        bb = bt.Blackboard.create()
        bb.set_int("count", 42)
        assert bb.get_int("count") == 42

    def test_set_get_double(self):
        bb = bt.Blackboard.create()
        bb.set_double("rate", 3.14)
        assert abs(bb.get_double("rate") - 3.14) < 0.001

    def test_set_get_bool(self):
        bb = bt.Blackboard.create()
        bb.set_bool("flag", True)
        assert bb.get_bool("flag") is True

    def test_get_keys(self):
        bb = bt.Blackboard.create()
        bb.set_string("a", "1")
        bb.set_int("b", 2)
        keys = bb.get_keys()
        assert "a" in keys
        assert "b" in keys


class TestBehaviorTreeFactory:
    """Test BehaviorTreeFactory."""

    def test_create_factory(self):
        factory = bt.BehaviorTreeFactory()
        assert factory is not None

    def test_register_simple_action(self):
        factory = bt.BehaviorTreeFactory()

        def my_action(node):
            return bt.NodeStatus.SUCCESS

        factory.register_simple_action("MyAction", my_action)

    def test_register_simple_condition(self):
        factory = bt.BehaviorTreeFactory()

        def my_condition(node):
            return bt.NodeStatus.SUCCESS

        factory.register_simple_condition("MyCondition", my_condition)


class TestTreeExecution:
    """Test tree creation and execution."""

    def test_create_and_execute_simple_tree(self):
        factory = bt.BehaviorTreeFactory()

        executed = []

        def action_a(node):
            executed.append("A")
            return bt.NodeStatus.SUCCESS

        def action_b(node):
            executed.append("B")
            return bt.NodeStatus.SUCCESS

        factory.register_simple_action("ActionA", action_a)
        factory.register_simple_action("ActionB", action_b)

        tree_xml = """
        <root BTCPP_format="4" main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <ActionA/>
                    <ActionB/>
                </Sequence>
            </BehaviorTree>
        </root>
        """

        tree = factory.create_tree_from_text(tree_xml)
        status = tree.tick_while_running()

        assert status == bt.NodeStatus.SUCCESS
        assert executed == ["A", "B"]

    def test_fallback_on_failure(self):
        factory = bt.BehaviorTreeFactory()

        executed = []

        def action_fail(node):
            executed.append("FAIL")
            return bt.NodeStatus.FAILURE

        def action_success(node):
            executed.append("SUCCESS")
            return bt.NodeStatus.SUCCESS

        factory.register_simple_action("ActionFail", action_fail)
        factory.register_simple_action("ActionSuccess", action_success)

        tree_xml = """
        <root BTCPP_format="4" main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Fallback>
                    <ActionFail/>
                    <ActionSuccess/>
                </Fallback>
            </BehaviorTree>
        </root>
        """

        tree = factory.create_tree_from_text(tree_xml)
        status = tree.tick_while_running()

        assert status == bt.NodeStatus.SUCCESS
        assert executed == ["FAIL", "SUCCESS"]

    def test_blackboard_in_action(self):
        """Test passing data between actions using Blackboard with int type."""
        factory = bt.BehaviorTreeFactory()

        def writer_action(node):
            bb = node.config().blackboard
            bb.set_int("counter", 42)
            return bt.NodeStatus.SUCCESS

        def reader_action(node):
            bb = node.config().blackboard
            val = bb.get_int("counter")
            assert val == 42
            return bt.NodeStatus.SUCCESS

        factory.register_simple_action("Writer", writer_action)
        factory.register_simple_action("Reader", reader_action)

        tree_xml = """
        <root BTCPP_format="4" main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <Writer/>
                    <Reader/>
                </Sequence>
            </BehaviorTree>
        </root>
        """

        bb = bt.Blackboard.create()
        tree = factory.create_tree_from_text(tree_xml, bb)
        status = tree.tick_while_running()

        assert status == bt.NodeStatus.SUCCESS


class TestGroot2:
    """Test Groot2 support."""

    def test_groot2_available_flag(self):
        # GROOT2_AVAILABLE should be a boolean
        assert isinstance(bt.GROOT2_AVAILABLE, bool)

    @pytest.mark.skip(reason="Groot2Publisher causes test to hang due to ZeroMQ thread")
    def test_groot2_publisher(self):
        import gc
        import time

        factory = bt.BehaviorTreeFactory()
        factory.register_simple_action("Noop", lambda n: bt.NodeStatus.SUCCESS)

        tree_xml = """
        <root BTCPP_format="4" main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Noop/>
            </BehaviorTree>
        </root>
        """

        tree = factory.create_tree_from_text(tree_xml)
        publisher = bt.Groot2Publisher(tree, 1668)  # Use different port
        assert publisher is not None

        # Explicitly cleanup to stop the ZeroMQ server thread
        # Order matters: publisher must be deleted before tree
        del publisher
        del tree
        del factory
        gc.collect()
        # Give ZeroMQ time to clean up sockets
        time.sleep(0.1)
