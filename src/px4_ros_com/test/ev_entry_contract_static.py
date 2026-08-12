#!/usr/bin/env python3
"""Structural static checks for the selected VISION_POSE entry contract."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import shlex
import sys
from typing import Dict, Iterable, List, Sequence, Tuple


SELECTED_NODE_IDENTITY = (
    "px4_ros_com",
    "fastlio_mavros_vision_bridge",
    "fastlio_mavros_vision_bridge",
)
PHYSICAL_BODY_TF_IDENTITY = (
    "tf2_ros",
    "static_transform_publisher",
    "base_link_to_body",
)
ADAPTER_DEFAULTS = {
    "start_mavros_vision_bridge": "true",
}
STACK_ADAPTER_VALUES = {
    "start_mavros_vision_bridge": "true",
}
# EV writers that must NOT reappear in any launch file. The former
# start_bridge / start_px4_ev_bridge adapters were deleted at the source, so the
# contract now enforces absence instead of a "false" default -- an argument that
# does not exist cannot be flipped to true by a second launch.
FORBIDDEN_ADAPTERS = (
    "start_bridge",
    "start_px4_ev_bridge",
)
STACK_LEVER_ARM_VALUES = {
    "body_to_sensor_x_m": "${MID360_BODY_TO_SENSOR_X_M}",
    "body_to_sensor_y_m": "${MID360_BODY_TO_SENSOR_Y_M}",
    "body_to_sensor_z_m": "${MID360_BODY_TO_SENSOR_Z_M}",
    "body_to_fastlio_yaw_rad": "${MID360_BODY_TO_FASTLIO_YAW_RAD}",
}
STACK_WORLD_YAW_VALUES = {
    # The fixed sensor installation is handled by body_to_fastlio_yaw_rad;
    # one world-frame alignment drives both position and attitude.
    "world_yaw_alignment_rad": "${world_yaw_alignment_rad}",
}
# Any node that can write EV data into the flight controller. Only the first one
# still exists; the other two are kept listed on purpose so that reintroducing
# either is detected as a second EV writer rather than silently allowed.
EV_EXECUTABLES = {
    "fastlio_mavros_vision_bridge",
    "fastlio_vehicle_visual_odometry",
    "fastlio_mavros_odometry_bridge",
}
DELETED_EV_EXECUTABLES = {
    "fastlio_vehicle_visual_odometry",
    "fastlio_mavros_odometry_bridge",
}


class ContractError(RuntimeError):
    """Raised when a selected-entry contract check fails closed."""


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ContractError(f"cannot read {path}: {exc}") from exc


def _call_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def _keyword(call: ast.Call, name: str) -> ast.AST:
    matches = [item.value for item in call.keywords if item.arg == name]
    if len(matches) != 1:
        raise ContractError(f"expected exactly one {name!r} keyword")
    return matches[0]


def _string(node: ast.AST, description: str) -> str:
    if not isinstance(node, ast.Constant) or type(node.value) is not str:
        raise ContractError(f"{description} must be a literal string")
    return node.value


def _node_identity(call: ast.Call) -> Tuple[str, str, str]:
    return (
        _string(_keyword(call, "package"), "Node package"),
        _string(_keyword(call, "executable"), "Node executable"),
        _string(_keyword(call, "name"), "Node name"),
    )


def _generate_function(tree: ast.Module) -> ast.FunctionDef:
    matches = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    ]
    if len(matches) != 1:
        raise ContractError("expected one generate_launch_description function")
    function = matches[0]
    arguments = function.args
    if function.decorator_list:
        raise ContractError("generate_launch_description must be undecorated")
    if (
        arguments.posonlyargs
        or arguments.args
        or arguments.vararg is not None
        or arguments.kwonlyargs
        or arguments.kwarg is not None
        or arguments.defaults
        or arguments.kw_defaults
    ):
        raise ContractError("generate_launch_description must have no parameters")
    return function


class _ScopeBindingVisitor(ast.NodeVisitor):
    """Find bindings of one name without entering child lexical scopes."""

    def __init__(self, target: str) -> None:
        self.target = target
        self.bindings: List[ast.AST] = []
        self.has_star_import = False

    def _bind(self, name: str | None, node: ast.AST) -> None:
        if name == self.target:
            self.bindings.append(node)

    def visit_Name(self, node: ast.Name) -> None:
        if isinstance(node.ctx, (ast.Store, ast.Del)):
            self._bind(node.id, node)

    def _visit_function_definition(self, node: ast.FunctionDef) -> None:
        self._bind(node.name, node)
        for decorator in node.decorator_list:
            self.visit(decorator)
        for argument in (
            list(node.args.posonlyargs)
            + list(node.args.args)
            + list(node.args.kwonlyargs)
        ):
            if argument.annotation is not None:
                self.visit(argument.annotation)
        for argument in (node.args.vararg, node.args.kwarg):
            if argument is not None and argument.annotation is not None:
                self.visit(argument.annotation)
        for default in list(node.args.defaults) + [
            value for value in node.args.kw_defaults if value is not None
        ]:
            self.visit(default)
        if node.returns is not None:
            self.visit(node.returns)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._visit_function_definition(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._visit_function_definition(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        self._bind(node.name, node)
        for decorator in node.decorator_list:
            self.visit(decorator)
        for base in node.bases:
            self.visit(base)
        for keyword in node.keywords:
            self.visit(keyword.value)

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            self._bind(alias.asname or alias.name.split(".")[0], node)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        for alias in node.names:
            if alias.name == "*":
                self.has_star_import = True
            else:
                self._bind(alias.asname or alias.name, node)

    def visit_Global(self, node: ast.Global) -> None:
        if self.target in node.names:
            self.bindings.append(node)

    def visit_Nonlocal(self, node: ast.Nonlocal) -> None:
        if self.target in node.names:
            self.bindings.append(node)

    def visit_ExceptHandler(self, node: ast.ExceptHandler) -> None:
        self._bind(node.name, node)
        if node.type is not None:
            self.visit(node.type)
        for statement in node.body:
            self.visit(statement)

    def visit_MatchAs(self, node: ast.MatchAs) -> None:
        self._bind(node.name, node)
        if node.pattern is not None:
            self.visit(node.pattern)

    def visit_MatchStar(self, node: ast.MatchStar) -> None:
        self._bind(node.name, node)

    def visit_MatchMapping(self, node: ast.MatchMapping) -> None:
        self._bind(node.rest, node)
        for key in node.keys:
            self.visit(key)
        for pattern in node.patterns:
            self.visit(pattern)

    def _visit_comprehension_parts(
        self, generators: Sequence[ast.comprehension], values: Sequence[ast.AST]
    ) -> None:
        for generator in generators:
            self.visit(generator.iter)
            for condition in generator.ifs:
                self.visit(condition)
        for value in values:
            self.visit(value)

    def visit_ListComp(self, node: ast.ListComp) -> None:
        self._visit_comprehension_parts(node.generators, [node.elt])

    def visit_SetComp(self, node: ast.SetComp) -> None:
        self._visit_comprehension_parts(node.generators, [node.elt])

    def visit_DictComp(self, node: ast.DictComp) -> None:
        self._visit_comprehension_parts(node.generators, [node.key, node.value])

    def visit_GeneratorExp(self, node: ast.GeneratorExp) -> None:
        self._visit_comprehension_parts(node.generators, [node.elt])


def _scope_bindings(
    statements: Sequence[ast.stmt], target: str
) -> Tuple[List[ast.AST], bool]:
    visitor = _ScopeBindingVisitor(target)
    for statement in statements:
        visitor.visit(statement)
    return visitor.bindings, visitor.has_star_import


class _FunctionScopeReturnVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.returns: List[ast.Return] = []

    def visit_Return(self, node: ast.Return) -> None:
        self.returns.append(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        return

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        return

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        return

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return


def _function_scope_returns(function: ast.FunctionDef) -> List[ast.Return]:
    visitor = _FunctionScopeReturnVisitor()
    for statement in function.body:
        visitor.visit(statement)
    return visitor.returns


class _OuterScopeStoreVisitor(ast.NodeVisitor):
    """Count writes in one function scope without entering nested scopes."""

    def __init__(self) -> None:
        self.counts: Dict[str, int] = {}

    def visit_Name(self, node: ast.Name) -> None:
        if isinstance(node.ctx, ast.Store):
            self.counts[node.id] = self.counts.get(node.id, 0) + 1

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        return

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        return

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        return

    def visit_Lambda(self, node: ast.Lambda) -> None:
        return

    def visit_ListComp(self, node: ast.ListComp) -> None:
        return

    def visit_SetComp(self, node: ast.SetComp) -> None:
        return

    def visit_DictComp(self, node: ast.DictComp) -> None:
        return

    def visit_GeneratorExp(self, node: ast.GeneratorExp) -> None:
        return


def _outer_scope_store_counts(function: ast.FunctionDef) -> Dict[str, int]:
    visitor = _OuterScopeStoreVisitor()
    for statement in function.body:
        visitor.visit(statement)
    return visitor.counts


def _direct_assignments(
    function: ast.FunctionDef,
) -> Dict[str, List[Tuple[int, ast.AST]]]:
    assignments: Dict[str, List[Tuple[int, ast.AST]]] = {}
    for position, statement in enumerate(function.body):
        targets: List[ast.AST] = []
        value = None
        if isinstance(statement, ast.Assign):
            targets = list(statement.targets)
            value = statement.value
        elif isinstance(statement, ast.AnnAssign):
            targets = [statement.target]
            value = statement.value
        if value is None:
            continue
        for target in targets:
            if isinstance(target, ast.Name):
                assignments.setdefault(target.id, []).append((position, value))
    return assignments


def _unique_direct_assignment(
    assignments: Dict[str, List[Tuple[int, ast.AST]]],
    store_counts: Dict[str, int],
    variable: str,
    description: str,
) -> Tuple[int, ast.AST]:
    matches = assignments.get(variable, [])
    if len(matches) != 1 or store_counts.get(variable, 0) != 1:
        raise ContractError(
            f"{description} variable {variable!r} must have exactly one direct assignment"
        )
    return matches[0]


def _resolve_launch_configuration(
    reference: ast.AST,
    expected: str,
    description: str,
    assignments: Dict[str, List[Tuple[int, ast.AST]]],
    store_counts: Dict[str, int],
    node_position: int,
) -> None:
    if not isinstance(reference, ast.Name):
        raise ContractError(f"{description} must reference a LaunchConfiguration variable")
    position, value = _unique_direct_assignment(
        assignments, store_counts, reference.id, description
    )
    if position >= node_position:
        raise ContractError(
            f"{description} LaunchConfiguration must be assigned before selected vision Node"
        )
    if (
        not isinstance(value, ast.Call)
        or _call_name(value.func) != "LaunchConfiguration"
        or len(value.args) != 1
        or value.keywords
        or _string(value.args[0], f"{description} LaunchConfiguration") != expected
    ):
        raise ContractError(f"{description} must bind LaunchConfiguration({expected!r})")


def _is_ev_node(call: ast.AST) -> bool:
    if not isinstance(call, ast.Call) or _call_name(call.func) != "Node":
        return False
    executable = _string(_keyword(call, "executable"), "Node executable")
    return executable in EV_EXECUTABLES


def _returned_actions(function: ast.FunctionDef) -> List[ast.AST]:
    returns = _function_scope_returns(function)
    if (
        len(returns) != 1
        or not function.body
        or function.body[-1] is not returns[0]
        or not isinstance(returns[0].value, ast.Call)
    ):
        raise ContractError(
            "generate_launch_description must end with its only Return"
        )
    launch_description = returns[0].value
    if _call_name(launch_description.func) != "LaunchDescription":
        raise ContractError("return value is not LaunchDescription")
    if len(launch_description.args) != 1 or not isinstance(
        launch_description.args[0], ast.List
    ):
        raise ContractError("LaunchDescription actions must be a literal list")
    return list(launch_description.args[0].elts)


def _launch_argument_defaults(actions: Iterable[ast.AST]) -> Dict[str, str]:
    values: Dict[str, List[str]] = {}
    for action in actions:
        if not isinstance(action, ast.Call) or _call_name(action.func) != "DeclareLaunchArgument":
            continue
        if len(action.args) != 1:
            raise ContractError("DeclareLaunchArgument name must be one literal argument")
        name = _string(action.args[0], "launch argument name")
        default = _string(_keyword(action, "default_value"), f"default for {name}")
        values.setdefault(name, []).append(default)

    # Reject adapters that select a deleted EV writer before anything else, so a
    # reintroduced argument fails closed even though it is not in ADAPTER_DEFAULTS.
    for name in FORBIDDEN_ADAPTERS:
        if name in values:
            raise ContractError(
                f"launch argument {name!r} selects a deleted EV writer and must "
                "not be declared"
            )

    resolved: Dict[str, str] = {}
    for name, expected in ADAPTER_DEFAULTS.items():
        observed = values.get(name, [])
        if observed != [expected]:
            raise ContractError(
                f"launch default {name} must be uniquely {expected!r}; got {observed!r}"
            )
        resolved[name] = observed[0]
    return resolved


def _parameter_entries(call: ast.Call) -> Dict[str, List[ast.AST]]:
    parameters = _keyword(call, "parameters")
    if not isinstance(parameters, ast.List):
        raise ContractError("selected vision Node parameters must be a literal list")
    entries: Dict[str, List[ast.AST]] = {}
    for item in parameters.elts:
        if not isinstance(item, ast.Dict):
            raise ContractError("selected vision Node parameters must contain literal dicts")
        for key, value in zip(item.keys, item.values):
            if key is None:
                raise ContractError("parameter dict expansion is not allowed")
            parameter_name = _string(key, "parameter name")
            entries.setdefault(parameter_name, []).append(value)
    return entries


def _one_parameter(entries: Dict[str, List[ast.AST]], name: str) -> ast.AST:
    matches = entries.get(name, [])
    if len(matches) != 1:
        raise ContractError(f"selected vision Node must define {name!r} exactly once")
    return matches[0]


def _validate_physical_body_tf(
    assignments: Dict[str, List[Tuple[int, ast.AST]]],
    store_counts: Dict[str, int],
) -> None:
    matches = [
        (variable, position, value)
        for variable, bindings in assignments.items()
        for position, value in bindings
        if isinstance(value, ast.Call)
        and _call_name(value.func) == "Node"
        and _node_identity(value) == PHYSICAL_BODY_TF_IDENTITY
    ]
    if len(matches) != 1:
        raise ContractError(
            "expected exactly one physical base_link-to-FAST-LIO body TF"
        )
    variable, position, call = matches[0]
    _, unique_value = _unique_direct_assignment(
        assignments, store_counts, variable, "physical body TF"
    )
    if unique_value is not call:
        raise ContractError("physical body TF binding is ambiguous")

    arguments = _keyword(call, "arguments")
    if not isinstance(arguments, ast.List):
        raise ContractError("physical body TF arguments must be a literal list")
    values = list(arguments.elts)
    expected_count = 16
    if len(values) != expected_count:
        raise ContractError(
            f"physical body TF must have {expected_count} argument entries"
        )

    expected_flags = (
        "--x", "--y", "--z", "--roll", "--pitch", "--yaw",
        "--frame-id", "--child-frame-id",
    )
    observed_flags = tuple(
        _string(values[index], "physical body TF flag")
        for index in range(0, expected_count, 2)
    )
    if observed_flags != expected_flags:
        raise ContractError(
            f"physical body TF flags are invalid: {observed_flags!r}"
        )

    for index, name in ((1, "body_to_sensor_x_m"),
                        (3, "body_to_sensor_y_m"),
                        (5, "body_to_sensor_z_m")):
        _resolve_launch_configuration(
            values[index], name, f"physical body TF {name}",
            assignments, store_counts, position,
        )

    if _string(values[7], "physical body TF roll") != "0" or \
            _string(values[9], "physical body TF pitch") != "0":
        raise ContractError("physical body TF roll and pitch must be zero")

    yaw = values[11]
    if (
        not isinstance(yaw, ast.Call)
        or _call_name(yaw.func) != "PythonExpression"
        or len(yaw.args) != 1
        or yaw.keywords
        or not isinstance(yaw.args[0], ast.List)
        or len(yaw.args[0].elts) != 3
        or _string(yaw.args[0].elts[0], "physical body TF yaw prefix")
        != "-1.0 * ("
        or _string(yaw.args[0].elts[2], "physical body TF yaw suffix") != ")"
    ):
        raise ContractError(
            "physical body TF yaw must negate body_to_fastlio_yaw_rad"
        )
    _resolve_launch_configuration(
        yaw.args[0].elts[1],
        "body_to_fastlio_yaw_rad",
        "physical body TF inverse installation yaw",
        assignments,
        store_counts,
        position,
    )
    if (
        _string(values[13], "physical body TF frame") != "base_link"
        or _string(values[15], "physical body TF child frame") != "body"
    ):
        raise ContractError("physical body TF must be base_link -> body")




def validate_launch(path: Path) -> Dict[str, str]:
    source = _read_text(path)
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as exc:
        raise ContractError(f"launch AST parse failed: {exc}") from exc

    function = _generate_function(tree)
    assignments = _direct_assignments(function)
    store_counts = _outer_scope_store_counts(function)
    actions = _returned_actions(function)

    selected = [
        (variable, position, value)
        for variable, bindings in assignments.items()
        for position, value in bindings
        if isinstance(value, ast.Call)
        and _call_name(value.func) == "Node"
        and _node_identity(value) == SELECTED_NODE_IDENTITY
    ]
    if len(selected) != 1:
        raise ContractError(
            "expected exactly one selected vision Node with identity "
            + "/".join(SELECTED_NODE_IDENTITY)
        )
    selected_variable, selected_position, selected_call = selected[0]
    _, unique_selected_value = _unique_direct_assignment(
        assignments, store_counts, selected_variable, "selected vision Node"
    )
    if unique_selected_value is not selected_call:
        raise ContractError("selected vision Node binding is ambiguous")

    returned_names = [item.id for item in actions if isinstance(item, ast.Name)]
    if returned_names.count(selected_variable) != 1:
        raise ContractError("selected vision Node is not returned exactly once")

    condition = _keyword(selected_call, "condition")
    if (
        not isinstance(condition, ast.Call)
        or _call_name(condition.func) != "IfCondition"
        or len(condition.args) != 1
        or condition.keywords
    ):
        raise ContractError(
            "selected vision Node condition must bind start_mavros_vision_bridge"
        )
    _resolve_launch_configuration(
        condition.args[0],
        "start_mavros_vision_bridge",
        "selected vision Node condition",
        assignments,
        store_counts,
        selected_position,
    )

    parameters = _parameter_entries(selected_call)
    input_topic = _one_parameter(parameters, "input_topic")
    _resolve_launch_configuration(
        input_topic,
        "healthy_odom_topic",
        "selected vision Node input_topic",
        assignments,
        store_counts,
        selected_position,
    )
    restamp = _one_parameter(parameters, "restamp_message")
    if not (
        isinstance(restamp, ast.Constant)
        and type(restamp.value) is bool
        and restamp.value is False
    ):
        raise ContractError("selected vision Node restamp_message must be bool False")
    for parameter_name in STACK_LEVER_ARM_VALUES:
        _resolve_launch_configuration(
            _one_parameter(parameters, parameter_name),
            parameter_name,
            f"selected vision Node {parameter_name}",
            assignments,
            store_counts,
            selected_position,
        )
    _resolve_launch_configuration(
        _one_parameter(parameters, "world_yaw_alignment_rad"),
        "world_yaw_alignment_rad",
        "selected vision Node unified world yaw",
        assignments,
        store_counts,
        selected_position,
    )
    _validate_physical_body_tf(assignments, store_counts)

    defaults = _launch_argument_defaults(actions)
    healthy_defaults: List[str] = []
    for action in actions:
        if not isinstance(action, ast.Call) or _call_name(action.func) != "DeclareLaunchArgument":
            continue
        if len(action.args) == 1 and _string(action.args[0], "launch argument name") == "healthy_odom_topic":
            healthy_defaults.append(
                _string(_keyword(action, "default_value"), "healthy_odom_topic default")
            )
    if healthy_defaults != ["/Odometry/healthy"]:
        raise ContractError(
            "healthy_odom_topic default must be uniquely '/Odometry/healthy'; "
            f"got {healthy_defaults!r}"
        )

    # Single EV writer, enforced structurally.
    #
    # This used to be a runtime OpaqueFunction that counted enabled EV adapters,
    # because three writers coexisted and only launch arguments kept them apart.
    # That check could be bypassed by starting two launches at once. The two
    # extra writers are now deleted at the source, so the contract enforces the
    # stronger property directly: exactly one EV Node may be returned, and the
    # deleted executables must not reappear.
    ev_node_indexes = []
    for index, action in enumerate(actions):
        if _is_ev_node(action):
            ev_node_indexes.append(index)
            continue
        if not isinstance(action, ast.Name):
            continue
        bindings = assignments.get(action.id, [])
        ev_bindings = [value for _, value in bindings if _is_ev_node(value)]
        if not ev_bindings:
            continue
        _unique_direct_assignment(
            assignments, store_counts, action.id, "returned EV Node"
        )
        if len(ev_bindings) != 1:
            raise ContractError(f"returned EV Node {action.id!r} binding is ambiguous")
        ev_node_indexes.append(index)
    if len(ev_node_indexes) != 1:
        raise ContractError(
            "exactly one EV writer Node may be returned; got "
            f"{len(ev_node_indexes)} -- see 坐标转换.md before adding any PX4 EV writer"
        )

    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or _call_name(call.func) != "Node":
            continue
        for keyword in call.keywords:
            if (
                keyword.arg == "executable"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value in DELETED_EV_EXECUTABLES
            ):
                raise ContractError(
                    f"deleted EV writer {keyword.value.value!r} must not be "
                    "reintroduced; the single EV path is "
                    "fastlio_mavros_vision_bridge -> /mavros/vision_pose/pose_cov"
                )

    identity = ",".join(
        (
            f"package={SELECTED_NODE_IDENTITY[0]}",
            f"executable={SELECTED_NODE_IDENTITY[1]}",
            f"name={SELECTED_NODE_IDENTITY[2]}",
        )
    )
    return {
        "identity": identity,
        "condition": "start_mavros_vision_bridge",
        "input": "/Odometry/healthy",
        "restamp": "false",
        "defaults": ",".join(f"{key}={value}" for key, value in defaults.items()),
        "validation_order": "single_ev_node_in_source",
    }


def _logical_lines(source: str) -> List[str]:
    logical: List[str] = []
    pending = ""
    for physical in source.splitlines():
        stripped = physical.rstrip()
        backslashes = len(stripped) - len(stripped.rstrip("\\"))
        if backslashes % 2 == 1:
            pending += stripped[:-1] + " "
            continue
        logical.append(pending + physical)
        pending = ""
    if pending:
        raise ContractError("shell source ends with an incomplete continuation")
    return logical


def _shell_tokens(source: str, description: str) -> List[str]:
    try:
        lexer = shlex.shlex(source, posix=True, punctuation_chars=";&|")
        lexer.whitespace_split = True
        lexer.commenters = "#"
        return list(lexer)
    except ValueError as exc:
        raise ContractError(f"{description} shell parse failed: {exc}") from exc


def _commands(tokens: Sequence[str]) -> Iterable[List[str]]:
    separators = {";", "&&", "||", "|", "&"}
    current: List[str] = []
    for token in tokens:
        if token in separators:
            if current:
                yield current
                current = []
        else:
            current.append(token)
    if current:
        yield current


def validate_stack(path: Path) -> Dict[str, str]:
    source = _read_text(path)
    matches: List[List[str]] = []
    for line_number, logical in enumerate(_logical_lines(source), start=1):
        if not logical.strip():
            continue
        top_level = _shell_tokens(logical, f"line {line_number}")
        if not top_level or top_level[0] != "open_window":
            continue
        if len(top_level) != 4:
            raise ContractError(
                f"line {line_number} open_window must have title, log name, and one command"
            )
        nested = _shell_tokens(top_level[3], f"open_window line {line_number}")
        for command in _commands(nested):
            if command[:4] == [
                "ros2",
                "launch",
                "px4_ros_com",
                "fastlio_mavros_autofix.launch.py",
            ]:
                matches.append(command)

    if len(matches) != 1:
        raise ContractError(
            "expected exactly one actual ros2 launch px4_ros_com "
            f"fastlio_mavros_autofix.launch.py command; got {len(matches)}"
        )
    command = matches[0]
    for name, expected in STACK_ADAPTER_VALUES.items():
        prefix = f"{name}:="
        values = [token[len(prefix) :] for token in command[4:] if token.startswith(prefix)]
        if values != [expected]:
            raise ContractError(
                f"stack launch token {name} must be uniquely {expected!r}; got {values!r}"
            )
    for name, expected in STACK_LEVER_ARM_VALUES.items():
        prefix = f"{name}:="
        values = [token[len(prefix) :] for token in command[4:] if token.startswith(prefix)]
        if values != [expected]:
            raise ContractError(
                f"stack launch token {name} must bind uniquely to {expected!r}; got {values!r}"
            )
    for name, expected in STACK_WORLD_YAW_VALUES.items():
        prefix = f"{name}:="
        values = [token[len(prefix) :] for token in command[4:] if token.startswith(prefix)]
        if values != [expected]:
            raise ContractError(
                f"stack launch token {name} must remain uniquely {expected!r}; got {values!r}"
            )
    # The launch file no longer declares these arguments, so passing one would
    # abort the launch at startup. Fail closed here instead, and catch the case
    # where a stale token is smuggled in through a nested command.
    for name in FORBIDDEN_ADAPTERS:
        prefix = f"{name}:="
        if any(token.startswith(prefix) for token in command[4:]):
            raise ContractError(
                f"stack launch token {name} selects a deleted EV writer and must "
                "not be passed"
            )
    return {"command": shlex.join(command)}


def validate_contract(launch_path: Path, stack_path: Path) -> Dict[str, str]:
    report = validate_launch(Path(launch_path))
    report.update(validate_stack(Path(stack_path)))
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch", required=True, type=Path)
    parser.add_argument("--stack", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = validate_contract(args.launch, args.stack)
    except ContractError as exc:
        print(f"STRUCTURAL_EV_ENTRY_CONTRACT=FAIL: {exc}", file=sys.stderr)
        return 2

    print("STRUCTURAL_EV_ENTRY_CONTRACT=PASS")
    print("LAUNCH_AST_PARSE_RESULT=PASS")
    print(f"SELECTED_VISION_NODE_IDENTITY={report['identity']}")
    print("VISION_CONDITION_BINDING_RESULT=PASS")
    print("VISION_INPUT_BINDING_RESULT=PASS:/Odometry/healthy")
    print("VISION_RESTAMP_RESULT=PASS:false")
    print("VISION_LEVER_ARM_BINDING_RESULT=PASS:body_FLU_control_center_to_FAST_LIO")
    print("VISION_WORLD_YAW_BINDING_RESULT=PASS:one_alignment_for_position_and_attitude")
    print("PHYSICAL_BODY_TF_RESULT=PASS:base_link_to_FAST_LIO_body")
    print(f"LAUNCH_DEFAULTS_RESULT=PASS:{report['defaults']}")
    print(f"SINGLE_EV_WRITER_RESULT=PASS:{report['validation_order']}")
    print("STACK_COMMAND_TOKEN_BINDING_RESULT=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
