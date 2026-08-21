# generated from rosidl_generator_py/resource/_idl.py.em
# with input from race_msgs:msg/GlobalPlannerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GlobalPlannerStatus(type):
    """Metaclass of message 'GlobalPlannerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('race_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'race_msgs.msg.GlobalPlannerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__global_planner_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__global_planner_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__global_planner_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__global_planner_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__global_planner_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GlobalPlannerStatus(metaclass=Metaclass_GlobalPlannerStatus):
    """Message class 'GlobalPlannerStatus'."""

    __slots__ = [
        '_header',
        '_global_goal_id',
        '_global_path_id',
        '_local_goal_seq',
        '_goal_active',
        '_final_goal_reached',
        '_distance_to_final',
        '_horizontal_speed',
        '_mode',
        '_reason',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'global_goal_id': 'uint64',
        'global_path_id': 'uint64',
        'local_goal_seq': 'uint64',
        'goal_active': 'boolean',
        'final_goal_reached': 'boolean',
        'distance_to_final': 'double',
        'horizontal_speed': 'double',
        'mode': 'string',
        'reason': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.global_goal_id = kwargs.get('global_goal_id', int())
        self.global_path_id = kwargs.get('global_path_id', int())
        self.local_goal_seq = kwargs.get('local_goal_seq', int())
        self.goal_active = kwargs.get('goal_active', bool())
        self.final_goal_reached = kwargs.get('final_goal_reached', bool())
        self.distance_to_final = kwargs.get('distance_to_final', float())
        self.horizontal_speed = kwargs.get('horizontal_speed', float())
        self.mode = kwargs.get('mode', str())
        self.reason = kwargs.get('reason', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.global_goal_id != other.global_goal_id:
            return False
        if self.global_path_id != other.global_path_id:
            return False
        if self.local_goal_seq != other.local_goal_seq:
            return False
        if self.goal_active != other.goal_active:
            return False
        if self.final_goal_reached != other.final_goal_reached:
            return False
        if self.distance_to_final != other.distance_to_final:
            return False
        if self.horizontal_speed != other.horizontal_speed:
            return False
        if self.mode != other.mode:
            return False
        if self.reason != other.reason:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def global_goal_id(self):
        """Message field 'global_goal_id'."""
        return self._global_goal_id

    @global_goal_id.setter
    def global_goal_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'global_goal_id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'global_goal_id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._global_goal_id = value

    @builtins.property
    def global_path_id(self):
        """Message field 'global_path_id'."""
        return self._global_path_id

    @global_path_id.setter
    def global_path_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'global_path_id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'global_path_id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._global_path_id = value

    @builtins.property
    def local_goal_seq(self):
        """Message field 'local_goal_seq'."""
        return self._local_goal_seq

    @local_goal_seq.setter
    def local_goal_seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'local_goal_seq' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'local_goal_seq' field must be an unsigned integer in [0, 18446744073709551615]"
        self._local_goal_seq = value

    @builtins.property
    def goal_active(self):
        """Message field 'goal_active'."""
        return self._goal_active

    @goal_active.setter
    def goal_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'goal_active' field must be of type 'bool'"
        self._goal_active = value

    @builtins.property
    def final_goal_reached(self):
        """Message field 'final_goal_reached'."""
        return self._final_goal_reached

    @final_goal_reached.setter
    def final_goal_reached(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'final_goal_reached' field must be of type 'bool'"
        self._final_goal_reached = value

    @builtins.property
    def distance_to_final(self):
        """Message field 'distance_to_final'."""
        return self._distance_to_final

    @distance_to_final.setter
    def distance_to_final(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_to_final' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance_to_final' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance_to_final = value

    @builtins.property
    def horizontal_speed(self):
        """Message field 'horizontal_speed'."""
        return self._horizontal_speed

    @horizontal_speed.setter
    def horizontal_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'horizontal_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'horizontal_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._horizontal_speed = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'mode' field must be of type 'str'"
        self._mode = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value
