# generated from rosidl_generator_py/resource/_idl.py.em
# with input from race_msgs:msg/LocalPathReference.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LocalPathReference(type):
    """Metaclass of message 'LocalPathReference'."""

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
                'race_msgs.msg.LocalPathReference')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__local_path_reference
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__local_path_reference
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__local_path_reference
            cls._TYPE_SUPPORT = module.type_support_msg__msg__local_path_reference
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__local_path_reference

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

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


class LocalPathReference(metaclass=Metaclass_LocalPathReference):
    """Message class 'LocalPathReference'."""

    __slots__ = [
        '_header',
        '_global_path_id',
        '_local_goal_seq',
        '_local_goal',
        '_points',
        '_continuation_points',
        '_arc_length',
        '_minimum_clearance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'global_path_id': 'uint64',
        'local_goal_seq': 'uint64',
        'local_goal': 'geometry_msgs/Point',
        'points': 'sequence<geometry_msgs/Point>',
        'continuation_points': 'sequence<geometry_msgs/Point>',
        'arc_length': 'double',
        'minimum_clearance': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.global_path_id = kwargs.get('global_path_id', int())
        self.local_goal_seq = kwargs.get('local_goal_seq', int())
        from geometry_msgs.msg import Point
        self.local_goal = kwargs.get('local_goal', Point())
        self.points = kwargs.get('points', [])
        self.continuation_points = kwargs.get('continuation_points', [])
        self.arc_length = kwargs.get('arc_length', float())
        self.minimum_clearance = kwargs.get('minimum_clearance', float())

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
        if self.global_path_id != other.global_path_id:
            return False
        if self.local_goal_seq != other.local_goal_seq:
            return False
        if self.local_goal != other.local_goal:
            return False
        if self.points != other.points:
            return False
        if self.continuation_points != other.continuation_points:
            return False
        if self.arc_length != other.arc_length:
            return False
        if self.minimum_clearance != other.minimum_clearance:
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
    def local_goal(self):
        """Message field 'local_goal'."""
        return self._local_goal

    @local_goal.setter
    def local_goal(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'local_goal' field must be a sub message of type 'Point'"
        self._local_goal = value

    @builtins.property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence and each value of type 'Point'"
        self._points = value

    @builtins.property
    def continuation_points(self):
        """Message field 'continuation_points'."""
        return self._continuation_points

    @continuation_points.setter
    def continuation_points(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'continuation_points' field must be a set or sequence and each value of type 'Point'"
        self._continuation_points = value

    @builtins.property
    def arc_length(self):
        """Message field 'arc_length'."""
        return self._arc_length

    @arc_length.setter
    def arc_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'arc_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'arc_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._arc_length = value

    @builtins.property
    def minimum_clearance(self):
        """Message field 'minimum_clearance'."""
        return self._minimum_clearance

    @minimum_clearance.setter
    def minimum_clearance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'minimum_clearance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'minimum_clearance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._minimum_clearance = value
