# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/LQRTrajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 's0'
# Member 'ut'
# Member 'sf'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LQRTrajectory(type):
    """Metaclass of message 'LQRTrajectory'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ACTION_ADD': 1,
        'ACTION_ABORT': 2,
        'ACTION_WARN_START': 3,
        'ACTION_WARN_FINAL': 4,
        'ACTION_WARN_IMPOSSIBLE': 5,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('quadrotor_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'quadrotor_msgs.msg.LQRTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lqr_trajectory
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lqr_trajectory
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lqr_trajectory
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lqr_trajectory
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lqr_trajectory

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ACTION_ADD': cls.__constants['ACTION_ADD'],
            'ACTION_ABORT': cls.__constants['ACTION_ABORT'],
            'ACTION_WARN_START': cls.__constants['ACTION_WARN_START'],
            'ACTION_WARN_FINAL': cls.__constants['ACTION_WARN_FINAL'],
            'ACTION_WARN_IMPOSSIBLE': cls.__constants['ACTION_WARN_IMPOSSIBLE'],
        }

    @property
    def ACTION_ADD(self):
        """Message constant 'ACTION_ADD'."""
        return Metaclass_LQRTrajectory.__constants['ACTION_ADD']

    @property
    def ACTION_ABORT(self):
        """Message constant 'ACTION_ABORT'."""
        return Metaclass_LQRTrajectory.__constants['ACTION_ABORT']

    @property
    def ACTION_WARN_START(self):
        """Message constant 'ACTION_WARN_START'."""
        return Metaclass_LQRTrajectory.__constants['ACTION_WARN_START']

    @property
    def ACTION_WARN_FINAL(self):
        """Message constant 'ACTION_WARN_FINAL'."""
        return Metaclass_LQRTrajectory.__constants['ACTION_WARN_FINAL']

    @property
    def ACTION_WARN_IMPOSSIBLE(self):
        """Message constant 'ACTION_WARN_IMPOSSIBLE'."""
        return Metaclass_LQRTrajectory.__constants['ACTION_WARN_IMPOSSIBLE']


class LQRTrajectory(metaclass=Metaclass_LQRTrajectory):
    """
    Message class 'LQRTrajectory'.

    Constants:
      ACTION_ADD
      ACTION_ABORT
      ACTION_WARN_START
      ACTION_WARN_FINAL
      ACTION_WARN_IMPOSSIBLE
    """

    __slots__ = [
        '_header',
        '_trajectory_id',
        '_action',
        '_r',
        '_start_yaw',
        '_final_yaw',
        '_s0',
        '_ut',
        '_sf',
        '_t_f',
        '_debug_info',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'trajectory_id': 'uint32',
        'action': 'uint32',
        'r': 'double',
        'start_yaw': 'double',
        'final_yaw': 'double',
        's0': 'double[6]',
        'ut': 'double[3]',
        'sf': 'double[6]',
        't_f': 'double',
        'debug_info': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.trajectory_id = kwargs.get('trajectory_id', int())
        self.action = kwargs.get('action', int())
        self.r = kwargs.get('r', float())
        self.start_yaw = kwargs.get('start_yaw', float())
        self.final_yaw = kwargs.get('final_yaw', float())
        if 's0' not in kwargs:
            self.s0 = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.s0 = kwargs.get('s0')
        if 'ut' not in kwargs:
            self.ut = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.ut = kwargs.get('ut')
        if 'sf' not in kwargs:
            self.sf = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.sf = kwargs.get('sf')
        self.t_f = kwargs.get('t_f', float())
        self.debug_info = kwargs.get('debug_info', str())

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
        if self.trajectory_id != other.trajectory_id:
            return False
        if self.action != other.action:
            return False
        if self.r != other.r:
            return False
        if self.start_yaw != other.start_yaw:
            return False
        if self.final_yaw != other.final_yaw:
            return False
        if any(self.s0 != other.s0):
            return False
        if any(self.ut != other.ut):
            return False
        if any(self.sf != other.sf):
            return False
        if self.t_f != other.t_f:
            return False
        if self.debug_info != other.debug_info:
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
    def trajectory_id(self):
        """Message field 'trajectory_id'."""
        return self._trajectory_id

    @trajectory_id.setter
    def trajectory_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trajectory_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'trajectory_id' field must be an unsigned integer in [0, 4294967295]"
        self._trajectory_id = value

    @builtins.property
    def action(self):
        """Message field 'action'."""
        return self._action

    @action.setter
    def action(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'action' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'action' field must be an unsigned integer in [0, 4294967295]"
        self._action = value

    @builtins.property
    def r(self):
        """Message field 'r'."""
        return self._r

    @r.setter
    def r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._r = value

    @builtins.property
    def start_yaw(self):
        """Message field 'start_yaw'."""
        return self._start_yaw

    @start_yaw.setter
    def start_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_yaw = value

    @builtins.property
    def final_yaw(self):
        """Message field 'final_yaw'."""
        return self._final_yaw

    @final_yaw.setter
    def final_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_yaw = value

    @builtins.property
    def s0(self):
        """Message field 's0'."""
        return self._s0

    @s0.setter
    def s0(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 's0' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 's0' numpy.ndarray() must have a size of 6"
            self._s0 = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 's0' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._s0 = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def ut(self):
        """Message field 'ut'."""
        return self._ut

    @ut.setter
    def ut(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'ut' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'ut' numpy.ndarray() must have a size of 3"
            self._ut = value
            return
        if __debug__:
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'ut' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._ut = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def sf(self):
        """Message field 'sf'."""
        return self._sf

    @sf.setter
    def sf(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'sf' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 'sf' numpy.ndarray() must have a size of 6"
            self._sf = value
            return
        if __debug__:
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'sf' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._sf = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def t_f(self):
        """Message field 't_f'."""
        return self._t_f

    @t_f.setter
    def t_f(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't_f' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 't_f' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._t_f = value

    @builtins.property
    def debug_info(self):
        """Message field 'debug_info'."""
        return self._debug_info

    @debug_info.setter
    def debug_info(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'debug_info' field must be of type 'str'"
        self._debug_info = value
