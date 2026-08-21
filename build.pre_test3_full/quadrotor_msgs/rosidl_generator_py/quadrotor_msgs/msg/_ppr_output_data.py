# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/PPROutputData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'pwm'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PPROutputData(type):
    """Metaclass of message 'PPROutputData'."""

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
            module = import_type_support('quadrotor_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'quadrotor_msgs.msg.PPROutputData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ppr_output_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ppr_output_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ppr_output_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ppr_output_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ppr_output_data

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


class PPROutputData(metaclass=Metaclass_PPROutputData):
    """Message class 'PPROutputData'."""

    __slots__ = [
        '_header',
        '_quad_time',
        '_des_thrust',
        '_des_roll',
        '_des_pitch',
        '_des_yaw',
        '_est_roll',
        '_est_pitch',
        '_est_yaw',
        '_est_angvel_x',
        '_est_angvel_y',
        '_est_angvel_z',
        '_est_acc_x',
        '_est_acc_y',
        '_est_acc_z',
        '_pwm',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'quad_time': 'uint16',
        'des_thrust': 'double',
        'des_roll': 'double',
        'des_pitch': 'double',
        'des_yaw': 'double',
        'est_roll': 'double',
        'est_pitch': 'double',
        'est_yaw': 'double',
        'est_angvel_x': 'double',
        'est_angvel_y': 'double',
        'est_angvel_z': 'double',
        'est_acc_x': 'double',
        'est_acc_y': 'double',
        'est_acc_z': 'double',
        'pwm': 'uint16[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint16'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.quad_time = kwargs.get('quad_time', int())
        self.des_thrust = kwargs.get('des_thrust', float())
        self.des_roll = kwargs.get('des_roll', float())
        self.des_pitch = kwargs.get('des_pitch', float())
        self.des_yaw = kwargs.get('des_yaw', float())
        self.est_roll = kwargs.get('est_roll', float())
        self.est_pitch = kwargs.get('est_pitch', float())
        self.est_yaw = kwargs.get('est_yaw', float())
        self.est_angvel_x = kwargs.get('est_angvel_x', float())
        self.est_angvel_y = kwargs.get('est_angvel_y', float())
        self.est_angvel_z = kwargs.get('est_angvel_z', float())
        self.est_acc_x = kwargs.get('est_acc_x', float())
        self.est_acc_y = kwargs.get('est_acc_y', float())
        self.est_acc_z = kwargs.get('est_acc_z', float())
        if 'pwm' not in kwargs:
            self.pwm = numpy.zeros(4, dtype=numpy.uint16)
        else:
            self.pwm = kwargs.get('pwm')

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
        if self.quad_time != other.quad_time:
            return False
        if self.des_thrust != other.des_thrust:
            return False
        if self.des_roll != other.des_roll:
            return False
        if self.des_pitch != other.des_pitch:
            return False
        if self.des_yaw != other.des_yaw:
            return False
        if self.est_roll != other.est_roll:
            return False
        if self.est_pitch != other.est_pitch:
            return False
        if self.est_yaw != other.est_yaw:
            return False
        if self.est_angvel_x != other.est_angvel_x:
            return False
        if self.est_angvel_y != other.est_angvel_y:
            return False
        if self.est_angvel_z != other.est_angvel_z:
            return False
        if self.est_acc_x != other.est_acc_x:
            return False
        if self.est_acc_y != other.est_acc_y:
            return False
        if self.est_acc_z != other.est_acc_z:
            return False
        if any(self.pwm != other.pwm):
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
    def quad_time(self):
        """Message field 'quad_time'."""
        return self._quad_time

    @quad_time.setter
    def quad_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quad_time' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'quad_time' field must be an unsigned integer in [0, 65535]"
        self._quad_time = value

    @builtins.property
    def des_thrust(self):
        """Message field 'des_thrust'."""
        return self._des_thrust

    @des_thrust.setter
    def des_thrust(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_thrust' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_thrust' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_thrust = value

    @builtins.property
    def des_roll(self):
        """Message field 'des_roll'."""
        return self._des_roll

    @des_roll.setter
    def des_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_roll' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_roll' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_roll = value

    @builtins.property
    def des_pitch(self):
        """Message field 'des_pitch'."""
        return self._des_pitch

    @des_pitch.setter
    def des_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_pitch = value

    @builtins.property
    def des_yaw(self):
        """Message field 'des_yaw'."""
        return self._des_yaw

    @des_yaw.setter
    def des_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_yaw = value

    @builtins.property
    def est_roll(self):
        """Message field 'est_roll'."""
        return self._est_roll

    @est_roll.setter
    def est_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_roll' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_roll' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_roll = value

    @builtins.property
    def est_pitch(self):
        """Message field 'est_pitch'."""
        return self._est_pitch

    @est_pitch.setter
    def est_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_pitch = value

    @builtins.property
    def est_yaw(self):
        """Message field 'est_yaw'."""
        return self._est_yaw

    @est_yaw.setter
    def est_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_yaw = value

    @builtins.property
    def est_angvel_x(self):
        """Message field 'est_angvel_x'."""
        return self._est_angvel_x

    @est_angvel_x.setter
    def est_angvel_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_angvel_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_angvel_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_angvel_x = value

    @builtins.property
    def est_angvel_y(self):
        """Message field 'est_angvel_y'."""
        return self._est_angvel_y

    @est_angvel_y.setter
    def est_angvel_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_angvel_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_angvel_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_angvel_y = value

    @builtins.property
    def est_angvel_z(self):
        """Message field 'est_angvel_z'."""
        return self._est_angvel_z

    @est_angvel_z.setter
    def est_angvel_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_angvel_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_angvel_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_angvel_z = value

    @builtins.property
    def est_acc_x(self):
        """Message field 'est_acc_x'."""
        return self._est_acc_x

    @est_acc_x.setter
    def est_acc_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_acc_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_acc_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_acc_x = value

    @builtins.property
    def est_acc_y(self):
        """Message field 'est_acc_y'."""
        return self._est_acc_y

    @est_acc_y.setter
    def est_acc_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_acc_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_acc_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_acc_y = value

    @builtins.property
    def est_acc_z(self):
        """Message field 'est_acc_z'."""
        return self._est_acc_z

    @est_acc_z.setter
    def est_acc_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'est_acc_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'est_acc_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._est_acc_z = value

    @builtins.property
    def pwm(self):
        """Message field 'pwm'."""
        return self._pwm

    @pwm.setter
    def pwm(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint16, \
                "The 'pwm' numpy.ndarray() must have the dtype of 'numpy.uint16'"
            assert value.size == 4, \
                "The 'pwm' numpy.ndarray() must have a size of 4"
            self._pwm = value
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
                 len(value) == 4 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'pwm' field must be a set or sequence with length 4 and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._pwm = numpy.array(value, dtype=numpy.uint16)
