# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/AuxCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'angle_corrections'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AuxCommand(type):
    """Metaclass of message 'AuxCommand'."""

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
                'quadrotor_msgs.msg.AuxCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__aux_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__aux_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__aux_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__aux_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__aux_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AuxCommand(metaclass=Metaclass_AuxCommand):
    """Message class 'AuxCommand'."""

    __slots__ = [
        '_current_yaw',
        '_kf_correction',
        '_angle_corrections',
        '_enable_motors',
        '_use_external_yaw',
    ]

    _fields_and_field_types = {
        'current_yaw': 'double',
        'kf_correction': 'double',
        'angle_corrections': 'double[2]',
        'enable_motors': 'boolean',
        'use_external_yaw': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current_yaw = kwargs.get('current_yaw', float())
        self.kf_correction = kwargs.get('kf_correction', float())
        if 'angle_corrections' not in kwargs:
            self.angle_corrections = numpy.zeros(2, dtype=numpy.float64)
        else:
            self.angle_corrections = kwargs.get('angle_corrections')
        self.enable_motors = kwargs.get('enable_motors', bool())
        self.use_external_yaw = kwargs.get('use_external_yaw', bool())

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
        if self.current_yaw != other.current_yaw:
            return False
        if self.kf_correction != other.kf_correction:
            return False
        if any(self.angle_corrections != other.angle_corrections):
            return False
        if self.enable_motors != other.enable_motors:
            return False
        if self.use_external_yaw != other.use_external_yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_yaw(self):
        """Message field 'current_yaw'."""
        return self._current_yaw

    @current_yaw.setter
    def current_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'current_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._current_yaw = value

    @builtins.property
    def kf_correction(self):
        """Message field 'kf_correction'."""
        return self._kf_correction

    @kf_correction.setter
    def kf_correction(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kf_correction' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'kf_correction' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._kf_correction = value

    @builtins.property
    def angle_corrections(self):
        """Message field 'angle_corrections'."""
        return self._angle_corrections

    @angle_corrections.setter
    def angle_corrections(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'angle_corrections' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 2, \
                "The 'angle_corrections' numpy.ndarray() must have a size of 2"
            self._angle_corrections = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'angle_corrections' field must be a set or sequence with length 2 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._angle_corrections = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def enable_motors(self):
        """Message field 'enable_motors'."""
        return self._enable_motors

    @enable_motors.setter
    def enable_motors(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enable_motors' field must be of type 'bool'"
        self._enable_motors = value

    @builtins.property
    def use_external_yaw(self):
        """Message field 'use_external_yaw'."""
        return self._use_external_yaw

    @use_external_yaw.setter
    def use_external_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_external_yaw' field must be of type 'bool'"
        self._use_external_yaw = value
