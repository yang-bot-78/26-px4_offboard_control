# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/OutputData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'radio_channel'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OutputData(type):
    """Metaclass of message 'OutputData'."""

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
                'quadrotor_msgs.msg.OutputData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__output_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__output_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__output_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__output_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__output_data

            from geometry_msgs.msg import Quaternion
            if Quaternion.__class__._TYPE_SUPPORT is None:
                Quaternion.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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


class OutputData(metaclass=Metaclass_OutputData):
    """Message class 'OutputData'."""

    __slots__ = [
        '_header',
        '_loop_rate',
        '_voltage',
        '_orientation',
        '_angular_velocity',
        '_linear_acceleration',
        '_pressure_dheight',
        '_pressure_height',
        '_magnetic_field',
        '_radio_channel',
        '_seq',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'loop_rate': 'uint16',
        'voltage': 'double',
        'orientation': 'geometry_msgs/Quaternion',
        'angular_velocity': 'geometry_msgs/Vector3',
        'linear_acceleration': 'geometry_msgs/Vector3',
        'pressure_dheight': 'double',
        'pressure_height': 'double',
        'magnetic_field': 'geometry_msgs/Vector3',
        'radio_channel': 'uint8[8]',
        'seq': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Quaternion'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 8),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.loop_rate = kwargs.get('loop_rate', int())
        self.voltage = kwargs.get('voltage', float())
        from geometry_msgs.msg import Quaternion
        self.orientation = kwargs.get('orientation', Quaternion())
        from geometry_msgs.msg import Vector3
        self.angular_velocity = kwargs.get('angular_velocity', Vector3())
        from geometry_msgs.msg import Vector3
        self.linear_acceleration = kwargs.get('linear_acceleration', Vector3())
        self.pressure_dheight = kwargs.get('pressure_dheight', float())
        self.pressure_height = kwargs.get('pressure_height', float())
        from geometry_msgs.msg import Vector3
        self.magnetic_field = kwargs.get('magnetic_field', Vector3())
        if 'radio_channel' not in kwargs:
            self.radio_channel = numpy.zeros(8, dtype=numpy.uint8)
        else:
            self.radio_channel = kwargs.get('radio_channel')
        self.seq = kwargs.get('seq', int())

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
        if self.loop_rate != other.loop_rate:
            return False
        if self.voltage != other.voltage:
            return False
        if self.orientation != other.orientation:
            return False
        if self.angular_velocity != other.angular_velocity:
            return False
        if self.linear_acceleration != other.linear_acceleration:
            return False
        if self.pressure_dheight != other.pressure_dheight:
            return False
        if self.pressure_height != other.pressure_height:
            return False
        if self.magnetic_field != other.magnetic_field:
            return False
        if any(self.radio_channel != other.radio_channel):
            return False
        if self.seq != other.seq:
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
    def loop_rate(self):
        """Message field 'loop_rate'."""
        return self._loop_rate

    @loop_rate.setter
    def loop_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'loop_rate' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'loop_rate' field must be an unsigned integer in [0, 65535]"
        self._loop_rate = value

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._voltage = value

    @builtins.property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            from geometry_msgs.msg import Quaternion
            assert \
                isinstance(value, Quaternion), \
                "The 'orientation' field must be a sub message of type 'Quaternion'"
        self._orientation = value

    @builtins.property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'angular_velocity' field must be a sub message of type 'Vector3'"
        self._angular_velocity = value

    @builtins.property
    def linear_acceleration(self):
        """Message field 'linear_acceleration'."""
        return self._linear_acceleration

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'linear_acceleration' field must be a sub message of type 'Vector3'"
        self._linear_acceleration = value

    @builtins.property
    def pressure_dheight(self):
        """Message field 'pressure_dheight'."""
        return self._pressure_dheight

    @pressure_dheight.setter
    def pressure_dheight(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pressure_dheight' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pressure_dheight' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pressure_dheight = value

    @builtins.property
    def pressure_height(self):
        """Message field 'pressure_height'."""
        return self._pressure_height

    @pressure_height.setter
    def pressure_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pressure_height' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pressure_height' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pressure_height = value

    @builtins.property
    def magnetic_field(self):
        """Message field 'magnetic_field'."""
        return self._magnetic_field

    @magnetic_field.setter
    def magnetic_field(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'magnetic_field' field must be a sub message of type 'Vector3'"
        self._magnetic_field = value

    @builtins.property
    def radio_channel(self):
        """Message field 'radio_channel'."""
        return self._radio_channel

    @radio_channel.setter
    def radio_channel(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'radio_channel' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 8, \
                "The 'radio_channel' numpy.ndarray() must have a size of 8"
            self._radio_channel = value
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
                 len(value) == 8 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'radio_channel' field must be a set or sequence with length 8 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._radio_channel = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def seq(self):
        """Message field 'seq'."""
        return self._seq

    @seq.setter
    def seq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seq' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'seq' field must be an unsigned integer in [0, 255]"
        self._seq = value
