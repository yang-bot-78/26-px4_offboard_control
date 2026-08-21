# generated from rosidl_generator_py/resource/_idl.py.em
# with input from race_msgs:msg/FlightAltitudeReference.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FlightAltitudeReference(type):
    """Metaclass of message 'FlightAltitudeReference'."""

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
                'race_msgs.msg.FlightAltitudeReference')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__flight_altitude_reference
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__flight_altitude_reference
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__flight_altitude_reference
            cls._TYPE_SUPPORT = module.type_support_msg__msg__flight_altitude_reference
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__flight_altitude_reference

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


class FlightAltitudeReference(metaclass=Metaclass_FlightAltitudeReference):
    """Message class 'FlightAltitudeReference'."""

    __slots__ = [
        '_header',
        '_flight_id',
        '_valid',
        '_target_agl_m',
        '_min_agl_m',
        '_max_agl_m',
        '_ground_z_local_ned',
        '_target_z_local_ned',
        '_ground_z_map',
        '_target_z_map',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'flight_id': 'uint64',
        'valid': 'boolean',
        'target_agl_m': 'double',
        'min_agl_m': 'double',
        'max_agl_m': 'double',
        'ground_z_local_ned': 'double',
        'target_z_local_ned': 'double',
        'ground_z_map': 'double',
        'target_z_map': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.flight_id = kwargs.get('flight_id', int())
        self.valid = kwargs.get('valid', bool())
        self.target_agl_m = kwargs.get('target_agl_m', float())
        self.min_agl_m = kwargs.get('min_agl_m', float())
        self.max_agl_m = kwargs.get('max_agl_m', float())
        self.ground_z_local_ned = kwargs.get('ground_z_local_ned', float())
        self.target_z_local_ned = kwargs.get('target_z_local_ned', float())
        self.ground_z_map = kwargs.get('ground_z_map', float())
        self.target_z_map = kwargs.get('target_z_map', float())

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
        if self.flight_id != other.flight_id:
            return False
        if self.valid != other.valid:
            return False
        if self.target_agl_m != other.target_agl_m:
            return False
        if self.min_agl_m != other.min_agl_m:
            return False
        if self.max_agl_m != other.max_agl_m:
            return False
        if self.ground_z_local_ned != other.ground_z_local_ned:
            return False
        if self.target_z_local_ned != other.target_z_local_ned:
            return False
        if self.ground_z_map != other.ground_z_map:
            return False
        if self.target_z_map != other.target_z_map:
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
    def flight_id(self):
        """Message field 'flight_id'."""
        return self._flight_id

    @flight_id.setter
    def flight_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flight_id' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'flight_id' field must be an unsigned integer in [0, 18446744073709551615]"
        self._flight_id = value

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def target_agl_m(self):
        """Message field 'target_agl_m'."""
        return self._target_agl_m

    @target_agl_m.setter
    def target_agl_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_agl_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_agl_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_agl_m = value

    @builtins.property
    def min_agl_m(self):
        """Message field 'min_agl_m'."""
        return self._min_agl_m

    @min_agl_m.setter
    def min_agl_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_agl_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_agl_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_agl_m = value

    @builtins.property
    def max_agl_m(self):
        """Message field 'max_agl_m'."""
        return self._max_agl_m

    @max_agl_m.setter
    def max_agl_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_agl_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_agl_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_agl_m = value

    @builtins.property
    def ground_z_local_ned(self):
        """Message field 'ground_z_local_ned'."""
        return self._ground_z_local_ned

    @ground_z_local_ned.setter
    def ground_z_local_ned(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ground_z_local_ned' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ground_z_local_ned' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ground_z_local_ned = value

    @builtins.property
    def target_z_local_ned(self):
        """Message field 'target_z_local_ned'."""
        return self._target_z_local_ned

    @target_z_local_ned.setter
    def target_z_local_ned(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_z_local_ned' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_z_local_ned' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_z_local_ned = value

    @builtins.property
    def ground_z_map(self):
        """Message field 'ground_z_map'."""
        return self._ground_z_map

    @ground_z_map.setter
    def ground_z_map(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ground_z_map' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ground_z_map' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ground_z_map = value

    @builtins.property
    def target_z_map(self):
        """Message field 'target_z_map'."""
        return self._target_z_map

    @target_z_map.setter
    def target_z_map(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_z_map' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'target_z_map' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._target_z_map = value
