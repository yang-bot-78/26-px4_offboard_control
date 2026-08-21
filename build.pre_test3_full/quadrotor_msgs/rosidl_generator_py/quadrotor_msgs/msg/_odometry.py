# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/Odometry.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Odometry(type):
    """Metaclass of message 'Odometry'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_ODOM_VALID': 0,
        'STATUS_ODOM_INVALID': 1,
        'STATUS_ODOM_LOOPCLOSURE': 2,
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
                'quadrotor_msgs.msg.Odometry')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__odometry
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__odometry
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__odometry
            cls._TYPE_SUPPORT = module.type_support_msg__msg__odometry
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__odometry

            from nav_msgs.msg import Odometry
            if Odometry.__class__._TYPE_SUPPORT is None:
                Odometry.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_ODOM_VALID': cls.__constants['STATUS_ODOM_VALID'],
            'STATUS_ODOM_INVALID': cls.__constants['STATUS_ODOM_INVALID'],
            'STATUS_ODOM_LOOPCLOSURE': cls.__constants['STATUS_ODOM_LOOPCLOSURE'],
        }

    @property
    def STATUS_ODOM_VALID(self):
        """Message constant 'STATUS_ODOM_VALID'."""
        return Metaclass_Odometry.__constants['STATUS_ODOM_VALID']

    @property
    def STATUS_ODOM_INVALID(self):
        """Message constant 'STATUS_ODOM_INVALID'."""
        return Metaclass_Odometry.__constants['STATUS_ODOM_INVALID']

    @property
    def STATUS_ODOM_LOOPCLOSURE(self):
        """Message constant 'STATUS_ODOM_LOOPCLOSURE'."""
        return Metaclass_Odometry.__constants['STATUS_ODOM_LOOPCLOSURE']


class Odometry(metaclass=Metaclass_Odometry):
    """
    Message class 'Odometry'.

    Constants:
      STATUS_ODOM_VALID
      STATUS_ODOM_INVALID
      STATUS_ODOM_LOOPCLOSURE
    """

    __slots__ = [
        '_curodom',
        '_kfodom',
        '_kfid',
        '_status',
    ]

    _fields_and_field_types = {
        'curodom': 'nav_msgs/Odometry',
        'kfodom': 'nav_msgs/Odometry',
        'kfid': 'uint32',
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Odometry'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Odometry'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from nav_msgs.msg import Odometry
        self.curodom = kwargs.get('curodom', Odometry())
        from nav_msgs.msg import Odometry
        self.kfodom = kwargs.get('kfodom', Odometry())
        self.kfid = kwargs.get('kfid', int())
        self.status = kwargs.get('status', int())

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
        if self.curodom != other.curodom:
            return False
        if self.kfodom != other.kfodom:
            return False
        if self.kfid != other.kfid:
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def curodom(self):
        """Message field 'curodom'."""
        return self._curodom

    @curodom.setter
    def curodom(self, value):
        if __debug__:
            from nav_msgs.msg import Odometry
            assert \
                isinstance(value, Odometry), \
                "The 'curodom' field must be a sub message of type 'Odometry'"
        self._curodom = value

    @builtins.property
    def kfodom(self):
        """Message field 'kfodom'."""
        return self._kfodom

    @kfodom.setter
    def kfodom(self, value):
        if __debug__:
            from nav_msgs.msg import Odometry
            assert \
                isinstance(value, Odometry), \
                "The 'kfodom' field must be a sub message of type 'Odometry'"
        self._kfodom = value

    @builtins.property
    def kfid(self):
        """Message field 'kfid'."""
        return self._kfid

    @kfid.setter
    def kfid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kfid' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'kfid' field must be an unsigned integer in [0, 4294967295]"
        self._kfid = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value
