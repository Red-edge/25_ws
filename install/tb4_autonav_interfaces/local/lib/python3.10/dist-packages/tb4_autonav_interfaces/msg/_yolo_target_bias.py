# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_YoloTargetBias(type):
    """Metaclass of message 'YoloTargetBias'."""

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
            module = import_type_support('tb4_autonav_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tb4_autonav_interfaces.msg.YoloTargetBias')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__yolo_target_bias
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__yolo_target_bias
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__yolo_target_bias
            cls._TYPE_SUPPORT = module.type_support_msg__msg__yolo_target_bias
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__yolo_target_bias

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


class YoloTargetBias(metaclass=Metaclass_YoloTargetBias):
    """Message class 'YoloTargetBias'."""

    __slots__ = [
        '_header',
        '_has_target',
        '_type',
        '_distance_m',
        '_u_norm',
        '_v_norm',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'has_target': 'boolean',
        'type': 'string',
        'distance_m': 'float',
        'u_norm': 'float',
        'v_norm': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.has_target = kwargs.get('has_target', bool())
        self.type = kwargs.get('type', str())
        self.distance_m = kwargs.get('distance_m', float())
        self.u_norm = kwargs.get('u_norm', float())
        self.v_norm = kwargs.get('v_norm', float())

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
        if self.has_target != other.has_target:
            return False
        if self.type != other.type:
            return False
        if self.distance_m != other.distance_m:
            return False
        if self.u_norm != other.u_norm:
            return False
        if self.v_norm != other.v_norm:
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
    def has_target(self):
        """Message field 'has_target'."""
        return self._has_target

    @has_target.setter
    def has_target(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_target' field must be of type 'bool'"
        self._has_target = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'type' field must be of type 'str'"
        self._type = value

    @builtins.property
    def distance_m(self):
        """Message field 'distance_m'."""
        return self._distance_m

    @distance_m.setter
    def distance_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_m = value

    @builtins.property
    def u_norm(self):
        """Message field 'u_norm'."""
        return self._u_norm

    @u_norm.setter
    def u_norm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'u_norm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'u_norm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._u_norm = value

    @builtins.property
    def v_norm(self):
        """Message field 'v_norm'."""
        return self._v_norm

    @v_norm.setter
    def v_norm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_norm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'v_norm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._v_norm = value
