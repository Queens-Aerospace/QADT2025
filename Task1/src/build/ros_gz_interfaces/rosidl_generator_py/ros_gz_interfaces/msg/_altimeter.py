# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/Altimeter.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Altimeter(type):
    """Metaclass of message 'Altimeter'."""

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
            module = import_type_support('ros_gz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros_gz_interfaces.msg.Altimeter')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__altimeter
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__altimeter
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__altimeter
            cls._TYPE_SUPPORT = module.type_support_msg__msg__altimeter
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__altimeter

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


class Altimeter(metaclass=Metaclass_Altimeter):
    """Message class 'Altimeter'."""

    __slots__ = [
        '_header',
        '_vertical_position',
        '_vertical_velocity',
        '_vertical_reference',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'vertical_position': 'double',
        'vertical_velocity': 'double',
        'vertical_reference': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
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
        self.vertical_position = kwargs.get('vertical_position', float())
        self.vertical_velocity = kwargs.get('vertical_velocity', float())
        self.vertical_reference = kwargs.get('vertical_reference', float())

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
        if self.vertical_position != other.vertical_position:
            return False
        if self.vertical_velocity != other.vertical_velocity:
            return False
        if self.vertical_reference != other.vertical_reference:
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
    def vertical_position(self):
        """Message field 'vertical_position'."""
        return self._vertical_position

    @vertical_position.setter
    def vertical_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vertical_position' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vertical_position' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vertical_position = value

    @builtins.property
    def vertical_velocity(self):
        """Message field 'vertical_velocity'."""
        return self._vertical_velocity

    @vertical_velocity.setter
    def vertical_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vertical_velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vertical_velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vertical_velocity = value

    @builtins.property
    def vertical_reference(self):
        """Message field 'vertical_reference'."""
        return self._vertical_reference

    @vertical_reference.setter
    def vertical_reference(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vertical_reference' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vertical_reference' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vertical_reference = value
