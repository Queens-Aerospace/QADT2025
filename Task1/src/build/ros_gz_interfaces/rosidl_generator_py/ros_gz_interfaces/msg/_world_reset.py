# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/WorldReset.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WorldReset(type):
    """Metaclass of message 'WorldReset'."""

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
                'ros_gz_interfaces.msg.WorldReset')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__world_reset
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__world_reset
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__world_reset
            cls._TYPE_SUPPORT = module.type_support_msg__msg__world_reset
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__world_reset

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ALL__DEFAULT': False,
            'TIME_ONLY__DEFAULT': False,
            'MODEL_ONLY__DEFAULT': False,
        }

    @property
    def ALL__DEFAULT(cls):
        """Return default value for message field 'all'."""
        return False

    @property
    def TIME_ONLY__DEFAULT(cls):
        """Return default value for message field 'time_only'."""
        return False

    @property
    def MODEL_ONLY__DEFAULT(cls):
        """Return default value for message field 'model_only'."""
        return False


class WorldReset(metaclass=Metaclass_WorldReset):
    """Message class 'WorldReset'."""

    __slots__ = [
        '_all',
        '_time_only',
        '_model_only',
    ]

    _fields_and_field_types = {
        'all': 'boolean',
        'time_only': 'boolean',
        'model_only': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.all = kwargs.get(
            'all', WorldReset.ALL__DEFAULT)
        self.time_only = kwargs.get(
            'time_only', WorldReset.TIME_ONLY__DEFAULT)
        self.model_only = kwargs.get(
            'model_only', WorldReset.MODEL_ONLY__DEFAULT)

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
        if self.all != other.all:
            return False
        if self.time_only != other.time_only:
            return False
        if self.model_only != other.model_only:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def all(self):  # noqa: A003
        """Message field 'all'."""
        return self._all

    @all.setter  # noqa: A003
    def all(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'all' field must be of type 'bool'"
        self._all = value

    @builtins.property
    def time_only(self):
        """Message field 'time_only'."""
        return self._time_only

    @time_only.setter
    def time_only(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'time_only' field must be of type 'bool'"
        self._time_only = value

    @builtins.property
    def model_only(self):
        """Message field 'model_only'."""
        return self._model_only

    @model_only.setter
    def model_only(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'model_only' field must be of type 'bool'"
        self._model_only = value
