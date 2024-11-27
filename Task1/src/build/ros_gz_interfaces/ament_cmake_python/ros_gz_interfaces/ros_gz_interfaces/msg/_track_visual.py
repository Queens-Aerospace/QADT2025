# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/TrackVisual.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrackVisual(type):
    """Metaclass of message 'TrackVisual'."""

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
                'ros_gz_interfaces.msg.TrackVisual')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__track_visual
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__track_visual
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__track_visual
            cls._TYPE_SUPPORT = module.type_support_msg__msg__track_visual
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__track_visual

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


class TrackVisual(metaclass=Metaclass_TrackVisual):
    """Message class 'TrackVisual'."""

    __slots__ = [
        '_header',
        '_name',
        '_id',
        '_inherit_orientation',
        '_min_dist',
        '_max_dist',
        '_is_static',
        '_use_model_frame',
        '_xyz',
        '_inherit_yaw',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'id': 'uint32',
        'inherit_orientation': 'boolean',
        'min_dist': 'double',
        'max_dist': 'double',
        'is_static': 'boolean',
        'use_model_frame': 'boolean',
        'xyz': 'geometry_msgs/Vector3',
        'inherit_yaw': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        self.id = kwargs.get('id', int())
        self.inherit_orientation = kwargs.get('inherit_orientation', bool())
        self.min_dist = kwargs.get('min_dist', float())
        self.max_dist = kwargs.get('max_dist', float())
        self.is_static = kwargs.get('is_static', bool())
        self.use_model_frame = kwargs.get('use_model_frame', bool())
        from geometry_msgs.msg import Vector3
        self.xyz = kwargs.get('xyz', Vector3())
        self.inherit_yaw = kwargs.get('inherit_yaw', bool())

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
        if self.name != other.name:
            return False
        if self.id != other.id:
            return False
        if self.inherit_orientation != other.inherit_orientation:
            return False
        if self.min_dist != other.min_dist:
            return False
        if self.max_dist != other.max_dist:
            return False
        if self.is_static != other.is_static:
            return False
        if self.use_model_frame != other.use_model_frame:
            return False
        if self.xyz != other.xyz:
            return False
        if self.inherit_yaw != other.inherit_yaw:
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
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

    @builtins.property
    def inherit_orientation(self):
        """Message field 'inherit_orientation'."""
        return self._inherit_orientation

    @inherit_orientation.setter
    def inherit_orientation(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'inherit_orientation' field must be of type 'bool'"
        self._inherit_orientation = value

    @builtins.property
    def min_dist(self):
        """Message field 'min_dist'."""
        return self._min_dist

    @min_dist.setter
    def min_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_dist' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_dist' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_dist = value

    @builtins.property
    def max_dist(self):
        """Message field 'max_dist'."""
        return self._max_dist

    @max_dist.setter
    def max_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_dist' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_dist' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_dist = value

    @builtins.property
    def is_static(self):
        """Message field 'is_static'."""
        return self._is_static

    @is_static.setter
    def is_static(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_static' field must be of type 'bool'"
        self._is_static = value

    @builtins.property
    def use_model_frame(self):
        """Message field 'use_model_frame'."""
        return self._use_model_frame

    @use_model_frame.setter
    def use_model_frame(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_model_frame' field must be of type 'bool'"
        self._use_model_frame = value

    @builtins.property
    def xyz(self):
        """Message field 'xyz'."""
        return self._xyz

    @xyz.setter
    def xyz(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'xyz' field must be a sub message of type 'Vector3'"
        self._xyz = value

    @builtins.property
    def inherit_yaw(self):
        """Message field 'inherit_yaw'."""
        return self._inherit_yaw

    @inherit_yaw.setter
    def inherit_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'inherit_yaw' field must be of type 'bool'"
        self._inherit_yaw = value
