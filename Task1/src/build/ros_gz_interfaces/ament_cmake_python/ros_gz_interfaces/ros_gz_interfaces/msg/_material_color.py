# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/MaterialColor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MaterialColor(type):
    """Metaclass of message 'MaterialColor'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FIRST': 0,
        'ALL': 1,
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
                'ros_gz_interfaces.msg.MaterialColor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__material_color
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__material_color
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__material_color
            cls._TYPE_SUPPORT = module.type_support_msg__msg__material_color
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__material_color

            from ros_gz_interfaces.msg import Entity
            if Entity.__class__._TYPE_SUPPORT is None:
                Entity.__class__.__import_type_support__()

            from std_msgs.msg import ColorRGBA
            if ColorRGBA.__class__._TYPE_SUPPORT is None:
                ColorRGBA.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FIRST': cls.__constants['FIRST'],
            'ALL': cls.__constants['ALL'],
        }

    @property
    def FIRST(self):
        """Message constant 'FIRST'."""
        return Metaclass_MaterialColor.__constants['FIRST']

    @property
    def ALL(self):
        """Message constant 'ALL'."""
        return Metaclass_MaterialColor.__constants['ALL']


class MaterialColor(metaclass=Metaclass_MaterialColor):
    """
    Message class 'MaterialColor'.

    Constants:
      FIRST
      ALL
    """

    __slots__ = [
        '_header',
        '_entity',
        '_ambient',
        '_diffuse',
        '_specular',
        '_emissive',
        '_shininess',
        '_entity_match',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'entity': 'ros_gz_interfaces/Entity',
        'ambient': 'std_msgs/ColorRGBA',
        'diffuse': 'std_msgs/ColorRGBA',
        'specular': 'std_msgs/ColorRGBA',
        'emissive': 'std_msgs/ColorRGBA',
        'shininess': 'double',
        'entity_match': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'Entity'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from ros_gz_interfaces.msg import Entity
        self.entity = kwargs.get('entity', Entity())
        from std_msgs.msg import ColorRGBA
        self.ambient = kwargs.get('ambient', ColorRGBA())
        from std_msgs.msg import ColorRGBA
        self.diffuse = kwargs.get('diffuse', ColorRGBA())
        from std_msgs.msg import ColorRGBA
        self.specular = kwargs.get('specular', ColorRGBA())
        from std_msgs.msg import ColorRGBA
        self.emissive = kwargs.get('emissive', ColorRGBA())
        self.shininess = kwargs.get('shininess', float())
        self.entity_match = kwargs.get('entity_match', int())

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
        if self.entity != other.entity:
            return False
        if self.ambient != other.ambient:
            return False
        if self.diffuse != other.diffuse:
            return False
        if self.specular != other.specular:
            return False
        if self.emissive != other.emissive:
            return False
        if self.shininess != other.shininess:
            return False
        if self.entity_match != other.entity_match:
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
    def entity(self):
        """Message field 'entity'."""
        return self._entity

    @entity.setter
    def entity(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import Entity
            assert \
                isinstance(value, Entity), \
                "The 'entity' field must be a sub message of type 'Entity'"
        self._entity = value

    @builtins.property
    def ambient(self):
        """Message field 'ambient'."""
        return self._ambient

    @ambient.setter
    def ambient(self, value):
        if __debug__:
            from std_msgs.msg import ColorRGBA
            assert \
                isinstance(value, ColorRGBA), \
                "The 'ambient' field must be a sub message of type 'ColorRGBA'"
        self._ambient = value

    @builtins.property
    def diffuse(self):
        """Message field 'diffuse'."""
        return self._diffuse

    @diffuse.setter
    def diffuse(self, value):
        if __debug__:
            from std_msgs.msg import ColorRGBA
            assert \
                isinstance(value, ColorRGBA), \
                "The 'diffuse' field must be a sub message of type 'ColorRGBA'"
        self._diffuse = value

    @builtins.property
    def specular(self):
        """Message field 'specular'."""
        return self._specular

    @specular.setter
    def specular(self, value):
        if __debug__:
            from std_msgs.msg import ColorRGBA
            assert \
                isinstance(value, ColorRGBA), \
                "The 'specular' field must be a sub message of type 'ColorRGBA'"
        self._specular = value

    @builtins.property
    def emissive(self):
        """Message field 'emissive'."""
        return self._emissive

    @emissive.setter
    def emissive(self, value):
        if __debug__:
            from std_msgs.msg import ColorRGBA
            assert \
                isinstance(value, ColorRGBA), \
                "The 'emissive' field must be a sub message of type 'ColorRGBA'"
        self._emissive = value

    @builtins.property
    def shininess(self):
        """Message field 'shininess'."""
        return self._shininess

    @shininess.setter
    def shininess(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shininess' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'shininess' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._shininess = value

    @builtins.property
    def entity_match(self):
        """Message field 'entity_match'."""
        return self._entity_match

    @entity_match.setter
    def entity_match(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'entity_match' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'entity_match' field must be an unsigned integer in [0, 255]"
        self._entity_match = value
