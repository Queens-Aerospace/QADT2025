# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/Light.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Light(type):
    """Metaclass of message 'Light'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'POINT': 0,
        'SPOT': 1,
        'DIRECTIONAL': 2,
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
                'ros_gz_interfaces.msg.Light')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__light
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__light
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__light
            cls._TYPE_SUPPORT = module.type_support_msg__msg__light
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__light

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

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
            'POINT': cls.__constants['POINT'],
            'SPOT': cls.__constants['SPOT'],
            'DIRECTIONAL': cls.__constants['DIRECTIONAL'],
        }

    @property
    def POINT(self):
        """Message constant 'POINT'."""
        return Metaclass_Light.__constants['POINT']

    @property
    def SPOT(self):
        """Message constant 'SPOT'."""
        return Metaclass_Light.__constants['SPOT']

    @property
    def DIRECTIONAL(self):
        """Message constant 'DIRECTIONAL'."""
        return Metaclass_Light.__constants['DIRECTIONAL']


class Light(metaclass=Metaclass_Light):
    """
    Message class 'Light'.

    Constants:
      POINT
      SPOT
      DIRECTIONAL
    """

    __slots__ = [
        '_header',
        '_name',
        '_type',
        '_pose',
        '_diffuse',
        '_specular',
        '_attenuation_constant',
        '_attenuation_linear',
        '_attenuation_quadratic',
        '_direction',
        '_range',
        '_cast_shadows',
        '_spot_inner_angle',
        '_spot_outer_angle',
        '_spot_falloff',
        '_id',
        '_parent_id',
        '_intensity',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'type': 'uint8',
        'pose': 'geometry_msgs/Pose',
        'diffuse': 'std_msgs/ColorRGBA',
        'specular': 'std_msgs/ColorRGBA',
        'attenuation_constant': 'float',
        'attenuation_linear': 'float',
        'attenuation_quadratic': 'float',
        'direction': 'geometry_msgs/Vector3',
        'range': 'float',
        'cast_shadows': 'boolean',
        'spot_inner_angle': 'float',
        'spot_outer_angle': 'float',
        'spot_falloff': 'float',
        'id': 'uint32',
        'parent_id': 'uint32',
        'intensity': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'ColorRGBA'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        self.type = kwargs.get('type', int())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        from std_msgs.msg import ColorRGBA
        self.diffuse = kwargs.get('diffuse', ColorRGBA())
        from std_msgs.msg import ColorRGBA
        self.specular = kwargs.get('specular', ColorRGBA())
        self.attenuation_constant = kwargs.get('attenuation_constant', float())
        self.attenuation_linear = kwargs.get('attenuation_linear', float())
        self.attenuation_quadratic = kwargs.get('attenuation_quadratic', float())
        from geometry_msgs.msg import Vector3
        self.direction = kwargs.get('direction', Vector3())
        self.range = kwargs.get('range', float())
        self.cast_shadows = kwargs.get('cast_shadows', bool())
        self.spot_inner_angle = kwargs.get('spot_inner_angle', float())
        self.spot_outer_angle = kwargs.get('spot_outer_angle', float())
        self.spot_falloff = kwargs.get('spot_falloff', float())
        self.id = kwargs.get('id', int())
        self.parent_id = kwargs.get('parent_id', int())
        self.intensity = kwargs.get('intensity', float())

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
        if self.type != other.type:
            return False
        if self.pose != other.pose:
            return False
        if self.diffuse != other.diffuse:
            return False
        if self.specular != other.specular:
            return False
        if self.attenuation_constant != other.attenuation_constant:
            return False
        if self.attenuation_linear != other.attenuation_linear:
            return False
        if self.attenuation_quadratic != other.attenuation_quadratic:
            return False
        if self.direction != other.direction:
            return False
        if self.range != other.range:
            return False
        if self.cast_shadows != other.cast_shadows:
            return False
        if self.spot_inner_angle != other.spot_inner_angle:
            return False
        if self.spot_outer_angle != other.spot_outer_angle:
            return False
        if self.spot_falloff != other.spot_falloff:
            return False
        if self.id != other.id:
            return False
        if self.parent_id != other.parent_id:
            return False
        if self.intensity != other.intensity:
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
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type' field must be an unsigned integer in [0, 255]"
        self._type = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'pose' field must be a sub message of type 'Pose'"
        self._pose = value

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
    def attenuation_constant(self):
        """Message field 'attenuation_constant'."""
        return self._attenuation_constant

    @attenuation_constant.setter
    def attenuation_constant(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'attenuation_constant' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'attenuation_constant' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._attenuation_constant = value

    @builtins.property
    def attenuation_linear(self):
        """Message field 'attenuation_linear'."""
        return self._attenuation_linear

    @attenuation_linear.setter
    def attenuation_linear(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'attenuation_linear' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'attenuation_linear' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._attenuation_linear = value

    @builtins.property
    def attenuation_quadratic(self):
        """Message field 'attenuation_quadratic'."""
        return self._attenuation_quadratic

    @attenuation_quadratic.setter
    def attenuation_quadratic(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'attenuation_quadratic' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'attenuation_quadratic' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._attenuation_quadratic = value

    @builtins.property
    def direction(self):
        """Message field 'direction'."""
        return self._direction

    @direction.setter
    def direction(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'direction' field must be a sub message of type 'Vector3'"
        self._direction = value

    @builtins.property  # noqa: A003
    def range(self):  # noqa: A003
        """Message field 'range'."""
        return self._range

    @range.setter  # noqa: A003
    def range(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'range' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._range = value

    @builtins.property
    def cast_shadows(self):
        """Message field 'cast_shadows'."""
        return self._cast_shadows

    @cast_shadows.setter
    def cast_shadows(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cast_shadows' field must be of type 'bool'"
        self._cast_shadows = value

    @builtins.property
    def spot_inner_angle(self):
        """Message field 'spot_inner_angle'."""
        return self._spot_inner_angle

    @spot_inner_angle.setter
    def spot_inner_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'spot_inner_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'spot_inner_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._spot_inner_angle = value

    @builtins.property
    def spot_outer_angle(self):
        """Message field 'spot_outer_angle'."""
        return self._spot_outer_angle

    @spot_outer_angle.setter
    def spot_outer_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'spot_outer_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'spot_outer_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._spot_outer_angle = value

    @builtins.property
    def spot_falloff(self):
        """Message field 'spot_falloff'."""
        return self._spot_falloff

    @spot_falloff.setter
    def spot_falloff(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'spot_falloff' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'spot_falloff' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._spot_falloff = value

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
    def parent_id(self):
        """Message field 'parent_id'."""
        return self._parent_id

    @parent_id.setter
    def parent_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'parent_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'parent_id' field must be an unsigned integer in [0, 4294967295]"
        self._parent_id = value

    @builtins.property
    def intensity(self):
        """Message field 'intensity'."""
        return self._intensity

    @intensity.setter
    def intensity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'intensity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'intensity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._intensity = value
