# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/SensorNoise.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SensorNoise(type):
    """Metaclass of message 'SensorNoise'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NONE': 0,
        'GAUSSIAN': 2,
        'GAUSSIAN_QUANTIZED': 3,
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
                'ros_gz_interfaces.msg.SensorNoise')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sensor_noise
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sensor_noise
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sensor_noise
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sensor_noise
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sensor_noise

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NONE': cls.__constants['NONE'],
            'GAUSSIAN': cls.__constants['GAUSSIAN'],
            'GAUSSIAN_QUANTIZED': cls.__constants['GAUSSIAN_QUANTIZED'],
        }

    @property
    def NONE(self):
        """Message constant 'NONE'."""
        return Metaclass_SensorNoise.__constants['NONE']

    @property
    def GAUSSIAN(self):
        """Message constant 'GAUSSIAN'."""
        return Metaclass_SensorNoise.__constants['GAUSSIAN']

    @property
    def GAUSSIAN_QUANTIZED(self):
        """Message constant 'GAUSSIAN_QUANTIZED'."""
        return Metaclass_SensorNoise.__constants['GAUSSIAN_QUANTIZED']


class SensorNoise(metaclass=Metaclass_SensorNoise):
    """
    Message class 'SensorNoise'.

    Constants:
      NONE
      GAUSSIAN
      GAUSSIAN_QUANTIZED
    """

    __slots__ = [
        '_header',
        '_type',
        '_mean',
        '_stddev',
        '_bias_mean',
        '_bias_stddev',
        '_precision',
        '_dynamic_bias_stddev',
        '_dynamic_bias_correlation_time',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'type': 'uint8',
        'mean': 'double',
        'stddev': 'double',
        'bias_mean': 'double',
        'bias_stddev': 'double',
        'precision': 'double',
        'dynamic_bias_stddev': 'double',
        'dynamic_bias_correlation_time': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
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
        self.type = kwargs.get('type', int())
        self.mean = kwargs.get('mean', float())
        self.stddev = kwargs.get('stddev', float())
        self.bias_mean = kwargs.get('bias_mean', float())
        self.bias_stddev = kwargs.get('bias_stddev', float())
        self.precision = kwargs.get('precision', float())
        self.dynamic_bias_stddev = kwargs.get('dynamic_bias_stddev', float())
        self.dynamic_bias_correlation_time = kwargs.get('dynamic_bias_correlation_time', float())

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
        if self.type != other.type:
            return False
        if self.mean != other.mean:
            return False
        if self.stddev != other.stddev:
            return False
        if self.bias_mean != other.bias_mean:
            return False
        if self.bias_stddev != other.bias_stddev:
            return False
        if self.precision != other.precision:
            return False
        if self.dynamic_bias_stddev != other.dynamic_bias_stddev:
            return False
        if self.dynamic_bias_correlation_time != other.dynamic_bias_correlation_time:
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
    def mean(self):
        """Message field 'mean'."""
        return self._mean

    @mean.setter
    def mean(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mean' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mean' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mean = value

    @builtins.property
    def stddev(self):
        """Message field 'stddev'."""
        return self._stddev

    @stddev.setter
    def stddev(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'stddev' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'stddev' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._stddev = value

    @builtins.property
    def bias_mean(self):
        """Message field 'bias_mean'."""
        return self._bias_mean

    @bias_mean.setter
    def bias_mean(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_mean' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_mean' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_mean = value

    @builtins.property
    def bias_stddev(self):
        """Message field 'bias_stddev'."""
        return self._bias_stddev

    @bias_stddev.setter
    def bias_stddev(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bias_stddev' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bias_stddev' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bias_stddev = value

    @builtins.property
    def precision(self):
        """Message field 'precision'."""
        return self._precision

    @precision.setter
    def precision(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'precision' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'precision' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._precision = value

    @builtins.property
    def dynamic_bias_stddev(self):
        """Message field 'dynamic_bias_stddev'."""
        return self._dynamic_bias_stddev

    @dynamic_bias_stddev.setter
    def dynamic_bias_stddev(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dynamic_bias_stddev' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dynamic_bias_stddev' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dynamic_bias_stddev = value

    @builtins.property
    def dynamic_bias_correlation_time(self):
        """Message field 'dynamic_bias_correlation_time'."""
        return self._dynamic_bias_correlation_time

    @dynamic_bias_correlation_time.setter
    def dynamic_bias_correlation_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dynamic_bias_correlation_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dynamic_bias_correlation_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dynamic_bias_correlation_time = value
