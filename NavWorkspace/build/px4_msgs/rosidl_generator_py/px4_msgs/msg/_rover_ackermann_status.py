# generated from rosidl_generator_py/resource/_idl.py.em
# with input from px4_msgs:msg/RoverAckermannStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RoverAckermannStatus(type):
    """Metaclass of message 'RoverAckermannStatus'."""

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
            module = import_type_support('px4_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'px4_msgs.msg.RoverAckermannStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rover_ackermann_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rover_ackermann_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rover_ackermann_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rover_ackermann_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rover_ackermann_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RoverAckermannStatus(metaclass=Metaclass_RoverAckermannStatus):
    """Message class 'RoverAckermannStatus'."""

    __slots__ = [
        '_timestamp',
        '_measured_forward_speed',
        '_adjusted_forward_speed_setpoint',
        '_steering_setpoint_normalized',
        '_adjusted_steering_setpoint_normalized',
        '_measured_lateral_acceleration',
        '_pid_throttle_integral',
        '_pid_lat_accel_integral',
    ]

    _fields_and_field_types = {
        'timestamp': 'uint64',
        'measured_forward_speed': 'float',
        'adjusted_forward_speed_setpoint': 'float',
        'steering_setpoint_normalized': 'float',
        'adjusted_steering_setpoint_normalized': 'float',
        'measured_lateral_acceleration': 'float',
        'pid_throttle_integral': 'float',
        'pid_lat_accel_integral': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.timestamp = kwargs.get('timestamp', int())
        self.measured_forward_speed = kwargs.get('measured_forward_speed', float())
        self.adjusted_forward_speed_setpoint = kwargs.get('adjusted_forward_speed_setpoint', float())
        self.steering_setpoint_normalized = kwargs.get('steering_setpoint_normalized', float())
        self.adjusted_steering_setpoint_normalized = kwargs.get('adjusted_steering_setpoint_normalized', float())
        self.measured_lateral_acceleration = kwargs.get('measured_lateral_acceleration', float())
        self.pid_throttle_integral = kwargs.get('pid_throttle_integral', float())
        self.pid_lat_accel_integral = kwargs.get('pid_lat_accel_integral', float())

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
        if self.timestamp != other.timestamp:
            return False
        if self.measured_forward_speed != other.measured_forward_speed:
            return False
        if self.adjusted_forward_speed_setpoint != other.adjusted_forward_speed_setpoint:
            return False
        if self.steering_setpoint_normalized != other.steering_setpoint_normalized:
            return False
        if self.adjusted_steering_setpoint_normalized != other.adjusted_steering_setpoint_normalized:
            return False
        if self.measured_lateral_acceleration != other.measured_lateral_acceleration:
            return False
        if self.pid_throttle_integral != other.pid_throttle_integral:
            return False
        if self.pid_lat_accel_integral != other.pid_lat_accel_integral:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timestamp' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timestamp = value

    @builtins.property
    def measured_forward_speed(self):
        """Message field 'measured_forward_speed'."""
        return self._measured_forward_speed

    @measured_forward_speed.setter
    def measured_forward_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'measured_forward_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'measured_forward_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._measured_forward_speed = value

    @builtins.property
    def adjusted_forward_speed_setpoint(self):
        """Message field 'adjusted_forward_speed_setpoint'."""
        return self._adjusted_forward_speed_setpoint

    @adjusted_forward_speed_setpoint.setter
    def adjusted_forward_speed_setpoint(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'adjusted_forward_speed_setpoint' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'adjusted_forward_speed_setpoint' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._adjusted_forward_speed_setpoint = value

    @builtins.property
    def steering_setpoint_normalized(self):
        """Message field 'steering_setpoint_normalized'."""
        return self._steering_setpoint_normalized

    @steering_setpoint_normalized.setter
    def steering_setpoint_normalized(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_setpoint_normalized' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'steering_setpoint_normalized' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._steering_setpoint_normalized = value

    @builtins.property
    def adjusted_steering_setpoint_normalized(self):
        """Message field 'adjusted_steering_setpoint_normalized'."""
        return self._adjusted_steering_setpoint_normalized

    @adjusted_steering_setpoint_normalized.setter
    def adjusted_steering_setpoint_normalized(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'adjusted_steering_setpoint_normalized' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'adjusted_steering_setpoint_normalized' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._adjusted_steering_setpoint_normalized = value

    @builtins.property
    def measured_lateral_acceleration(self):
        """Message field 'measured_lateral_acceleration'."""
        return self._measured_lateral_acceleration

    @measured_lateral_acceleration.setter
    def measured_lateral_acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'measured_lateral_acceleration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'measured_lateral_acceleration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._measured_lateral_acceleration = value

    @builtins.property
    def pid_throttle_integral(self):
        """Message field 'pid_throttle_integral'."""
        return self._pid_throttle_integral

    @pid_throttle_integral.setter
    def pid_throttle_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_throttle_integral' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pid_throttle_integral' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pid_throttle_integral = value

    @builtins.property
    def pid_lat_accel_integral(self):
        """Message field 'pid_lat_accel_integral'."""
        return self._pid_lat_accel_integral

    @pid_lat_accel_integral.setter
    def pid_lat_accel_integral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_lat_accel_integral' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pid_lat_accel_integral' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pid_lat_accel_integral = value
