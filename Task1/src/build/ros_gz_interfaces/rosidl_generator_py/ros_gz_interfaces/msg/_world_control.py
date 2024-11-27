# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/WorldControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WorldControl(type):
    """Metaclass of message 'WorldControl'."""

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
                'ros_gz_interfaces.msg.WorldControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__world_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__world_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__world_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__world_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__world_control

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from ros_gz_interfaces.msg import WorldReset
            if WorldReset.__class__._TYPE_SUPPORT is None:
                WorldReset.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MULTI_STEP__DEFAULT': 0,
        }

    @property
    def MULTI_STEP__DEFAULT(cls):
        """Return default value for message field 'multi_step'."""
        return 0


class WorldControl(metaclass=Metaclass_WorldControl):
    """Message class 'WorldControl'."""

    __slots__ = [
        '_pause',
        '_step',
        '_multi_step',
        '_reset',
        '_seed',
        '_run_to_sim_time',
    ]

    _fields_and_field_types = {
        'pause': 'boolean',
        'step': 'boolean',
        'multi_step': 'uint32',
        'reset': 'ros_gz_interfaces/WorldReset',
        'seed': 'uint32',
        'run_to_sim_time': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'WorldReset'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pause = kwargs.get('pause', bool())
        self.step = kwargs.get('step', bool())
        self.multi_step = kwargs.get(
            'multi_step', WorldControl.MULTI_STEP__DEFAULT)
        from ros_gz_interfaces.msg import WorldReset
        self.reset = kwargs.get('reset', WorldReset())
        self.seed = kwargs.get('seed', int())
        from builtin_interfaces.msg import Time
        self.run_to_sim_time = kwargs.get('run_to_sim_time', Time())

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
        if self.pause != other.pause:
            return False
        if self.step != other.step:
            return False
        if self.multi_step != other.multi_step:
            return False
        if self.reset != other.reset:
            return False
        if self.seed != other.seed:
            return False
        if self.run_to_sim_time != other.run_to_sim_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pause(self):
        """Message field 'pause'."""
        return self._pause

    @pause.setter
    def pause(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pause' field must be of type 'bool'"
        self._pause = value

    @builtins.property
    def step(self):
        """Message field 'step'."""
        return self._step

    @step.setter
    def step(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'step' field must be of type 'bool'"
        self._step = value

    @builtins.property
    def multi_step(self):
        """Message field 'multi_step'."""
        return self._multi_step

    @multi_step.setter
    def multi_step(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'multi_step' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'multi_step' field must be an unsigned integer in [0, 4294967295]"
        self._multi_step = value

    @builtins.property
    def reset(self):
        """Message field 'reset'."""
        return self._reset

    @reset.setter
    def reset(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import WorldReset
            assert \
                isinstance(value, WorldReset), \
                "The 'reset' field must be a sub message of type 'WorldReset'"
        self._reset = value

    @builtins.property
    def seed(self):
        """Message field 'seed'."""
        return self._seed

    @seed.setter
    def seed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seed' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'seed' field must be an unsigned integer in [0, 4294967295]"
        self._seed = value

    @builtins.property
    def run_to_sim_time(self):
        """Message field 'run_to_sim_time'."""
        return self._run_to_sim_time

    @run_to_sim_time.setter
    def run_to_sim_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'run_to_sim_time' field must be a sub message of type 'Time'"
        self._run_to_sim_time = value
