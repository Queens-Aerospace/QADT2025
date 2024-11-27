# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/EntityFactory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EntityFactory(type):
    """Metaclass of message 'EntityFactory'."""

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
                'ros_gz_interfaces.msg.EntityFactory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__entity_factory
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__entity_factory
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__entity_factory
            cls._TYPE_SUPPORT = module.type_support_msg__msg__entity_factory
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__entity_factory

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ALLOW_RENAMING__DEFAULT': False,
            'RELATIVE_TO__DEFAULT': 'world',
        }

    @property
    def ALLOW_RENAMING__DEFAULT(cls):
        """Return default value for message field 'allow_renaming'."""
        return False

    @property
    def RELATIVE_TO__DEFAULT(cls):
        """Return default value for message field 'relative_to'."""
        return 'world'


class EntityFactory(metaclass=Metaclass_EntityFactory):
    """Message class 'EntityFactory'."""

    __slots__ = [
        '_name',
        '_allow_renaming',
        '_sdf',
        '_sdf_filename',
        '_clone_name',
        '_pose',
        '_relative_to',
    ]

    _fields_and_field_types = {
        'name': 'string',
        'allow_renaming': 'boolean',
        'sdf': 'string',
        'sdf_filename': 'string',
        'clone_name': 'string',
        'pose': 'geometry_msgs/Pose',
        'relative_to': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name = kwargs.get('name', str())
        self.allow_renaming = kwargs.get(
            'allow_renaming', EntityFactory.ALLOW_RENAMING__DEFAULT)
        self.sdf = kwargs.get('sdf', str())
        self.sdf_filename = kwargs.get('sdf_filename', str())
        self.clone_name = kwargs.get('clone_name', str())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        self.relative_to = kwargs.get(
            'relative_to', EntityFactory.RELATIVE_TO__DEFAULT)

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
        if self.name != other.name:
            return False
        if self.allow_renaming != other.allow_renaming:
            return False
        if self.sdf != other.sdf:
            return False
        if self.sdf_filename != other.sdf_filename:
            return False
        if self.clone_name != other.clone_name:
            return False
        if self.pose != other.pose:
            return False
        if self.relative_to != other.relative_to:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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

    @builtins.property
    def allow_renaming(self):
        """Message field 'allow_renaming'."""
        return self._allow_renaming

    @allow_renaming.setter
    def allow_renaming(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'allow_renaming' field must be of type 'bool'"
        self._allow_renaming = value

    @builtins.property
    def sdf(self):
        """Message field 'sdf'."""
        return self._sdf

    @sdf.setter
    def sdf(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sdf' field must be of type 'str'"
        self._sdf = value

    @builtins.property
    def sdf_filename(self):
        """Message field 'sdf_filename'."""
        return self._sdf_filename

    @sdf_filename.setter
    def sdf_filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sdf_filename' field must be of type 'str'"
        self._sdf_filename = value

    @builtins.property
    def clone_name(self):
        """Message field 'clone_name'."""
        return self._clone_name

    @clone_name.setter
    def clone_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'clone_name' field must be of type 'str'"
        self._clone_name = value

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
    def relative_to(self):
        """Message field 'relative_to'."""
        return self._relative_to

    @relative_to.setter
    def relative_to(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'relative_to' field must be of type 'str'"
        self._relative_to = value
