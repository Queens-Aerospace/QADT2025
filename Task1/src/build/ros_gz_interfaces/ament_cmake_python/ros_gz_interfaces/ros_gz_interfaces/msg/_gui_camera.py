# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/GuiCamera.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GuiCamera(type):
    """Metaclass of message 'GuiCamera'."""

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
                'ros_gz_interfaces.msg.GuiCamera')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gui_camera
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gui_camera
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gui_camera
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gui_camera
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gui_camera

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from ros_gz_interfaces.msg import TrackVisual
            if TrackVisual.__class__._TYPE_SUPPORT is None:
                TrackVisual.__class__.__import_type_support__()

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


class GuiCamera(metaclass=Metaclass_GuiCamera):
    """Message class 'GuiCamera'."""

    __slots__ = [
        '_header',
        '_name',
        '_view_controller',
        '_pose',
        '_track',
        '_projection_type',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'string',
        'view_controller': 'string',
        'pose': 'geometry_msgs/Pose',
        'track': 'ros_gz_interfaces/TrackVisual',
        'projection_type': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'TrackVisual'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', str())
        self.view_controller = kwargs.get('view_controller', str())
        from geometry_msgs.msg import Pose
        self.pose = kwargs.get('pose', Pose())
        from ros_gz_interfaces.msg import TrackVisual
        self.track = kwargs.get('track', TrackVisual())
        self.projection_type = kwargs.get('projection_type', str())

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
        if self.view_controller != other.view_controller:
            return False
        if self.pose != other.pose:
            return False
        if self.track != other.track:
            return False
        if self.projection_type != other.projection_type:
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

    @builtins.property
    def view_controller(self):
        """Message field 'view_controller'."""
        return self._view_controller

    @view_controller.setter
    def view_controller(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'view_controller' field must be of type 'str'"
        self._view_controller = value

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
    def track(self):
        """Message field 'track'."""
        return self._track

    @track.setter
    def track(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import TrackVisual
            assert \
                isinstance(value, TrackVisual), \
                "The 'track' field must be a sub message of type 'TrackVisual'"
        self._track = value

    @builtins.property
    def projection_type(self):
        """Message field 'projection_type'."""
        return self._projection_type

    @projection_type.setter
    def projection_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'projection_type' field must be of type 'str'"
        self._projection_type = value
