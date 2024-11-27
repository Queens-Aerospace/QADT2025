# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/JointWrench.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JointWrench(type):
    """Metaclass of message 'JointWrench'."""

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
                'ros_gz_interfaces.msg.JointWrench')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joint_wrench
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joint_wrench
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joint_wrench
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joint_wrench
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joint_wrench

            from geometry_msgs.msg import Wrench
            if Wrench.__class__._TYPE_SUPPORT is None:
                Wrench.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

            from std_msgs.msg import UInt32
            if UInt32.__class__._TYPE_SUPPORT is None:
                UInt32.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class JointWrench(metaclass=Metaclass_JointWrench):
    """Message class 'JointWrench'."""

    __slots__ = [
        '_header',
        '_body_1_name',
        '_body_1_id',
        '_body_2_name',
        '_body_2_id',
        '_body_1_wrench',
        '_body_2_wrench',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'body_1_name': 'std_msgs/String',
        'body_1_id': 'std_msgs/UInt32',
        'body_2_name': 'std_msgs/String',
        'body_2_id': 'std_msgs/UInt32',
        'body_1_wrench': 'geometry_msgs/Wrench',
        'body_2_wrench': 'geometry_msgs/Wrench',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'UInt32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'UInt32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Wrench'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from std_msgs.msg import String
        self.body_1_name = kwargs.get('body_1_name', String())
        from std_msgs.msg import UInt32
        self.body_1_id = kwargs.get('body_1_id', UInt32())
        from std_msgs.msg import String
        self.body_2_name = kwargs.get('body_2_name', String())
        from std_msgs.msg import UInt32
        self.body_2_id = kwargs.get('body_2_id', UInt32())
        from geometry_msgs.msg import Wrench
        self.body_1_wrench = kwargs.get('body_1_wrench', Wrench())
        from geometry_msgs.msg import Wrench
        self.body_2_wrench = kwargs.get('body_2_wrench', Wrench())

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
        if self.body_1_name != other.body_1_name:
            return False
        if self.body_1_id != other.body_1_id:
            return False
        if self.body_2_name != other.body_2_name:
            return False
        if self.body_2_id != other.body_2_id:
            return False
        if self.body_1_wrench != other.body_1_wrench:
            return False
        if self.body_2_wrench != other.body_2_wrench:
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
    def body_1_name(self):
        """Message field 'body_1_name'."""
        return self._body_1_name

    @body_1_name.setter
    def body_1_name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'body_1_name' field must be a sub message of type 'String'"
        self._body_1_name = value

    @builtins.property
    def body_1_id(self):
        """Message field 'body_1_id'."""
        return self._body_1_id

    @body_1_id.setter
    def body_1_id(self, value):
        if __debug__:
            from std_msgs.msg import UInt32
            assert \
                isinstance(value, UInt32), \
                "The 'body_1_id' field must be a sub message of type 'UInt32'"
        self._body_1_id = value

    @builtins.property
    def body_2_name(self):
        """Message field 'body_2_name'."""
        return self._body_2_name

    @body_2_name.setter
    def body_2_name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'body_2_name' field must be a sub message of type 'String'"
        self._body_2_name = value

    @builtins.property
    def body_2_id(self):
        """Message field 'body_2_id'."""
        return self._body_2_id

    @body_2_id.setter
    def body_2_id(self, value):
        if __debug__:
            from std_msgs.msg import UInt32
            assert \
                isinstance(value, UInt32), \
                "The 'body_2_id' field must be a sub message of type 'UInt32'"
        self._body_2_id = value

    @builtins.property
    def body_1_wrench(self):
        """Message field 'body_1_wrench'."""
        return self._body_1_wrench

    @body_1_wrench.setter
    def body_1_wrench(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'body_1_wrench' field must be a sub message of type 'Wrench'"
        self._body_1_wrench = value

    @builtins.property
    def body_2_wrench(self):
        """Message field 'body_2_wrench'."""
        return self._body_2_wrench

    @body_2_wrench.setter
    def body_2_wrench(self, value):
        if __debug__:
            from geometry_msgs.msg import Wrench
            assert \
                isinstance(value, Wrench), \
                "The 'body_2_wrench' field must be a sub message of type 'Wrench'"
        self._body_2_wrench = value
