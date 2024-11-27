# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:msg/Contact.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'depths'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Contact(type):
    """Metaclass of message 'Contact'."""

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
                'ros_gz_interfaces.msg.Contact')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__contact
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__contact
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__contact
            cls._TYPE_SUPPORT = module.type_support_msg__msg__contact
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__contact

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from ros_gz_interfaces.msg import Entity
            if Entity.__class__._TYPE_SUPPORT is None:
                Entity.__class__.__import_type_support__()

            from ros_gz_interfaces.msg import JointWrench
            if JointWrench.__class__._TYPE_SUPPORT is None:
                JointWrench.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Contact(metaclass=Metaclass_Contact):
    """Message class 'Contact'."""

    __slots__ = [
        '_collision1',
        '_collision2',
        '_positions',
        '_normals',
        '_depths',
        '_wrenches',
    ]

    _fields_and_field_types = {
        'collision1': 'ros_gz_interfaces/Entity',
        'collision2': 'ros_gz_interfaces/Entity',
        'positions': 'sequence<geometry_msgs/Vector3>',
        'normals': 'sequence<geometry_msgs/Vector3>',
        'depths': 'sequence<double>',
        'wrenches': 'sequence<ros_gz_interfaces/JointWrench>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'Entity'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'Entity'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'JointWrench')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from ros_gz_interfaces.msg import Entity
        self.collision1 = kwargs.get('collision1', Entity())
        from ros_gz_interfaces.msg import Entity
        self.collision2 = kwargs.get('collision2', Entity())
        self.positions = kwargs.get('positions', [])
        self.normals = kwargs.get('normals', [])
        self.depths = array.array('d', kwargs.get('depths', []))
        self.wrenches = kwargs.get('wrenches', [])

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
        if self.collision1 != other.collision1:
            return False
        if self.collision2 != other.collision2:
            return False
        if self.positions != other.positions:
            return False
        if self.normals != other.normals:
            return False
        if self.depths != other.depths:
            return False
        if self.wrenches != other.wrenches:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def collision1(self):
        """Message field 'collision1'."""
        return self._collision1

    @collision1.setter
    def collision1(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import Entity
            assert \
                isinstance(value, Entity), \
                "The 'collision1' field must be a sub message of type 'Entity'"
        self._collision1 = value

    @builtins.property
    def collision2(self):
        """Message field 'collision2'."""
        return self._collision2

    @collision2.setter
    def collision2(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import Entity
            assert \
                isinstance(value, Entity), \
                "The 'collision2' field must be a sub message of type 'Entity'"
        self._collision2 = value

    @builtins.property
    def positions(self):
        """Message field 'positions'."""
        return self._positions

    @positions.setter
    def positions(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Vector3) for v in value) and
                 True), \
                "The 'positions' field must be a set or sequence and each value of type 'Vector3'"
        self._positions = value

    @builtins.property
    def normals(self):
        """Message field 'normals'."""
        return self._normals

    @normals.setter
    def normals(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Vector3) for v in value) and
                 True), \
                "The 'normals' field must be a set or sequence and each value of type 'Vector3'"
        self._normals = value

    @builtins.property
    def depths(self):
        """Message field 'depths'."""
        return self._depths

    @depths.setter
    def depths(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'depths' array.array() must have the type code of 'd'"
            self._depths = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'depths' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._depths = array.array('d', value)

    @builtins.property
    def wrenches(self):
        """Message field 'wrenches'."""
        return self._wrenches

    @wrenches.setter
    def wrenches(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import JointWrench
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, JointWrench) for v in value) and
                 True), \
                "The 'wrenches' field must be a set or sequence and each value of type 'JointWrench'"
        self._wrenches = value
