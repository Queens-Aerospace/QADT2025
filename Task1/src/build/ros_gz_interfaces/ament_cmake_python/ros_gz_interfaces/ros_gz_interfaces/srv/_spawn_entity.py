# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros_gz_interfaces:srv/SpawnEntity.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SpawnEntity_Request(type):
    """Metaclass of message 'SpawnEntity_Request'."""

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
                'ros_gz_interfaces.srv.SpawnEntity_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__spawn_entity__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__spawn_entity__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__spawn_entity__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__spawn_entity__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__spawn_entity__request

            from ros_gz_interfaces.msg import EntityFactory
            if EntityFactory.__class__._TYPE_SUPPORT is None:
                EntityFactory.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SpawnEntity_Request(metaclass=Metaclass_SpawnEntity_Request):
    """Message class 'SpawnEntity_Request'."""

    __slots__ = [
        '_entity_factory',
    ]

    _fields_and_field_types = {
        'entity_factory': 'ros_gz_interfaces/EntityFactory',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['ros_gz_interfaces', 'msg'], 'EntityFactory'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from ros_gz_interfaces.msg import EntityFactory
        self.entity_factory = kwargs.get('entity_factory', EntityFactory())

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
        if self.entity_factory != other.entity_factory:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def entity_factory(self):
        """Message field 'entity_factory'."""
        return self._entity_factory

    @entity_factory.setter
    def entity_factory(self, value):
        if __debug__:
            from ros_gz_interfaces.msg import EntityFactory
            assert \
                isinstance(value, EntityFactory), \
                "The 'entity_factory' field must be a sub message of type 'EntityFactory'"
        self._entity_factory = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SpawnEntity_Response(type):
    """Metaclass of message 'SpawnEntity_Response'."""

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
                'ros_gz_interfaces.srv.SpawnEntity_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__spawn_entity__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__spawn_entity__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__spawn_entity__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__spawn_entity__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__spawn_entity__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SpawnEntity_Response(metaclass=Metaclass_SpawnEntity_Response):
    """Message class 'SpawnEntity_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_SpawnEntity(type):
    """Metaclass of service 'SpawnEntity'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ros_gz_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros_gz_interfaces.srv.SpawnEntity')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__spawn_entity

            from ros_gz_interfaces.srv import _spawn_entity
            if _spawn_entity.Metaclass_SpawnEntity_Request._TYPE_SUPPORT is None:
                _spawn_entity.Metaclass_SpawnEntity_Request.__import_type_support__()
            if _spawn_entity.Metaclass_SpawnEntity_Response._TYPE_SUPPORT is None:
                _spawn_entity.Metaclass_SpawnEntity_Response.__import_type_support__()


class SpawnEntity(metaclass=Metaclass_SpawnEntity):
    from ros_gz_interfaces.srv._spawn_entity import SpawnEntity_Request as Request
    from ros_gz_interfaces.srv._spawn_entity import SpawnEntity_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
