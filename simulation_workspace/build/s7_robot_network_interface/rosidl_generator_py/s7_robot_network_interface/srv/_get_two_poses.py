# generated from rosidl_generator_py/resource/_idl.py.em
# with input from s7_robot_network_interface:srv/GetTwoPoses.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTwoPoses_Request(type):
    """Metaclass of message 'GetTwoPoses_Request'."""

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
            module = import_type_support('s7_robot_network_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                's7_robot_network_interface.srv.GetTwoPoses_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_two_poses__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_two_poses__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_two_poses__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_two_poses__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_two_poses__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTwoPoses_Request(metaclass=Metaclass_GetTwoPoses_Request):
    """Message class 'GetTwoPoses_Request'."""

    __slots__ = [
        '_robot_id',
    ]

    _fields_and_field_types = {
        'robot_id': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())

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
        if self.robot_id != other.robot_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'robot_id' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._robot_id = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTwoPoses_Response(type):
    """Metaclass of message 'GetTwoPoses_Response'."""

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
            module = import_type_support('s7_robot_network_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                's7_robot_network_interface.srv.GetTwoPoses_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_two_poses__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_two_poses__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_two_poses__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_two_poses__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_two_poses__response

            from geometry_msgs.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTwoPoses_Response(metaclass=Metaclass_GetTwoPoses_Response):
    """Message class 'GetTwoPoses_Response'."""

    __slots__ = [
        '_pickup_pose',
        '_delivery_pose',
    ]

    _fields_and_field_types = {
        'pickup_pose': 'geometry_msgs/Pose2D',
        'delivery_pose': 'geometry_msgs/Pose2D',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose2D
        self.pickup_pose = kwargs.get('pickup_pose', Pose2D())
        from geometry_msgs.msg import Pose2D
        self.delivery_pose = kwargs.get('delivery_pose', Pose2D())

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
        if self.pickup_pose != other.pickup_pose:
            return False
        if self.delivery_pose != other.delivery_pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def pickup_pose(self):
        """Message field 'pickup_pose'."""
        return self._pickup_pose

    @pickup_pose.setter
    def pickup_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'pickup_pose' field must be a sub message of type 'Pose2D'"
        self._pickup_pose = value

    @property
    def delivery_pose(self):
        """Message field 'delivery_pose'."""
        return self._delivery_pose

    @delivery_pose.setter
    def delivery_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'delivery_pose' field must be a sub message of type 'Pose2D'"
        self._delivery_pose = value


class Metaclass_GetTwoPoses(type):
    """Metaclass of service 'GetTwoPoses'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('s7_robot_network_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                's7_robot_network_interface.srv.GetTwoPoses')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_two_poses

            from s7_robot_network_interface.srv import _get_two_poses
            if _get_two_poses.Metaclass_GetTwoPoses_Request._TYPE_SUPPORT is None:
                _get_two_poses.Metaclass_GetTwoPoses_Request.__import_type_support__()
            if _get_two_poses.Metaclass_GetTwoPoses_Response._TYPE_SUPPORT is None:
                _get_two_poses.Metaclass_GetTwoPoses_Response.__import_type_support__()


class GetTwoPoses(metaclass=Metaclass_GetTwoPoses):
    from s7_robot_network_interface.srv._get_two_poses import GetTwoPoses_Request as Request
    from s7_robot_network_interface.srv._get_two_poses import GetTwoPoses_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
