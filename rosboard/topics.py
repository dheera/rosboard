import os

from rosidl_adapter.parser import parse_message_string
from rosidl_runtime_py import get_interface_path

from rosboard.ros_init import rospy


def get_all_topics():
    all_topics = {}
    for topic_tuple in rospy.get_published_topics():
        topic_name = topic_tuple[0]
        topic_type = topic_tuple[1]
        if type(topic_type) is list:
            topic_type = topic_type[0] # ROS2
        all_topics[topic_name] = topic_type
    return all_topics

def update_all_topics_with_typedef(full_topics, topics=None):
    if topics is None:
        topics = get_all_topics()
    topic_names = set(topics.keys())
    cached_topic_names = set(full_topics.keys())
    # Get only the topics that are not cached
    missing_topics = topic_names - cached_topic_names
    if missing_topics:
        for topic_name in missing_topics:
            topic_type = topics[topic_name]
            type_def = get_typedef_full_text(topic_type)
            full_topics[topic_name] = {"type": topic_type, "typedef": type_def}

def get_all_topics_with_typedef():
    topics = get_all_topics()
    full_topics = {}
    for topic_name, topic_type in topics.items():
        full_typedef = get_typedef_full_text(topic_type)
        full_topics[topic_name] = {"type": topic_type, "typedef": full_typedef}
    return full_topics

# TODO: Add ros1 support for this function
def get_typedef_full_text(ty):
    """Returns the full text (similar to `gendeps --cat`) for the specified message type"""
    try:
        return stringify_field_types(ty)
    except Exception as e:
        return f"# failed to get full definition text for {ty}: {str(e)}"

def extract_msg_path_from_idl(idl_path):
    with open(idl_path, 'r', encoding='utf-8') as idl_file:
        idl_content = idl_file.read()
    
    # Try to extract the original .msg file path from content. Usually in the first lines
    # there is a part that says "// with input from <path>" and path is related to the original .msg file
    msg_file_info = [line for line in idl_content.split('\n') if '// with input from' in line]
    if msg_file_info:
        msg_path = msg_file_info[0].split('// with input from')[-1].strip()
        # Remove "msg/" from the path, since the actual path doesn't have it
        msg_path = msg_path.replace('msg/', '')
        # Find parent directory of the idl just before the share folder
        share_dir = idl_path.split('share')[0]
        # Construct the full path to the .msg file
        return os.path.join(share_dir, 'share', msg_path)
    
    return idl_path

# Taken from https://github.com/RobotWebTools/rosbridge_suite/blob/7d78af16d30d0ffe232abcc65d0928ce90bd61f7/rosapi/src/rosapi/stringify_field_types.py#L5
# Modified to account custom message types
def stringify_field_types(root_type):
    definition = ""
    seen_types = set()
    deps = [root_type]
    is_root = True
    while deps:
        ty = deps.pop()
        parts = ty.split("/")
        if not is_root:
            definition += "\n================================================================================\n"
            definition += f"MSG: {ty}\n"
        is_root = False

        msg_name = parts[2] if len(parts) == 3 else parts[1]
        interface_name = ty if len(parts) == 3 else f"{parts[0]}/msg/{parts[1]}"

        # Try to get the .msg file first
        msg_path = get_interface_path(interface_name)

        # If custom we get instead the .idl implementation
        if msg_path.endswith('.idl'):
            msg_path = extract_msg_path_from_idl(msg_path)

        with open(msg_path, encoding="utf-8") as msg_file:
            msg_definition = msg_file.read()
        definition += msg_definition

        spec = parse_message_string(parts[0], msg_name, msg_definition)
        for field in spec.fields:
            is_builtin = field.type.pkg_name is None
            if not is_builtin:
                field_ty = f"{field.type.pkg_name}/{field.type.type}"
                if field_ty not in seen_types:
                    deps.append(field_ty)
                    seen_types.add(field_ty)

    return definition
