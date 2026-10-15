"""Robot input topics and identifiers for namespaced route scoring."""


def robot_identity(namespace, configured_id=''):
    leaf = namespace.strip('/').rsplit('/', 1)[-1]
    robot = configured_id or leaf or 'robot'
    if leaf and robot != leaf:
        raise ValueError('robot_id must match the final robot namespace component')
    return robot


def robot_topic(topic, namespace):
    if not isinstance(topic, str) or not topic.strip():
        raise ValueError('Robot input topic must be non-empty')
    if not topic.startswith('/'):
        return topic
    prefix = namespace.rstrip('/') + '/'
    if prefix == '/' or not topic.startswith(prefix):
        raise ValueError('Robot input topics must belong to this node namespace')
    return topic[len(prefix):]
