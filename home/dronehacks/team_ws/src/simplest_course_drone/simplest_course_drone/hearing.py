"""Bookkeeping for listening to many topics: name matching, rates, and a printable table. Pure Python."""
import re

HEAVY_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/PointCloud2')


def resolve(listed, available):
    """The live spelling of a listed topic: itself, or its highest PX4 version (_v<N>), or None."""
    if listed in available:
        return listed
    versions = [(int(m.group(1)), name) for name in available
                for m in [re.fullmatch(re.escape(listed) + r'_v(\d+)', name)] if m]
    return max(versions)[1] if versions else None


def extras(available, wanted, never):
    """Topics on the graph that are not on our list, minus the ones we must not touch."""
    return sorted(t for t in available if t not in wanted and not any(n in t for n in never))


class Ear:
    """What we have heard on one topic."""

    def __init__(self, topic, type_name, note=''):
        self.topic, self.type_name, self.note = topic, type_name, note
        self.count = 0
        self.bytes = 0
        self.last_t = None
        self.preview = ''
        self._window = []                      # receipt times in the last 5 s, for the rate

    @property
    def heavy(self):
        return self.type_name in HEAVY_TYPES

    def heard(self, t, size=0, preview=''):
        self.count += 1
        self.bytes += size
        self.last_t = t
        if preview:
            self.preview = preview
        self._window.append(t)

    def rate(self, now, window_s=5.0):
        self._window = [x for x in self._window if now - x <= window_s]
        return len(self._window) / window_s


def table(now, ears, missing):
    """One block of text: a row per topic, then the listed topics nobody publishes."""
    rows = ['{:<44} {:>6} {:>8}  {}'.format('topic', 'Hz', 'last', 'latest')]
    for e in sorted(ears, key=lambda e: e.topic):
        last = '-' if e.last_t is None else '{:.1f}s'.format(now - e.last_t)
        rows.append('{:<44} {:>6.1f} {:>8}  {}'.format(e.topic[:44], e.rate(now), last, e.preview[:90]))
    if missing:
        rows.append('not on the graph: ' + ', '.join(missing))
    return '\n'.join(rows)


def short(d, limit=90):
    """A dict of message fields as one short line: scalars first, arrays shortened."""
    parts = []
    for k, v in d.items():
        if isinstance(v, dict):
            v = short(v, 40)
        elif isinstance(v, (list, tuple)):
            v = '[{} items]'.format(len(v)) if len(v) > 4 else list(v)
        elif isinstance(v, float):
            v = round(v, 3)
        parts.append('{}={}'.format(k, v))
        if sum(len(p) for p in parts) > limit:
            break
    return ' '.join(parts)[:limit]
