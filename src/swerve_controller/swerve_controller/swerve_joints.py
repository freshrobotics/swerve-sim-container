"""
provides enums for enumerating all 8 swerve joints and iterators
"""

from enum import Enum
from typing import Final, Iterator, Tuple


class JointKind(Enum):
    steering = 0
    wheel = 1


class JointEnd(Enum):
    front = 0
    rear = 1


class JointSide(Enum):
    right = 0
    left = 1


JointKey = Tuple[JointKind, JointEnd, JointSide]
JointPosition = Tuple[JointEnd, JointSide]


def joints() -> Iterator[JointKey]:
    for kind in JointKind:
        for end in JointEnd:
            for side in JointSide:
                yield (kind, end, side)


def joint_positions() -> Iterator[JointPosition]:
    for end in JointEnd:
        for side in JointSide:
            yield (end, side)


def joint_name_from_key(key: JointKey) -> str:
    kind, end, side = key
    return f"{kind.name}_{end.name}_{side.name}"


joint_names: Final[Tuple[str, ...]] = tuple(
    joint_name_from_key(key) for key in joints()
)
