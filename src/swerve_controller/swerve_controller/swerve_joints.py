"""
provides enums for enumerating all 8 swerve joints and iterators
"""

from enum import Enum
from typing import Final, Iterator, Tuple


class JointKind(Enum):
    STEERING = "steering"
    WHEEL = "wheel"


class JointEnd(Enum):
    FRONT = "front"
    REAR = "rear"


class JointSide(Enum):
    RIGHT = "right"
    LEFT = "left"


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
    return f"{kind.value}_{end.value}_{side.value}"


joint_names: Final[Tuple[str, ...]] = tuple(
    joint_name_from_key(key) for key in joints()
)
