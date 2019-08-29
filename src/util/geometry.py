""""
Geometry module.
"""

import math
from typing import Optional, Tuple, Iterable

import numpy

from src.entity.geometry import Segment, Ray
from src.entity.type import Radian
from src.entity.vector import Vector2


def ray_segment_intersection(ray: Ray, segment: Segment
                             ) -> Tuple[Optional[Vector2], Optional[float]]:
    """
    Get the intersection point between a ray and a segment. Returns None if they do not intersect
    and the intersection coordinates and the distance from the origin of the ray instead.

    Find intersection point:
    origin + direction * x = start + (end - start) * y
    origin ^ (end-start) + direction ^ (end-start) * x = start ^ (end-start)
    direction ^ (end-start) * x = start ^ (end-start) - origin ^ (end-start)
    direction ^ (end-start) * x = (start - origin) ^ (end-start)
    A * x = B
    x = B / A

    Then...
    intersection_point = origin + direction * x

    To find Y:
    intersection_point = start + (end - start) * y
    intersection_point - start = (end - start) * y
    (intersection_point - start) . intersection_point = (end - start) . intersection_point * y
    y = (intersection_point - start) . intersection_point / (end - start) . intersection_point
    """

    origin = numpy.array([ray.origin.x, ray.origin.y])
    direction = numpy.array([ray.direction.x, ray.direction.y])

    start = numpy.array([segment.start.x, segment.start.y])
    end = numpy.array([segment.end.x, segment.end.y])

    coef_a = numpy.cross(direction, (end - start))
    if coef_a == 0:
        return None, None  # Collinear, will never intersect.

    coef_b = numpy.cross(start - origin, end - start)
    v_x = coef_b / coef_a
    if v_x < 0:
        return None, None  # Ray hit behind.

    intersection_point = origin + direction * v_x
    divisor = numpy.dot(end - start, intersection_point)
    if divisor == 0:
        return None, None

    v_y = numpy.dot(intersection_point - start, intersection_point) / divisor
    if v_y < 0 or v_y > 1:
        return None, None  # Ray did not hit segment.

    return Vector2(*intersection_point), v_x


def ray_segments_intersection(ray: Ray, segments: Iterable[Segment]
                              ) -> Tuple[Optional[Vector2], Optional[float]]:
    """
    Compute the intersection of a ray and a set of segments.
    """
    closest_distance = math.inf
    closest_point = None

    for seg in segments:
        point, distance = ray_segment_intersection(ray, seg)
        if distance is None:
            continue
        if distance < closest_distance:
            closest_distance = distance
            closest_point = point

    if closest_point is None:
        return None, None

    return closest_point, closest_distance


def segment_segment_intersection(sgmt1: Segment,
                                 sgmt2: Segment) -> Optional[Vector2]:
    """
    Check if a segment intersect with another segment.
    """
    origin = sgmt2.start

    direction = sgmt2.end - sgmt2.start
    distance = direction.norm()

    direction_normalized = direction / distance
    ray = Ray(origin=origin, direction=direction_normalized)

    pos, ray_dist = ray_segment_intersection(ray, sgmt1)
    if ray_dist is None:
        return None

    print(ray_dist, distance)
    if ray_dist > distance:
        return None  # Too far away on the ray.

    return pos


def does_segment_intersect(sgmt1: Segment,
                           segments: Iterable[Segment]) -> bool:
    """
    Check if a segment intersects with a set of other segments.
    """
    for sgmt2 in segments:
        pos = segment_segment_intersection(sgmt1, sgmt2)
        if pos is not None:
            return True

    return False


def forward(angle: Radian) -> Vector2:
    """
    Get the front direction relative to the orientation.
    """
    return Vector2(
        math.cos(angle),
        math.sin(angle),
    )


def backward(angle: Radian) -> Vector2:
    """
    Get the back direction relative to the orientation.
    """
    return -forward(angle)


def right(angle: Radian) -> Vector2:
    """
    Get the right direction relative to the orientation.
    """
    return Vector2(
        math.cos(angle + math.pi / 2),
        math.sin(angle + math.pi / 2),
    )


def left(angle: Radian) -> Vector2:
    """
    Get the left direction relative to the orientation.
    """
    return -right(angle)
