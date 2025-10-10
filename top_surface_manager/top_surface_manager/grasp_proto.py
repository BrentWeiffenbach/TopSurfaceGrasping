import numpy as np
from dataclasses import dataclass
from typing import List
import math
from typing import Optional, Tuple

@dataclass
class Point32:
    x: float
    y: float
    z: float = 0.0
    def __str__(self):
        return f"({np.round(self.x, 2)}, {round(self.y,2)})"

@dataclass
class Polygon:
    points: List[Point32]

def getIntersectingPoint(
    rayPoints: Tuple[Point32, Point32], 
    sidePoints: Tuple[Point32, Point32]
) -> Optional[Point32]:
    (pt, centroid) = rayPoints
    (curr, next) = sidePoints
    print(f'intersection between {((pt.x, pt.y), (centroid.x, centroid.y)), ((curr.x, curr.y), (next.x, next.y))}')

    if pt == curr: # or pt == next: 
        print('point == curr')
        return None

    # Method 1
    a_ray = centroid.y - pt.y
    b_ray = pt.x - centroid.x
    c_ray = a_ray*pt.x + b_ray*pt.y

    a_side = next.y - curr.y
    b_side = curr.x - next.x
    c_side = a_side*curr.x + b_side*curr.y

    ## find intersection
    d = a_ray*b_side - a_side*b_ray
    if abs(d) < 1e-9: 
        print('lines are parallel')
        return None
    else: 
        x = (b_side*c_ray - b_ray*c_side)/d
        y = (a_ray*c_side - a_side*c_ray)/d

    if x >= min(curr.x, next.x) and x <= max(curr.x, next.x) and y >= min(curr.y, next.y) and y <= max(curr.y, next.y):

        intersection = Point32(x=x, y=y)
        return intersection
    else: 
        print('does not fit in intersection')
        return None


def getOpposite(currPt, centroid, geometry):
    """
    return the opposite point from the current point
    """

    points_list = list(geometry.points)
    points_list.append(Point32(0.0, 0.0))
    intersectingPoints = []
    for i in range(len(points_list)-1):
        curr = points_list[i]
        next = points_list[i+1]
        if currPt == curr or currPt == next: 
            # print(f'skipped: {curr}, {next}')
            continue

        # print(f"check intersection: {((pt.x, pt.y), (centroid.x, centroid.y)), ((curr.x, curr.y), (next.x, next.y))}")
        
        point = getIntersectingPoint(rayPoints=(currPt, centroid), sidePoints=(curr, next))
        if point is not None:
            intersectingPoints.append(point)
    
    ## choose opposite point from intersections
    if len(intersectingPoints) > 0:
        dist_pt_to_centroid = math.hypot(centroid.x - currPt.x, centroid.y - currPt.y)
        farthest_intersection = None
        max_dist = 0.0

        for intersection in intersectingPoints:
            dist_pt_to_intersection = math.hypot(intersection.x - currPt.x, intersection.y - currPt.y)
            
            # Only consider intersections beyond the centroid
            if dist_pt_to_intersection > dist_pt_to_centroid:
                # Choose the farthest valid intersection (opposite side)
                if dist_pt_to_intersection > max_dist:
                    max_dist = dist_pt_to_intersection
                    farthest_intersection = intersection

        return farthest_intersection  # None if no valid intersection found
    else:
        return None

def getNextPoint(point, centroid, geometry, thetaDelta): 
    """
    point = current point
    centroid: center of polygon
    geometry: 
    thetaDelta: increments of theta to change by in radians 
    """
    points_list = list(geometry.points)


    ## Method 1
    # dx = point.x - centroid.x
    # dy = point.y - centroid.y

    # dx_new = dx*math.cos(thetaDelta) - dy*math.sin(thetaDelta)
    # dy_new = dy*math.sin(thetaDelta) + dy*math.cos(thetaDelta)

    # x_dummy = centroid.x + dx_new
    # y_dummy = centroid.y + dy_new

    ## Method 2 
    if abs(point.x - centroid.x) < 0.00001: # vertical line
        if point.y > centroid.y:
            newTheta = math.pi/2 + thetaDelta
        else: 
            newTheta = 3*math.pi/2 + thetaDelta
    else: 
        m_old = (point.y - centroid.y)/(point.x - centroid.x)
        newTheta = math.atan(m_old) + thetaDelta
        
    m_new = math.tan(newTheta)
    b_new = centroid.y-m_new*centroid.x

    if m_new > 100000000000: # vertical line through centroid
        dummyPoint = Point32(centroid.x, centroid.y+10)
    else: 
        x_dummy = centroid.x + 10
        y_dummy = m_new*x_dummy + b_new    

        print(f'm: {m_new}\tb: {b_new}\tx_dummy: {x_dummy}\ty_dummy: {y_dummy}')

        dummyPoint = Point32(x_dummy, y_dummy)

    intersectingPoints = []
    for i in range(len(points_list)-1):
        curr = points_list[i]
        next = points_list[i+1]

        ip = getIntersectingPoint(rayPoints=(dummyPoint, centroid), sidePoints=(curr, next))
        if ip is not None:
            intersectingPoints.append(ip)
    
    min_diff = math.inf
    print(f'intersection points: {intersectingPoints}')
    if len(intersectingPoints) > 0:
        for intersection in intersectingPoints: 
            # calculate the difference in theta and find the one that == delta theta
            angleInter = math.atan2(intersection.y - centroid.y, intersection.x - centroid.x)
            anglePt = math.atan2(point.y - centroid.y, point.x - centroid.x)
            # CCW angle difference
            diff = (angleInter - anglePt) % (2*math.pi)
            delta_error = abs(diff - thetaDelta)
            # print(f'angleInter: {angleInter}\tanglePt: {anglePt}\tdiff: {diff} \totherDiff: {(abs((abs(angleInter) - abs(anglePt))) - thetaDelta)}')
            if delta_error < min_diff:
                min_diff = delta_error
                closest_point = intersection

        return closest_point
    else: 
        return None
    


def get_all_possible_grasps(geometry, centroid, thetaDelta):
    """calculate all possible grasp points"""

    graspPoints = []
    theta = 0

    # start somewhere, draw a line through the center to the other side to get second point
    points_list = list(geometry.points)
    print(f"points list: {points_list}")
    maxDist = 0
    farthest = None
    for p in points_list: 
        dist = math.dist((centroid.x, centroid.y), (p.x, p.y))
        if dist > maxDist: 
            maxDist = dist
            farthest = p
    
    currPoint = farthest

    while theta < 2*math.pi:
        print(f'\ncurrPoint: {currPoint}\ttheta: {theta}')

        # get opposite point
        oppPoint = getOpposite(currPoint, centroid, geometry)
        print(f"opposite: {oppPoint}\n")
        if oppPoint is None:
            print(f"opposite is None")
            return  
        graspPoints.append((currPoint, oppPoint))

        theta += thetaDelta
        currPoint = getNextPoint(currPoint, centroid, geometry, thetaDelta)
        if currPoint == None: 
            print("currentPoint is None")
            return

    # get rid of duplicates
    unique_grasps = []

    for p1, p2 in graspPoints:
        is_duplicate = False
        
        for o1, o2 in unique_grasps: 
            d1 = math.dist((o1.x, o1.y), (p1.x, p1.y))
            d2 = math.dist((o2.x, o2.y), (p2.x, p2.y))
            
            # Check if both points match (or reversed match)
            if (d1 < 0.001 and d2 < 0.001) or \
            (math.dist((o1.x, o1.y), (p2.x, p2.y)) < 0.001 and \
                math.dist((o2.x, o2.y), (p1.x, p1.y)) < 0.001):
                is_duplicate = True
                break
        
        if not is_duplicate:
            unique_grasps.append((p1, p2))

    ## Debugging
    print(f'calculated {len(unique_grasps)} total grasp positions')

    for (p1, p2) in unique_grasps:
        print(p1, p2)

    return unique_grasps

if __name__ == "__main__":

    polygon = Polygon(points=[
        Point32(0.0, 0.0),
        Point32(1.0, 0.0),
        Point32(1.0, 1.0),
        Point32(0.0, 1.0)
    ])

    grasps = get_all_possible_grasps(polygon, centroid=Point32(0.5, 0.5), thetaDelta=math.radians(45))