#include <cmath>
#include <iostream>
#include "scene_object.h"
#include <vector>
#include <algorithm>

using namespace std;

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	bool intersected = false;
	
 	Ray3D raySpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
	double t_value = -(raySpace.origin[2] / raySpace.dir[2]);
	
	Point3D crossPoint = raySpace.origin + t_value * raySpace.dir;
	
	if ((t_value > 0) 
	    && (crossPoint[0] >= -0.5 && crossPoint[0] <= 0.5)
	    && (crossPoint[1] >= -0.5 && crossPoint[1] <= 0.5)
	    && (t_value < ray.intersection.t_value || ray.intersection.none))
	{
	    intersected = true;
	    ray.intersection.normal = worldToModel.transpose() * Vector3D(0, 0, 1);
	    ray.intersection.point = modelToWorld * crossPoint;
	    ray.intersection.t_value = t_value;
	    ray.intersection.none = false;
	}

	return intersected;
}


bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	// uses logic based on http://www.csee.umbc.edu/~olano/435f02/ray-sphere.html
	bool occured = false;
	bool overwrite = false;
	
	Ray3D raySpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
	Vector3D rayOriginVector = raySpace.origin - Point3D(0, 0, 0);
	
	double a = raySpace.dir.dot(raySpace.dir);
	double b = 2 * raySpace.dir.dot(rayOriginVector);
	double c = rayOriginVector.dot(rayOriginVector) - 1;
	
	double discriminant_val = (b * b) - (4 * a * c);
	
	double t_value;
	
	if (discriminant_val >= 0) { 
		double root1, root2;
		quadSolve(a, b, c, root1, root2);
		
		if (root1 > 0 || root2 > 0)
		{
			occured = true;
			if (root1 > root2)
			{
				t_value = root2;
				if (root2 < 0)
				{
					t_value = root1;
				}
			}
			
			if (root1 < root2)
			{
				t_value = root1;
				if (root1 < 0)
				{
					t_value = root2;
				}
			}
		}
	}
	
	if (occured && (ray.intersection.none || t_value < ray.intersection.t_value)) {
		overwrite = true;
		Point3D intersectionPointSpace = raySpace.origin + t_value * raySpace.dir;
		ray.intersection.point = modelToWorld * intersectionPointSpace;
		ray.intersection.normal = worldToModel.transpose() * Vector3D(intersectionPointSpace[0], intersectionPointSpace[1], intersectionPointSpace[2]);
		ray.intersection.t_value = t_value;
		ray.intersection.none = false;
	}
	
	return occured && overwrite;
}

bool NullObject::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
    return false;
}

UnitTriangle::UnitTriangle(Vector3D norm, Point3D a, Point3D b, Point3D c) : n(norm), p0(a), p1(b), p2(c) {}

bool UnitTriangle::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) 
{

	bool overwrite = false;

	Ray3D raySpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

	double denom = raySpace.dir.dot(n);
	if (isSameDouble(denom,0)) {
		return false;
	}
	double t_value = -(raySpace.origin - p0).dot(n) / denom;
	Point3D planeIntersect = raySpace.point_at(t_value);

	// see if the point is contained in the 3 half planes
	if (((p1 - p0).cross(planeIntersect - p0)).dot(n) >= 0 &&
		((p2 - p1).cross(planeIntersect - p1)).dot(n) >= 0 &&
		((p0 - p2).cross(planeIntersect - p2)).dot(n) >= 0
		&& (t_value > 0)
		&& (ray.intersection.none || t_value < ray.intersection.t_value)) 
	{
		overwrite = true;
		ray.intersection.normal = n;
		ray.intersection.point = modelToWorld * planeIntersect;
		ray.intersection.t_value = t_value;
		ray.intersection.none = false;
	}
	
	return overwrite;
}

bool inUnitCircle(Point3D point) {
	double x = point[0];
	double z = point[2];
	return pow(x, 2) + pow(z, 2) <= 1.0;
}

bool UnitCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {

	// a cylinder: unit circle in xz plane, y from -0.5 to 0.5
	bool overwrite = false;
	bool hit_top = false;
	bool hit_bottom = false;

	double t_value;
	Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
	vector<double> possible_t_values;

	double px = rayModelSpace.origin[0];
	double py = rayModelSpace.origin[1];
	double pz = rayModelSpace.origin[2];

	double dx = rayModelSpace.dir[0];
	double dy = rayModelSpace.dir[1];
	double dz = rayModelSpace.dir[2];

	double a = pow(dx, 2) + pow(dz, 2);
	double b = 2 * (px * dx + pz * dz);
	double c = pow(px, 2) + pow(pz, 2) - 1.0;

	double discriminant_val = (b * b) - (4 * a * c);
	if (discriminant_val >= 0) 
	{
		double root1, root2;
		quadSolve(a, b, c, root1, root2);
		if (root1 > 0) {
			possible_t_values.push_back(root1);
		}
		if (root2 > 0) {
			possible_t_values.push_back(root2);
		}
	}

	// check top and bottom also:
	double t_top = (0.5 - py) / dy;
	double t_bottom = (-0.5 - py) / dy;

	if (inUnitCircle(rayModelSpace.point_at(t_top))) {
		possible_t_values.push_back(t_top);	
	}

	if (inUnitCircle(rayModelSpace.point_at(t_bottom))) {
		possible_t_values.push_back(t_bottom);	
	}
	
	// smallest t value that is greater than 0 and meets the height requirements:
	std::vector<double> qualifying_t_values;
	for(unsigned int i = 0; i < possible_t_values.size(); i++) {
		double t = possible_t_values.at(i);
	    if (t > 0) {
	    	Point3D point = rayModelSpace.point_at(t);
	    	double y_val = point[1];
	    	if ((y_val >= -0.5) && (y_val <= 0.5)) {
				qualifying_t_values.push_back(t);
	    	}
	    }
	}

	if (qualifying_t_values.size() > 0) {
		t_value = *std::min_element(qualifying_t_values.begin(), qualifying_t_values.end());
		if (isSameDouble(t_value, t_top)) {
			hit_top = true;
		}

		if (isSameDouble(t_value, t_bottom)) {
			hit_bottom = true;
		} 
	}
	

	if ((qualifying_t_values.size() > 0) 
		&& (ray.intersection.none || t_value < ray.intersection.t_value))
	{
		overwrite = true;
		Point3D intersectionPointModelSpace = rayModelSpace.point_at(t_value);
		ray.intersection.point = modelToWorld * intersectionPointModelSpace;
		Vector3D normal;
		if (hit_top) {
			normal = Vector3D(0, 1.0, 0); 
		} else if (hit_bottom) {
			normal = Vector3D(0, -1.0, 0);
		} else {
			normal = Vector3D(intersectionPointModelSpace[0], 0, intersectionPointModelSpace[2]);
		}
		normal.normalize();
		ray.intersection.normal = worldToModel.transpose() * normal;
		ray.intersection.t_value = t_value;
		ray.intersection.none = false;
	}

	return overwrite;
}

bool UnitCone::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {

	// a cone: unit circle in xz plane, y from 0 to 1

	bool overwrite = false;
	bool hit_bottom = false;

	double t_value;
	Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
	vector<double> possible_t_values;

	double px = rayModelSpace.origin[0];
	double py = rayModelSpace.origin[1];
	double pz = rayModelSpace.origin[2];

	double dx = rayModelSpace.dir[0];
	double dy = rayModelSpace.dir[1];
	double dz = rayModelSpace.dir[2];

	double a = pow(dx, 2) + pow(dz, 2) - pow(dy, 2);
	double b = 2 * (px * dx + pz * dz) + 2 * dy - 2 * dy * py;
	double c = pow(px, 2) + pow(pz, 2) - pow(py, 2) - 1.0 + 2.0 * py;

	double discriminant_val = (b * b) - (4 * a * c);
	if (discriminant_val >= 0) {
		double root1, root2;
		quadSolve(a, b, c, root1, root2);
		if (root1 > 0) {
			possible_t_values.push_back(root1);
		}
		if (root2 > 0) {
			possible_t_values.push_back(root2);
		}
	}

	// check bottom also:
	double t_bottom = -py / dy;
	if (inUnitCircle(rayModelSpace.point_at(t_bottom))) {
		possible_t_values.push_back(t_bottom);	
	}
	
	// we want the smallest t value that is greater than 0 and meets the height requirements:
	std::vector<double> qualifying_t_values;
	for(unsigned int i = 0; i < possible_t_values.size(); i++) {
		double t = possible_t_values[i];
	    if (t > 0) {
	    	Point3D point = rayModelSpace.point_at(t);
	    	double y_val = point[1];
	    	if ((y_val >= 0) && (y_val <= 1)) {
				qualifying_t_values.push_back(t);
	    	}
	    }
	}
	
	if (qualifying_t_values.size() > 0) {
		t_value = *std::min_element(qualifying_t_values.begin(), qualifying_t_values.end());
		if (isSameDouble(t_value, t_bottom)) {
			hit_bottom = true;
		} 
	}
	

	if ((qualifying_t_values.size() > 0) 
		&& (ray.intersection.none || t_value < ray.intersection.t_value)) 
	{
		overwrite = true;
		Point3D intersectionPointModelSpace = rayModelSpace.point_at(t_value);
		ray.intersection.point = modelToWorld * intersectionPointModelSpace;
		Vector3D normal;
		if (hit_bottom) {
			normal = Vector3D(0, 0, -1.0);
		} else {
			normal = Vector3D(intersectionPointModelSpace[0], 1 - intersectionPointModelSpace[1], intersectionPointModelSpace[2]);
		}
		normal.normalize();
		ray.intersection.normal = worldToModel.transpose() * normal;
		ray.intersection.t_value = t_value;
		ray.intersection.none = false;
	}

	return overwrite;
}


