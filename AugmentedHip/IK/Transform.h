#pragma once

#include "../vector3.h"
#include "../quat.h"

struct Transform {

public:
	Transform() {}
	Transform(Vector3 pos) : position(pos) {}

public:
	Vector3 position;
	Quat rotation;
	

public:
	// Applies the parent transform (other) to the current transform, and returns the final value
	static Transform apply(const Transform& parent, const Transform& child) {
		Transform appliedTransform;
		appliedTransform.position = parent.position + child.rotation.xform(child.position);
		appliedTransform.rotation = parent.rotation * child.rotation;
		return appliedTransform;
	}

	static Quat fromTo(const Vector3& lhs, const Vector3& rhs) {
		// https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
		Quat q;
		Vector3 a = lhs.cross(rhs);
		q.x = a.x;
		q.y = a.y;
		q.z = a.z;
		q.w = sqrt((lhs.length_squared()) * (rhs.length_squared())) + lhs.dot(rhs);
		q.normalize();

		return q;
	}

	static Transform lerp(const Transform& left, const Transform& right, double delta = 0.5) {
		Transform ret;
		ret.position = left.position.lerp(right.position, delta);
		// ret.rotation = left.rotation.slerp(right.rotation, delta);

		return ret;
	}

	Transform lerp(const Vector3& pos, double delta = 0.5) {
		Transform ret;
		ret.position = this->position.lerp(pos, delta);
		return ret;
	}

	Transform operator- (const Transform& v) const {
		return Transform(this->position - v.position);
		// yes ik no rotation bla bla i got IK figuring that out fuck you :D
	}
};