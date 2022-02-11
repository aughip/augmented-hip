#pragma once
#include "../vector3.h"
#include <vector>
#include "Transform.h"

class FABRIKSolver {
protected:
	std::vector<Transform> mIKChain;
	unsigned int mNumSteps;
	float mThreshold;
	std::vector<Vector3> mWorldChain;
	std::vector<float> mLengths;
protected:
	void IKChainToWorld();
	void IterateForward(const Vector3& goal);
	void IterateBackward(const Vector3& base);
	void WorldToIKChain();
public:
	FABRIKSolver();

	unsigned int Size();
	void Resize(unsigned int newSize);

	Transform GetLocalTransform(unsigned int index);
	void SetLocalTransform(unsigned int index, const Transform& t);
	Transform GetGlobalTransform(unsigned int index);

	unsigned int GetNumSteps();
	void SetNumSteps(unsigned int numSteps);
	
	float GetThreshold();
	void SetThreshold(float value);

	bool Solve(const Transform& target);
};