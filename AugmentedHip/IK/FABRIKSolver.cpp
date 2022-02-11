#include "FABRIKSolver.h"

FABRIKSolver::FABRIKSolver() {
	mNumSteps = 2;
	mThreshold = 0.00001f;
}

unsigned int FABRIKSolver::GetNumSteps() {
	return mNumSteps;
}
void FABRIKSolver::SetNumSteps(unsigned int numSteps) {
	mNumSteps = numSteps;
}
float FABRIKSolver::GetThreshold() {
	return mThreshold;
}
void FABRIKSolver::SetThreshold(float value) {
	mThreshold = value;
}

unsigned int FABRIKSolver::Size() {
	return mIKChain.size();
}
void FABRIKSolver::Resize(unsigned int newSize) {
	mIKChain.resize(newSize);
	mWorldChain.resize(newSize);
	mLengths.resize(newSize);
}

Transform FABRIKSolver::GetLocalTransform(unsigned int index) {
	return mIKChain[index];
}
void FABRIKSolver::SetLocalTransform(unsigned int index, const Transform& t) {
	mIKChain[index] = t;
}

Transform FABRIKSolver::GetGlobalTransform(unsigned int index) {
	unsigned int size = (unsigned int)mIKChain.size();
	Transform world = mIKChain[index];
	for (int i = (int)index - 1; i >= 0; --i) {
		world = Transform::apply(mIKChain[i], world);
	}
	return world;
}

void FABRIKSolver::IKChainToWorld() {
	unsigned int size = Size();
	for (unsigned int i = 0; i < size; ++i) {
		Transform world = GetGlobalTransform(i);
		mWorldChain[i] = world.position;
		if (i >= 1) {
			Vector3 prev = mWorldChain[i - 1];
			mLengths[i] = (world.position - prev).length();
		}
	}
	if (size > 0) {
		mLengths[0] = 0.0f;
	}
}

void FABRIKSolver::WorldToIKChain() {
	unsigned int size = Size();
	if (size == 0) { return; }
	for (unsigned int i = 0; i < size - 1; ++i) {
		Transform world = GetGlobalTransform(i);
		Transform next = GetGlobalTransform(i + 1);
		Vector3 position = world.position;
		Quat rotation = world.rotation;
		Vector3 toNext = next.position - position;
		toNext = (rotation.inverse()).xform(toNext);
		Vector3 toDesired = mWorldChain[i + 1] - position;
		toDesired = (rotation.inverse()).xform(toDesired);
		Quat delta = Transform::fromTo(toNext, toDesired);
		mIKChain[i].rotation = delta * mIKChain[i].rotation;
	}
}

void FABRIKSolver::IterateBackward(const Vector3& goal) {
	int size = (int)Size();
	if (size > 0) {
		mWorldChain[size - 1] = goal;
	}
	for (int i = size - 2; i >= 0; --i) {
		Vector3 direction = (mWorldChain[i] - mWorldChain[i + 1]);
		direction.normalize();
		Vector3 offset = direction * mLengths[i + 1];
		mWorldChain[i] = mWorldChain[i + 1] + offset;
	}
}

void FABRIKSolver::IterateForward(const Vector3& base) {
	unsigned int size = Size();
	if (size > 0) {
		mWorldChain[0] = base;
	}
	for (int i = 1; i < size; ++i) {
		Vector3 direction = (mWorldChain[i] - mWorldChain[i - 1]);
		direction.normalize();
		Vector3 offset = direction * mLengths[i];
		mWorldChain[i] = mWorldChain[i - 1] + offset;
	}
}

bool FABRIKSolver::Solve(const Transform& target) {
	unsigned int size = Size();
	if (size == 0) { return false; }
	unsigned int last = size - 1;
	float thresholdSq = mThreshold * mThreshold;

	IKChainToWorld();
	Vector3 goal = target.position;
	Vector3 base = mWorldChain[0];

	for (unsigned int i = 0; i < mNumSteps; ++i) {
		Vector3 effector = mWorldChain[last];
		if ((goal - effector).length_squared() < thresholdSq) {
			WorldToIKChain();
			return true;
		}
		IterateBackward(goal);
		IterateForward(base);

		// Constraints
		//WorldToIKChain();
		// -> APPLY CONSTRAINTS HERE!
		//IKChainToWorld();
	}

	WorldToIKChain();
	Vector3 effector = GetGlobalTransform(last).position;
	if ((goal - effector).length_squared() < thresholdSq) {
		return true;
	}
	return false;
}