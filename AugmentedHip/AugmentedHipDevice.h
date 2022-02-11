#pragma once

#include <openvr_driver.h>
#include "driverlog.h"
#include "basis.h"
#include "quaternion.h"

#include "AbstractDevice.h"
#include "AugmentedHipSettings.h"

#include "IK/FABRIKSolver.h"

class AugmentedHipDevice : public AbstractDevice {

private:

	vr::VRInputComponentHandle_t haptic;
	AugmentedHipSettings settings;

	bool is_calibrating = false;
	bool is_down_calibrating = false;

	Basis last_basis;

	bool activated = false;
public:
	unsigned int id = 0;
	unsigned int port_no = 0;

	AugmentedHipDevice(const int& id, AugmentedHipSettings settings_v);
	~AugmentedHipDevice();

	vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) override;

	void Deactivate() override;
	void EnterStandby() override;

	void* GetComponent(const char* pchComponentNameAndVersion) override;

	void PowerOff() override;

	/** debug request from a client */
	void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;

	vr::DriverPose_t GetPose() override;

	void RunFrame(vr::TrackedDevicePose_t* poses) override;

	void ProcessEvent(const vr::VREvent_t& vrEvent) override;

	const char* GetSerialNumber() const override;
	const char* GetModelNumber() const override;
	const char* GetId() const override;

	void AutoCalibHeight();

	void send_invalid_pose();
	const Basis& get_last_basis();

	// Yes i know this is a shit IK system. I'm going to improve it with a better rig later
	FABRIKSolver headChainSolver = FABRIKSolver();
	FABRIKSolver rightLegChainSolver = FABRIKSolver();
	FABRIKSolver leftLegChainSolver = FABRIKSolver();

	// Variables
	Vector3 hmd_pos;
	Vector3 left_leg_pos;
	Vector3 right_leg_pos;

	// My behated
	Quat identity = Quat::Quat(0, 0, 0, 1);

	Quat hmd_tracker = Quat::Quat(0, 0, 0, 1);
	Quat left_tracker = Quat::Quat(0, 0, 0, 1);
	Quat right_tracker = Quat::Quat(0, 0, 0, 1);
	Quat balls_tracker = Quat::Quat(0, 0, 0, 1);
	Quat hip_tracker = Quat::Quat(0, 0, 0, 1);

	// This tracker's serial code
	std::string augHipSerialCode;

	// Measurements are currently rough; taken from https://hpc.invisualdata.com/, using a 183cm human as a basis
#define SPINE_HEIGHT 0.604
#define NECK_HEIGHT 0.077
#define THIGH_HEIGHT 0.458
#define LOWER_LEG_HEIGHT 0.457

	inline double getHeightRatio() {
		return settings.realPlayerHeight / 1.83;
	}

#define MIN_AUTO_HEIGHT 1.0
#define MAX_AUTO_HEIGHT 2.0
};