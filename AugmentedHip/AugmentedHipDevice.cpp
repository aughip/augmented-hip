#include "AugmentedHipDevice.h"

AugmentedHipDevice::AugmentedHipDevice(
	const int& id_v,
	AugmentedHipSettings settings_v
)
	:
		settings(settings_v),
		id(id_v)
{
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

	m_sSerialNumber = "AUG_HIP_" + std::to_string(id);
	m_sModelNumber = "AUGMNT_HIP_" + std::to_string(id);

	// Init FABRIK solvers
	headChainSolver.Resize(3);
	rightLegChainSolver.Resize(3);
	leftLegChainSolver.Resize(3);
}

AugmentedHipDevice::~AugmentedHipDevice()
{

}

vr::EVRInitError AugmentedHipDevice::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
	vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, ("/devices/htc/vive_tracker" + m_sSerialNumber).c_str(), "TrackerRole_Waist");

	// Taken from https://github.com/KinectToVR/k2vr-application/blob/dff6aa8deee966d5fe9fba93fbacbd59343fedba/driver_KinectToVR/K2Tracker.cpp#L159
	
	m_unObjectId = unObjectId;
	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
	augHipSerialCode = "htc/vive_tracker" + m_sSerialNumber;

	// strings for this to use the vive puck render model
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Vive. Tracker PVT");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");

	// Tracker type
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_NeverTracked_Bool, false);
	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);

	// register as a vive tracker
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, augHipSerialCode.c_str());
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/tracker/waist_profile.json");

	// Properties to make this appear as though it were a real vive tracker, but not actually offer things like firmware updates
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Identifiable_Bool, true);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_RemindUpdate_Bool, false);
	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid); // should this be OptOut? see IsRoleAllowedAsHand

	// Set role to waist tracker
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker_waist");

	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, -1);

	// Disable HMD specific props; this is a tracker
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);

	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, 2);

	// Battery
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceCanPowerOff_Bool, true);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1);

	// Wireless
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ConnectedWirelessDongle_String, "D0000BE000");

	// "Firmware"
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingFirmwareVersion_String,
		"1541800000 RUNNER-WATCHMAN$runner-watchman@runner-watchman 2018-01-01 FPGA 512(2.56/0/0) BL 0 VRC 1541800000 Radio 1518800000");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String,
		"product 128 rev 2.5.6 lot 2000/0/0 0");

	// Updating stuff
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_UpdateAvailable_Bool, false);
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdate_Bool, false);
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdateURL_String,
		"https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 2214720000);
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, 1541800000);
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_FPGAVersion_Uint64, 512);
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_VRCVersion_Uint64, 1514800000);
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_RadioVersion_Uint64, 1518800000);
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_DongleVersion_Uint64, 8933539758);

	// YOU CAN'T VIBRATE THE AIR UNLESS YOU'RE LISTENING TO REALLY LOUD DRUM AND BASS!!!!
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Identifiable_Bool, false);

	// Vive tracker icons
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String,
		"{htc}/icons/tracker_status_off.png");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String,
		"{htc}/icons/tracker_status_searching.gif");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String,
		"{htc}/icons/tracker_status_searching_alert.gif");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String,
		"{htc}/icons/tracker_status_ready.png");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String,
		"{htc}/icons/tracker_status_ready_alert.png");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String,
		"{htc}/icons/tracker_status_error.png");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String,
		"{htc}/icons/tracker_status_standby.png");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String,
		"{htc}/icons/tracker_status_ready_low.png");

	// vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &haptic);

	activated = true;

	return vr::VRInitError_None;
}

void AugmentedHipDevice::Deactivate()
{
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
}

void AugmentedHipDevice::EnterStandby() {}

void* AugmentedHipDevice::GetComponent(const char* pchComponentNameAndVersion)
{
	// override this to add a component to a driver
	return NULL;
}

void AugmentedHipDevice::PowerOff() {}

void AugmentedHipDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {}

// Stolen from owotrack
vr::DriverPose_t AugmentedHipDevice::GetPose()
{
	vr::DriverPose_t pose = { 0 };
	pose.poseIsValid = false;
	pose.result = vr::TrackingResult_Calibrating_OutOfRange;
	pose.deviceIsConnected = false;

	pose.vecPosition[0] = 9999999;
	pose.vecPosition[1] = 9999999;
	pose.vecPosition[2] = 9999999;

	pose.vecVelocity[0] = 0.0;
	pose.vecVelocity[1] = 0.0;
	pose.vecVelocity[2] = 0.0;

	pose.qWorldFromDriverRotation = quaternion::init(0, 0, 0, 1);
	pose.qDriverFromHeadRotation = quaternion::init(0, 0, 0, 1);

	return pose;
}


void AugmentedHipDevice::send_invalid_pose() {
	vr::DriverPose_t pose = GetPose();

	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, pose, sizeof(pose));
}

void AugmentedHipDevice::RunFrame(vr::TrackedDevicePose_t* poses) {
	if (!activated) return;

	vr::DriverPose_t pose = { 0 };
	pose.poseIsValid = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	int trackedDevicePositionContributorCount = 0;

	// Find the trackers we want to pull off this algorithm
	for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
		if (poses[i].bDeviceIsConnected && poses[i].bPoseIsValid) {
			auto balls = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);
			auto deviceID = vr::VRProperties()->GetInt32Property(balls, vr::Prop_DeviceClass_Int32);
			auto ballsHD_RTX = vr::VRProperties()->GetStringProperty(balls, vr::Prop_RegisteredDeviceType_String);

			switch (deviceID) {
				case vr::TrackedDeviceClass_GenericTracker:
					if (ballsHD_RTX != augHipSerialCode) {

						// Generic tracker
						auto trackerRole = vr::VRProperties()->GetStringProperty(balls, vr::Prop_ControllerType_String);

						if (trackerRole == "vive_tracker_left_foot")
						{
							left_tracker = from_hmdMatrix(poses[i].mDeviceToAbsoluteTracking).get_rotation_quat();

							left_leg_pos.x = poses[i].mDeviceToAbsoluteTracking.m[0][3];
							left_leg_pos.y = poses[i].mDeviceToAbsoluteTracking.m[1][3];
							left_leg_pos.z = poses[i].mDeviceToAbsoluteTracking.m[2][3];
						}
						else if (trackerRole == "vive_tracker_right_foot") {

							right_tracker = from_hmdMatrix(poses[i].mDeviceToAbsoluteTracking).get_rotation_quat();

							right_leg_pos.x = poses[i].mDeviceToAbsoluteTracking.m[0][3];
							right_leg_pos.y = poses[i].mDeviceToAbsoluteTracking.m[1][3];
							right_leg_pos.z = poses[i].mDeviceToAbsoluteTracking.m[2][3];
						}

						pose.vecPosition[0] += poses[i].mDeviceToAbsoluteTracking.m[0][3];
						pose.vecPosition[1] += poses[i].mDeviceToAbsoluteTracking.m[1][3];
						pose.vecPosition[2] += poses[i].mDeviceToAbsoluteTracking.m[2][3];

						trackedDevicePositionContributorCount++;
						
					}
					break;
				case vr::TrackedDeviceClass_Controller:
					// A controller
					break;
				case vr::TrackedDeviceClass_HMD:

					// HMD

					Basis offset_basis;
					offset_basis.set(
						poses[i].mDeviceToAbsoluteTracking.m[0][0],
						poses[i].mDeviceToAbsoluteTracking.m[0][1],
						poses[i].mDeviceToAbsoluteTracking.m[0][2],
						poses[i].mDeviceToAbsoluteTracking.m[1][0],
						poses[i].mDeviceToAbsoluteTracking.m[1][1],
						poses[i].mDeviceToAbsoluteTracking.m[1][2],
						poses[i].mDeviceToAbsoluteTracking.m[2][0],
						poses[i].mDeviceToAbsoluteTracking.m[2][1],
						poses[i].mDeviceToAbsoluteTracking.m[2][2]
					);

					Quat quat = from_hmdMatrix(poses[i].mDeviceToAbsoluteTracking).get_rotation_quat();

					hmd_tracker = from_hmdMatrix(poses[i].mDeviceToAbsoluteTracking).get_rotation_quat();

					pose.vecPosition[0] += poses[i].mDeviceToAbsoluteTracking.m[0][3];
					pose.vecPosition[1] += poses[i].mDeviceToAbsoluteTracking.m[1][3];
					pose.vecPosition[2] += poses[i].mDeviceToAbsoluteTracking.m[2][3];

					hmd_pos.x = poses[i].mDeviceToAbsoluteTracking.m[0][3];
					hmd_pos.y = poses[i].mDeviceToAbsoluteTracking.m[1][3];
					hmd_pos.z = poses[i].mDeviceToAbsoluteTracking.m[2][3];

					trackedDevicePositionContributorCount++;

					break;
			}
		}
	}

	// If we have less than 3 devices then disable the tracker
	if (trackedDevicePositionContributorCount < 3) {
		// identity rotation
		pose.qRotation = quaternion::init(0,0,0,1);

		// fallback with hmd rotation
		pose.vecPosition[0] = hmd_pos.x;
		pose.vecPosition[1] = hmd_pos.y;
		pose.vecPosition[2] = hmd_pos.z;

		// "hip offset"
		Vector3 rotatedOffsetFromHead = hip_tracker.xform(settings.offset_local_tracker);
		pose.vecPosition[0] += rotatedOffsetFromHead.x;
		pose.vecPosition[1] += rotatedOffsetFromHead.y;
		pose.vecPosition[2] += rotatedOffsetFromHead.z;

		// deactivate
		send_invalid_pose();
		return;
	} else {

		// automagically try calibrating our height
		AutoCalibHeight();

		// average orientation of legs
		balls_tracker = left_tracker.slerp(right_tracker, 0.5);

		Quat final_hip_rot;
		final_hip_rot = balls_tracker;
		final_hip_rot *= Quat(Vector3(Math_PI/2.0, Math_PI, 0));
		final_hip_rot.normalize();

#define PrtVec3(vec) ("{ x: " + std::to_string(vec.x) + " y: " + std::to_string(vec.y) + " z: " + std::to_string(vec.z) + " }")

		double delta = left_tracker.dot(right_tracker);

		// Calculate the average position of the trackers to get a REALLY rough estimate as to hip position
		// This will get refined in the IK pass below
		pose.vecPosition[0] /= trackedDevicePositionContributorCount;
		pose.vecPosition[1] /= trackedDevicePositionContributorCount;
		pose.vecPosition[2] /= trackedDevicePositionContributorCount;

		// Prepare for Inverse Kinematics
		Transform headBone = Transform(hmd_pos);
		Transform leftLegBone = Transform(left_leg_pos);
		Transform rightLegBone = Transform(right_leg_pos);
		Vector3 hipPosition = Vector3(pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);
		Transform targetTransform = Transform(hipPosition);

#define CalcBone(bone, rot, delta) Transform(delta * Vector3(0, 1, 0))

		auto waistTrans = CalcBone(headBone, hmd_tracker, -SPINE_HEIGHT * getHeightRatio());
		auto waistEstimate = hmd_pos + waistTrans.position;

		// Inverse painematics
		headChainSolver.SetLocalTransform(0, headBone); // head
		headChainSolver.SetLocalTransform(1, CalcBone(headBone, hmd_tracker, -NECK_HEIGHT * getHeightRatio())); // neck
		headChainSolver.SetLocalTransform(2, CalcBone(headBone, hmd_tracker, -SPINE_HEIGHT * getHeightRatio())); // "hip"
		headChainSolver.Solve(targetTransform);

		rightLegChainSolver.SetLocalTransform(0, rightLegBone); // foot
		rightLegChainSolver.SetLocalTransform(1, rightLegBone.lerp(rightLegBone, hipPosition, LOWER_LEG_HEIGHT* getHeightRatio()) - rightLegBone); // knee
		rightLegChainSolver.SetLocalTransform(2, rightLegBone.lerp(rightLegBone, hipPosition, THIGH_HEIGHT* getHeightRatio()) - rightLegBone); // "hip"
		rightLegChainSolver.Solve(targetTransform);

		leftLegChainSolver.SetLocalTransform(0, leftLegBone); // foot
		leftLegChainSolver.SetLocalTransform(1, leftLegBone.lerp(leftLegBone, hipPosition, LOWER_LEG_HEIGHT* getHeightRatio()) - leftLegBone); // knee
		leftLegChainSolver.SetLocalTransform(2, leftLegBone.lerp(leftLegBone, hipPosition, THIGH_HEIGHT* getHeightRatio()) - leftLegBone); // "hip"
		leftLegChainSolver.Solve(targetTransform);
		
		// apply IK pos
		Vector3 trackerPos = headChainSolver.GetGlobalTransform(2).position * 0.5;
		trackerPos += rightLegChainSolver.GetGlobalTransform(2).position * 0.25;
		trackerPos += leftLegChainSolver.GetGlobalTransform(2).position * 0.25;

		pose.vecPosition[0] = trackerPos.x;
		pose.vecPosition[1] = trackerPos.y;
		pose.vecPosition[2] = trackerPos.z;

		// rotation
		
		// leg directions
		Vector3 leftLegDir = (trackerPos - left_leg_pos).normalized();
		Vector3 rightLegDir = (trackerPos - right_leg_pos).normalized();

		// since we dont actually have a spine direction with this IK rig, we have to guess it using the hmd position...
		Vector3 spineDir = (trackerPos - hmd_pos).normalized();
		spineDir = final_hip_rot.xform(Vector3(0, 1, 0)).normalized();
		
		// hip tracker direction
		Vector3 roughAngle = leftLegDir.cross(rightLegDir).normalized();

		// interpolate between hip and spine
		roughAngle = (roughAngle * 0.4 + (spineDir * 0.6)).normalized();

		// angle between hip and HMD
		double dotProd = roughAngle.dot(hmd_tracker.xform(Vector3(0, 0, 1)));

		// flip if we're facing the negative direction to fix the cross product's result
		if (dotProd < 0) {
			roughAngle = -(leftLegDir.cross(rightLegDir).normalized());

			// interpolate between hip and spine
			roughAngle = (roughAngle * 0.4 + (-spineDir * 0.6)).normalized();
		}

		final_hip_rot = Basis::looking_at(roughAngle, Vector3(0, 1, 0)).get_rotation_quat().normalized();
		pose.qRotation = quaternion::from_Quat(final_hip_rot);
	}

	// values no one gives a shit about
	pose.vecVelocity[0] = 0.0;
	pose.vecVelocity[1] = 0.0;
	pose.vecVelocity[2] = 0.0;
	pose.qWorldFromDriverRotation = quaternion::init(0, 0, 0, 1);
	pose.qDriverFromHeadRotation = quaternion::init(0, 0, 0, 1);

	// submit
	vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, pose, sizeof(pose));
}

void AugmentedHipDevice::ProcessEvent(const vr::VREvent_t& vrEvent)
{
}

const Basis& AugmentedHipDevice::get_last_basis() {
	return last_basis;
}

const char* AugmentedHipDevice::GetSerialNumber() const { return m_sSerialNumber.c_str(); }
const char* AugmentedHipDevice::GetModelNumber() const { return m_sModelNumber.c_str(); }

const char* AugmentedHipDevice::GetId() const { return GetSerialNumber(); }

void AugmentedHipDevice::AutoCalibHeight() {

	// Average the legs' position, then calculate the distance between them and our hmd to get a good enough guess on player height
	Vector3 avg_leg_pos = (left_leg_pos + right_leg_pos) * 0.5;

	double guesstimatedHeight = avg_leg_pos.distance_to(hmd_pos);

	guesstimatedHeight = CLAMP(guesstimatedHeight, MIN_AUTO_HEIGHT, MAX_AUTO_HEIGHT);

	// THIS IS BAD, as it will make you shorter when you enter a "brace for impact" position
	// TODO: better algorithm for auto height calib that takes an average over time, maximising for tallest height or something
	settings.realPlayerHeight = guesstimatedHeight;

}