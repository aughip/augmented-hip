#include "DeviceProvider.h"

int DeviceProvider::add_tracker() {

	unsigned int id = trackers.size();

	AugmentedHipSettings defaults;

	defaults.anchor_device_id = 0;
	defaults.offset_global = Vector3(0.0, 0.0, 0.0);
	defaults.offset_local_device = Vector3(0.0, 0.0, 0.0);
	defaults.offset_local_tracker = Vector3(0.0, -0.73, 0.0);

	defaults.local_rot_euler = Vector3(0.0, 0.0, 0.0);
	defaults.global_rot_euler = Vector3(0.0, 0.0, 0.0);

	defaults.yaw_offset = 0.0;

	defaults.should_predict_position = false;

	AugmentedHipDevice* tracker = new AugmentedHipDevice(id, defaults);
	vr::VRServerDriverHost()->TrackedDeviceAdded(tracker->GetSerialNumber(), vr::TrackedDeviceClass_GenericTracker, tracker);

	devices.push_back(tracker);
	trackers.push_back(tracker);

	return (int) id;

}

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {

	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());
	DriverLog("Initialising Augmented Hip 0.2...");

	poses = (vr::TrackedDevicePose_t*)malloc(sizeof(vr::TrackedDevicePose_t) * vr::k_unMaxTrackedDeviceCount);
	unsigned int id = add_tracker();

	if (trackers.size() <= id) return vr::EVRInitError::VRInitError_Init_Internal;
	AugmentedHipDevice* tracker = trackers[id];

	if (tracker == nullptr) {
		return vr::EVRInitError::VRInitError_Init_Internal;
	}
	else {
		tracker->Activate(id);
		vr::VRServerDriverHost()->TrackedDeviceAdded(tracker->GetSerialNumber(), vr::TrackedDeviceClass_Controller, tracker);
	}

	return vr::EVRInitError::VRInitError_None;
}

void DeviceProvider::Cleanup() {
	CleanupDriverLog();
	for (auto v : trackers) {
		delete ((AugmentedHipDevice*)(v));
	}
}

void DeviceProvider::RunFrame() {
	
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, poses, vr::k_unMaxTrackedDeviceCount);

	for (auto v : devices) {
		if (v == nullptr) continue;
		v->RunFrame(poses);
	}

	vr::VREvent_t vrEvent;
	while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
		for (auto v : devices) {
			if (v == nullptr) continue;
			v->ProcessEvent(vrEvent);
		}
	}
}