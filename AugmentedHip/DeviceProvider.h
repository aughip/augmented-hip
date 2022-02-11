#pragma once

#include <openvr_driver.h>
#include "driverlog.h"

#include "AbstractDevice.h"
#include "AugmentedHipDevice.h"

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
private:

	int add_tracker();

	std::vector<AugmentedHipDevice*> trackers;
	std::vector<AbstractDevice*> devices;

	vr::TrackedDevicePose_t* poses;
public:
	virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
};