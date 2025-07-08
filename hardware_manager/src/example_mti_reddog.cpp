#include "callback_handler.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cassert>

using namespace std;

XsControl* control = nullptr;
XsDevice* device = nullptr;
XsPortInfo mtPort;
CallbackHandler callback;
SensorData sensorData;  // ✅ 僅儲存一筆當前資料

int init() {
    cout << "Creating XsControl object..." << endl;
    control = XsControl::construct();
    assert(control != 0);

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    for (auto const &portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty()) {
        cerr << "No MTi device found. Aborting." << endl;
        return -1;
    }

    cout << "Found device @ port: " << mtPort.portName().toStdString() << endl;

    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) {
        cerr << "Could not open port. Aborting." << endl;
        return -1;
    }

    device = control->device(mtPort.deviceId());
    assert(device != 0);
    device->addCallbackHandler(&callback);

    if (!device->gotoConfig()) {
        cerr << "Failed to enter config mode." << endl;
        return -1;
    }


	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

    if (device->deviceId().isImu()) {
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
    } else if (device->deviceId().isVru() || device->deviceId().isAhrs()) {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
    } else if (device->deviceId().isGnss()) {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
        configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
        configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
    }

    if (!device->setOutputConfiguration(configArray)) {
        cerr << "Failed to configure device." << endl;
        return -1;
    }

    if (device->createLogFile("logfile.mtb") != XRV_OK) {
        cerr << "Failed to create log file." << endl;
        return -1;
    }

    return 0;
}

int start() {
    if (!device->gotoMeasurement()) {
        cerr << "Failed to enter measurement mode." << endl;
        return -1;
    }

    if (!device->startRecording()) {
        cerr << "Failed to start recording." << endl;
        return -1;
    }

    cout << "Recording data for 10 seconds..." << endl;
    int64_t startTime = XsTime::timeStampNow();
   	while (XsTime::timeStampNow() - startTime <= 10000)
	{
		if (callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();

			if (packet.containsCalibratedData())
			{   
                sensorData.acc = packet.calibratedAcceleration();
                sensorData.gyr = packet.calibratedGyroscopeData();
                sensorData.mag = packet.calibratedMagneticField();
                // cout << "\r"
				    // << "Acc X:" << sensorData.acc[0]
					// << ", Acc Y:" << sensorData.acc[1]
					// << ", Acc Z:" << sensorData.acc[2];

                cout << "\r"
				    << " |Gyr X:" << sensorData.gyr[0]
					<< ", Gyr Y:" << sensorData.gyr[1]
					<< ", Gyr Z:" << sensorData.gyr[2];

				// cout << " |Mag X:" << sensorData.mag[0]
				// 	<< ", Mag Y:" << sensorData.mag[1]
				// 	<< ", Mag Z:" << sensorData.mag[2];
			}

			if (packet.containsOrientation())
			{
				sensorData.quat = packet.orientationQuaternion();
                sensorData.euler = packet.orientationEuler();
				// cout << "q0:" << sensorData.quat.w()
				// 	<< ", q1:" << sensorData.quat.x()
				// 	<< ", q2:" << sensorData.quat.y()
				// 	<< ", q3:" << sensorData.quat.z();

				// cout << " |Roll:" << sensorData.euler.roll()
				// 	<< ", Pitch:" << sensorData.euler.pitch()
				// 	<< ", Yaw:" << sensorData.euler.yaw();
			}

			if (packet.containsLatitudeLongitude())
			{
				sensorData.latlon = packet.latitudeLongitude();
				// cout << " |Lat:" << sensorData.latlon[0]
				// 	<< ", Lon:" << sensorData.latlon[1];
			}

			if (packet.containsAltitude())
                sensorData.altitude = packet.altitude();
				// cout << " |Alt:" << sensorData.altitude;

			if (packet.containsVelocity())
			{   
                sensorData.velocity = packet.velocity(XDI_CoordSysEnu);
				// cout << " |E:" << sensorData.velocity[0]
				// 	<< ", N:" << sensorData.velocity[1]
				// 	<< ", U:" << sensorData.velocity[2];
			}

			cout << flush;
		}
		XsTime::msleep(0);
	}
    cout << "\n" << string(79, '-') << "\n";
    cout << endl;
    return 0;
}

void stop() {
    device->stopRecording();
    device->closeLogFile();
    control->closePort(mtPort.portName().toStdString());
    control->destruct();
}

int main() {
    if (init() != 0) return -1;
    if (start() != 0) return -1;
    stop();
    cout << "\nSuccessful exit. Press [ENTER] to continue." << endl;
    cin.get();
    return 0;
}
