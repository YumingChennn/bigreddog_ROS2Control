#include "callback_handler.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <cassert>
#include <deque>
#include <algorithm>
#include <vector>

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

    // ---- 濾波相關變數 ----
    const int medianWindow = 3;
    std::deque<XsVector3> gyroHistory;
    XsVector3 prevSmoothedGyr(0.0, 0.0, 0.0);
    double alpha = 0.1;

    std::vector<XsVector3> filteredGyrLog; // 儲存所有濾波後資料

    while (XsTime::timeStampNow() - startTime <= 10000) {
        if (callback.packetAvailable()) {
            XsDataPacket packet = callback.getNextPacket();

            if (packet.containsCalibratedData()) {
                sensorData.acc = packet.calibratedAcceleration();
                XsVector3 gyroscope = packet.calibratedGyroscopeData();
                sensorData.mag = packet.calibratedMagneticField();

                // === Step 1: Median Filter ===
                gyroHistory.push_back(gyroscope);
                if (gyroHistory.size() > medianWindow)
                    gyroHistory.pop_front();

                // 若長度不足 medianWindow，跳過本次處理
                if (gyroHistory.size() < medianWindow)
                    continue;

                XsVector3 medianGyr = gyroscope;
                for (int i = 0; i < 3; ++i) {
                    std::vector<double> axis_vals;
                    for (const auto& v : gyroHistory)
                        axis_vals.push_back(v[i]);
                    std::sort(axis_vals.begin(), axis_vals.end());
                    medianGyr[i] = axis_vals[medianWindow / 2];
                }

                // === Step 2: Exponential Moving Average ===
                XsVector3 smoothedGyr;
                if (filteredGyrLog.empty()) {
                    smoothedGyr = medianGyr;  // 第一筆直接採用
                } else {
                    for (int i = 0; i < 3; ++i) {
                        smoothedGyr[i] = alpha * medianGyr[i] + (1 - alpha) * prevSmoothedGyr[i];
                    }
                }

                prevSmoothedGyr = smoothedGyr;
                sensorData.gyr = smoothedGyr;

                // 存入 log vector
                filteredGyrLog.push_back(smoothedGyr);
            }

            // 可選擇是否保留其他資訊處理
            if (packet.containsOrientation()) {
                sensorData.quat = packet.orientationQuaternion();
                sensorData.euler = packet.orientationEuler();
            }

            if (packet.containsLatitudeLongitude()) {
                sensorData.latlon = packet.latitudeLongitude();
            }

            if (packet.containsAltitude()) {
                sensorData.altitude = packet.altitude();
            }

            if (packet.containsVelocity()) {
                sensorData.velocity = packet.velocity(XDI_CoordSysEnu);
            }

            cout << flush;
        }
        XsTime::msleep(0);
    }

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
