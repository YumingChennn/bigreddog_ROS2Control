#ifndef CALLBACK_HANDLER_H
#define CALLBACK_HANDLER_H

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>

#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <list>

struct SensorData {
    XsVector acc;
    XsVector gyr;
    XsVector mag;
    XsQuaternion quat;
    XsEuler euler;
    XsVector latlon;
    double altitude = 0;
    XsVector velocity;
};

bool isSensorDataValid(const SensorData& data); 

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5);
    virtual ~CallbackHandler() throw();

    bool packetAvailable() const;
    XsDataPacket getNextPacket();

protected:
    virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet);

private:
    mutable xsens::Mutex m_mutex;
    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};


#endif // CALLBACK_HANDLER_H
