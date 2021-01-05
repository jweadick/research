#ifndef POLICESERVICE_H_
#define POLICESERVICE_H_

#include "artery/application/ItsG5Service.h"

// forward declaration
namespace traci { class VehicleController; }

class PoliceService : public artery::ItsG5Service
{
    public:
        void trigger() override;
        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

	private:
	traci::VehicleController* mVehicleController = nullptr;
	
    protected:
        void initialize() override;
        void slowDown();
        void speedUp();
        bool ackRxd = false;
        int count = 0;
};

#endif /* POLICESERVICE_H_ */
