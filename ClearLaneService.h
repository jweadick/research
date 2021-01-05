#ifndef CLEARLANESERVICE_H_
#define CLEARLANESERVICE_H_

#include "artery/application/ItsG5Service.h"

// forward declaration
namespace traci { class VehicleController; }


class ClearLaneService : public artery::ItsG5Service
{

	public:
		void trigger() override;
		void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
		bool packetRxd = false;
		bool motiveProceed = false;
		
    protected:
        void initialize() override;
        
    private:
        traci::VehicleController* mVehicleController = nullptr;
        void slowDown();
   
};

#endif /* CLEARLANESERVICE_H_ */

