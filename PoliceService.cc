#include "PoliceService.h"
#include "police_msgs/MotiveReq_m.h"
#include "mathCar_msgs/MathCar_m.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <sstream>

using namespace omnetpp;
using namespace vanetza;

Define_Module(PoliceService)

void PoliceService::initialize()
{
    ItsG5BaseService::initialize();
    //part of SUMO
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
}

void PoliceService::indicate(const vanetza::btp::DataIndication& ind, omnetpp::cPacket* packetMathCar)
{
	Enter_Method("MotiveService indicate");
	const MathCar* mathCarInfo = check_and_cast<const MathCar*>(packetMathCar);
	ackRxd = true;
	if(mathCarInfo->getMotiveProceedMsg())
	{
		slowDown();
	}
}

void PoliceService::trigger()
{
    Enter_Method("MotiveService trigger");
    													//to counter this only sending once if Rxd by a MathCar, have MathCars send "ready to RX messages" in future
    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getLiteAPI().vehicle();
    count++;
    
    if(!ackRxd && count > 5){
    	//look into DataRequestA if time is an issue
    	btp::DataRequestB req;
    	req.destination_port = host_cast<PoliceService::port_type>(getPortNumber());
    	req.gn.transport_type = geonet::TransportType::SHB;
    	req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
    	req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
	
    	
    	//getting position variables
    	libsumo::TraCIPosition temp_position = vehicle_api.getPosition(id);
    	std::stringstream ss;
    	ss.str(temp_position.getString());
    	double vehiclePositionX = 0.0;
    	double vehiclePositionY = 0.0;
    	ss >> vehiclePositionX >> vehiclePositionY;
    	
    	//getting id to char * format
    	int n = id.length();
    	char id_array[n + 1];
    	std::strcpy(id_array, id.c_str());
    	
    	auto packetData = new MotiveReq();
    	packetData->setVehicleId(id_array);
    	packetData->setVehicleSpeed(vehicle_api.getSpeed(id));
    	packetData->setVehiclePositionX(vehiclePositionX);
    	packetData->setVehiclePositionY(vehiclePositionY);
    	packetData->setVehicleAngle(vehicle_api.getAngle(id));
    	packetData->setByteLength(10);
    	request(req, packetData);
    	
    }
    if(vehicle_api.getSpeed(id) <= 5.0)
    {
    	speedUp();
    }
    
}

void PoliceService::slowDown(){
	mVehicleController->setMaxSpeed(5 * units::si::meter_per_second);
}

void PoliceService::speedUp(){
	mVehicleController->setMaxSpeed(45 * units::si::meter_per_second);
}

/*
#include "PoliceService.h"
#include "police_msgs/PoliceClearLane_m.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>

using namespace omnetpp;
using namespace vanetza;

Define_Module(PoliceService)

void PoliceService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_const<traci::VehicleController>();
}

void PoliceService::trigger()
{
    Enter_Method("PoliceService trigger");
    btp::DataRequestB req;
    req.destination_port = host_cast<PoliceService::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getLiteAPI().vehicle();

    auto packet = new PoliceClearLane();
    packet->setEdgeName(vehicle_api.getRoadID(id).c_str());
    packet->setLaneIndex(vehicle_api.getLaneIndex(id));
    packet->setByteLength(40);
    request(req, packet);
}
*/
