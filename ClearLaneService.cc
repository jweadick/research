#include "ClearLaneService.h"
#include "police_msgs/MotiveReq_m.h"
#include "mathCar_msgs/MathCar_m.h"
#include "artery/traci/VehicleController.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <sstream>
#include <cmath>

using namespace omnetpp;
using namespace vanetza;

Define_Module(ClearLaneService)

void ClearLaneService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
}

//use flag to set trigger function

void ClearLaneService::trigger()
{
	Enter_Method("ClearLaneService trigger");
	if(packetRxd)
	{
		//prepares btp packet
		btp::DataRequestB reqMathCar;
    	reqMathCar.destination_port = host_cast<ClearLaneService::port_type>(getPortNumber());
    	reqMathCar.gn.transport_type = geonet::TransportType::SHB;
    	reqMathCar.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
    	reqMathCar.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
    	
    	if(motiveProceed)
    	{
    		//reset flags
    		motiveProceed = false;
    		
    		//send MOTIVEProceed signal
    		auto mathCarAck = new MathCar();
    		mathCarAck->setRxdPCL(true);
    		mathCarAck->setMotiveProceedMsg(true);
    		mathCarAck->setByteLength(40);
    		request(reqMathCar, mathCarAck);
    	}
    	else{
    		//create packet
    		auto mathCarAck = new MathCar();
    		mathCarAck->setRxdPCL(true);
    		mathCarAck->setMotiveProceedMsg(false);
    		mathCarAck->setByteLength(40);
    		request(reqMathCar, mathCarAck);
    	}
    	packetRxd = false;
    }
    

}

void ClearLaneService::indicate(const vanetza::btp::DataIndication& ind, omnetpp::cPacket* packetData)
{
    Enter_Method("MathCarService indicate");
    auto motiveVehicMsg = check_and_cast<const MotiveReq*>(packetData);
    packetRxd = true;
    
    //get MOTIVE vehicle data
    double motiveVehicAngle = motiveVehicMsg->getVehicleAngle(); //Angle is in reference from (0,0) in degrees
    double motiveVehicPosX = motiveVehicMsg->getVehiclePositionX();
    double motiveVehicPosY = motiveVehicMsg->getVehiclePositionY();
    double motiveVehicSpeed = motiveVehicMsg->getVehicleSpeed(); //m/s
    
    //get mathCar vehicle data
    auto& vehicle_api = mVehicleController->getLiteAPI().vehicle();
    const std::string id = mVehicleController->getVehicleId();
    
    	//converting position variables to double format
    	libsumo::TraCIPosition temp_position = vehicle_api.getPosition(id);
    	std::stringstream ss;
    	ss.str(temp_position.getString());
    	double vehiclePositionX = 0.0;
    	double vehiclePositionY = 0.0;
    	ss >> vehiclePositionX >> vehiclePositionY;
    	
    //continue getting mathCar data
    double mathCarAngle = vehicle_api.getAngle(id); //Angle is in reference from (0,0) in degrees
    double mathCarPosX = vehiclePositionX;
    double mathCarPosY = vehiclePositionY;
    double mathCarSpeed = vehicle_api.getSpeed(id); //meters/second
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    
    //find final positions of mathCar and motiveVehic and see if they're in range
    
    //MOTIVE car math variable initializers
    double motiveProcessingTime = 10; //seconds
    double motiveVehicHypotenuse = 0;
  	double motiveVehicTempAngle = 0;
    double motiveFinalX = 0;
    double motiveFinalY = 0;
    
    
    //find estimated distance after motiveProcessingTime
    motiveVehicHypotenuse = motiveProcessingTime * motiveVehicSpeed;
  
  	//find trig angle to be used
   	if(motiveVehicAngle >= 90 && motiveVehicAngle <= 270)
   	{
   		motiveVehicTempAngle = abs(180 - motiveVehicAngle);
   	}
   	else if(motiveVehicAngle > 270)
   	{
   		motiveVehicTempAngle = abs(360 - motiveVehicAngle);
   	}
   	//Add or subtract distance traveled in X or Y direction as necessary
   	if(motiveVehicAngle <= 90 || motiveVehicAngle >= 270)
   	{
   		motiveFinalX = motiveVehicPosX + (motiveVehicHypotenuse * cos(motiveVehicTempAngle * (PI/180.0)));
   	}
   	else{
   		motiveFinalX = motiveVehicPosX - (motiveVehicHypotenuse * cos(motiveVehicTempAngle * (PI/180.0)));
   	}
   	if(motiveVehicAngle >= 180)
   	{
   		motiveFinalY = motiveVehicPosY - (motiveVehicHypotenuse * sin(motiveVehicTempAngle * (PI/180.0)));
   	}
   	else{
   		motiveFinalY = motiveVehicPosY + (motiveVehicHypotenuse * sin(motiveVehicTempAngle * (PI/180.0)));
   	}
   	
   	//mathCar variable initializers
   	double mathCarHypotenuse = 0;
   	double mathCarFinalX = 0;
   	double mathCarFinalY = 0;
   	double mathCarTempAngle = 0;
   	
   	//mathCar math let's gooooooooo
   	mathCarHypotenuse = motiveProcessingTime * mathCarSpeed;
   	
   	//find trig angle to be used
   	if(mathCarAngle >= 90 && mathCarAngle <= 270)
   	{
   		mathCarTempAngle = abs(180 - mathCarAngle);
   	}
   	else if(mathCarAngle > 270)
   	{
   		mathCarTempAngle = abs(360 - mathCarAngle);
   	}
   	
   	//Add or subtract distance traveled in X or Y direction as necessary
   	if(mathCarAngle <= 90 || mathCarAngle >= 270)
   	{
   		mathCarFinalX = motiveVehicPosX + (mathCarHypotenuse * cos(mathCarTempAngle * (PI/180.0)));
   	}
   	else{
   		mathCarFinalX = mathCarPosX - (mathCarHypotenuse * cos(mathCarTempAngle * (PI/180.0)));
   	}
   	if(mathCarAngle >= 180)
   	{
   		mathCarFinalY = mathCarPosY - (mathCarHypotenuse * sin(mathCarTempAngle * (PI/180.0)));
   	}
   	else{
   		mathCarFinalY = mathCarPosY + (mathCarHypotenuse * sin(mathCarTempAngle * (PI/180.0)));
   	}
   	
   	//find distance between two final positions
   	double finalDist = sqrt(pow((motiveFinalX - mathCarFinalX), 2) + pow((motiveFinalY - mathCarFinalY), 2));
   	//the big boy trigger
   	if(finalDist <= 250)
   	{
   		motiveProceed = true;
   	}
    
    delete motiveVehicMsg;
}

void ClearLaneService::slowDown(){
	mVehicleController->setMaxSpeed(5 * units::si::meter_per_second);
}



/*
#include "ClearLaneService.h"
#include "police_msgs/PoliceClearLane_m.h"
#include "artery/traci/VehicleController.h"

using namespace omnetpp;
using namespace vanetza;

Define_Module(ClearLaneService)

void ClearLaneService::initialize()
{
    ItsG5BaseService::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
}

void ClearLaneService::indicate(const vanetza::btp::DataIndication& ind, omnetpp::cPacket* packet)
{
    Enter_Method("ClearLaneService indicate");
    auto clearLaneMessage = check_and_cast<const PoliceClearLane*>(packet);

    const std::string id = mVehicleController->getVehicleId();
    auto& vehicle_api = mVehicleController->getLiteAPI().vehicle();
    if (vehicle_api.getRoadID(id) == clearLaneMessage->getEdgeName()) {
        if (vehicle_api.getLaneIndex(id) != clearLaneMessage->getLaneIndex()) {
            slowDown();
        }
    }

    delete clearLaneMessage;
}

void ClearLaneService::slowDown()
{
    mVehicleController->setMaxSpeed(25 * units::si::meter_per_second);
}
*/


