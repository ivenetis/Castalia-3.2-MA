#ifndef _VCL_MIP_MH_H_
#define _VCL_MIP_MH_H_

#include <climits>
#include "VirtualApplication.h"
#include "VCL_MIP_MH_PosPkt_m.h"
#include "VCL_MIP_MH_MA_Pkt_m.h"

#define MAX_NUM_OF_SENSORS		(1000 + 1)
#define NO_ID				(-1000)

#define WAIT_FOR_POSITIONS_INTERVAL	(5.0)
#define REQUEST_POSITIONS_INTERVAL	(300.0)
#define SEND_PACKET_DELAY		(0.5)

#define NO_REPORT_RECEIVED	(0)
#define REPORT_RECEIVED		(1)
#define REPORT_END_OF_TURN	(-1)

#define MAX_TRANSMISSION_LEVEL	(TransmissionLevel[0])
#define MAX_TRANSMISSION_POWER	(TransmissionPower[0])
#define MAX_TRANSMISSION_RANGE	(TransmissionRange[0])

//#define LOG_ENABLED

#define REQUEST_POSITIONS_PACKET	"VCL_MIP_MH REQUEST_POSITIONS_PACKET"
#define SEND_POSITION_PACKET		"VCL_MIP_MH SEND_POSITION_PACKET"
#define END_OF_TURN_PACKET		"VCL_MIP_MH END_OF_TURN_PACKET"
#define MA_PACKET			"VCL_MIP_MH MA_PACKET"

/*
 * The following definitions have to be updated if the Mobile Agent packet definition changes (.msg file in current directory).
 */
#define getInitSize(nodesInPath)	(3 * sizeof(int) + codeSize + nodesInPath * sizeof(Path_t))
#define getPathStart(MA_PktDataStruct)	((Path_t *)(((char *)((MA_PktDataStruct).data)) + codeSize))
//#define getDataStart(MA_PktDataStruct)	((void *)((char *)((MA_PktDataStruct).data) + (MA_PktDataStruct).nodesInPath * sizeof(Path_t)))
//#define getDataEnd(MA_PktDataStruct)	((void *)((char *)(&(MA_PktDataStruct)) + (MA_PktDataStruct).currentSize))

namespace VCL_MIP_MH_NS {

	typedef struct {
		int	nodeID;
		int	prev;
		double	xCoord;
		double	yCoord;
		double	distance;
		double	level;
		double	power;
		double	cost;
		bool	used;
	} Info_t;

	typedef struct {
		int	nodeID;
		bool	collect;
		double	level;
	} Path_t;

	enum Timers {
		REQUEST_SAMPLE		=	1,
		REQUEST_POSITIONS	=	2,
		SEND_PACKET		=	3,
		FIND_PATH		=	4,
		WAIT_FOR_POSITIONS	=	5,
	};

	class VCL_MIP_MH:public VirtualApplication {
	private:
		double	THRESHOLD;
		double	sigma;
		double	startTime;
		double	endTime;
		double	instantiationDelay;
		double	processingDelay;
		double	initialEnergy;
		double	startEnergy;
		double	spentEnergy;
		double	aggregationEnergy;
		int	ID;
		int	prevNodeID;
		int	sampleInterval;
		int	sensorsReported;
		int	numberOfSensors;
		int	reported;
		int	listSize;
		int	bytesCollected;
		int	codeSize;
		int	DijkstraType;
		int	iterations;
		bool	createMAPI;
		void DijkstraDistance ( int source, vector<double> &MinVect, vector<int> &PrevVect );
		void DijkstraPower ( int source, vector<double> &MinVect, vector<int> &PrevVect );

	protected:
		virtual void startup();
		void fromNetworkLayer ( ApplicationPacket *, const char *, double, double );
		void timerFiredCallback ( int );
		void handleSensorReading ( SensorReadingMessage * );
		void finishSpecific();
	};

	double *TransmissionLevel;
	double *TransmissionPower;
	double *TransmissionRange;

	/*
	 * These variables should be in the memory of the sink. However, in Castalia it is not possible
	 * to define a separate structure for the sink and another one for the sensors.
	 * Having these variables in the common structure would lead to excessive use of memory.
	 * To avoid this we declare the variables global and make certain that only the sink uses them.
	 */
	Info_t	Info[MAX_NUM_OF_SENSORS][MAX_NUM_OF_SENSORS];

	int		savedS;
	int		totalNodes;
	int		totalItineraries;
	int		totalIntermediateNodes;
	int		maxIntermediateNodes;
	int		maxIntermediateNodesItinerary;
	int		maxItineraryWithIntermediateNodes1;
	int		maxItineraryWithIntermediateNodes2;
	int		maxItineraryWithoutIntermediateNodes1;
	int		maxItineraryWithoutIntermediateNodes2;
	double		maxItineraryTravelDistance;
	double		totalTravelDistance;
	double		MaxTransmissionCost;
	set<int>	Vleft;
	set<int>	Vgroup;
	vector<double>	G;
	list<double>	time;
	vector<int>	PrevVect;
	vector<double>	MinVect;
	Path_t		*Path;
	MA_PktData	Fill;
	MA_Pkt		*MA;
	FILE		*MAPIfd;
}

using namespace VCL_MIP_MH_NS;

#endif	// _VCL_MIP_MH_H_
