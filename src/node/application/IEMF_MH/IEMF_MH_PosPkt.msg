cplusplus {{
#include "ApplicationPacket_m.h"
}}

class ApplicationPacket;

namespace IEMF_MH_NS;

struct PosPktData {
	int	nodeID;
	double	xCoord;
	double	yCoord;
}

packet PosPkt extends ApplicationPacket {
	PosPktData extraData;
}

