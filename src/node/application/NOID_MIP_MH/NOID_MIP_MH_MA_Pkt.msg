cplusplus {{
#include "ApplicationPacket_m.h"
}}

class ApplicationPacket;

namespace NOID_MIP_MH_NS;

struct MA_PktData {
	int	currentSize;
	int	nextNodeID;
	int	nodesInPath;
	char	data[524288];
}

packet MA_Pkt extends ApplicationPacket {
	MA_PktData extraData;
}

