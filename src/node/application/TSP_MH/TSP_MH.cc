#include "TSP_MH.h"

Define_Module ( TSP_MH );

/******************************************************************************/

void TSP_MH::startup()
{
	ID			= atoi ( SELF_NETWORK_ADDRESS );
	sampleInterval		= par ( "sampleInterval" );
	bytesCollected		= par ( "bytesCollected" );
	codeSize		= par ( "codeSize" );
	createMAPI		= par ( "createMAPI" );
	DijkstraType		= par ( "DijkstraType" );
	instantiationDelay	= par ( "instantiationDelay" );
	processingDelay		= par ( "processingDelay" );
	aggregationEnergy	= par ( "aggregationEnergy" );
	initialEnergy		= resMgrModule->par ( "initialEnergy" );
	numberOfSensors		= getParentModule()->getParentModule()->par ( "numNodes" );
	THRESHOLD		= 10;
	spentEnergy		= 0.0;
	if ( isSink ) {
		double	PLd0, d0, pathLossExponent, sensitivity;
		int	i = 0;

		prevNodeID		= 0;
		Info[ID][ID].nodeID	= 0;
		Info[ID][ID].xCoord	= mobilityModule->getLocation().x;
		Info[ID][ID].yCoord	= mobilityModule->getLocation().y;
		trace() << fixed << setprecision( 2 ) << " SINK: Position is x= " << mobilityModule->getLocation().x << " and y= " << mobilityModule->getLocation().y;

		PLd0 = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "PLd0" );
		d0   = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "d0" );
		pathLossExponent = getParentModule()->getParentModule()->getSubmodule("wirelessChannel")->par ( "pathLossExponent" );

		for (list <RXmode_type>::iterator it = radioModule->getRXmodeListBegin(); it != radioModule->getRXmodeListEnd(); it++) {
			if (it->name.compare("IDEAL") == 0) {
				sensitivity = it->sensitivity;
				break;
			}
		}

		listSize = radioModule->getTxLevelListSize();

		TransmissionLevel = (double *)malloc(listSize * sizeof(double));
		if (TransmissionLevel == NULL) {
			trace() << "Could not allocate memory for TransmissionLevel";
			exit(0);
		}

		TransmissionPower = (double *)malloc(listSize * sizeof(double));
		if (TransmissionPower == NULL) {
			trace() << "Could not allocate memory for TransmissionPower";
			exit(0);
		}

		TransmissionRange = (double *)malloc(listSize * sizeof(double));
		if (TransmissionRange == NULL) {
			trace() << "Could not allocate memory for TransmissionRange";
			exit(0);
		}

		for (list <TxLevel_type>::iterator it = radioModule->getTxLevelListBegin(); it != radioModule->getTxLevelListEnd(); it++, i++) {
			TransmissionLevel[i] = it->txOutputPower;
			TransmissionPower[i] = it->txPowerConsumed;
			TransmissionRange[i] = d0 * pow(10.0, (TransmissionLevel[i] - sensitivity - PLd0) / (10.0 * pathLossExponent));
			trace() << "Transmission range is " << setw( 5 ) << TransmissionRange[i] << " for transmission level " << TransmissionLevel[i];
		}

		setTimer ( REQUEST_POSITIONS, 0.5 );
	} else {
		prevNodeID		= NO_ID;
		trace() << fixed << setprecision(2) << "!SINK: Position is x= " << mobilityModule->getLocation().x << " and y= " << mobilityModule->getLocation().y;
		setTimer ( REQUEST_SAMPLE, sampleInterval );
	}
}

/******************************************************************************/

/*
 * Triggered when a packet is received
 */
void TSP_MH::fromNetworkLayer ( ApplicationPacket * genericPacket, const char *source, double rssi, double lqi )
{
	string packetName ( genericPacket->getName() );

	if ( packetName.compare ( REQUEST_POSITIONS_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		/*
		 * If this node has not already sent it's position.
		 */
		if ( prevNodeID == NO_ID ) {
			prevNodeID = atoi ( source );

			PosPktData	Fill;
			PosPkt		*newPktSend = new PosPkt ( SEND_POSITION_PACKET, APPLICATION_PACKET );
			Fill.nodeID = ID;
			Fill.xCoord = mobilityModule->getLocation().x;
			Fill.yCoord = mobilityModule->getLocation().y;
			newPktSend->setExtraData ( Fill );
			newPktSend->setByteLength ( sizeof ( Fill ) );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPktSend, source );

			PosPkt	*newPktRequest = new PosPkt ( REQUEST_POSITIONS_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPktRequest, BROADCAST_NETWORK_ADDRESS );
		}
	}
	else if ( packetName.compare ( SEND_POSITION_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);
		PosPktData	Sender = rcvPacket->getExtraData();

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		/*
		 * If the packet is received from the sink, update the information for the node from where the packet was originally sent.
		 */
		if ( isSink ) {
			/*
			 * Information is updated only if it arrives in time.
			 * Otherwise, the information is discarded, since the sink might
			 * already have calculated the distances and sorted the information.
			 */
			if ( reported != REPORT_END_OF_TURN ) {
				Info[Sender.nodeID][Sender.nodeID].nodeID = Sender.nodeID;
				Info[Sender.nodeID][Sender.nodeID].xCoord = Sender.xCoord;
				Info[Sender.nodeID][Sender.nodeID].yCoord = Sender.yCoord;
				sensorsReported++;
				reported = REPORT_RECEIVED;
			}
			/*
			 * prevNodeID could be NO_ID if an END_OF_TURN_PACKET arrived in the meantime.
			 * Normally though (if WAIT_FOR_POSITIONS_INTERVAL is large enough) this should never happen.
			 */
		} else if ( prevNodeID != NO_ID ) {
			stringstream	prevNodeIDss;
			prevNodeIDss << prevNodeID;
			string		prevNodeIDstr = prevNodeIDss.str();

			PosPkt	*newPkt = new PosPkt ( SEND_POSITION_PACKET, APPLICATION_PACKET );
			newPkt->setExtraData ( Sender );
			newPkt->setByteLength ( sizeof ( Sender ) );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, prevNodeIDstr.c_str() );
		}
	}
	else if ( packetName.compare ( END_OF_TURN_PACKET ) == 0 ) {
		PosPkt		*rcvPacket = check_and_cast<PosPkt *>(genericPacket);

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		if ( ( prevNodeID != NO_ID ) && ( !isSink ) ) {
			startEnergy = resMgrModule->getSpentEnergy();

			prevNodeID = NO_ID;

			PosPkt	*newPkt = new PosPkt ( END_OF_TURN_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );
		}
	} else if ( packetName.compare ( MA_PACKET ) == 0 ) {
		MA_Pkt		*rcvPacket = check_and_cast<MA_Pkt *>(genericPacket);
		MA_PktData	Sender = rcvPacket->getExtraData();

#if defined(LOG_ENABLED)
		trace() << "Received [" << packetName << "] from " << source;
#endif
		if ( isSink && ( Sender.nextNodeID == Sender.nodesInPath ) ) {
			endTime = simTime().dbl();
			trace() << "Mobile agent returned to sink from node " << source << " with a packet of size " << Sender.currentSize << "!";
		} else {
			Path_t	*Path = getPathStart(Sender);

			if ( Path[Sender.nextNodeID - 1].collect == 1 ) {
				Sender.currentSize += bytesCollected;

				trace() << "Data collected from node " << Path[Sender.nextNodeID - 1].nodeID;
			} else {
				trace() << "Only wandering through node " << Path[Sender.nextNodeID - 1].nodeID;
			}

			resMgrModule->consumeEnergy( aggregationEnergy );
			spentEnergy = resMgrModule->getSpentEnergy();

			stringstream	nextNodeIDss;
			nextNodeIDss << Path[Sender.nextNodeID].nodeID;
			string		nextNodeIDstr = nextNodeIDss.str();

#if defined(LOG_ENABLED)
			trace() << "Sending to node " << nextNodeIDstr << " with level " << Path[Sender.nextNodeID].level << " a message with size " << Sender.currentSize;
#endif
			MA_Pkt		*MA = new MA_Pkt ( MA_PACKET, APPLICATION_PACKET );
			Sender.nextNodeID++;
			MA->setExtraData ( Sender );
			MA->setByteLength ( Sender.currentSize );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, Path[Sender.nextNodeID - 1].level ) );
			toNetworkLayer ( MA, nextNodeIDstr.c_str() );
		}
	}
}

/******************************************************************************/

void TSP_MH::timerFiredCallback ( int index )
{
	switch ( index ) {

	case REQUEST_SAMPLE: {
		if ( !isSink ) {
			setTimer ( REQUEST_SAMPLE, sampleInterval );
			requestSensorReading();
		}
		break;
	}

	case REQUEST_POSITIONS: {
		sensorsReported	= 1;
		reported = NO_REPORT_RECEIVED;

		for ( int i = 1; i < numberOfSensors; i++ ) {
			Info[i][i].nodeID = NO_ID;
		}

		setTimer ( WAIT_FOR_POSITIONS, WAIT_FOR_POSITIONS_INTERVAL );

		PosPkt *newPkt = new PosPkt ( REQUEST_POSITIONS_PACKET, APPLICATION_PACKET );
		toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
		toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );
		break;
	}

	case SEND_PACKET: {
		/*
		 * Send the mobile agent out.
		 */
		stringstream	nextNodeIDss;
		nextNodeIDss << Path[Fill.nextNodeID].nodeID;
		string		nextNodeIDstr = nextNodeIDss.str();

#if defined(LOG_ENABLED)
		trace() << "Sending to node " << nextNodeIDstr << " with level " << Path[Fill.nextNodeID].level << " a message with size " << Fill.currentSize;
#endif
		Fill.nextNodeID++;
		MA = new MA_Pkt ( MA_PACKET, APPLICATION_PACKET );
		MA->setExtraData ( Fill );
		MA->setByteLength ( Fill.currentSize );
		toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, Path[Fill.nextNodeID - 1].level ) );
		toNetworkLayer ( MA, nextNodeIDstr.c_str() );

		break;
	}

	case FIND_PATH: {
		double	minCost, travelDistance;
		int	i, j, k, l, v, currentNode, minNode, nodesInPath, prevNodesInPath, intermediateNodes;
		FILE		*LKHfd;
		char		readBuffer[256];

		/*
		 * Write the file that LKH requires to run.
		 * Node IDs have to be reset in the range 1 to 'sensorsReported'.
		 */
		LKHfd = fopen ( "fromCastalia.tsp", "w" );
		fprintf ( LKHfd, "NAME: fromCastalia\n" );
		fprintf ( LKHfd, "COMMENT: fromCastalia\n" );
		fprintf ( LKHfd, "TYPE: ATSP\n" );
		fprintf ( LKHfd, "DIMENSION: %d\n", sensorsReported );
		fprintf ( LKHfd, "EDGE_WEIGHT_TYPE: EXPLICIT\n" );
		fprintf ( LKHfd, "EDGE_WEIGHT_FORMAT: FULL_MATRIX\n" );
		fprintf ( LKHfd, "EDGE_DATA_FORMAT: ADJ_LIST\n" );
		fprintf ( LKHfd, "EDGE_DATA_SECTION\n" );

		k = 1;
		for ( i = 0; i < numberOfSensors; i++ ) {
			if ( Info[i][i].nodeID != NO_ID ) {
				fprintf ( LKHfd, "%3d ", k );
//				trace() << i << " -> " << k;
				l = 1;
				for ( j = 0; j < numberOfSensors; j++ ) {
					if ( Info[j][j].nodeID != NO_ID ) {
						if ( ( k != l ) ) {
							fprintf ( LKHfd, "%3d ", l );
						}
						l++;
					}
				}
				fprintf ( LKHfd, "-1\n" );
				k++;
			}
		}
		fprintf ( LKHfd, "-1\n" );

		/*
		 * The weight of an edge in LKH must be an integer. Therefore, we multiply the cost of an edge
		 * by a power of 10, in order to keep some decimal points of the cost and improve accuracy.
		 */
		fprintf ( LKHfd, "EDGE_WEIGHT_SECTION\n" );
		for ( i = 0; i < numberOfSensors; i++ ) {
			if ( Info[i][i].nodeID != NO_ID ) {
				for ( j = 0; j < numberOfSensors; j++ ) {
					if ( Info[j][j].nodeID != NO_ID ) {
						if ( i != j ) {
							fprintf ( LKHfd, "%15u ", (long)round ( Info[i][j].cost * 100000.0 ) );
						} else {
							fprintf ( LKHfd, "%15u ", 0 );
						}
					}
				}
			}
			fprintf ( LKHfd, "\n" );
		}
		fprintf ( LKHfd, "EOF" );
		fclose ( LKHfd );

		/*
		 * Run LKH. The file 'toCastalia.par' has been created outside of Castalia,
		 * but uses the file created previously.
		 */
		system ( "./LKH toCastalia.par > /dev/null" );

		/*
		 * The file 'output.txt' has been set in 'toCastalia.par' to contain the final path produced by LKH.
		 * This file contains some initial lines that are not required in our case. We only need the produced
		 * path, without the first node (which is the sink and its ID has been reset to 1 above).
		 */
		LKHfd = fopen ( "output.txt", "r" );

		for ( i = 0; i < 6; i++ ) {
			fgets ( readBuffer, 256, LKHfd );
		}

		fgets ( readBuffer, 256, LKHfd );
		if ( !strcmp ( readBuffer, "1" ) ) {
			trace() << "ERROR: First node returned from TSP is not node 0.";
			fclose ( LKHfd );
			exit(0);
		}

		Path = getPathStart(Fill);
		nodesInPath = 0;
		currentNode = 0;
		Info[currentNode][currentNode].used = true;
		for ( i = 0; i < sensorsReported - 1; i++ ) {
			/*
			 * Read the next node ID for the path. That ID has to be converted back
			 * into the ID of the node in Castalia and not LKH.
			 *
			 * We keep the variables 'minNode' and 'minCost', despite them not having the
			 * same meaning as in other algorithms implemented. But this saves us from
			 * making further changes in the rest of the code.
			 */
			fgets ( readBuffer, 256, LKHfd );
			for ( j = 1, minNode = 0; j <= atoi ( readBuffer ); j++, minNode++ ) {
				if ( Info[minNode][minNode].nodeID == NO_ID ) {
					j--;
				}
			}
			minNode--;

			minCost = Info[currentNode][minNode].cost;
			if ( minCost <= MaxTransmissionCost ) {
				Path[nodesInPath].nodeID   = minNode;
				Path[nodesInPath].collect  = true;
				Path[nodesInPath].level    = Info[currentNode][minNode].level;
				if ( createMAPI == true ) {
					fprintf ( MAPIfd, "%d\t%d\n", 0, currentNode );
				}
				nodesInPath++;
				trace() << "Next hop is from node " << currentNode << " to node " << minNode << " with level= " << Info[currentNode][minNode].level;
			} else {
				trace() << "Next hop is from node " << currentNode << " to node " << minNode << ". Path to follow:";
				/*
				 * Dijkstra's algorithm returns the path to be followed in the reverse order.
				 * Firstly, we count how many nodes we have to visit to reach the destination node.
				 * Secondly, we reverse again their order when inserting them into the path to be followed by the mobile agent.
				 * Thirdly, we setup correctly the required transmission power level to move from one node to the next one.
				 */
				intermediateNodes = 0;
				for ( v = minNode; v != currentNode; v = Info[currentNode][v].prev ) {
					intermediateNodes++;
				}

				l = nodesInPath + intermediateNodes - 1;
				for ( v = minNode; v != currentNode; v = Info[currentNode][v].prev ) {
					Path[l].nodeID   = v;
					Path[l].collect  = false;	// Do not collect information from intermediate nodes
					l--;
					nodesInPath++;
				}
				k = 0;
				for ( l = l + 1; l < nodesInPath; l++) {
					Path[l].level = Info[Path[l - 1].nodeID][Path[l].nodeID].level;
					if ( createMAPI == true ) {
						fprintf ( MAPIfd, "%d\t%d\n", k, Path[l - 1].nodeID );
						k = 1;
					}
					trace() << "  " << Path[l].nodeID << ", " << Path[l].level;
				}
				Path[nodesInPath - 1].collect = true;	// Collect information from destination node
			}
			Info[minNode][minNode].used = true;
			currentNode = minNode;
		}

		/*
		 * We reached the last available node. Now find the shortest path back to the sink.
		 *
		 * If required, the information is written to a MAPI file. The return path is
		 * distinguished by the previous two cases by marking the nodes of the
		 * return path with a "2".
		 */
		prevNodesInPath = nodesInPath;
		intermediateNodes = 0;
		for ( v = 0; v != minNode; v = Info[currentNode][v].prev ) {
			intermediateNodes++;
		}

		l = nodesInPath + intermediateNodes - 1;
		for ( v = 0; v != minNode; v = Info[currentNode][v].prev ) {
			Path[l].nodeID  = v;
			Path[l].collect = false;
			/*
			 * Since we are going to visit nodes when sending back to the sink the aqcuired data,
			 * we will collect data from these nodes during the return trip. Therefore, any previous
			 * visit to the nodes of the return path do not require information to be collected.
			 */
			for ( i = 0; i < prevNodesInPath; i++ ) {
				if ( ( Path[i].nodeID == v ) && ( Path[i].collect == true) ) {
					Path[i].collect = false;
					Path[l].collect  = true;
				}
			}
			l--;
			nodesInPath++;
		}
		/*
		 * The previous procedure will also collect information from the sink, which is not required.
		 */
		Path[nodesInPath - 1].collect = false;

		trace() << "Path back to sink:";
		k = 0;
		for ( l = l + 1; l < nodesInPath; l++) {
			Path[l].level = Info[Path[l - 1].nodeID][Path[l].nodeID].level;
			if ( createMAPI == true ) {
				fprintf ( MAPIfd, "%d\t%d\n", k, Path[l - 1].nodeID );
				k = 2;
			}
			trace() << "  " << Path[l].nodeID << ", " << Path[l].level;
		}

		trace() << "Path starting from sink:";
		intermediateNodes = 0;
		travelDistance = Info[0][Path[0].nodeID].distance;
		totalTravelDistance += Info[0][Path[0].nodeID].distance;
		for ( i = 0; i < nodesInPath; i++ ) {
			trace() << "  " << setw( 3 ) << Path[i].nodeID << ", level = " << setw( 6 ) << Path[i].level << ", collect = " << Path[i].collect;
			if ( i < nodesInPath - 1 ) {
				travelDistance += Info[Path[i].nodeID][Path[i + 1].nodeID].distance;
				totalTravelDistance += Info[Path[i].nodeID][Path[i + 1].nodeID].distance;

				if ( Path[i].collect == 0 ) {
					intermediateNodes++;
					totalIntermediateNodes++;
				}
			}
		}

		totalItineraries++;
		totalNodes += ( nodesInPath - 1 );
		if ( intermediateNodes > maxIntermediateNodes ) {
			maxIntermediateNodes = intermediateNodes;
			maxIntermediateNodesItinerary = nodesInPath;
		}
		if ( nodesInPath == maxItineraryWithIntermediateNodes1 ) {
			if ( nodesInPath - intermediateNodes > maxItineraryWithoutIntermediateNodes1 ) {
				maxItineraryWithoutIntermediateNodes1 = nodesInPath - intermediateNodes;
			}
		} else if ( nodesInPath > maxItineraryWithIntermediateNodes1 ) {
			maxItineraryWithIntermediateNodes1    = nodesInPath;
			maxItineraryWithoutIntermediateNodes1 = nodesInPath - intermediateNodes;
		}
		if ( nodesInPath - intermediateNodes == maxItineraryWithoutIntermediateNodes2 ) {
			if ( nodesInPath > maxItineraryWithIntermediateNodes2 ) {
				maxItineraryWithIntermediateNodes2    = nodesInPath;
			}
		} else if ( nodesInPath - intermediateNodes > maxItineraryWithoutIntermediateNodes2 ) {
			maxItineraryWithIntermediateNodes2    = nodesInPath;
			maxItineraryWithoutIntermediateNodes2 = nodesInPath - intermediateNodes;
		}
		if ( travelDistance > maxItineraryTravelDistance ) {
			maxItineraryTravelDistance = travelDistance;
		}

		if ( createMAPI == true ) {
			fprintf ( MAPIfd, "%d\t%d\n", 0, 0 );

			fprintf ( MAPIfd, "\n" );

			fprintf ( MAPIfd, "[Coordinates]\n" );
			for ( i = 0; i < numberOfSensors; i++ ) {
				VirtualMobilityManager *nodeMobilityModule = check_and_cast<VirtualMobilityManager*>(getParentModule()->getParentModule()->getSubmodule("node",i)->getSubmodule("MobilityManager"));
				double x = nodeMobilityModule->getLocation().x;
				double y = nodeMobilityModule->getLocation().y;
				fprintf ( MAPIfd, "%f\t%f\t%d\t\"%d\"\n", x, y, ( Info[i][i].used == true ? 1 : 0 ), i );
			}

			fclose ( MAPIfd );
		}

		Fill.currentSize = getInitSize(nodesInPath);
		Fill.nextNodeID  = 0;
		Fill.nodesInPath = nodesInPath;

		time.push_back( instantiationDelay + nodesInPath * processingDelay );

		setTimer( SEND_PACKET, SEND_PACKET_DELAY + instantiationDelay );

		break;
	}

	case WAIT_FOR_POSITIONS: {
		if ( ( sensorsReported == numberOfSensors ) || ( reported == NO_REPORT_RECEIVED ) ) {
			int		i, j, k;
			double		field_x = getParentModule()->getParentModule()->par ( "field_x" );
			double		field_y = getParentModule()->getParentModule()->par ( "field_y" );
			stringstream	out;

//			if ( sensorsReported == 1 ) {
//				trace() << "Fatal error: No node reported its position.";
//				exit(0);
//			}

			reported = REPORT_END_OF_TURN;

			PosPkt	*newPkt = new PosPkt ( END_OF_TURN_PACKET, APPLICATION_PACKET );
			toNetworkLayer ( createRadioCommand ( SET_TX_OUTPUT, MAX_TRANSMISSION_LEVEL ) );
			toNetworkLayer ( newPkt, BROADCAST_NETWORK_ADDRESS );

			if ( createMAPI == true ) {
				MAPIfd = fopen ( "TSP_MH.mapi", "w" );

				fprintf ( MAPIfd, "[Field]\n" );
				fprintf ( MAPIfd, "%f\t%f\n\n", field_x, field_y );

				fprintf ( MAPIfd, "[Nodes]\n" );
				fprintf ( MAPIfd, "%d\n\n", numberOfSensors );
			}

			/*
			 * Setup information according to the data received from every node.
			 * The structure Info[i][j] describes the information between nodes i and j.
			 * More specifically, the information kept is as follows:
			 *   - If both nodes have reported their position, their distance can be calculated. Otherwise, their distance is set to INFINITY.
			 *   - If the distance previously calculated is less or equal to the maximum transmission range of a node,
			 *     the two nodes can exchange information directly. In this case, the smallest transmission level that can cover the distance is selected.
			 *     This is to preserve as much as possible the energy of the nodes.
			 *   - If the distance previously calculated is larger than the maximum transmission range of a node,
			 *     the two nodes cannot directly exchange information and the corresponding transmission power and level are set to INFINITY.
			 */
			sensorsReported = numberOfSensors;
			for ( i = 0; i < numberOfSensors; i++ ) {
				VirtualMobilityManager *nodeMobilityModule = check_and_cast<VirtualMobilityManager*>(getParentModule()->getParentModule()->getSubmodule("node",i)->getSubmodule("MobilityManager"));
				Info[i][i].xCoord = nodeMobilityModule->getLocation().x;
				Info[i][i].yCoord = nodeMobilityModule->getLocation().y;
				Info[i][i].distance = 0.0;	// Distance from myself
				Info[i][i].level    = 0.0;
				Info[i][i].power    = 0.0;
				Info[i][i].cost     = 0.0;
				Info[i][i].prev     = -1;
				Info[i][i].used     = false;

				for ( j = 0; j < i; j++ ) {
					Info[i][j].distance = sqrt ( pow ( Info[i][i].xCoord - Info[j][j].xCoord, 2) + pow ( Info[i][i].yCoord - Info[j][j].yCoord, 2) );
					Info[j][i].distance = Info[i][j].distance;
				}
			}

			/*
			 * Now that the distances between nodes have been correctly set up, we can use Diskstra's algorithm
			 * to calculate the minimum cost to send information from one node to another. If the communication
			 * cannot be performed directly, we save the path that has to be followed.
			 */
			for ( i = 0; i < numberOfSensors; i++ ) {
				if ( DijkstraType == 1) {
					DijkstraDistance ( i, MinVect, PrevVect );
					MaxTransmissionCost = MAX_TRANSMISSION_RANGE;
				} else if ( DijkstraType == 2 ) {
					DijkstraPower ( i, MinVect, PrevVect );
					MaxTransmissionCost = MAX_TRANSMISSION_POWER;
				} else {
					trace() << "Unknown type (=" << DijkstraType << ") of Dijkstra algorithm.";
				}
				for ( j = 0; j < numberOfSensors; j++ ) {
					Info[i][j].cost = MinVect[j];
					Info[i][j].prev = PrevVect[j];
				}
			}

			for ( i = 1; i < numberOfSensors; i++ ) {
				if ( Info[i][0].prev == -1) {
					sensorsReported--;
					Info[i][i].nodeID = NO_ID;
					for ( j = 0; j < numberOfSensors; j++ ) {
						if ( i != j ) {
							Info[i][j].distance = INFINITY;
							Info[j][i].distance = INFINITY;
							Info[i][j].level    = INFINITY;
							Info[j][i].level    = INFINITY;
							Info[i][j].power    = INFINITY;
							Info[j][i].power    = INFINITY;
							Info[i][j].cost     = INFINITY;
							Info[j][i].cost     = INFINITY;
							Info[i][j].prev     = -1;
							Info[j][i].prev     = -1;
						}
					}
				} else {
					Info[i][i].nodeID = i;
					for ( j = 0; j < i; j++ ) {
						if (Info[i][j].distance <= MAX_TRANSMISSION_RANGE ) {
							k = listSize - 1;
							while ( Info[i][j].distance > TransmissionRange[k] ) {
								k--;
							}
							Info[i][j].level = TransmissionLevel[k];
							Info[j][i].level = Info[i][j].level;
							Info[i][j].power = TransmissionPower[k];
							Info[j][i].power = Info[i][j].power;
						} else {
							Info[i][j].level = INFINITY;
							Info[j][i].level = INFINITY;
							Info[i][j].power = INFINITY;
							Info[j][i].power = INFINITY;
						}
					}
				}
			}

			if ( createMAPI == true ) {
				fprintf ( MAPIfd , "[CommunicationCost]\n" );
				for ( i = 0; i < numberOfSensors; i++ ) {
					for ( j = 0; j < numberOfSensors - 1; j++ ) {
						if ( Info[i][j].cost <= MaxTransmissionCost ) {
							fprintf ( MAPIfd, "%f\t", Info[i][j].cost );
						} else {
							fprintf ( MAPIfd, "Infinity\t" );	// The tool that reads this information is written in Java, which uses "Infinity" as the literal to describe infinity.
						}
					}
					if ( Info[i][numberOfSensors - 1].cost <= MaxTransmissionCost ) {
						fprintf ( MAPIfd, "%f\n", Info[i][numberOfSensors - 1].cost );
					} else {
						fprintf ( MAPIfd, "Infinity\n" );
					}
				}
				fprintf ( MAPIfd, "\n" );
			}

//#if defined(LOG_ENABLED)
#if 1
			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].distance << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].level << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].power << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].cost << " ";
				}
				trace() << out.str();
			}

			trace() << "";

			for ( i = 0; i < numberOfSensors; i++ ) {
				out.str("");
				for ( j = 0; j < numberOfSensors; j++ ) {
					out << fixed << setprecision( 2 ) << setw ( 6 ) << Info[i][j].prev << " ";
				}
				trace() << out.str();
			}
#endif

			totalNodes = 0;
			totalItineraries = 0;
			totalIntermediateNodes = 0;
			maxIntermediateNodes = 0;
			maxIntermediateNodesItinerary = 0;
			maxItineraryWithIntermediateNodes1 = 0;
			maxItineraryWithIntermediateNodes2 = 0;
			maxItineraryWithoutIntermediateNodes1 = 0;
			maxItineraryWithoutIntermediateNodes2 = 0;
			maxItineraryTravelDistance = 0.0;
			totalTravelDistance = 0.0;

			/*
			 * Now calculate the path that the mobile agent has to follow.
			 * First, the current node finds the node with the smallest cost to send the data.
			 * If the next node is within the transmission range of the current node, the information is sent directly.
			 * Otherwise, the information from running Dijkstra's algorithm is used to calculate an alternative path
			 * to get to the next node.
			 *
			 * If required, the information is written to a MAPI file. The two cases described above
			 * (directly sending data or through an alternative path) are marked with a "0" or a "1"
			 * respectively in the MAPI file.
			 */
			if ( createMAPI == true ) {
				fprintf ( MAPIfd, "[Path]\n" );
			}

			time.clear();

			startTime = simTime().dbl() + 0.1;

			setTimer( FIND_PATH, 0.1 );

//			setTimer ( REQUEST_POSITIONS, REQUEST_POSITIONS_INTERVAL );
		} else {
			reported = NO_REPORT_RECEIVED;
			setTimer ( WAIT_FOR_POSITIONS, WAIT_FOR_POSITIONS_INTERVAL );
		}
		break;
	}
	}
}

/******************************************************************************/

void TSP_MH::handleSensorReading ( SensorReadingMessage * sensorMsg )
{
	double	sensValue = sensorMsg->getSensedValue();

	if ( !isSink ) {
		if ( sensValue > THRESHOLD ) {
#if defined(LOG_ENABLED)
			trace() << "Sensed value is " << sensValue << " from sensor at (" << mobilityModule->getLocation().x << ", " << mobilityModule->getLocation().y << ")";
#endif
		}
	}
}

/******************************************************************************/

void TSP_MH::finishSpecific()
{
	list<double>::iterator it, max_it, secondMax_it;
	double max, secondMax;

	if ( isSink ) {
		if ( time.size() == 1 ) {
			it = time.begin();
			trace() << setprecision ( 8 ) << "Total time = " << ( endTime - startTime - 2 * SEND_PACKET_DELAY + *it );
		} else {
			max_it = time.begin();
			for ( it = time.begin(); it != time.end(); it++ ) {
				if ( *it > *max_it ) {
					max_it = it;
				}
			}
			max = *max_it;

			time.erase(max_it);

			secondMax_it = time.begin();
			for ( it = time.begin(); it != time.end(); it++ ) {
				if ( *it > *secondMax_it ) {
					secondMax_it = it;
				}
			}
			secondMax = *secondMax_it;

			trace() << setprecision ( 8 ) << "Total time                       = " << ( endTime - startTime - 2 * SEND_PACKET_DELAY + ( max - secondMax ) );
		}
		trace() << setprecision ( 8 ) << "Total itineraries                = " << totalItineraries;
		trace() << setprecision ( 8 ) << "Total travel distance            = " << totalTravelDistance;
		trace() << setprecision ( 8 ) << "Maximum travel distance          = " << maxItineraryTravelDistance;
		trace() << setprecision ( 8 ) << "Total nodes reported             = " << sensorsReported - 1;
		trace() << setprecision ( 8 ) << "Total nodes visited              = " << totalNodes;
		trace() << setprecision ( 8 ) << "Total intermediate nodes         = " << totalIntermediateNodes;
		trace() << setprecision ( 8 ) << "Maximum intermediate nodes added = " << maxIntermediateNodes;
		trace() << setprecision ( 8 ) << "  Total nodes in itinerary       = " << maxIntermediateNodesItinerary;
		trace() << setprecision ( 8 ) << "Maximum itinerary with intermediate nodes:";
		trace() << setprecision ( 8 ) << "  With intermediate nodes        = " << maxItineraryWithIntermediateNodes1;
		trace() << setprecision ( 8 ) << "  Without intermediate nodes     = " << maxItineraryWithoutIntermediateNodes1;
		trace() << setprecision ( 8 ) << "Maximum itinerary without intermediate nodes:";
		trace() << setprecision ( 8 ) << "  With intermediate nodes        = " << maxItineraryWithIntermediateNodes2;
		trace() << setprecision ( 8 ) << "  Without intermediate nodes     = " << maxItineraryWithoutIntermediateNodes2;
	} else {
		if ( Info[ID][ID].nodeID != NO_ID ) {
			if (spentEnergy == 0.0 ) {
				spentEnergy = startEnergy;
			}
			trace() << "Energy spent = " << spentEnergy - startEnergy;
		} else {
			trace() << "Energy spent = " << 0.0;
		}
	}
}

/******************************************************************************/

void TSP_MH::DijkstraDistance ( int source, vector<double> &MinVect, vector<int> &PrevVect )
{
	set<pair<double, int> > vertex_queue;

	double distance_through_u;

	MinVect.clear();
	MinVect.resize ( numberOfSensors, INFINITY );
	MinVect[source] = 0.0;
	PrevVect.clear();
	PrevVect.resize ( numberOfSensors, -1 );

	vertex_queue.insert ( make_pair ( MinVect[source], source ) );

	while ( !vertex_queue.empty() ) {
		double	dist = vertex_queue.begin()->first;
		int	u    = vertex_queue.begin()->second;
		vertex_queue.erase ( vertex_queue.begin() );

		// Visit each edge exiting u
		for ( int v = 0; v < numberOfSensors; v++ ) {
			if ( ( Info[u][v].distance <= MAX_TRANSMISSION_RANGE ) && ( u != v ) ) {
				distance_through_u = dist + Info[u][v].distance;
				if ( distance_through_u < MinVect[v] ) {
					vertex_queue.erase ( make_pair ( MinVect[v], v ) );

					MinVect[v] = distance_through_u;
					PrevVect[v] = u;
					vertex_queue.insert ( make_pair ( MinVect[v], v ) );

				}
			}
		}
	}
}

/******************************************************************************/

void TSP_MH::DijkstraPower ( int source, vector<double> &MinVect, vector<int> &PrevVect )
{
	set<pair<double, int> > vertex_queue;

	double power_through_u;

	MinVect.clear();
	MinVect.resize ( numberOfSensors, INFINITY );
	MinVect[source] = 0.0;
	PrevVect.clear();
	PrevVect.resize ( numberOfSensors, -1 );

	vertex_queue.insert ( make_pair ( MinVect[source], source ) );

	while ( !vertex_queue.empty() ) {
		double	dist = vertex_queue.begin()->first;
		int	u    = vertex_queue.begin()->second;
		vertex_queue.erase ( vertex_queue.begin() );

		// Visit each edge exiting u
		for ( int v = 0; v < numberOfSensors; v++ ) {
			if ( ( Info[u][v].power <= MAX_TRANSMISSION_POWER ) && ( u != v ) ) {
				power_through_u = dist + Info[u][v].power;
				if ( power_through_u < MinVect[v] ) {
					vertex_queue.erase ( make_pair ( MinVect[v], v ) );

					MinVect[v] = power_through_u;
					PrevVect[v] = u;
					vertex_queue.insert ( make_pair ( MinVect[v], v ) );

				}
			}
		}
	}
}

/******************************************************************************/
