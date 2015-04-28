#include "NOID_MIP_MH.h"

Define_Module ( NOID_MIP_MH );

/******************************************************************************/

void NOID_MIP_MH::startup()
{
	ID			= atoi ( SELF_NETWORK_ADDRESS );
	sampleInterval		= par ( "sampleInterval" );
	bytesCollected		= par ( "bytesCollected" );
	codeSize		= par ( "codeSize" );
	createMAPI		= par ( "createMAPI" );
	DijkstraType		= par ( "DijkstraType" );
	a			= par ( "a" );
	f			= par ( "f" );
	instantiationDelay	= par ( "instantiationDelay" );
	processingDelay		= par ( "processingDelay" );
	aggregationEnergy	= par ( "aggregationEnergy" );
	multiplier		= par ( "multiplier" );
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
void NOID_MIP_MH::fromNetworkLayer ( ApplicationPacket * genericPacket, const char *source, double rssi, double lqi )
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

void NOID_MIP_MH::timerFiredCallback ( int index )
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

		setTimer( FIND_PATH, SEND_PACKET_DELAY );

		break;
	}

	case FIND_PATH: {
		int	i, j, k, l, s, v, currentNode, nodesInPath, nextNode, intermediateNodes, nodes, connected;
		double	t, min_t, travelDistance;
		set<int>::iterator		it_i, it_j, min_it_i, min_it_j;
		vector< double >::iterator	it_C_i, it_C_j, min_it_C_i, min_it_C_j;
		vector< set<int> >::iterator	it_set_i, it_set_j, min_it_set_i, min_it_set_j;
		vector< tree<int> >::iterator	it_tree_i, it_tree_j, min_it_tree_i, min_it_tree_j;
		tree<int>::iterator		loc_i, loc_j, current_it, it;
		tree<int>			tempTree;
		tree<int>::post_order_iterator	it_post;

		if ( savedS > 0 ) {
			goto OnlySendNextMA;
		}

		for ( connected = 0; connected < sensorsReported - 1; connected++ ) {
			min_t = INFINITY;
			for ( it_set_i = I.begin(), it_C_i = C.begin(), it_tree_i = workingBranch.begin(); it_set_i != I.end(); it_set_i++, it_C_i++, it_tree_i++ ) {
				for ( it_set_j = I.begin(), it_C_j = C.begin(), it_tree_j = workingBranch.begin(); it_set_j != I.end(); it_set_j++, it_C_j++, it_tree_j++ ) {
					if ( it_set_i != it_set_j ) {
						for ( it_i = it_set_i->begin(); it_i != it_set_i->end(); it_i++ ) {
							for ( it_j = it_set_j->begin(); it_j != it_set_j->end(); it_j++ ) {
								if ( (*it_i) != (*it_j) ) {
#if defined(LOG_ENABLED)
									trace() << "Checking node " << (*it_i) << " from set " << ( it_set_i - I.begin() ) << " with node " << (*it_j) << " from set " << ( it_set_j - I.begin() );
#endif
									if ( (*it_i) == 0 ) {
										nodes = it_set_j->size();
									} else if ( (*it_j) == 0 ) {
										nodes = it_set_i->size();
									} else {
										nodes = it_set_i->size() + it_set_j->size();
									}
#if defined(LOG_ENABLED)
									trace() << "  distance = " << Info[*it_i][*it_j].distance << ", nodes = " << nodes << ", f = " << f << ", bytesCollected = " << bytesCollected << ", C = " << *it_C_i;
#endif
									t = Info[*it_i][*it_j].distance + ( nodes * ( nodes + 1 ) / 2 ) * f * bytesCollected - *it_C_i;
									if ( ( (*it_i) == 0 ) || ( (*it_j) == 0 ) ) {
										t += ( multiplier * codeSize );
									}
#if defined(LOG_ENABLED)
									trace() << "  New t = " << t;
#endif
									if ( t < min_t ) {
										min_t         = t;
										min_it_set_i  = it_set_i;
										min_it_set_j  = it_set_j;
										min_it_i      = it_i;
										min_it_j      = it_j;
										min_it_C_i    = it_C_i;
										min_it_C_j    = it_C_j;
										min_it_tree_i = it_tree_i;
										min_it_tree_j = it_tree_j;
									}
								}
							}
						}
					}
				}
			}

#if defined(LOG_ENABLED)
			trace() << "min_t = " << min_t << ", min_i = " << (*min_it_i) << ", min_j = " << (*min_it_j) << ", min_set_i = " << ( min_it_set_i - I.begin() ) << ", min_set_j = " << ( min_it_set_j - I.begin() );
			trace() << "min_it_C_i = " << (*min_it_C_i) << ", min_it_C_j = " << (*min_it_C_j);
#endif

			if ( (*min_it_i) == 0 ) {
#if defined(LOG_ENABLED)
				trace() << "1. Tree i";
				kptree::print_tree_bracketed((*min_it_tree_i), trace());
				trace() << "1. Tree j";
				kptree::print_tree_bracketed((*min_it_tree_j), trace());
#endif
				tempTree.clear();
				tempTree.set_head( 0 );
				tempTree.append_child( tempTree.begin(), min_it_tree_j->begin() );
				branch.push_back( tempTree );
				min_it_tree_i->append_child( min_it_tree_i->begin(), min_it_tree_j->begin() );
#if defined(LOG_ENABLED)
				trace() << "1. Tree i";
				kptree::print_tree_bracketed((*min_it_tree_i), trace());
#endif
				workingBranch.erase( min_it_tree_j );
				min_it_set_i->insert ( min_it_set_j->begin(), min_it_set_j->end() );
				I.erase ( min_it_set_j );
				*min_it_C_i = *min_it_C_i < *min_it_C_j ? *min_it_C_i : *min_it_C_j;
				C.erase ( min_it_C_j );
			} else if ( (*min_it_j) == 0 ) {
#if defined(LOG_ENABLED)
				trace() << "2. Tree i";
				kptree::print_tree_bracketed((*min_it_tree_i), trace());
				trace() << "2. Tree j";
				kptree::print_tree_bracketed((*min_it_tree_j), trace());
#endif
				tempTree.clear();
				tempTree.set_head( 0 );
				tempTree.append_child( tempTree.begin(), min_it_tree_i->begin() );
				branch.push_back( tempTree );
				min_it_tree_j->append_child( min_it_tree_j->begin(), min_it_tree_i->begin() );
#if defined(LOG_ENABLED)
				trace() << "2. Tree j";
				kptree::print_tree_bracketed((*min_it_tree_j), trace());
#endif
				workingBranch.erase( min_it_tree_i );
				min_it_set_j->insert ( min_it_set_i->begin(), min_it_set_i->end() );
				I.erase ( min_it_set_i );
				*min_it_C_i = *min_it_C_i < *min_it_C_j ? *min_it_C_i : *min_it_C_j;
				C.erase ( min_it_C_i );
			} else {
#if defined(LOG_ENABLED)
				trace() << "3. Tree i";
				kptree::print_tree_bracketed((*min_it_tree_i), trace());
				trace() << "3. Tree j";
				kptree::print_tree_bracketed((*min_it_tree_j), trace());
#endif
				loc_i = find( min_it_tree_i->begin(), min_it_tree_i->end(), *min_it_i );
				loc_j = find( min_it_tree_j->begin(), min_it_tree_j->end(), *min_it_j );
				it = min_it_tree_j->append_child( loc_j );
				it = min_it_tree_j->move_ontop( it, loc_i );

				it = find( min_it_tree_j->begin(), min_it_tree_j->end(), *min_it_i );

				if ( min_it_tree_i->size() > 0 ) {
					it = min_it_tree_j->append_child( it );
					it = min_it_tree_j->move_ontop( it, min_it_tree_i->begin() );
				}
#if defined(LOG_ENABLED)
				trace() << "3. Tree j";
				kptree::print_tree_bracketed((*min_it_tree_j), trace());
#endif
				workingBranch.erase( min_it_tree_i );
				min_it_set_j->insert ( min_it_set_i->begin(), min_it_set_i->end() );
				I.erase ( min_it_set_i );
				*min_it_C_j = *min_it_C_j < *min_it_C_i ? *min_it_C_j : *min_it_C_i;
				C.erase ( min_it_C_i );
			}

#if defined(LOG_ENABLED)
			trace() << "";

			for ( int i = 0; i < I.size(); i++ ) {
				trace() << "Nodes included in set " << i << " :";
				for ( set<int>::iterator it = I[i].begin(); it != I[i].end(); it++ ) {
					trace() << "  " << (*it);
				}
			}
#endif
		}

//#if defined(LOG_ENABLED)
		for ( int i = 0; i < branch.size(); i++ ) {
			trace() << "Size of branch " << i << " = " <<  branch[i].size();
			kptree::print_tree_bracketed(branch[i], trace());
		}
//#endif

OnlySendNextMA:
		/*
		 * Finally, we iterate over all branches to create the path that each MA has to follow.
		 */
		for ( s = savedS; s < branch.size(); s++ ) {
			trace() << "Traversing nodes in branch " << s;

			Path = getPathStart(Fill);
			nodesInPath = 0;
			currentNode = 0;
			Info[currentNode][currentNode].used = true;

			for ( it_post = branch[s].begin_post(); it_post != branch[s].end_post(); it_post++) {
				nextNode = *it_post;
				if ( Info[currentNode][nextNode].cost <= MaxTransmissionCost ) {
					Path[nodesInPath].nodeID   = nextNode;
					Path[nodesInPath].collect  = true;
					Path[nodesInPath].level    = Info[currentNode][nextNode].level;
					if ( createMAPI == true ) {
						fprintf ( MAPIfd, "%d\t%d\n", 0, currentNode );
					}
					nodesInPath++;
					trace() << "  Next hop is from node " << currentNode << " to node " << nextNode << " with level= " << Info[currentNode][nextNode].level;
				} else {
					trace() << "  Next hop is from node " << currentNode << " to node " << nextNode << ". Path to follow:";
					/*
					 * Dijkstra's algorithm returns the path to be followed in the reverse order.
					 * Firstly, we count how many nodes we have to visit to reach the destination node.
					 * Secondly, we reverse again their order when inserting them into the path to be followed by the mobile agent.
					 * Thirdly, we setup correctly the required transmission power level to move from one node to the next one.
					 */
					intermediateNodes = 0;
					for ( v = nextNode; v != currentNode; v = Info[currentNode][v].prev ) {
						intermediateNodes++;
					}

					l = nodesInPath + intermediateNodes - 1;
					for ( v = nextNode; v != currentNode; v = Info[currentNode][v].prev ) {
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
				Info[nextNode][nextNode].used = true;
				currentNode = nextNode;
			}

			Path[nodesInPath - 1].collect = 0;

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

			Fill.currentSize = getInitSize(nodesInPath);
			Fill.nextNodeID  = 0;
			Fill.nodesInPath = nodesInPath;

			time.push_back( ( savedS + 1 ) * instantiationDelay + nodesInPath * processingDelay );

			savedS = s + 1;		// Restart from next node.

			setTimer( SEND_PACKET, SEND_PACKET_DELAY + instantiationDelay );
			return;
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

		break;
	}

	case WAIT_FOR_POSITIONS: {
		if ( ( sensorsReported == numberOfSensors ) || ( reported == NO_REPORT_RECEIVED ) ) {
			int		i, j, k;
			double		currentDiameter;
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
				MAPIfd = fopen ( "NOID_MIP_MH.mapi", "w" );

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

#if defined(LOG_ENABLED)
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

			/*
			 * Clear the vector that holds the sets of nodes that belong to each itinerary.
			 */
			for ( i = 0; i < I.size(); i++ ) {
				I[i].clear();
			}
			I.clear();

			/*
			 * Resize the vector of itineraries to hold as many sensors have reported (including the sink).
			 */
			I.resize( sensorsReported );

			/*
			 * Clear the vector that holds the minimum costs of all itineraries to the sink node.
			 */
			C.clear();

			/*
			 * Resize the vector of minimum costs to hold as many sensors have reported (including the sink).
			 */
			C.resize( sensorsReported );

			for ( i = 0; i < branch.size(); i++ ) {
				branch[i].clear();
			}
			branch.clear();

			for ( i = 0; i < workingBranch.size(); i++ ) {
				workingBranch[i].clear();
			}
			workingBranch.clear();
			workingBranch.resize( sensorsReported );

			/*
			 * Insert every node that reported its position into a separate set and into a tree.
			 */
			for ( i = 0, j = 0; i < numberOfSensors; i++ ) {
				if ( Info[i][i].nodeID != NO_ID ) {
					I[j].insert( i );
					C[j] = Info[i][0].distance;
					workingBranch[j].insert( workingBranch[j].begin(), i );
					j++;
				}
			}

			for ( i = 0; i < I.size(); i++ ) {
				trace() << "Nodes included in set " << i << " :";
				for ( set<int>::iterator it = I[i].begin(); it != I[i].end(); it++ ) {
					trace() << "  " << (*it);
				}
			}

			trace() << "sensorsReported is " << sensorsReported;
			trace() << "Multiplier is " << multiplier;

			savedS = 0;
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

void NOID_MIP_MH::handleSensorReading ( SensorReadingMessage * sensorMsg )
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

void NOID_MIP_MH::finishSpecific()
{
	list<double>::iterator it, max_it, secondMax_it;
	double max, secondMax;

	if ( isSink ) {
//		if ( time.size() == 0 ) {
//			trace() << setprecision ( 8 ) << "Total time = " << 0.0;
//		} else
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

void NOID_MIP_MH::DijkstraDistance ( int source, vector<double> &MinVect, vector<int> &PrevVect )
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

void NOID_MIP_MH::DijkstraPower ( int source, vector<double> &MinVect, vector<int> &PrevVect )
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
