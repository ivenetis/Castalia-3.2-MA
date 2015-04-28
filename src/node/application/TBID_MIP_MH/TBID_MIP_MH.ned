//****************************************************************************
//*  Copyright: National ICT Australia,  2007 - 2010                         *
//*  Developed at the ATP lab, Networked Systems research theme              *
//*  Author(s): Athanassios Boulis, Dimosthenis Pediaditakis                 *
//*  This file is distributed under the terms in the attached LICENSE file.  *
//*  If you do not find this file, copies can be found by writing to:        *
//*                                                                          *
//*      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
//*      Attention:  License Inquiry.                                        *
//*                                                                          *
//****************************************************************************/

package node.application.TBID_MIP_MH;

@namespace(TBID_MIP_MH_NS);

// The sensor node module. Connects to the wireless channel in order to communicate
// with other nodes. Connects to psysical processes so it can sample them.

simple TBID_MIP_MH like node.application.iApplication {
 parameters:
 	string applicationID		= default ("TBID_MIP_MH");
	bool collectTraceInfo		= default (true);
	double a			= default (1.0);
	double f			= default (1.0);
	double instantiationDelay	= default (0.01);	// in seconds
	double processingDelay		= default (0.05);	// in seconds
	double aggregationEnergy	= default (0.000000005);// in Joules
	int priority			= default (1);
	int packetHeaderOverhead	= default (8);		// in bytes
	int constantDataPayload		= default (2);		// in bytes
	int sampleInterval		= default (1000);	// in ms
	int DijkstraType		= default (1);
	int bytesCollected		= default (200);
	int codeSize			= default (1024);
	bool isSink			= default (false);
	bool createMAPI			= default (false);

 gates:
 	output toCommunicationModule;
	output toSensorDeviceManager;
	input fromCommunicationModule;
	input fromSensorDeviceManager;
	input fromResourceManager;
}

