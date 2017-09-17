package moban.mobility;
//***************************************************************************
//* file:        MoBANCoordinator.ned
//*
//* author:      Majid Nabi <m.nabi@tue.nl>
//*
//*              http://www.es.ele.tue.nl/nes
//*
//*
//* copyright:   (C) 2010 Electronic Systems group(ES),
//*              Eindhoven University of Technology (TU/e), the Netherlands.
//*
//*
//*              This program is free software; you can redistribute it
//*              and/or modify it under the terms of the GNU General Public
//*              License as published by the Free Software Foundation; either
//*              version 2 of the License, or (at your option) any later
//*              version.
//*              For further information see file COPYING
//*              in the top level directory
//***************************************************************************
//* part of:    MoBAN (Mobility Model for wireless Body Area Networks)
//* description:     Coordinator module of the MoBAN mobility model
//***************************************************************************
//* Citation of the following publication is appreciated if you use MoBAN for
//* a publication of your own.
//*
//* M. Nabi, M. Geilen, T. Basten. MoBAN: A Configurable Mobility Model for Wireless Body Area Networks.
//* In Proc. of the 4th Int'l Conf. on Simulation Tools and Techniques, SIMUTools 2011, Barcelona, Spain, 2011.
//*
//* BibTeX:
//*	@inproceedings{MoBAN,
//* 	author = "M. Nabi and M. Geilen and T. Basten.",
//* 	title = "{MoBAN}: A Configurable Mobility Model for Wireless Body Area Networks.",
//*    	booktitle = "Proceedings of the 4th Int'l Conf. on Simulation Tools and Techniques.",
//*    	series = {SIMUTools '11},
//*    	isbn = {978-963-9799-41-7},
//*	year = {2011},
//*    	location = {Barcelona, Spain},
//*	publisher = {ICST} }
//*
//**************************************************************************/




simple MoBANCoordinator
{
    parameters:
    	@class(MoBANCoordinator);
        bool debug; // debug switch
       
        double updateInterval @unit(s); // Time interval to update the nodes position (seconds).
        
        int numNodes;			//Number of sendor nodes belong to this WBAN
        
        xml postureSpecFile;    // The input file that includes the specification of all postures
        xml configFile;     // The configuration file for setting probability vectors, distributions and correlations
        
        bool useMobilityPattern; // Set if the logged mobility pattern is used.
        string mobilityPatternFile;  // The input file for mobility pattern if it is going to be used.
           
}

