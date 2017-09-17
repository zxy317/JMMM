/* -*- mode:c++ -*- ********************************************************
 * file:        MoBANLocal.cc
 *
 * author:      Majid Nabi <m.nabi@tue.nl>
 *
 *
 *              http://www.es.ele.tue.nl/nes
 *
 * copyright:   (C) 2010 Electronic Systems group(ES),
 *              Eindhoven University of Technology (TU/e), the Netherlands.
 *
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:    MoBAN (Mobility Model for wireless Body Area Networks)
 * description:     Implementation of the local module of the MoBAN mobility model
 ***************************************************************************
 * Citation of the following publication is appreciated if you use MoBAN for
 * a publication of your own.
 *
 * M. Nabi, M. Geilen, T. Basten. MoBAN: A Configurable Mobility Model for Wireless Body Area Networks.
 * In Proc. of the 4th Int'l Conf. on Simulation Tools and Techniques, SIMUTools 2011, Barcelona, Spain, 2011.
 *
 * BibTeX:
 *		@inproceedings{MoBAN,
 * 		author = "M. Nabi and M. Geilen and T. Basten.",
 * 	 	title = "{MoBAN}: A Configurable Mobility Model for Wireless Body Area Networks.",
 *    	booktitle = "Proceedings of the 4th Int'l Conf. on Simulation Tools and Techniques.",
 *    	series = {SIMUTools '11},
 *    	isbn = {978-963-9799-41-7},
 *	    year = {2011},
 *    	location = {Barcelona, Spain},
 *	    publisher = {ICST} }
 *
 **************************************************************************/


#include <FWMath.h>
#include "MoBANLocal.h"
#include"math.h"
#define pi 3.14159

Define_Module(MoBANLocal);

void MoBANLocal::initialize(int stage) //pass the stage£¬valid values are only 0 and 1
{
    BaseMobility::initialize(stage);
    yidongcount=0;
    if (stage == 0) {  //stage = 0 means initialization, sending the first message

    	selfidx=this->getParentModule()->getIndex();
		speed = 1;
    	move.setSpeed(speed);//It should be done in the first stage for sure. If it is not set, then no move message will be initiated. Any value is okay, but Zero!

    	numSteps = 0;
        step = -1;
        stepSize = Coord(0,0,0);

        posture=0;//Any initial value
        steputimes=(0.1)/par("updateInterval").doubleValue();

        stepv=0;
    	nowanglev=(0/180)*pi;
    	nextanglev=(0/180)*pi;


        ParaSettingBasedonPosture(posture);

    	steputimes=(0.1)/par("updateInterval").doubleValue();
    	stepu=stepu/steputimes;
    	stepv=stepv/steputimes;

        BBMoBANMessage groupMove;        // subscribe to blackboard to receive items published by the MoBAN coordinator
        catRefMove = utility->subscribe(this, &groupMove, findHost()->getId());
    }
    else if( stage == 1 ){   //Not sure when the stage = 1, refercencPoint will be updated, resulting in a quiescent state below will not update startpos
    	referencePoint = move.getStartPos();



    }

    anglecount=0;

    selfidx=this->getParentModule()->getIndex();

   // posture=1;




    EV<<"reference id chushihua OK"<<referenceindex<<"direction chushihua OK"<<direction<<endl;


}

/**
 * Calculate a new random position within a sphere around the reference point with the given radius.
 * It also calculates the number of steps the node needs to reach this position with the given speed
 */
void MoBANLocal::setTargetPosition()  //To change the words as long as you modify the code here, every time you move, u and v are not a zero value.
{





	Coord currentRelativePosition;  //This is also a variable for the coordinate type
	FILE *fp;

	if (speed != 0)
	{
		// Find a uniformly random position within a sphere around the reference point
		double x  = uniform(-1*radius, radius);
		double y  = uniform(-1*radius, radius);
		double z =  uniform(-1*radius, radius);
		while (x*x+y*y+z*z > radius*radius)
		{
			x  = uniform(-1*radius, radius);
			y  = uniform(-1*radius, radius);
			z =  uniform(-1*radius, radius);
		}

		targetPos.setX(x);
		targetPos.setY(y);
		targetPos.setZ(z);

		EV<<"point1"<<endl;







		//In addition to the nodes on the trunk other nodes are set up like this
		if (referenceindex!=-1)
		{
		cModule* referencenode=this->getParentModule()->getParentModule()->getSubmodule("node",referenceindex);

	    EV<<"referencenode chushihua OK"<<endl;

	 	double x1=referencePoint.getX(); //In the simulation inside the referencepoint is treated as the current absolute coordinates
		double y1=referencePoint.getY();
		double z1=referencePoint.getZ();


		//double x1=this->par("x");
		//double y1=this->par("y");
		//double z1=this->par("z");


		//  Before stepmove each point should store two sets of coordinates: before the current move and before the start of the current move, now use the back of the set of coordinates

		double xr=referencenode->getSubmodule("mobility")->par("x");
		double yr=referencenode->getSubmodule("mobility")->par("y");
		double zr=referencenode->getSubmodule("mobility")->par("z");


		double sinnowangle;


	    //endSimulation();

		sinnowangle=(z1-zr)/ownR;
//      This comment out, all angles start to set nowangle
//		if (anglecount==0)
//		{
//	    nowangle=asin(sinnowangle);
//	    this->par("nowangle")=nowangle;
//	    anglecount++;
//		}
	    //Determine whether 'm' is in range

        EV<<mupperbound<<"XX"<<mlowerbound<<endl;

        if(posture==1)  //walking
        {
			        //Horizontal offset in the vertical direction
					if ((referenceindex==7)||(referenceindex==8))  
						{
							 //Situation of the node 11
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==7))
										 {
														 yidongcount=0;
														 //direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(3*steputimes))&&(referenceindex==7))
							 {
								stepu=(10.0*pi)/180;
								stepu=stepu/steputimes;
											  direction=-1;
							 }
							 if((yidongcount==(int)(3*steputimes))&&(referenceindex==7))
							 {
											stepu=(8*pi)/180;
								            stepu=stepu/steputimes;
														  direction=1;
							 }

							 if((yidongcount==(int)(6*steputimes))&&(referenceindex==7))
							 {
								              stepu=(54.0/4.0*pi)/180;
											  stepu=stepu/steputimes;
											  direction=1;
							 }


							 if((yidongcount==(int)(10*steputimes))&&(referenceindex==7))
							 {
								             stepu=(9.0*pi)/180;				
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=-1;
							 }
							 if((yidongcount==(int)(12*steputimes))&&(referenceindex==7))
										 {
											             stepu=(5.0/4.0*pi)/180;										
														 stepu=stepu/steputimes;
														 //direction=(-1)*direction;
														 direction=-1;
										 }
							 if((yidongcount==(int)(16*steputimes))&&(referenceindex==7))
							 {
								 
									 stepu=(25.0/8.0*pi)/180;
									 direction=-1;
                                     stepu=stepu/steputimes;
											 //direction=(-1)*direction;

							 }

							 //Situation of the node 12
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==8))
										 {
														 yidongcount=0;
														// direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(8*steputimes))&&(referenceindex==8))
							 {
                                                stepu=(22.0/8.0*pi)/180;
												direction=-1;
												stepu=stepu/steputimes;
															 //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(8*steputimes))&&(referenceindex==8))
							 {
								             stepu=(0*pi)/180;
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=-1;
							 }
							 if((yidongcount==(int)(10*steputimes))&&(referenceindex==8))
							 {
								             stepu=(38.0/5.0*pi)/180;
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=-1;
							 }

							 if((yidongcount==(int)(15*steputimes))&&(referenceindex==8))
							 {
								             stepu=(10*pi)/180;
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=1;
							 }

							 if((yidongcount==(int)(22*steputimes))&&(referenceindex==8))
							 {
								             stepu=(5.0*pi)/180;
											 stepu=stepu/steputimes;
											 direction=-1;
											 //direction=(-1)*direction;

							 }

						}

					if ((referenceindex==3)||(referenceindex==4))  //Calculate the movement range with the relative, calculate the coordinates with absolute
						{
							 //Stituation of the node 7
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==3))
										 {
											             yidongcount=0;
													     //direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(6*steputimes))&&(referenceindex==3))
							 {
								              stepu=(9*pi)/180;
								              stepu=stepu/steputimes;
								              direction=1;
							 }
							 if((yidongcount==(int)(6*steputimes))&&(referenceindex==3))
							 {
											 stepu=(3.0*pi)/180;
										     stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										     direction=1;
							 }
							 if((yidongcount==(int)(9*steputimes))&&(referenceindex==3))
							 {
							 	            stepu=(0*pi)/180;
							 				stepu=stepu/steputimes;
						                    //direction=(-1)*direction;
							 				direction=1;
							 }
							 if((yidongcount==(int)(11*steputimes))&&(referenceindex==3))
							 {
									stepu=(3*pi)/180;
									 direction=-1;
								    stepu=stepu/steputimes;
										     //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(12*steputimes))&&(referenceindex==3))
							 {
							 		stepu=(5*pi)/180;
							 		 direction=-1;
							 		stepu=stepu/steputimes;
							 										     //direction=(-1)*direction;

							 }




							 //Situation of the node 8
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==4))
										 {
											             yidongcount=0;
													    // direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(12*steputimes))&&(referenceindex==4))
							 {
								        stepu=(5*pi)/180;
										direction=-1;
										stepu=stepu/steputimes;
														     //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(12*steputimes))&&(referenceindex==4))
							 {
										stepu=(9*pi)/180;							
										stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										direction=1;
							 }
							 if((yidongcount==(int)(18*steputimes))&&(referenceindex==4))
							 {
								        stepu=(3.0*pi)/180;
										stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										direction=1;
							 }
							 if((yidongcount==(int)(21*steputimes))&&(referenceindex==4))
							 {
										stepu=(0*pi)/180;								
										     stepu=stepu/steputimes;
										     direction=1;
										     //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(23*steputimes))&&(referenceindex==4))
							 {
										stepu=(3*pi)/180;
							 				stepu=stepu/steputimes;
							 				direction=-1;
							 				//direction=(-1)*direction;

							 }
						}
							 


							 if ((referenceindex==1)||(referenceindex==2))
								{
								
										if(yidongcount==(int)(24*steputimes))
												{
													yidongcount=0;
													direction=direction*(-1);
												}
										if(yidongcount==(int)(12*steputimes))
												{
													 direction=direction*(-1);
												}

									


								}
							if ((referenceindex==5)||(referenceindex==6))
							{

								
										if(yidongcount==(int)(24*steputimes))
												{
													yidongcount=0;
													direction=direction*(-1);
												}
										if(yidongcount==(int)(12*steputimes))
												{
													 direction=direction*(-1);
												}

									

							}

			




       //The following is the calculation of the horizontal offset
       //Calculate the horizontal offset of nodes 7 and 8
		if ((referenceindex==3)||(referenceindex==4))  //Calculate the movement range with the relative, calculate the coordinates with absolute
		{

			
				if(referenceindex==3)
				{
					if(yidongcount==(int)(24*steputimes))
					        {
								yidongcount=0;
					        }
					if(yidongcount<(int)(4*steputimes))
							{
						      stepv=(11.0/4.0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=1;
							}
					if(yidongcount==(int)(4*steputimes))
							{
						      stepv=(11.0/18.0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=-1;
							}
					if(yidongcount==(int)(22*steputimes))
							{
								stepv=(0*pi)/180;
								stepv=stepv/steputimes;
								directionv=-1;
							}


				}
				if(referenceindex==4)
				{
					if(yidongcount==(int)(24*steputimes))
					        {
								yidongcount=0;
					        }
					if(yidongcount<(int)(10*steputimes))
							{
						      stepv=(0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=-1;
							}
					if(yidongcount==(int)(10*steputimes))
							{
						      stepv=(7.0/6.0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=1;
							}
					if(yidongcount==(int)(16*steputimes))
												{
											      stepv=(7.0/6.0*pi)/180;
											      stepv=stepv/steputimes;
											      directionv=-1;
												}
					if(yidongcount==(int)(22*steputimes))
												{
											      stepv=(0*pi)/180;
											      stepv=stepv/steputimes;
											      directionv=-1;
												}

				}
			

		}


		//Calculate the horizontal offset of nodes 5 and 6
		if ((referenceindex==1)||(referenceindex==2))  //Calculate the movement range with the relative, calculate the coordinates with absolute
		{

					
						if(referenceindex==1)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(14*steputimes))
									{
								      stepv=(0.5*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(14*steputimes))
									{
								      stepv=(7.0/6.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(20*steputimes))
									{
										stepv=(0*pi)/180;
										stepv=stepv/steputimes;
										directionv=1;
									}


						}
						if(referenceindex==2)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(2*steputimes))
									{
								      stepv=(0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(2*steputimes))
									{
								      stepv=(18.0/8.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(10*steputimes))
																{
															      stepv=(7.0/6.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}
							if(yidongcount==(int)(16*steputimes))
																{
															      stepv=(0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=1;
																}
							if(yidongcount==(int)(20*steputimes))
																{
															      stepv=(11.0/4.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}

						}
					

				}


		//Calculate the horizontal offset of nodes 9 and 10
		if ((referenceindex==5)||(referenceindex==6))  //Calculate the movement range with the relative, calculate the coordinates with absolute
				{


						if(referenceindex==5)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(8*steputimes))
									{
								      stepv=(21.0/8.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(8*steputimes))
									{
								      stepv=(1*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(12*steputimes))
									{
										stepv=(1*pi)/180;
										stepv=stepv/steputimes;
										directionv=-1;
									}
							if(yidongcount==(int)(16*steputimes))
																{
																	stepv=(21.0/8.0*pi)/180;
																	stepv=stepv/steputimes;
																	directionv=1;
																}


						}
						if(referenceindex==6)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(4*steputimes))
									{
								      stepv=(0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(4*steputimes))
									{
								      stepv=(17.0/6.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(10*steputimes))
																{
															      stepv=(17.0/14.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=1;
																}

						}
					

				}

		//Calculate the horizontal offset of nodes 11 and 12
				if ((referenceindex==8)||(referenceindex==7))  //Calculate the movement range with the relative, calculate the coordinates with absolute
				{

							
								if(referenceindex==7)
								{
									if(yidongcount==(int)(24*steputimes))
									        {
												yidongcount=0;
									        }
									if(yidongcount<(int)(4*steputimes))
											{
										      stepv=(7.0/4.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=1;
											}
									if(yidongcount==(int)(4*steputimes))
											{
										      stepv=(12.0/6.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=-1;
											}
									if(yidongcount==(int)(10*steputimes))
											{
												stepv=(5.0/4.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}
									if(yidongcount==(int)(14*steputimes))
											{
												stepv=(0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}


								}
								if(referenceindex==8)
								{
									if(yidongcount==(int)(24*steputimes))
									        {
												yidongcount=0;
									        }
									if(yidongcount<(int)(8*steputimes))
											{
										      stepv=(4.0/8.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=-1;
											}
									if(yidongcount==(int)(8*steputimes))
											{
										      stepv=(3.0/4.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=1;
											}
									if(yidongcount==(int)(12*steputimes))
											{
											  stepv=(5.0/4.0*pi)/180;
											  stepv=stepv/steputimes;
											  directionv=-1;
											}
									if(yidongcount==(int)(16*steputimes))
											{
												stepv=(9.0/6.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}
									if(yidongcount==(int)(22*steputimes))
											{
												stepv=(3.0/2.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=-1;
											}
								}
							}
						
				
		
		}
        
		
		
		
		
		
		
		
		
		
		
		
		/***************************Little Young is here to update*************************/
		
		
		if(posture==0)  //The settings of the running posture
        {
			
			
			
			
			        //Horizontal offset in the vertical direction
					if ((referenceindex==7)||(referenceindex==8))  
						{
							 //Situation of the node 11
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==7))
										 {
														 yidongcount=0;
														 //direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(4*steputimes))&&(referenceindex==7))
							 {
								stepu=(6.0*pi)/180;
								stepu=stepu/steputimes;
								direction=-1;
							 }
							 if((yidongcount==(int)(4*steputimes))&&(referenceindex==7))
							 {
											stepu=(0*pi)/180;
								            stepu=stepu/steputimes;
											direction=1;
							 }

							 if((yidongcount==(int)(7*steputimes))&&(referenceindex==7))
							 {
								              stepu=(63.0/6.0*pi)/180;
											  stepu=stepu/steputimes;
											  direction=-1;
							 }
							 if((yidongcount==(int)(13*steputimes))&&(referenceindex==7))
							 							 {
							 								              stepu=(0*pi)/180;
							 											  stepu=stepu/steputimes;
							 											  direction=-1;
							 							 }
							 if((yidongcount==(int)(14*steputimes))&&(referenceindex==7))
							 							 {
							 								              stepu=(106.0/8.0*pi)/180;
							 											  stepu=stepu/steputimes;
							 											  direction=1;
							 							 }
							 if((yidongcount==(int)(22*steputimes))&&(referenceindex==7))
							 							 {
							 								              stepu=(19.0/2.0*pi)/180;
							 											  stepu=stepu/steputimes;
							 											  direction=-1;
							 							 }




							 //Situation of the node 12
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==8))
										 {
														 yidongcount=0;
														// direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(1*steputimes))&&(referenceindex==8))
							 {
                                                stepu=(4.0*pi)/180;
												direction=-1;
												stepu=stepu/steputimes;
															 //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(1*steputimes))&&(referenceindex==8))
							 {
								             stepu=(100.0/9.0*pi)/180;
											 stepu=stepu/steputimes;
											 direction=1;
							 }
							 if((yidongcount==(int)(10*steputimes))&&(referenceindex==8))
							 {
								             stepu=(47.0/7.0*pi)/180;
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=-1;
							 }

							 if((yidongcount==(int)(17*steputimes))&&(referenceindex==8))
							 {
								             stepu=(3.0*pi)/180;
											 stepu=stepu/steputimes;
											 //direction=(-1)*direction;
											 direction=1;
							 }
							 if((yidongcount==(int)(19*steputimes))&&(referenceindex==8))
							 							 {
							 								             stepu=(55.0/5.0*pi)/180;
							 											 stepu=stepu/steputimes;
							 											 //direction=(-1)*direction;
							 											 direction=-1;
							 							 }


						}

					if ((referenceindex==3)||(referenceindex==4))  //Calculate the movement range with the relative, calculate the coordinates with absolute
						{
							 //Situation of the node 7
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==3))
										 {
											             yidongcount=0;
													     //direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(8*steputimes))&&(referenceindex==3))
							 {
								              stepu=(62.0/8.0*pi)/180;
								              stepu=stepu/steputimes;
								              direction=-1;
							 }
							 if((yidongcount==(int)(8*steputimes))&&(referenceindex==3))
							 {
											 stepu=(78.0/10.0*pi)/180;
										     stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										     direction=1;
							 }
							 if((yidongcount==(int)(18*steputimes))&&(referenceindex==3))
							 {
							 	            stepu=(16.0/6.0*pi)/180;
							 				stepu=stepu/steputimes;
						                    //direction=(-1)*direction;
							 				direction=-1;
							 }





							 //Situation of the node 8
							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==4))
										 {
											             yidongcount=0;
													    // direction=(-1)*direction;
										 }
							 if((yidongcount<(int)(6*steputimes))&&(referenceindex==4))
							 {
								        stepu=(48.0/6.0*pi)/180;
										direction=1;
										stepu=stepu/steputimes;
														     //direction=(-1)*direction;

							 }
							 if((yidongcount==(int)(6*steputimes))&&(referenceindex==4))
							 {
										stepu=(21.0/7.0*pi)/180;
										stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										direction=-1;
							 }
							 if((yidongcount==(int)(13*steputimes))&&(referenceindex==4))
							 {
								        stepu=(57.0/7.0*pi)/180;
										stepu=stepu/steputimes;
										     //direction=(-1)*direction;
										direction=-1;
							 }
							 if((yidongcount==(int)(20*steputimes))&&(referenceindex==4))
							 {
										stepu=(30.0/4.0*pi)/180;
										     stepu=stepu/steputimes;
										     direction=1;
										     //direction=(-1)*direction;

							 }

						}
							 


							 if ((referenceindex==1)||(referenceindex==2))
							 {
							 							 //Situation of the node 5
							 							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==1))
							 										 {
							 											             yidongcount=0;
							 													     //direction=(-1)*direction;
							 										 }
							 							 if((yidongcount<(int)(6*steputimes))&&(referenceindex==1))
							 							 {
							 								              stepu=(31.0/6.0*pi)/180;
							 								              stepu=stepu/steputimes;
							 								              direction=1;
							 							 }
							 							 if((yidongcount==(int)(6*steputimes))&&(referenceindex==1))
							 							 {
							 											 stepu=(49.0/14.0*pi)/180;
							 										     stepu=stepu/steputimes;
							 										     //direction=(-1)*direction;
							 										     direction=-1;
							 							 }
							 							 if((yidongcount==(int)(20*steputimes))&&(referenceindex==1))
							 							 {
							 							 	            stepu=(18.0/4.0*pi)/180;
							 							 				stepu=stepu/steputimes;
							 						                    //direction=(-1)*direction;
							 							 				direction=1;
							 							 }





							 							 //Situation of the node 6
							 							 if((yidongcount==(int)(24*steputimes))&&(referenceindex==2))
							 										 {
							 											             yidongcount=0;
							 													    // direction=(-1)*direction;
							 										 }
							 							 if((yidongcount<(int)(8*steputimes))&&(referenceindex==2))
							 							 {
							 								        stepu=(25.0/8.0*pi)/180;
							 										direction=-1;
							 										stepu=stepu/steputimes;
							 														     //direction=(-1)*direction;

							 							 }
							 							 if((yidongcount==(int)(8*steputimes))&&(referenceindex==2))
							 							 {
							 										stepu=(4.0*pi)/180;
							 										stepu=stepu/steputimes;
							 										     //direction=(-1)*direction;
							 										direction=1;
							 							 }
							 							 if((yidongcount==(int)(18*steputimes))&&(referenceindex==2))
							 							 {
							 								        stepu=(15.0/6.0*pi)/180;
							 										stepu=stepu/steputimes;
							 										     //direction=(-1)*direction;
							 										direction=-1;
							 							 }

							 						}
							if ((referenceindex==5)||(referenceindex==6))
							{
														 //Situation of the node 9
														 if((yidongcount==(int)(24*steputimes))&&(referenceindex==5))
																	 {
																		             yidongcount=0;
																				     //direction=(-1)*direction;
																	 }
														 if((yidongcount<(int)(6*steputimes))&&(referenceindex==5))
														 {
															              stepu=(40.0/6.0*pi)/180;
															              stepu=stepu/steputimes;
															              direction=1;
														 }
														 if((yidongcount==(int)(6*steputimes))&&(referenceindex==5))
														 {
																		 stepu=(63.0/9.0*pi)/180;
																	     stepu=stepu/steputimes;
																	     //direction=(-1)*direction;
																	     direction=-1;
														 }
														 if((yidongcount==(int)(15*steputimes))&&(referenceindex==5))
														 {
														 	            stepu=(0*pi)/180;
														 				stepu=stepu/steputimes;
													                    //direction=(-1)*direction;
														 				direction=1;
														 }
														 if((yidongcount==(int)(20*steputimes))&&(referenceindex==5))
														 {
																stepu=(23.0/4.0*pi)/180;
																 direction=1;
															    stepu=stepu/steputimes;
																	     //direction=(-1)*direction;

														 }





														 //Situation of the node 10
														 if((yidongcount==(int)(24*steputimes))&&(referenceindex==6))
																	 {
																		             yidongcount=0;
																				    // direction=(-1)*direction;
																	 }
														 if((yidongcount<(int)(2*steputimes))&&(referenceindex==6))
														 {
															        stepu=(10.0*pi)/180;
																	direction=-1;
																	stepu=stepu/steputimes;
																					     //direction=(-1)*direction;

														 }
														 if((yidongcount==(int)(2*steputimes))&&(referenceindex==6))
														 {
																	stepu=(3.0*pi)/180;
																	stepu=stepu/steputimes;
																	     //direction=(-1)*direction;
																	direction=-1;
														 }
														 if((yidongcount==(int)(4*steputimes))&&(referenceindex==6))
														 {
															        stepu=(2.0*pi)/180;
																	stepu=stepu/steputimes;
																	     //direction=(-1)*direction;
																	direction=-1;
														 }
														 if((yidongcount==(int)(6*steputimes))&&(referenceindex==6))
														 {
																	stepu=(0.5*pi)/180;
																	     stepu=stepu/steputimes;
																	     direction=-1;
																	     //direction=(-1)*direction;

														 }
														 if((yidongcount==(int)(8*steputimes))&&(referenceindex==6))
														 {
																	stepu=(3*pi)/180;
														 				stepu=stepu/steputimes;
														 				direction=1;
														 				//direction=(-1)*direction;

														 }
														 if((yidongcount==(int)(10*steputimes))&&(referenceindex==6))
														 														 {
														 																	stepu=(53.0/6.0*pi)/180;
														 														 				stepu=stepu/steputimes;
														 														 				direction=1;
														 														 				//direction=(-1)*direction;

														 														 }
														 if((yidongcount==(int)(16*steputimes))&&(referenceindex==6))
														 														 {
														 																	stepu=(4.5*pi)/180;
														 														 				stepu=stepu/steputimes;
														 														 				direction=1;
														 														 				//direction=(-1)*direction;

														 														 }
														 if((yidongcount==(int)(18*steputimes))&&(referenceindex==6))
														 														 {
														 																	stepu=(2.0*pi)/180;
														 														 				stepu=stepu/steputimes;
														 														 				direction=-1;
														 														 				//direction=(-1)*direction;

														 														 }
														 if((yidongcount==(int)(20*steputimes))&&(referenceindex==6))
														 														 {
														 																	stepu=(33.0/4.0*pi)/180;
														 														 				stepu=stepu/steputimes;
														 														 				direction=-1;
														 														 				//direction=(-1)*direction;

														 														 }
													}

			




       //The following is the calculation of the horizontal offset
       //Calculate the horizontal offset of nodes 7 and 8
		if ((referenceindex==3)||(referenceindex==4))  //Calculate the movement range with the relative, calculate the coordinates with absolute
		{

				//node 7
				if(referenceindex==3)
				{
					if(yidongcount==(int)(24*steputimes))
					        {
								yidongcount=0;
					        }
					if(yidongcount<(int)(1*steputimes))
							{
						      stepv=(2.1*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=1;
							}
					if(yidongcount==(int)(1*steputimes))
							{
						      stepv=(0.25*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=1;
							}
					if(yidongcount==(int)(3*steputimes))
							{
								stepv=(3.6/4.0*pi)/180;
								stepv=stepv/steputimes;
								directionv=-1;
							}
					if(yidongcount==(int)(7*steputimes))
												{
													stepv=(2.6/3.0*pi)/180;
													stepv=stepv/steputimes;
													directionv=1;
												}
					if(yidongcount==(int)(10*steputimes))
												{
													stepv=(16.2/5.0*pi)/180;
													stepv=stepv/steputimes;
													directionv=-1;
												}
					if(yidongcount==(int)(15*steputimes))
												{
													stepv=(12.8/5.0*pi)/180;
													stepv=stepv/steputimes;
													directionv=1;
												}
					if(yidongcount==(int)(20*steputimes))
												{
													stepv=(1.8/4.0*pi)/180;
													stepv=stepv/steputimes;
													directionv=1;
												}


				}
				//node 8
				if(referenceindex==4)
				{
					if(yidongcount==(int)(24*steputimes))
					        {
								yidongcount=0;
					        }
					if(yidongcount<(int)(2*steputimes))
							{
						      stepv=(7.8/2.0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=1;
							}
					if(yidongcount==(int)(2*steputimes))
							{
						      stepv=(14.3/13.0*pi)/180;
						      stepv=stepv/steputimes;
						      directionv=-1;
							}
					if(yidongcount==(int)(15*steputimes))
												{
											      stepv=(6.3/4.0*pi)/180;
											      stepv=stepv/steputimes;
											      directionv=1;
												}
					if(yidongcount==(int)(19*steputimes))
												{
											      stepv=(4.1/3.0*pi)/180;
											      stepv=stepv/steputimes;
											      directionv=-1;
												}
					if(yidongcount==(int)(22*steputimes))
																	{
																      stepv=(4.3/2.0*pi)/180;
																      stepv=stepv/steputimes;
																      directionv=1;
																	}

				}
			

		}


		//Calculate the horizontal offset of nodes 5 and 6
		if ((referenceindex==1)||(referenceindex==2))  //Calculate the movement range with the relative, calculate the coordinates with absolute
		{

					   //node 5
						if(referenceindex==1)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(3*steputimes))
									{
								      stepv=(2.0/3.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(3*steputimes))
									{
								      stepv=(13.0/6.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(9*steputimes))
									{
										stepv=(0*pi)/180;
										stepv=stepv/steputimes;
										directionv=1;
									}
							if(yidongcount==(int)(12*steputimes))
																{
																	stepv=(1.8*pi)/180;
																	stepv=stepv/steputimes;
																	directionv=1;
																}
							if(yidongcount==(int)(22*steputimes))
																{
																	stepv=(1.5*pi)/180;
																	stepv=stepv/steputimes;
																	directionv=-1;
																}


						}
						//node 6
						if(referenceindex==2)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(9*steputimes))
									{
								      stepv=(19.0/9.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(9*steputimes))
									{
								      stepv=(19.0/12.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(21*steputimes))
																{
															      stepv=(0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}


						}
					

				}


		//Calculate the horizontal offset of nodes 9 and 10
		if ((referenceindex==5)||(referenceindex==6))  //Calculate the movement range with the relative, calculate the coordinates with absolute
				{

						//node 9
						if(referenceindex==5)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(8*steputimes))
									{
								      stepv=(30.0/8.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(8*steputimes))
									{
								      stepv=(0.5*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=-1;
									}
							if(yidongcount==(int)(10*steputimes))
									{
										stepv=(3.7*pi)/180;
										stepv=stepv/steputimes;
										directionv=-1;
									}
							if(yidongcount==(int)(20*steputimes))
																{
																	stepv=(0*pi)/180;
																	stepv=stepv/steputimes;
																	directionv=1;
																}
							if(yidongcount==(int)(22*steputimes))
																{
																	stepv=(4.0*pi)/180;
																	stepv=stepv/steputimes;
																	directionv=1;
																}


						}
						//node 10
						if(referenceindex==6)
						{
							if(yidongcount==(int)(24*steputimes))
							        {
										yidongcount=0;
							        }
							if(yidongcount<(int)(3*steputimes))
									{
								      stepv=(19.0/3.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(3*steputimes))
									{
								      stepv=(9.0/5.0*pi)/180;
								      stepv=stepv/steputimes;
								      directionv=1;
									}
							if(yidongcount==(int)(8*steputimes))
																{
															      stepv=(0.5*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}
							if(yidongcount==(int)(10*steputimes))
																{
															      stepv=(1.5*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}
							if(yidongcount==(int)(12*steputimes))
																{
															      stepv=(34.0/7.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}
							if(yidongcount==(int)(19*steputimes))
																{
															      stepv=(2.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=-1;
																}
							if(yidongcount==(int)(21*steputimes))
																{
															      stepv=(2.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=1;
																}
							if(yidongcount==(int)(22*steputimes))
																{
															      stepv=(6.0*pi)/180;
															      stepv=stepv/steputimes;
															      directionv=1;
																}

						}
					

				}

		//Calculate the horizontal offset of nodes 11 and 12
				if ((referenceindex==8)||(referenceindex==7))  //Calculate the movement range with the relative, calculate the coordinates with absolute
				{

							    //node 11
								if(referenceindex==7)
								{
									if(yidongcount==(int)(24*steputimes))
									        {
												yidongcount=0;
									        }
									if(yidongcount<(int)(4*steputimes))
											{
										      stepv=(1.8/4.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=1;
											}
									if(yidongcount==(int)(4*steputimes))
											{
										      stepv=(7.0/6.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=-1;
											}
									if(yidongcount==(int)(10*steputimes))
											{
												stepv=(10.9/4.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}
									if(yidongcount==(int)(14*steputimes))
											{
												stepv=(11.9/6.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=-1;
											}
									if(yidongcount==(int)(20*steputimes))
																				{
																					stepv=(0.3*pi)/180;
																					stepv=stepv/steputimes;
																					directionv=-1;
																				}
									if(yidongcount==(int)(21*steputimes))
																				{
																					stepv=(6.5/3.0*pi)/180;
																					stepv=stepv/steputimes;
																					directionv=1;
																				}


								}

								//node 12
								if(referenceindex==8)
								{
									if(yidongcount==(int)(24*steputimes))
									        {
												yidongcount=0;
									        }
									if(yidongcount<(int)(2*steputimes))
											{
										      stepv=(2.6*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=-1;
											}
									if(yidongcount==(int)(2*steputimes))
											{
										      stepv=(6.7/6.0*pi)/180;
										      stepv=stepv/steputimes;
										      directionv=1;
											}
									if(yidongcount==(int)(8*steputimes))
											{
											  stepv=(7.1/4.0*pi)/180;
											  stepv=stepv/steputimes;
											  directionv=-1;
											}
									if(yidongcount==(int)(12*steputimes))
											{
												stepv=(9.0/6.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}
									if(yidongcount==(int)(16*steputimes))
											{
												stepv=(9.6/6.0*pi)/180;
												stepv=stepv/steputimes;
												directionv=1;
											}
									if(yidongcount==(int)(22*steputimes))
																				{
																					stepv=(2*pi)/180;
																					stepv=stepv/steputimes;
																					directionv=-1;
																				}
								}
							}
						
				
		
		
			
		}




























		    //Update the vertical angle
			double referangle=referencenode->getSubmodule("mobility")->par("nowangle");
			nowangle = nowangle - referangle;//relatively
			nextangle = nowangle+stepu*direction;//relatively

	        nowangle=nextangle;  //relatively
		    nextangle=nextangle+referangle;//relatively
		    this->par("nowangle")=nextangle;
            nowangle=nextangle;

            //Update the horizontal angle
			double referanglev=referencenode->getSubmodule("mobility")->par("nowanglev");
			nowanglev = nowanglev - referanglev;//relatively
			nextanglev = nowanglev+stepv*directionv;//relatively

	        nowanglev=nextanglev;  //relatively
		    nextanglev=nextanglev+referanglev;//relatively
		    this->par("nowanglev")=nextanglev;
            nowanglev=nextanglev;



            yidongcount++;





	    double newx1=xr + ownR*sin(nextanglev);
	    double newy1=yr + ownR*cos(nextangle)*cos(nextanglev);
	    double newz1=zr + ownR*sin(nextangle)*cos(nextanglev);

		targetPos.setX(newx1-x1);
		targetPos.setY(newy1-y1);
		targetPos.setZ(newz1-z1);

		//ev<<"node12 undertaking"<<"    "<<selfidx<<"XXXXXX"<<newy1<<"XXXXXX"<<newz1<<endl;
										    //endSimulation();

	    //Complete the coordinate update


		//When the speed is not equal to 0, take a random direction
		//This place is sure to change. First to distinguish the node's id. 13 nodes in 4 cases (1, sink 2, shoulder hip 3, elbow knee, 4, wrist ankle
        //Then whether the mobility model can be applied directly?
		//When applying the movement mode, pay attention to setting the trajectory of each move not to a straight line, there are two ways, one is updated under the arc above the targetpos
		//The other is to modify their own mobility module. This part needs to think about how to do.
		//In short, the logical distinction between the place should be carried out here


		double distance;
		simtime_t totalTime;


		currentRelativePosition = move.getStartPos()- referencePoint;  //startpos has been updated. The referencepoint has not changed
		distance = currentRelativePosition.distance(targetPos);



		//move.setSpeed(distance+0.01);

		totalTime = distance / move.getSpeed();    //Every move is a single step

		if (totalTime >= updateInterval)
			numSteps = FWMath::round(totalTime / updateInterval);
		else
			numSteps = 1; //The selected target position is quite close so that in less than one step, it will be reached with the given velocity.

		stepSize = (targetPos - currentRelativePosition)/numSteps;
		stepTarget = (move.getStartPos()-referencePoint) + stepSize;

		//ev<<"nodeX undertaking"<<"    "<<selfidx<<"XXXXXX"<<distance<<"XXXXXX"<<totalTime<<endl;
		//									    endSimulation();


	}
	else
	{
		numSteps = 1;
		targetPos = referencePoint;
		stepSize = Coord(0,0,0);
		stepTarget = Coord(0,0,0);
	}



	move.setStart(insideWorld(stepTarget + referencePoint),simTime());

    step = 0;

	EV << "new targetPos: " << targetPos.info() << " numSteps=" << numSteps<< endl;

	//Increase the output to the file statement
	//The node location output to the corresponding file inside to contain three steps: the first to take the current node id, the second creation file, the third output,
	int index =(*this).getParentModule()->getIndex();
	std::string indexs;
	std::string postures;
	      std::stringstream stream1,stream2;
	      stream1<<index;
          indexs=stream1.str();   //Here you can also use stream >> string_temp

          stream2<<posture;
          postures=stream2.str();

          EV<<"node id"<<indexs<<endl;

	std::string filepath="C:\\data\\"+postures+"MyModel"+indexs+".txt";

	EV<<"path"<< filepath.c_str() <<endl;
    if ((fp=fopen(filepath.c_str(),"a"))!=NULL)
    {fprintf(fp,"%lf %lf %lf\n",(stepTarget + referencePoint).getX(),(stepTarget + referencePoint).getY(), (stepTarget + referencePoint).getZ());}
    fclose(fp);


}
}

/**
 * Move the host if the destination is not reached yet. Otherwise
 * calculate a new random position. This function is called by the
 * BaseMobility module considering the given update time interval.
*/
void MoBANLocal::makeMove()
{
    // increment number of steps
    step++;

    if( step == numSteps ){
    	EV << "destination reached. " << move.info() << endl;
    	setTargetPosition();  //Reach the point of time, and then look for the next target.
	}
    else if( step < numSteps ){
    	// step forward
    	stepTarget += stepSize;
    	move.setStart(insideWorld(stepTarget+referencePoint),simTime());
    }
    else{
    	error("step cannot be bigger than numSteps");
    }


}

/**
* This function is called once something is written to the blackboard of this node. If the written item has specific category dedicated for the
* type that MoBAN coordinator writes, the item will be read and the corresponding variables are set.
*/
void MoBANLocal::receiveBBItem(int category, const BBItem *details, int scopeModuleId)
{


    if(category == catRefMove)
    {


    	BBMoBANMessage m(*static_cast<const BBMoBANMessage*>(details));
    	referencePoint= m.position;
 //       this->par("x")=referencePoint.getX();
  //      this->par("y")=referencePoint.getY();
  //      this->par("z")=referencePoint.getZ();

    	if(posture!=m.posture)//The parameter reset is only executed when the change is made
    	{
    	   posture= m.posture;
    	   ParaSettingBasedonPosture(posture);
    	}


    	speed = m.speed;
    	if ( !FWMath::close(speed,0.0) )
    		 move.setSpeed(speed); // IF WE SET ZERO, it is not going to move anymore

        radius = m.radius;

    	move.setStart(insideWorld(stepTarget+referencePoint),simTime());

        EV<<"Node "<< getParentModule()->getIndex() <<" received new reference point."<<endl;
        EV<< "New speed:" << speed <<" , new radius: "<< radius <<endl;


        //ParaSettingBasedonPosture(posture);
    }
}

/**
 * Gets a position and return the nearest point inside the simulation area if the point is outside the area
*/

double MoBANLocal::getnowangle()
{
   return nowangle;

}


//This function is used to control the movement range not to exceed the simulation area
Coord MoBANLocal::insideWorld(Coord apoint)
{
	double xmax, ymax, zmax;

	Coord NearestBorder = apoint;

	xmax = world->getPgs()->getX();
	ymax = world->getPgs()->getY();
	zmax = world->getPgs()->getZ();

	if (NearestBorder.getX() < 0)
		NearestBorder.setX(0.0);

	if (NearestBorder.getY() < 0)
		NearestBorder.setY(0.0);

	if (NearestBorder.getZ() < 0)
		NearestBorder.setZ(0.0);

	if (NearestBorder.getX() > xmax)
		NearestBorder.setX(xmax);

	if (NearestBorder.getY() > ymax)
		NearestBorder.setY(ymax);

	if (NearestBorder.getZ() > zmax)
		NearestBorder.setZ(zmax);

	return NearestBorder;

}


void MoBANLocal::ParaSettingBasedonPosture(int posture)
{

    if (posture==1)   //walking time parameter settings
{
	switch (selfidx)
	//Here is the main operation,choose a different reference point and relative motion radius according to the node id
	{
	case 0://Chest sink
	case 1://Left shoulder
    case 2://Right shoulder
	case 3://The left hip
	case 4://The right hip, the four nodes and the sink node to move relatively static
		referenceindex=-1;
		ownR=0.0;
        stepu=0.0;
        mupperbound=0.0;
        mlowerbound=0.0;
		direction=0;


		stepv=0;//Horizontal movement angular velocity
		nowanglev=0;//Current horizontal angle
        directionv=0;
		this->par("nowanglev")=nowanglev;

    break;

	case 5:

		referenceindex=1;
		ownR=33;
		stepu=(4.5*pi)/180;
		mupperbound=(28*pi)/180;
        mlowerbound=-(26*pi)/180;
        direction=-1;

		stepv=(0.5*pi)/180;//Horizontal movement angular velocity
		nowanglev=(-9*pi)/180;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

        nowangle=(28*pi)/180;
        this->par("nowangle")=nowangle;


		break;

	case 6:

		referenceindex=2;
		ownR=33;
		stepu=(2.5*pi)/180;
		mupperbound=(8*pi)/180;
        mlowerbound=-(22*pi)/180;
        direction=1;

        stepv=(0*pi)/180;//Horizontal movement angular velocity
        nowanglev=(-3*pi)/180;//Current horizontal angle
        directionv=-1;
		this->par("nowanglev")=nowanglev;

        nowangle=-(20*pi)/180;
        this->par("nowangle")=nowangle;



		break;
	case 7:
		referenceindex=3;
		ownR=43;
	//	stepu=(2.5*pi)/180;
	//	mupperbound=(15*pi)/180;
    //    mlowerbound=-(15*pi)/180;
		stepu=(5*pi)/180;
		mupperbound=(30*pi)/180;
        mlowerbound=-(30*pi)/180;
        direction=1;

        stepv=(11.0/4.0*pi)/180;//Horizontal movement angular velocity
        nowanglev=(-6*pi)/180;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

        nowangle=-(30*pi)/180;
        this->par("nowangle")=nowangle;

		break;
	case 8:

		referenceindex=4;
		ownR=43;
//		stepu=(2.5*pi)/180;
//		mupperbound=(15*pi)/180;
//        mlowerbound=-(15*pi)/180;
		stepu=(5*pi)/180;
		mupperbound=(30*pi)/180;
        mlowerbound=-(30*pi)/180;
        direction=-1;

        stepv=(0*pi)/180;//Horizontal movement angular velocity
        nowanglev=(-5*pi)/180;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

        nowangle=(30*pi)/180;
        this->par("nowangle")=nowangle;

		break;

	case 9:
		referenceindex=5;
		ownR=25;
		//stepu=(1.25*pi)/180;
		//mupperbound=(15*pi)/180;
        //mlowerbound=0;

		stepu=(7.25*pi)/180;

		stepv=(2*pi)/180;
		nowanglev=(15*pi)/180;
		directionv=-1;
		this->par("nowanglev")=nowanglev;

		stepv=(21.0/8.0*pi)/180;//Horizontal movement angular velocity
		nowanglev=(13*pi)/180;//Current horizontal angle
		directionv=-1;

		mupperbound=(65*pi)/180;
        mlowerbound=-(22*pi)/180;

        nowangle=(66*pi)/180;



        this->par("nowangle")=nowangle;

        direction=-1;


		break;
	case 10:

		referenceindex=6;

		ownR=25;
		//stepu=(1.25*pi)/180;
		//mupperbound=(15*pi)/180;
        //mlowerbound=0;

		stepu=(4.25*pi)/180;

		stepv=(0*pi)/180;//Horizontal movement angular velocity
		nowanglev=(8*pi)/180;//Current horizontal angle
		directionv=1;
		this->par("nowanglev")=nowanglev;

        /*********************There is a problem here, how assigned twice? (Problem solved by little Young)*****************************/


		mupperbound=(51*pi)/180;
        mlowerbound=-(0*pi)/180;

        nowangle=-(0*pi)/180;



        this->par("nowangle")=nowangle;
		direction=1;


		break;
	    case 11:
		referenceindex=7;

		ownR=47;
		stepu=(0*pi)/180;
		mupperbound=0;
        mlowerbound=-(30*pi)/180;
        direction=-1;

        stepv=(7.0/4.0*pi)/180;//Horizontal movement angular velocity
        nowanglev=(5*pi)/180;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

        nowangle=-(30*pi)/180;
        this->par("nowangle")=nowangle;

		break;
	case 12:
		referenceindex=8;
		ownR=47;
		stepu=(0*pi)/180;
		mupperbound=0;
        mlowerbound=-(60*pi)/180;
		direction=1;

		stepv=(4.0/8.0*pi)/180;//Horizontal movement angular velocity
		nowanglev=(-2*pi)/180;//Current horizontal angle
		directionv=-1;
		this->par("nowanglev")=nowanglev;

		nowangle=-(0*pi)/180;
        this->par("nowangle")=nowangle;

        break;

	case 13:
		referenceindex=-1;
				ownR=0.0;
		        stepu=0.0;
		        mupperbound=0.0;
		        mlowerbound=0.0;
				direction=0;
		stepv=0;//Horizontal movement angular velocity
		nowanglev=0;//Current horizontal angle
        directionv=0;
		this->par("nowanglev")=nowanglev;

				this->par("nowangle")=nowangle;
		break;
	}
}





/**********************************Little Young is here to update*******************************************/
 if(posture==0) //The settings of the running
{
    	switch (selfidx)
    	//Here is the main operation, choose a different reference point and relative motion radius according to the node id
    	{
    	case 0://Chest sink
    	case 1://Left shoulder
        case 2://Right shoulder
        case 3://The left hip
    	case 4://The right hip, the four nodes and the sink node to move relatively static
    		referenceindex=-1;
    		ownR=0.0;
            stepu=0.0;
            mupperbound=0.0;
            mlowerbound=0.0;
    		direction=0;
            nowangle=0;
			
		stepv=0;//Horizontal movement angular velocity
		nowanglev=0;//Current horizontal angle
        directionv=0;
		this->par("nowanglev")=nowanglev;

        break;

    	case 5:

    		referenceindex=1;
    		ownR=33;
    		stepu=(31.0/6.0*pi)/180;
    		mupperbound=(30*pi)/180;
            mlowerbound=-(30*pi)/180;
            direction=1;
            nowangle=(-28.0*pi)/180;
            this->par("nowangle")=nowangle;
			
			stepv=(2.0/3.0*pi)/180;;//Horizontal movement angular velocity
		    nowanglev=(-2*pi)/180;;//Current horizontal angle
            directionv=-1;
	     	this->par("nowanglev")=nowanglev;

    		break;

    	case 6:

    		referenceindex=2;
    		ownR=33;
    		stepu=(25.0/8.0*pi)/180;
    		mupperbound=(30*pi)/180;
            mlowerbound=-(30*pi)/180;
            direction=-1;
            nowangle=-(22*pi)/180;
            this->par("nowangle")=nowangle;
			
			stepv=(19.0/9.0*pi)/180;;//Horizontal movement angular velocity
		    nowanglev=(16.0*pi)/180;;//Current horizontal angle
            directionv=-1;
		    this->par("nowanglev")=nowanglev;

    		break;
    	case 7:
    		referenceindex=3;
    		ownR=43;
    		stepu=(62.0/8.0*pi)/180;
    		mupperbound=(45*pi)/180;
            mlowerbound=-(45*pi)/180;
            direction=-1;

            nowangle=(26*pi)/180;
            this->par("nowangle")=nowangle;
			
			stepv=(2.1*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(3.6*pi)/180;;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

    		break;
    	case 8:

    		referenceindex=4;
    		ownR=43;
    		stepu=(48.0/6.0*pi)/180;
    		mupperbound=(45*pi)/180;
            mlowerbound=-(45*pi)/180;
            direction=1;

            nowangle=(-7*pi)/180;
            this->par("nowangle")=nowangle;
			
			stepv=(7.8/2.0*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(-2*pi)/180;;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;

    		break;


    	case 9:
    		referenceindex=5;
    		ownR=25;
    		//stepu=(1.25*pi)/180;
    		//mupperbound=(15*pi)/180;
            //mlowerbound=0;

    		stepu=(40.0/6.0*pi)/180;
    		nowangle=(80*pi)/180;
    		direction=1;

    		mupperbound=(150*pi)/180;
            mlowerbound=(30*pi)/180;

            this->par("nowangle")=nowangle;
			
			stepv=(30.0/8.0*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(8.0*pi)/180;;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;




    		break;
    	case 10:

    		referenceindex=6;

    		ownR=25;
    		//stepu=(1.25*pi)/180;
    		//mupperbound=(15*pi)/180;
            //mlowerbound=0;

    		stepu=(10*pi)/180;
    		nowangle=(60*pi)/180;
    		direction=-1;

    		mupperbound=(150*pi)/180;
            mlowerbound=(30*pi)/180;


            this->par("nowangle")=nowangle;
			
			stepv=(19.0/3.0*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(-12*pi)/180;;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;




    		break;
    	case 11:
    		referenceindex=7;

    		ownR=47;
    		stepu=(6.0*pi)/180;

    		mupperbound=0;
            mlowerbound=-(90*pi)/180;

            nowangle=-(13*pi)/180;
            direction=-1;
            this->par("nowangle")=nowangle;
			
			stepv=(9.8/4.0*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(4*pi)/180;;//Current horizontal angle
        directionv=1;
		this->par("nowanglev")=nowanglev;








    		break;
    	case 12:
    		referenceindex=8;
    		ownR=47;

    		stepu=(4.0*pi)/180;
    		nowangle=-(89*pi)/180;
    		direction=-1;

    		mupperbound=0;
            mlowerbound=-(90*pi)/180;

            this->par("nowangle")=nowangle;
			
			stepv=(2.6*pi)/180;;//Horizontal movement angular velocity
		nowanglev=(0*pi)/180;;//Current horizontal angle
        directionv=-1;
		this->par("nowanglev")=nowanglev;





            break;

    	case 13:
    		referenceindex=-1;
    				ownR=0.0;
    		        stepu=0.0;
    		        mupperbound=0.0;
    		        mlowerbound=0.0;
    				direction=0;
					
					stepv=0;//Horizontal movement angular velocity
		nowanglev=0;//Current horizontal angle
        directionv=0;
		this->par("nowanglev")=nowanglev;
    		break;
    	}
}



}



void  MoBANLocal::handleSelfMsg(cMessage * msg)
{

	makeMove();
    updatePosition();



    if( !moveMsg->isScheduled() && move.getSpeed() > 0) {
    	scheduleAt(simTime() + updateInterval, msg);
    } else {
    	delete msg;
    	moveMsg = NULL;
    }


}

