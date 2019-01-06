/***************************************************************************

    file                 : robot_base.cpp
    created              : Mon 13 Feb 11:40:23 GMT 2017
    copyright            : (C) 2002 Author

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
robot_base(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("carter");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

int getGear(tCarElt *car)
{
	/*
	gear up and gear down are both seperate tasks i can make in the bt

	"is not at red line?" can be the bool at start to check if it even needs
	to shift gear or now

	can also be done with a switch such as in the drive command it just isnt as
	efficent for driving but its the way ive worked out for up shifting
	*/

	//gear up

	if (car->_gear <= 0) return 1;
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine / gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*0.9 < car->_speed_x) {
		return car->_gear + 1;
	}
	//gear down
	else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine / gr_down;
		if (car->_gear > 1 && omega*wr*0.9 > car->_speed_x + 4.0) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
	//always return gear no matter what
}



/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 

    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 

	//this code below will completely steer the car around any track *at slow speed only*
	//just implement a behaviour tree for gears and stuff as well as trying to do it for steering
	//and use this code for the steering section

	float angle;
	//tuning modifier
	const float SC = 1;


	angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
	angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;

	// the steer value calculation is now between 1.0 and -1.0
	car->ctrl.steer = angle / car->_steerLock;

	//Behaviour tree to control all of these 
	car->ctrl.accelCmd = 1; 
	car->ctrl.brakeCmd = 0.0;
	//getGear(car);


	//this is potienally another way of switching gears
	int gearNumber;
	if (car->priv.enginerpm >= car->priv.enginerpmRedLine)
	{
		switch (car->_gear)
		{
			case 0:
				gearNumber = 1;
				break;
			case 1:
				gearNumber = 2;
				break;
			case 2:
				gearNumber = 3;
				break;
			case 3:
				gearNumber = 4;
				break;
			default:
				break;
		}
	}
	car->ctrl.gear = gearNumber;


	  /*
	 * add the driving code here to modify the
	 * car->_steerCmd
	 * car->_accelCmd
	 * car->_brakeCmd
	 * car->_gearCmd
	 * car->_clutchCmd
	 */

    car->ctrl.brakeCmd = 1.0; /* all brakes on ... */ 
    /*  
     * add the driving code here to modify the 
     * car->_steerCmd 
     * car->_accelCmd 
     * car->_brakeCmd 
     * car->_gearCmd 
     * car->_clutchCmd 
     */ 
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}



/*
Utilities
*/




float getAllowedSpeed(tTrackSeg *segment)
{
	//if segment is straight then get max speed
	if (segment->type == TR_STR)
	{
		return FLT_MAX;
	}
	//otherwise work out max speed based on friction*radius of the turn (to avoid spinouts)
	else
	{
		float mu = segment->surface->kFriction;
		return sqrt(mu*segment->radius);
	}
}
