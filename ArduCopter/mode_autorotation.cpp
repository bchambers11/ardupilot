#include "Copter.h"

#include "mode.h"

#if MODE_AUTOROTO_ENABLED == ENABLED

/*
 * Init and run calls for acro flight mode
 */

//variables for state machine
AutoRotationState state = AutoRot_Takeoff;
bool NRreached = false;
bool landing = false;
//variables for detecting engine failure
private float oneMinusAlpha = 0.5;
private float avg = 0;
private float weightedSum = 0;
private float weightedCount = 0;
private float reading, lastReading, acc;

bool Copter::ModeAutoRoto::init(bool ignore_checks)
{
  //initialize state
  AutoRotationState state = AutoRot_Takeoff;
   // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
           (get_pilot_desired_throttle(channel_throttle->get_control_in(), copter.g2.acro_thr_mid) > copter.get_non_takeoff_throttle())) {
       return false;
   }

   return true;
}

void Copter::ModeAutoRoto::run()
{

  //state determination could turn into its own function
  if (_inav.get_altitude() > 40 && state == AutoRot_Takeoff)       //change 40 to a predefed value
  {
      state = AutoRot_Idle;
  }
  else if (engineFailed == true && state == AutoRot_Idle)
  {
      state = AutoRot_Inititate;
  }
  else if (NRreached == true && state == AutoRot_Inititate)
  {
      state = AutoRot_Freefall;
  }
  else if(landing == true && state == AutoRot_Freefall)
  {
    state = AutoRot_Landing;
  }

  switch (state) {

    case AutoRot_Takeoff:
        //go to 40 meters
        pos_control->set_alt_target(40); //not sure the units here yet
        break;

    case AutoRot_Idle:
        pos_control->set_alt_target(40);//hold at 40m
        //need to alllow user to cut engine
        engineFailed = detectEngineFailure();
        break;

    case AutoRot_Inititate:
        minimize_tau();   //increase the rotor speed to 1.2 NR
        if(rpm_sensor.get_rpm > 1.15*NR)   //NR needs to be predef also
        {
          NRreached = true;
        }
        break;

    case AutoRot_Freefall:
        zero_tau();      //keep the rotor speed constant
        //check for if it is time to land
        landing = checkForLanding();  //this function will determine if its time to land
        break;

    case AutoRot_Landing:
        maximize_F();  //Maximize upward acceleration to slow down 

  }





}

//implemented how Tom initially wrote it
//this will average over all samples, should only be last couple
private bool detectEngineFailure()
{
  lastReading = reading;
  reading = rpm_sensor.get_RPM();  //this will need to be changed once we have hall effect
  acc = lastReading - reading;

  weightedSum = acc+oneMinusAlpha*weightedSum;
  weightedCount = 1 + weightedCount;
  avg = weightedSum/weightedCount;

  if(avg<cutoff) { return true; }
  else { return false; }


}
