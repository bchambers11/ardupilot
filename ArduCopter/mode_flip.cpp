#include "Copter.h"

#if MODE_FLIP_ENABLED == ENABLED

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          Flip_Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          Flip_Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          Flip_Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define FLIP_THR_INC        0.20f   // throttle increase during Flip_Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        0.24f   // throttle decrease during Flip_Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

//state variables
private float V_z;
private float rpm;
private float height;

bool engineFailed = false;
bool NRreached = false;
bool landing = false;
uint16_t idle_count = 0;

private float oneMinusAlpha = 0.5;
private float avg = 0;
private float weightedSum = 0;
private float weightedCount = 0;
private float reading, lastReading, acc;

// flip_init - initialise flip controller
bool Copter::ModeFlip::init(bool ignore_checks)
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

// run - runs the flip controller
// should be called at 100hz or more
void Copter::ModeFlip::run()
{
  //update states
  update_states();
  //stabilize helicopter
  attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);


  //state determination could turn into its own function
  if (height > 40 && state == AutoRot_Takeoff)       //change 40 to a predefed value
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
        //if the wait time has passsed cut the engine -> can make this random
        if(idle_count > WAIT_TIME)    //need define for WAIT_TIME
        {
          _motors.rcwrite(AP_MOTORS_HELI_SINGLE_RSC, 0);     //_motors should be accessible from here(?)
        }
        idle_count ++;

        break;

    case AutoRot_Inititate:
        min_tau();   //increase the rotor speed to 1.2 NR
        if(rpm > 1.15*NR)   //NR needs to be predef also
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
        max_F();  //Maximize upward acceleration to slow down

  }
}

private void update_states()
{
  rpm = RPM.get_rpm(0);
  //these two functions for getting variables need to be tested
  height = _inav.get_altitude();
  V_z = _inav.get_velocity_z();
}

private bool detectEngineFailure()
{
  lastReading = reading;
  reading = RPM.get_rpm(0);  //still needs to be tested
  acc = lastReading - reading;

  weightedSum = acc+oneMinusAlpha*weightedSum;
  weightedCount = 1 + weightedCount;
  avg = weightedSum/weightedCount;

  if(avg<cutoff) { return true; }
  else { return false; }

}

private void zero_tau()
{
  phi_desired = 0.2272322/(rpm - 649.935288)
  - 1403.8166*V_z/(rpm + 65.3905)
  - 0.0199812*V_z
  - 4.0994168;

  phi_desired_scaled = phi_desired * k; //throttle must be 0->1
  attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);

}

private void max_tau()
{
  phi_desired = -4.27058949/(rpm + 1.02102643)
 - 18.66648244*V_z*V_z/(rpm - 12.31192228)
 + 13.48316539*V_z/rpm
 + 0.02156654*V_z*V_z
 - 1.45204798*V_z
 + 14.98632206;

 phi_desired_scaled = phi_desired*k;
 attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);
}

private void max_F()
{
  phi_desired = 3.31448333/(rpm + 46.4440510)
 - 24.6467786*V_z*V_z/(rpm + 9.07472533)
 + 55.4865572*V_z/rpm
 + 0.0234441090*V_z*V_z
 - 1.18983342*V_z
 + 8.71212789;

phi_desired_scaled = phi_desired*k;
attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);

}



#endif
