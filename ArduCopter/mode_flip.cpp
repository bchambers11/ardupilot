#include "Copter.h"

#if MODE_FLIP_ENABLED == ENABLED


#define WAIT_TIME 100
#define NR        2000
#define K         1

//state variables
float V_z;
float rpm;
float height;

bool engineFailed = false;
bool NRreached = false;
bool landing = false;
uint16_t idle_count = 0;

float oneMinusAlpha = 0.5;
float avg = 0;
float weightedSum = 0;
float weightedCount = 0;
float reading, lastReading, acc;

AutoRotationState state = AutoRot_Takeoff;

// flip_init - initialise flip controller
bool Copter::ModeFlip::init(bool ignore_checks)
{
  //initialize state

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
          //FIX THIS ASAP
          //rc_write(AP_MOTORS_HELI_SINGLE_RSC, 0);     //_motors should be accessible from here(?)
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

void update_states()
{
  //FIX
  //rpm = RPM.get_rpm(0);
  rpm = 0;
  //these two functions for getting variables need to be tested
  height = _inav.get_altitude();
  V_z = _inav.get_velocity_z();
}

bool detectEngineFailure()
{
  lastReading = reading;
  //FIX
  //reading = RPM.get_rpm(0);  //still needs to be tested
  reading = 0;
  acc = lastReading - reading;

  weightedSum = acc+oneMinusAlpha*weightedSum;
  weightedCount = 1 + weightedCount;
  avg = weightedSum/weightedCount;

  if(avg<cutoff) { return true; }
  else { return false; }

}

void zero_tau()
{
  phi_desired = 0.2272322/(rpm - 649.935288)
  - 1403.8166*V_z/(rpm + 65.3905)
  - 0.0199812*V_z
  - 4.0994168;

  phi_desired_scaled = phi_desired * K; //throttle must be 0->1
  attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);

}

void max_tau()
{
  phi_desired = -4.27058949/(rpm + 1.02102643)
 - 18.66648244*V_z*V_z/(rpm - 12.31192228)
 + 13.48316539*V_z/rpm
 + 0.02156654*V_z*V_z
 - 1.45204798*V_z
 + 14.98632206;

 phi_desired_scaled = phi_desired*K;
 attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);
}

void max_F()
{
  phi_desired = 3.31448333/(rpm + 46.4440510)
 - 24.6467786*V_z*V_z/(rpm + 9.07472533)
 + 55.4865572*V_z/rpm
 + 0.0234441090*V_z*V_z
 - 1.18983342*V_z
 + 8.71212789;

phi_desired_scaled = phi_desired*K;
attitude_control->set_throttle_out(phi_desired_scaled, false, g.throttle_filt);

}

bool checkForLanding()
{
  //comparing energy will happen here
  return false;
}



#endif
