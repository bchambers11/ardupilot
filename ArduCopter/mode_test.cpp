#include "Copter.h"

//#include "mode.h"

bool Copter::ModeAutoRoto::init(bool ignore_checks)
{
  
   // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
           (get_pilot_desired_throttle(channel_throttle->get_control_in(), copter.g2.acro_thr_mid) > copter.get_non_takeoff_throttle())) {
       return false;
   }

   return true;
}

void Copter::ModeAutoRoto::run()
{
  //PUT TEST CODE HERE
  DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt,rpm1",
                                         "smq", // units: seconds, meters
                                         "FB0", // mult: 1e-6, 1e-2
                                         "QfQ", // format: uint64_t, float
                                         AP_HAL::micros64(),
                                         (double)alt_in_cm)
                                         (double)rpm_in_rot/s;

  DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt",
                                        "sm", // units: seconds, meters
                                        "FB", // mult: 1e-6, 1e-2
                                        "Qf", // format: uint64_t, float
                                        AP_HAL::micros64(),
                                       (double)alt_in_cm);

}

//ADD FUNCTIONS TO BE CALLED IN run() down here
