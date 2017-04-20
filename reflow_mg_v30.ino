/*******************************************************************************
* Title: Reflow Oven Controller
*
* Author: John Groezinger
* Date: 20 April 2017
*
* Notes regarding usage
*
* This code can best be described as a project grown out of conrol.  What was meant to be
* a simple Floaster Oven build based on rocketscream hardware/software turned into an application of a time delay PID
* control loop with production implications and some self education.
*
* But, fun, nonetheless.  So this is a quick overview of steps to consider when using this code.
* The code is a bit of an engineer's sandbox so there are a number of configuration options
* that can be invoked to characterize the oven, then options for configuring the oven
* for best results, and lastly a number of checks to turn this into a safer and production level oven.
* However, don't assume that all options necessariy make sense for your application.
* Finally, there are some embedded tools for debugging and verification
* Like most engineering, options are left for future work
*
* To determine proper oven parameters, I'd recommend you start by enabling the constant on feature ifdef. CONSTANT_TEMP
* This will add a 4th option to the menu which will simply set the oven to a constant temp.
* Take a look in the defines and you will find a way to set the constant temp, an optional method to disable the PID up to a point close to the setpoint,
* and ways to apply one or two PID parameter sets to the oven.
*
* For code verification, if interested, there are a few configurations to consider.
*
* There are two major DEUBG options.  
* ALLPRINT is the most useful and will dump a variety of internal state change timestamps to the console.
* ALPRINT_WIN adds even more information regarding specific window the shift in the PID. It tends to be voluminous but sometimes useful.
* The NOHEAT config prevents the SSR from ever turning on.  Most useful when doing simulations.
*
* So speaking of SIM simulation, by enabling this option and one of the datasets, you can input a fixed
* set of stimuli into the oven.  Descriptions of the datasets are included.  It was most useful for testing
* code but you can watch the response of the PID algorithm with it as well.
*
* Moving into actually using the oven, I'd recommend you review the optimized parameters from the constant temp for determination 
* of what makes sense.
*
* Of primary interest are the PID Tuning Parameters for preheat, soak and reflow.
* I'd recommend you do experiments here as well as use the information from the CONSTANT run to help you decide on what makes sense.
* Options here are one pid set for everything (ONEPIDSET defined), one pid set for each stage (PID_NEAR_FAR_PREHEAT and PID_NEAR_FAR_REFLOW not defined),
* and two PID sets for PREHEAT and REFLOW based on the respective TEMP parameter if the NEAR_FAR parameter is defined.
* Also, See bangbang below.
*
* Changing the setpoints is possible for Lead and LeadFree profiles as a set of defines.
* Please note that the Preheat -> Soak transition is the same for both profiles.
*
* There are a variety of user pref config options for frequency of reading the thermocouple and processing rate,
* debounce options for the switch, and buzzer noise times.  
*
* There are defines for the slope in SOAK stage (5C every 9 seconds).
* The window size and sample time can be changed but be sure you read the notes from the PID library author.
* These are tightly tied to together and frankly should be the same.
*
* From a production / safety point of view, I put in place a number of checks.
* These include ramp rate checks, over and under temp checks, and maximum time checks.  
* You should review this for your oven so they don't get actuated under normal conditions.
* You may note that some of the checks are redundant but they are there nonetheless.
* In general, the oven attempts to do "the right thing".
* If there is a Preheat error, the oven aborts into cooldown.  I don't attempt recovery.
* If there is a soak error, it will do the same.  Checks here include overall time and not getting hot enough (still below reflow and still safely abort)
* Reflow is a bit of a quandry.  You are already this far and things are working so you don't want to screw up the board.
* I opted to put in a bangbang check which has parameters defined to turn the oven full on to try and hit peak temp as a last resort.
* Ways to defeat this are noted.  All of these parameters and more detailed information are below.
*
* Lastly actual usage is a bit different from the original rocket scream code. 
*   - By pressing the button (switch 1), you will scroll through 3 options, Pb free profile, Pb profile, and Summary.
*     Holding the button for 2 seconds (until it displays "Ready!") will execute the option
*   - Lead and nonLead profiles start executate as you would think, Summary displays information run since plugin (it is volatile)
*   - If you wish to use serial port, set to 115.2Kbaud
*   - The serial information has been augmented to include commas for easier input into Excel and changing output to DutyCycle
*     Executing the summary option will result in a nonvolatile summary of oven uses being printed to serial port.
*   - Both at the end of runs on the serial port and on the summary screen on the LCD, a number of useful metrics are printed including
*     times and temperaturs (e.g., total run time, peak temp, time in reflow, etc)
*   - Pressing the button at any time during a run will cancel exactly as before.
*   - The buzzer has two modes.  Quick short beeps are acknowledgement of start AND a cue to start opening the door for reflow cool down
*     The second mode is long beeps indicating an error.
*   - The oven used to display "cool" after reaching peak temp which, while true, is confusing since the oven stage is actually reflow.
*     This is now called RF2 and after the oven drops down from max temp, a target temp will be displayed in the uppper right corner
*     which will decrease at 1C/sec.  This is meant to cue how to open the door to achieve optimal cooling.
*
*******************************************************************************
*
* Version: 2.1 13 March 2017
*   13 March 2017 
*   - Stripped out all ifdefs for old controller and started fixing indents
*   - added ifdef NOHEAT for output testing.  
*   - Changed windowsize from 2000 to 1000.  
*     2000 just doesn't seem right when the Compute loop is set to 1000.
*   - PID parms testing resulted in honing in on 50/0.1/10
*   - Added long switch press support to start oven
*   - Adding dual profile support with short switch press
*   - Confirmed loop time is about 150uS
*   - Confirmed windowsize of 2000 isn't right, left at 1000
*   - Added documentation below on testing
*   - Reviewed Brett B PID treatise 
*   - Added loopIterations to count loop passes. Debug use
*   - Reviewed a number of solder profiles and this seems to be in the middle
*   - Given to MM for review
*
* Version 2.2 16 March 2017
*   - Put in place output metrics
*   - Changed sensor sampling time to 100mS (from 1S) to avoid phasing problems.
*   - Potential BUG fixed, a second loop of the oven would not load PID PREHEAT parameters
*   - Added more prints to serial output and cleanup labels on same
*   - Add commas rather than spaces to output
*   - Add column to serial output for percentage of output
*   - HACK: Turned off SSR control when in cool down state (there is a logic BUG here)
*     When I get a chance, look at turning off the PID
*   - Remove parameter loading during cool down (see above)
*   - Add summary of run to serial output 
*   - Fix if / if statement below (consolidate)
*   - I see no need to changing the PID parameters.  The oven seems to work
*     ok with constant but this is mine.  
*   - Consider moving average to get rid of wiggles on thermocouple input.
*     Not done: After increasing thermocouple loop rate, most of this went away.
*   - Add defaults to case switch statements.  Probably print error and shut off oven and hang.
*   - Allow summary statistics to be seen on display with short button press
*   - Temp display of 0.00C is has too many sig digits, it only does 0.25, 0.50, etc
*     Good enough for now
*   - Added compile time macro output to serial print
*   - Cooling is displayed after peak attained.  However, it really is still 
*     in reflow.  An additional state could fix this.
*   - Add beep / display when to open door
*   - Review when PID parameters are loaded 
*   - Sending to MM for review
*
* Version 2.3 19 March 2017
*   - buzzer Chirp at beginning of oven start
*   - started experiment of turning off the PID with manual setting during cooling
*   - put in double check on oven state for turning on oven relay
*   - and manually turned output to 0
*   - turned off the PID during cool down and lose the HACK
*   - Cleanup flag variables w/enum
*   - Added a few more inits
*   - Changed test in SSR ON loop to be => rather than just > to get full 100% of SSR
*   - Added ALLPRINT and ALLPRINT_WIN
*   - Added ramp rate output
*   - Reviewed V1.7 of Brett PID auto tune code and decided not to implement any features of it
*     Even Brett questions usefulness of autotune.
*   - Reviewed Panasonic code port and added ramp feature to printout
*   - Changed time tests for thermcouple and display to be >= and not just > which fundamentally
*     is a bit weird but did not result in any drift
*   - Fixed bug in SSR loop which would turn the SSR every cycle even when it should not.
*   - Put in place 11mS fix for getting windowStartTime much closer to PID calc time
*     This woudl result in a consistent error of 1% in the output loop
*   - Put in place a + or - during cooling.  A + means the oven is cooling too fast (shut the door).
*     "-" is open the door.  blank is just right.  This may need some work to give the correct feel but it's not horrid.
*   - Most important.  By using the ALLPRINT macros, validated the control of the SSR aligned with the output of the
*     PID.  This found 2 more bugs and seems to have all but eliminated some of the nondeterministic nature of the code.
*   - Serial to 115200 baud
*   - Release to MM 
*
* V2.4 24 March 2017
*   - Added moving average to input to smooth out some of the bumps
*   - At the same time, kicked up the read frequency of the thermocouples
*     Since the moving average is by 3, the "interesting" rate of change of temp is about 1C/sec (it will max out at 3C/sec)
*     and the significance of the reading is 0.25C, I decided to do two things.  I wanted to capture changes on the thermocouple
*     close to the 0.25C with the moving average and secondly I decided to intentionally decouple the reading from the rest of 
*     the PID loop (1 sec).  Therefore, a 0.25C change represents about 0.25 second and to get that through the moving average
*     would be 3x that rate or sample at 12 second or 83ms/update.  This also has the benefit of decoupleing the temp 
*     loop from the PID control loop.  Which essentially addresses the observation below.
*
*     NOTE from original code inspection: Biggest issue I see is the thermocouple, display, and PID 
*     calc all run on their own timers
*     E.g., thermcouple reading is not in sync with the PID loop.  
*     For now I sped up the the thermcouple loop to 10X the display speed / PID calc speed but 
*     that really isn't the best way to do this.
*     Not sure I like all the calls to millis(), potential out of phase things...
*
*     Case in point on above comment.  The windowStartTime is tightly tied to the PID
*     compute loop since they are both running on the same timeframe.  Thus, a phase difference
*     results directly in an output error.  In this case, the windowSizeTime clock started
*     too early (about 11mS) resulting directly in an 11mS decrease in actual ON time
*     from that requested by output.  I have put in place a patch to start the windowStartTime
*     right before the PID Calc loop for the first time to attempt to minimize this.
*     This did reduce the 11mS delay to close to zero
*   - Printed out PID parms with output change with ALLPRINT
*   - Released to MM
*
* V2.5 24 March 2017
*   - Added "State" to printout for easy grepping
*   - Fixed end of reflow determination logic bug
*   - Adding constant temp operation for tuning purposes
*   - Per Mark review, added profile information to printout
*   - Moved Data printout windowing information to ALLPRINT_WIN
*   - Moved the thermocouple loop from 83 to 80 to give it a rate of 12.5Hz 
*     thereby allowing to move w.r.t. 1 sec windows
*     That was original intent but didn't do it correct
*   - As a result of the constant temp testing which showed the oven doing massive overshoot
*     and reading an article on floasters, I decided to put in place code to not run 
*     the PID all the time during preheat as an experiment.  This was very enlightening.
*     It basically led me down the road of a time delay PID loop.
*     Experimenting with manual control of output until temp within a certain degrees and then turn on PID
*     Also experimenting with 2 PID sets in coming to the setpoing.
*   - Fixed - Real bug.  Canceling in the middle of a beep leaves the buzzer on
*   - Completed tested with revised parameters with thermocouple on pwb surface.  Deleted D parameters
*     and only used I parameters in middle section.  Pretty close.
*   - MM
*
* V2.6 31 March 2017
*   - Putting in framework for error testing and recovery for preheat stage
*   - Cleanup of defines
*   - Put SIMulator in place
*   - Put tests in place for reflow, soak, and preheat as well as cycle overall tests
*   - Put in place run status framework
*   - Fixed max temp / time bug to only record when reflow is on
*   - Added documentation on additional test cases during runtime, see defines below
*   - Release to MM
*
* V2.7 11 April
*   - Added debug macros and dual PID during preheat - big improvement in results
*   - Changed token handling during cooldown to display target temp
*   - Added debug macros and dual PID during reflow - big improvement in results
*   - Decreased requirement for presoak ramp during first 30 seconds (10C -> 5C)
*
* V2.8 17 April
*   - Added buzzer on cancel
*   - Added EEPROM tracking of usage
*
* V3.0 20 April 2017
*   - Minor doc edits and release to GIT
*
* Still to do and / or think about
*   - Consider programming with defined function rather than programmatic implementation. 
*     This would allow error detection on low ramp rates.  For example, the oven will not error 
*     if there is no heat.
*   - Consider reorganization of code overall to allow for varying length of button press and menuing
*     The 1 sec display loop makes display response feel slow and this is related to loop architecture
*   - Anticipated bugs.  This uses millis() all over the place which has a wraparound time of 55 days
*   - A thermocouple error displays TC Error on the LCD and immediately aborts with no information written to log.
*     Unsure of how to improve this currently
*
*******************************************************************************
*
* Oven Testing - Groezinger - 16 March 2017
*   Hamilton Beach Model 31146 Wal Mart oven
*   Thermocouple with no board, no middle tray in oven, drip tray in bottom
*
*   Testing starting with I and D set to zero
*   P term 50, 100, 200 testing showed an Integral term is needed.
*   At 50, oven would never reach setpoints
*   At 200, oven would hunt
*   In particular, at 50, the oven would miss the 200 soak temp (at 190)
*   and then proceed into reflow (since this is time based)
*
*   Adding I term
*   Honing in on P of 100 and setting I term to 1 and 0.2
*   Observations were significant hunting still and overshoot 
*   It was clear the I term was contributing too much and I felt that 
*   in combo with the P term was driving more hunting than seen before.
*
*   Reduced P term to 50
*   I term of 0.5 showed "riding the peaks" in the soak phase and still
*   lag of peak.  Reduced to 0.1 gave a nice response thru the soak phase
*   with a bit of overshoot and lag.
*
*   P=50, I=0.1, D term testing
*   Decided to address the overshoot with the D term.  This is a minor
*   adjustment and the oven would be fine without it.
*   0.1, 1.0, 10 terms tested with no difference between 0.1 and 1.0.
*   From 1.0 to 10 I observed slightly less overshoot on peak and slightly
*   less lag.  I observed slightly less undershoot on the soak.
*   A run at 50 yielded no dicernable improvement.
*   Again, a minor observation. Settled in on 10
*   
*   Optimal Parameters, KP=50, KI=0.1, KD=10
*
*   At this time, I decided to retest with windowSize of 2000 since I felt
*   things were working and this was the original value.  Bottom Line, 
*   the oven control did not work at all with significant misses on attaining
*   setpoints.  Back to windowSize of 1000.  I am not saying that 2000 is
*   wrong, but I can't figure out how that works and I don't believe it 
*   does since this is time based and everything else is 1 sec.
*
* Oven Testing - Groezinger - 17 March 2017
*
*   As an experiment, I decided to change only the following:
*   #define PID_KI_PREHEAT 0.05
*   Leaving all the rest at 0.1
*   The results were amazing.  The oven way undershot on the soak and reflow stage.
*   Unsure what this means but I suspect the change in parms changed 
*   the algorithm in a way not expected.  Since I have honed in on one setting,
*   I have added a define to stop all but one loading of the parameters.
*   Note on 19 April.  Beleive this was due to bugs found in the way the PID control was done.
*
* Oven Testing - Groezinger 23 March 2017
*
*   With the ALlPRINT serial output, I decided to do some additional testing to 
*   understand the PID library workings.  Indeed, setting the windowsize to 2000
*   and leaving the PID Calculation at 1000 is not a good plan.  The PID does 
*   do a recalc in the middle of a window, the output is invariably reduced,
*   and the result is that you can never attain desired heat.  Changing the PID calc window
*   to 2000 with this would be an alternative.
*
* Oven Testing - Groezinger 30 March 2017
*
*  Constant temp testing
*  10% Duty Cycle is about 100C
*  23% 145C
*  40% 192C
*  49% 240C
*
* Additional information regarding ALLPRINT output (for debugging / verification)
*
*  In the SSR loop you will see a set of prints:
*  SSRON / SSROFF indicates exactly this
*  The loopInteration is just a counter which gets incremented every time through loop()
*  It is most useful simply to tag output and compare to other output
*  output is the current output of the PID 0-1000
*  It also is mS that the SSR is supposed to be on out of a 1 second loop
*  now is the time at the start of the PID loop
*  time is the current time (close to above but not quite the same)
*  lastPIDChange is the last timestampe when the PID changed the output value
*  windowStartTime is the time that the current 1 second output window started  
*
*  So you can check that the SSR turned off at the correct time by taking the windowStartTime and
*  adding the output and comparing this against the SSROFF statement time.
*
* Testing with boards
*
* Profile Selection
*
*   Lead Free
*
*   Preheat stage -> Oven limits ramp to around 2C/sec (max of 2.5C/sec ok)
*                    Curve does not put a limit on this stage
*                    RESULT: 1.8C/sec
*   Preheat->Soak 150
*                    RESULT: ~100 sec (depends on start temp)
*   Soak stage -> PID limits ramp to around 0.5C/sec (max 0.6C sec ok)
*                    RESULT: ~0.6C/sec
*   Soak->Reflow 200
*                    RESULT: ~ 160 sec (total from start)
*   Reflow stage -> Oven limits ramp to around 1C/sec (max of ?/sec ok)
*   Reflow->Cooling 245 (e.g. Peak)
*                    RESULT: Attain peak in 50 sec, (~205 sec) need to open door 
*   Cooling stage -> Oven door closed limits to 0.5C/sec (max of ?/sec ok)
*                    Open door limits to 1.0C/sec (recommended)
* 
*   Kester Sn63Pb37 Recommended Profile (Lead)
*
*   Preheat stage -> Oven limits ramp to around 2C/sec (max of 2.5C/sec ok)
*                    Curve does not put a limit on this stage
*                    RESULT: 1.8C/sec
*   Preheat->Soak 150  Recommended attain in 90sec
*                    RESULT: ~90 sec
*   Soak stage -> PID limits ramp to around 0.5C/sec (max 0.6C sec ok)
*                    RESULT: ~0.6C/sec
*   Soak->Reflow 180 Recommended 2-4 minute from start
*                    RESULT: ~ 150 sec
*   Reflow stage -> Oven limits ramp to around 1C/sec (1.3-1.6C/sec rec)
*                    Recommended in Reflow Zone 45-75 sec (inc cooling)
*                    RESULT: Attain peak in ~35 sec, need to open door 
*   Reflow->Cooling 225 (e.g. Peak)
*   Cooling stage -> Oven door closed limits to 0.5C/sec
*                    Recommended 183C-> <40C over 3 minutes minimum
*                    Slightly Open door limits to 1.0C/sec (recommended)
* 
*******************************************************************************
* 
* Title: Reflow Oven Controller
*
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)

* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
*
* Revision	Description
* ========	===========
* 3.0		Groezinger (see notes elsewhere)
* 1.20		Adds supports for v1.60 (and above) of Reflow Oven Controller 
*			Shield:
*			- Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*			to be used for user application).
*			- Uses analog based switch (allowing D2 & D3 to be used for user 
*			application).	
*			Adds waiting state when temperature too hot to start reflow process.
*			Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10		Arduino IDE 1.0 compatible.
* 1.00		Initial public release.
*******************************************************************************/

// ***** INCLUDES *****
#include <LiquidCrystal.h>
#include <MAX31855.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define MAJOR_VERSION 3
#define MINOR_VERSION 0

//
// **** DEBUG AND OPERATIONAL CONFIG DEFINES ***
//
// Define to not turn on heater
// #define NOHEAT

// Extended print statements to serial, ALLPRINT gives SSR on/off information and PID calculations
// #define ALLPRINT

// ALLPRINT_WIN gives information relating to window frame calculation (a bit too much, generally)
// #define ALLPRINT_WIN

// add constant state option to top level menu if defined
// #define CONSTANT_TEMP
// Review options below for proper configuration of constant state

// Simulator input - Must add one data set to use
// Sims will end either when an error detected, a manual abort is done, or it hits last value which is set to max temp
// #define SIM
// #define SIM_DATA_OK_1C
// #define SIM_DATA_FAIL_30
// #define SIM_DATA_OK_30
// #define SIM_DATA_FAIL_60
// #define SIM_DATA_OK_60
// #define SIM_DATA_FAIL_149
// #define SIM_DATA_OK_150
// #define SIM_DATA_OK_SOAK
// #define SIM_DATA_OK_SOAK_FAST
// #define SIM_DATA_FAIL_SOAK_FAST_SLOW_PEAK
// #define SIM_DATA_FAIL_SOAK_FAST_MISS_END

// EEPROM
// #define EEPROM_DUMP
// #define EEPROM_RESET

//
// ***** TYPE DEFINITIONS *****
//
typedef enum REFLOW_STATE
{
	REFLOW_STATE_IDLE,
	REFLOW_STATE_IDLE_LEAD,
	REFLOW_STATE_IDLE_SUMMARY,
	REFLOW_STATE_PREHEAT,
	REFLOW_STATE_SOAK,
	REFLOW_STATE_REFLOW,
	REFLOW_STATE_REFLOW_2,
	REFLOW_STATE_COOL,
	REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
	REFLOW_STATE_ERROR,
	REFLOW_STATE_IDLE_CONSTANT,
	REFLOW_STATE_RUN_CONSTANT,
	REFLOW_STATE_PREHEAT_ERROR,
	REFLOW_STATE_SOAK_ERROR,
	REFLOW_STATE_REFLOW_ERROR,
	REFLOW_STATE_CYCLE_ERROR,
	REFLOW_STATE_CANCEL
} reflowState_t;

typedef enum REFLOW_STATUS
{
	REFLOW_STATUS_OFF,
	REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH
{
	SWITCH_NONE,
	SWITCH_1,	
	SWITCH_2
}	switch_t;

typedef enum DEBOUNCE_STATE
{
	DEBOUNCE_STATE_IDLE,
	DEBOUNCE_STATE_CHECK,
	DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum SWITCH_LONG_PRESS_FLAG
{
	SWITCH_LONG_PRESS_FLAG_NO,
	SWITCH_LONG_PRESS_FLAG_YES
} switchLongPressFlag_t;

typedef enum PROFILE_LEAD
{
	PROFILE_LEAD_YES,
	PROFILE_LEAD_FREE
} profileLead_t;

typedef enum RUN_COMPLETION_STATUS
{
	RUN_COMPLETION_STATUS_NEVERRUN,
	RUN_COMPLETION_STATUS_INPROCESS,
	RUN_COMPLETION_STATUS_OK,
	RUN_COMPLETION_STATUS_CANCEL,
	RUN_COMPLETION_STATUS_PREHEAT_ERROR,
	RUN_COMPLETION_STATUS_SOAK_ERROR,
	RUN_COMPLETION_STATUS_REFLOW_ERROR,
	RUN_COMPLETION_STATUS_CYCLE_ERROR
} runCompletionStatus_t;

//
// ***** SETPOINT CONSTANTS FOR LEAD AND NONLEAD PROFILES *****
//
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 200
#define TEMPERATURE_SOAK_MAX_LEAD 180
#define TEMPERATURE_REFLOW_MAX 245
#define TEMPERATURE_REFLOW_MAX_LEAD 220
#define TEMPERATURE_COOL_MIN 100

//
// ***** THERMOCOUPLE *****
//
// How often to read the thermocouple, changed to be faster than 
// display or PID loop and actually not in phase at 12.5Hz
#define SENSOR_SAMPLING_TIME 80

// 3 way moving average for input if defined
#define MOVING_AVERAGE

// use 3 way median filter instead - IMPORTANT, MUST define MOVING_AVERAGE to use this!
#define MEDIAN_AVERAGE

//
// ***** SWITCH *****
//
#define DEBOUNCE_PERIOD_MIN 50

// Minimum time in mS to depress switch to start cycle
#define LONG_SWITCH_PRESS 2000

//
// ***** SOUND *****
//
// Number of on/off cycles to chirp (must be even) and mS of time for each
#define BUZZER_CHIRP_NORM_STEPS 6
#define BUZZER_CHIRP_NORM_TIME 50
#define	BUZZER_CHIRP_ERROR_STEPS 10
#define BUZZER_CHIRP_ERROR_TIME 500

//
// ***** PID TUNING PARAMETERS *****
//
// One load of PID parameters (PREHEAT only) if defined across PREHEAT, SOAK, REFLOW states
// #define ONEPIDSET

// If defined, will use the second set of parameters when within the window during preheat
#define PID_NEAR_FAR_PREHEAT
#define PID_NEAR_FAR_TEMP_PREHEAT 20

// If defined, will use the second set of parameters when within the window during reflow
#define PID_NEAR_FAR_REFLOW
#define PID_NEAR_FAR_TEMP_REFLOW 20

// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 50 
#define PID_KI_PREHEAT 0.0
#define PID_KD_PREHEAT 0
// ***** PRE-HEAT STAGE 2 *****
#define PID_KP_PREHEAT_2 25 
#define PID_KI_PREHEAT_2 1.0
#define PID_KD_PREHEAT_2 0
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 100
#define PID_KI_SOAK 2.0
#define PID_KD_SOAK 0
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 50
#define PID_KI_REFLOW 0.0
#define PID_KD_REFLOW 0
// ***** REFLOW STAGE 2 *****
#define PID_KP_REFLOW_2 25 
#define PID_KI_REFLOW_2 0.1
#define PID_KD_REFLOW_2 0

// The temperature increment and how often to change during the soak period
// Also note that this results in 10 steps at 9 seconds apiece or 90 seconds for Pb Free.
// A better approach would be functional driven
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000

// The PID will revaluate every 1 sec
#define PID_SAMPLE_TIME 1000

// The output value of the PID is 0 to windowsize
// Set to 1000 since everything is based on 1 sec and mS units
// In retrospect, PID_SAMPLE_TIME and PID_WINDOW_SIZE should use the same define
#define PID_WINDOW_SIZE 1000

//
// ***** RAMP RATE TESTING AND SAFETY STUFF *****
//
// These are for checking oven functionality and safety stops in the event performance is not good enough
// This is number of seconds from start of run until start of soak.  
// If not met, oven will abort and turn off with Abort PH error
// To defeat, set to a large number
// There is a SIM for this. Another way is to set PreHeat PID parms to Ki=0 and Kp low (10 or something)
// Use SIM_DATA_FAIL_149 SIM_DATA_OK_150
#define PREHEAT_STAGE_MAX_TIME_TO_SOAK 150

// This consists of a ramprate test done at start of run + 30 sec.  Oven temp must increase by the PREHEAT_STAGE_MIN_INCREASE_INIT
// If not met, oven will abort and turn off with Abort PH error
// To defeat, set increase to 0
// Use SIM_DATA_FAIL_30 and SIM_DATA_OK_30 to verify
#define PREHEAT_STAGE_ERROR_CHECK_TIME_INIT 30
#define PREHEAT_STAGE_MIN_INCREASE_INIT 5

// This consists of two more checks done at run + init + 30 and run + init + 2 * 30.
// Oven temp must increase by 30 at each checkpoint
// If not met, oven will abort and turn off with Abort PH error
// To defeat, set increase to 0
// Use SIM_DATA_FAIL_60 and SIM_DATA_OK_60 to verify
#define PREHEAT_STAGE_ERROR_CHECK_TIME 30
#define PREHEAT_STAGE_MIN_INCREASE 30

// This test is maximum time from run start to start of reflow.
// If not met, oven will abort and turn off with Abort SK error
// To defeat, set to a large number
// Please note that this is actually a code error check since the above test for 150 max time + 10 steps at 9 seconds meets the 240 always.
// Use SIM_DATA_OK_SOAK
// For SIM testing, decrease time to 239 or less
#define PREHEAT_STAGE_MAX_TIME_TO_REFLOW 240

// Temp must be within this temp window of SOAK end temp at the end of SOAK
// If not met, oven will abort and turn off with Abort SK error
// To defeat, set to a large number
// Use SIM_DATA_FAIL_SOAK_FAST_MISS_END
#define SOAK_STAGE_TEMP_WITHIN_MAX_ERROR 10

// This is a safety shutoff, we do not want to hit these as the reflow may be screwed up
// Use SLOW_PEAK SIM array and adjust number below to 155
// If not met, oven will abort and turn off with Abort RF error
// To defeat, set to a large number
#define PREHEAT_STAGE_MAX_TIME_TO_PEAK 330

// If the reflow is not ramping up at 3 C / 10 sec, then switch to full on to try and save the run
// Reflow stage switch to bang bang criteria
// To defeat, set MIN increase to 0
// There is a SIM for this. Another way is to set Reflow PID parms to Ki=0 and Kp low (10 or something)
// Use SIM_DATA_FAIL_SOAK_FAST_SLOW_PEAK
#define REFLOW_STAGE_MIN_INCREASE 3
#define REFLOW_STAGE_ERROR_CHECK_TIME 10

// Cycle error
// To test temp, use any SIM input with a 275 in the array for the max temp or use constant temp and decrease the number
// To test max time, recommend reduce the max time to something like 15 and observe with any sim array
// Max temp will shut the oven off with Abort CY error due to over temp.  Time is similar but measured from run start to COMPLETE.
// Please note this currently includes cool down
// To defeat, set to a large number
// Use SIM_DATA_FAIL_SOAK_FAST_MISS_END
#define CYCLE_MAX_TEMP 275
#define CYCLE_MAX_TIME 600

// Please note that there are no tests for maximum ramp rate testing on PreHeat or Reflow assuming the oven limits this per testing
// Please note there are no ramp rate checks in soak because we did a ramp rate test in preheat

//
// ***** CONSTANT TEMP OPTION AND PID PARAMETER SETS *****
//
// The following section only applies if CONSTANT_TEMP is defined
// Primarily meant for oven testing and helping define needed algorithms
// The temp it will run at if the constant state option is enabled
#define TEMPERATURE_CONSTANT 150

// Defeat PID during constant temp until within PID_ON_WHEN_WITHIN degrees
// Please note that the PID appears to have some logic that prevents large sudden
// changes in output when going from manual to auto.  In short, having the oven
// in manual at 100% duty cycle and then flipping to auto when getting close
// did not work too well as the PID would only slowly drop the output down.
// Consequently, I chose to leave the PID on during the entire cycle but change
// the parameters (adding in the integral term) when the temp got close to setpoint.
// #define PID_OFF_DURING_PREHEAT
// #define PID_ON_WHEN_WITHIN 20

// if PID_OFF_DURING_PREHEAT is defined, this is the duty cycle used during that time
// #define CONSTANT_OUTPUT_1 1000

// If defined, enable 2 phase PID during constant temp
// Intended use is for oven testing to determine if changing the PID parameters makes a difference
// If not defined, only 1 phase PID during constant temp (FAR parameters)
#define PID_NEAR_FAR

// these are used for constant temp operation for oven testing.
// If the temp is greater than PID_NEAR_FAR_TEMP away from setpoint, the FAR parms are used.
#define PID_KP_CONSTANT_FAR 50 
#define PID_KI_CONSTANT_FAR 0.0
#define PID_KD_CONSTANT_FAR 0
#define PID_KP_CONSTANT_NEAR 25
#define PID_KI_CONSTANT_NEAR 0.1
#define PID_KD_CONSTANT_NEAR 0

#define PID_NEAR_FAR_TEMP 20

//
// ***** DISPLAY *****
//
// Amount of time to display messages in ms
// such as the signon message and messages in the summary screen
#define DISPLAY_DELAY 2000

// how much must temp drop below max before cueing starts
#define SLOWFAST_TEMP_WINDOW 3

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
	"OK noPb",
	"OK Pb",
	"Summary",
	"Pre-heat",
	"Soak",
	"Reflow",
	"RF2",
	"Cool",
	"Complete",
	"Wait,hot",
	"Error",
	"OK Const",
	"Constant",
	"Abort PH",
	"Abort SK",
	"Abort RF",
	"Abort CY",
	"Cancel"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
	int ssrPin = 5;
	int thermocoupleSOPin = A3;
	int thermocoupleCSPin = A2;
	int thermocoupleCLKPin = A1;
	int lcdRsPin = 7;
	int lcdEPin = 8;
	int lcdD4Pin = 9;
	int lcdD5Pin = 10;
	int lcdD6Pin = 11;
	int lcdD7Pin = 12;
	int ledRedPin = 4;
	int buzzerPin = 6;
	int switchPin = A0;

//
// ***** PID CONTROL VARIABLES *****
//
// Current desired temperature by the PID loop
double setpoint;

// Current temp measured (it is input to the PID loop)
// and previous temp for ramp rate calculation printed out in the serial output
double input;
double lastinput;

// Sliding temp record for deciding if to put into bang bang
double bangbangInput;
unsigned long bangbangTime;

// Moving average values for last 3 temps, a pointer for current one, and a working location
double input0;
double input1;
double input2;
int inputPtr;
double inputtmp;

// Output of PID control loop (subject to windowSize)
// and lastoutput is prior used to determine if a new value has been calculated.  
// Note: I learned that the PIDCompute function does return a value indicating a new output
// If correct, there is no need for lastoutput
double output;
double lastoutput;

// PID cooefficients
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;

// Input to the PID and it is now hardwired to 1 seconds
int windowSize;

// windowStartTime is used to determine when to adjust the output to the SSR frame
unsigned long windowStartTime;

// Records last time we had a PID output change
// Doesn't do anything except provide information in the print loop for when the current PID changes
// This should always be 1 sec increments
// This was useful to debug issues regarding time frame synch between the sliding window and the PID
long lastPIDChange;

//
// ***** DISPLAY *****
//
// nextCheck used to determine if to update LCD display
// Hardwired to 1 sec updates
unsigned long nextCheck;

// Temperature cue for cooling
double slowFastTemp;

// nextCheck used to determine if to read thermocouple
unsigned long nextRead;

// timerSoak used to determine if the current microPeriod has expired
// If it has, then the temp will be incremented
unsigned long timerSoak;

// Seconds timer
// Is used only for displaying time to serial port
int timerSeconds;

//
// ***** SOUND *****
//
// buzzerPeriod used to determine if time to turn off the buzzer
unsigned long buzzerPeriod;

// Number of counts to chirp buzzer for door open, must be even
int buzzerChirp;

//
// ***** OVEN STATE MACHINE *****
//
// Reflow oven controller state machine state variable
reflowState_t reflowState;

// Reflow oven controller status
reflowStatus_t reflowStatus;

//
// ***** SWITCH *****
//
// Switch debounce state machine state variable
debounceState_t debounceState;

// Switch debounce timer
// variable set to current time when switch first detected depressed
long lastDebounceTime;

// Switch press status
switch_t switchStatus;

// Switch Long Press Detection timer in mS, starts at zero
long switchLongPressTimer;

// Switch Long Press Flag
switchLongPressFlag_t switchLongPressFlag;

//
// ***** PROFILE AND METRICS *****
//
// Leaded Profile Flag
profileLead_t profileLead;

// Metrics on Last Run
long runStartTime;
double runStartTemp;
long runPreHeatEndTime;
double runPreHeatEndTemp;
long runSoakEndTime;
double runSoakEndTemp;
long runPeakTime;
double runPeakTemp;
long runReflowEndTime;
long runEndTime;
double runEndTemp;

// indicates if last run was successful or type of error
runCompletionStatus_t runCompletionStatus;

// Loop counter to simply count iterations of loop for debug purposes
long loopIteration;

// EEPROM interface
// In future rev, I will get more clever and put in place real structures in the EEPROM
// But for now, this is very simple.  Everything is in words
// Byte 0,1 - Key - 0xFFFF, never programmed, 0xAA55 programmed once
// Byte 2,3 - Version of structure same as version of code 
// Byte 4,5 - Number of cycles attempted (word)
// Byte 6,7 - Number of cycles completed (word)
// ...and so on

// The following is V2.8 format for EEPROM
#define EEPROM_KEY 0
#define EEPROM_VERSION 2
#define EEPROM_CYCLES_ATTEMPTED 4
#define EEPROM_CYCLES_COMPLETED 6
#define EEPROM_CYCLES_CANCELLED 8
#define EEPROM_CYCLES_ABORTEDPH 10
#define EEPROM_CYCLES_ABORTEDSK 12
#define EEPROM_CYCLES_ABORTEDRF 14
#define EEPROM_CYCLES_ABORTEDCY 16
#define EEPROM_RESERVED 18

// Length of pseudo structure
#define EEPROM_LENGTH 20 

// access variables
int address = 0;
word value;
word tmpvalue;


// ***** SIMULATOR *****

#ifdef SIM_DATA_OK_1C
// incremented 1C / sec model
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	60,61,62,63,64,65,66,67,68,69,
	70,71,72,73,74,75,76,77,78,79,
	80,81,82,83,84,85,86,87,88,89,
	90,91,92,93,94,95,96,97,98,99,
	100,101,102,103,104,105,106,107,108,109,
	110,111,112,113,114,115,116,117,118,119,
	120,121,122,123,124,125,126,127,128,129,
	130,131,132,133,134,135,136,137,138,139,
	140,141,142,143,144,145,146,147,148,149,
	150,151,152,153,154,155,156,157,158,159,
	160,161,162,163,164,165,166,167,168,169,
	170,171,172,173,174,175,176,177,178,179,
	180,181,182,183,184,185,186,187,188,189,
	190,191,192,193,194,195,196,197,198,199,
	200,201,202,203,204,205,206,207,208,209,
	210,211,212,213,214,215,216,217,218,219,
	220,221,222,223,224,225,226,227,228,229,
	230,231,232,233,234,235,236,237,238,239,
	240,241,242,243,244,245,246,247,248,249,
	250,251,252,253,254,255,256,257,258,259,
	260,261,262,263,264,265,266,267,268,269,
	275 };
#endif

#ifdef SIM_DATA_FAIL_30
// halted at 9 up increase for 30 seconds
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	29,29,29,29,29,29,29,29,29,29,
	29,29,29,29,29,29,29,29,29,29,
	30,31,32,33,34,35,36,37,38,39,
	275 };
#endif

#ifdef SIM_DATA_OK_30
// halted at 10 up increase for 30 seconds
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	29,29,29,29,29,29,29,29,29,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,30,30,30,30,30,30,30,30,30,
	30,31,32,33,34,35,36,37,38,39,
	275 };
#endif

#ifdef SIM_DATA_FAIL_60
// halted at 39 up increase for 60 seconds barely failing
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	29,29,29,29,29,29,29,29,29,30,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	59,59,59,59,59,59,59,59,59,59,
	59,59,59,59,59,59,59,59,59,59,
	60,61,62,63,64,65,66,67,68,69,
	275 };
#endif

#ifdef SIM_DATA_OK_60
// halted at 40 up increase for 60 seconds barely passing
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	29,29,29,29,29,29,29,29,29,30,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,60,
	60,60,60,60,60,60,60,60,60,60,
	60,60,60,60,60,60,60,60,60,60,
	60,60,60,60,60,60,60,60,60,60,
	60,60,60,60,60,60,60,60,60,60,
	60,60,60,60,60,60,60,60,60,60,
	60,61,62,63,64,65,66,67,68,69,
	275 };
#endif

#ifdef SIM_DATA_FAIL_149
// incremented 1C / sec model halting at 149 just barely failing move into soak
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	60,61,62,63,64,65,66,67,68,69,
	70,71,72,73,74,75,76,77,78,79,
	80,81,82,83,84,85,86,87,88,89,
	90,91,92,93,94,95,96,97,98,99,
	100,101,102,103,104,105,106,107,108,109,
	110,111,112,113,114,115,116,117,118,119,
	120,121,122,123,124,125,126,127,128,129,
	130,131,132,133,134,135,136,137,138,139,
	140,141,142,143,144,145,146,147,148,149,
	149,149,149,149,149,149,149,149,149,149,
	150,149,149,149,149,149,149,149,149,149,
	149,149,149,149,149,149,149,149,149,149,
	149,149,149,149,149,149,149,149,149,149,
	149,149,149,149,149,149,149,149,149,149,
	149,149,149,149,149,149,149,149,149,149,
	149,149,149,149,149,149,149,149,149,149,
	275 };
#endif

#ifdef SIM_DATA_OK_150
// incremented 1C / sec model barely passing into soak
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	60,61,62,63,64,65,66,67,68,69,
	70,71,72,73,74,75,76,77,78,79,
	80,81,82,83,84,85,86,87,88,89,
	90,91,92,93,94,95,96,97,98,99,
	100,101,102,103,104,105,106,107,108,109,
	110,111,112,113,114,115,116,117,118,119,
	120,121,122,123,124,125,126,127,128,129,
	130,131,132,133,134,135,136,137,138,139,
	140,141,142,143,144,145,146,147,148,149,
	149,149,149,149,149,149,149,149,149,150,
	150,150,150,150,150,150,150,150,150,150,
	150,150,150,150,150,150,150,150,150,150,
	150,150,150,150,150,150,150,150,150,150,
	150,150,150,150,150,150,150,150,150,150,
	275 };
#endif

#ifdef SIM_DATA_OK_SOAK
// incremented 1C / sec model barely passing into soak and then incrementing
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	60,61,62,63,64,65,66,67,68,69,
	70,71,72,73,74,75,76,77,78,79,
	80,81,82,83,84,85,86,87,88,89,
	90,91,92,93,94,95,96,97,98,99,
	100,101,102,103,104,105,106,107,108,109,
	110,111,112,113,114,115,116,117,118,119,
	120,121,122,123,124,125,126,127,128,129,
	130,131,132,133,134,135,136,137,138,139,
	140,141,142,143,144,145,146,147,148,149,
	149,149,149,149,149,149,149,149,149,150,
	150,150,150,151,152,153,154,155,155,155,
	155,156,157,158,159,160,160,160,160,160,
	161,162,163,164,165,165,165,165,165,166,
	167,168,169,170,170,170,170,170,171,172,
	173,174,175,175,175,175,175,176,177,178,
	179,180,180,180,180,180,181,182,183,184,
	185,185,185,185,185,185,186,187,188,189,
	190,190,190,190,191,192,193,194,195,195,
	195,195,195,196,197,198,199,200,200,200,
	200,201,202,203,204,205,206,207,208,209,
	210,211,212,213,214,215,216,217,218,219,
	220,221,222,223,224,225,226,227,228,229,
	230,231,232,233,234,235,236,237,238,239,
	240,241,242,243,244,245,246,247,248,249,
	250,251,252,253,254,255,256,257,258,259,
	260,261,262,263,264,265,266,267,268,269,
	275 };
#endif

#ifdef SIM_DATA_OK_SOAK_FAST
// incremented 1C / sec model barely passing into soak and then incrementing
// same as above but faster execution
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
//	20,21,22,23,24,25,26,27,28,29,
//	30,31,32,33,34,35,36,37,38,39,
//	40,41,42,43,44,45,46,47,48,49,
//	50,51,52,53,54,55,56,57,58,59,
//	60,61,62,63,64,65,66,67,68,69,
//	70,71,72,73,74,75,76,77,78,79,
//	80,81,82,83,84,85,86,87,88,89,
//	90,91,92,93,94,95,96,97,98,99,
//	100,101,102,103,104,105,106,107,108,109,
//	110,111,112,113,114,115,116,117,118,119,
//	120,121,122,123,124,125,126,127,128,129,
//	130,131,132,133,134,135,136,137,138,139,
//	140,141,142,143,144,145,146,147,148,149,
//	149,149,149,149,149,149,149,149,149,150,
	150,150,150,151,152,153,154,155,155,155,
	155,156,157,158,159,160,160,160,160,160,
	161,162,163,164,165,165,165,165,165,166,
	167,168,169,170,170,170,170,170,171,172,
	173,174,175,175,175,175,175,176,177,178,
	179,180,180,180,180,180,181,182,183,184,
	185,185,185,185,185,185,186,187,188,189,
	190,190,190,190,191,192,193,194,195,195,
	195,195,195,196,197,198,199,200,200,200,
	200,201,202,203,204,205,206,207,208,209,
//	210,211,212,213,214,215,216,217,218,219,
//	220,221,222,223,224,225,226,227,228,229,
//	230,231,232,233,234,235,236,237,238,239,
	240,241,242,243,244,245,246,247,248,249,
	250,251,252,253,254,255,256,257,258,259,
	260,261,262,263,264,265,266,267,268,269,
	275 };
#endif

#ifdef SIM_DATA_FAIL_SOAK_FAST_SLOW_PEAK
// incremented 1C / sec model barely passing into soak and then missing
// the peak so bang bang needs to kick in
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	150,150,150,151,152,153,154,155,155,155,
	155,156,157,158,159,160,160,160,160,160,
	161,162,163,164,165,165,165,165,165,166,
	167,168,169,170,170,170,170,170,171,172,
	173,174,175,175,175,175,175,176,177,178,
	179,180,180,180,180,180,181,182,183,184,
	185,185,185,185,185,185,186,187,188,189,
	190,190,190,190,191,192,193,194,195,195,
	195,195,195,196,197,198,199,200,200,200,
	200,201,202,203,204,205,206,207,208,209,
	210,211,212,213,214,215,216,217,218,219,
	220,221,222,223,224,225,226,227,228,229,
	230,231,232,233,234,235,236,237,238,239,
	241,241,241,241,241,241,241,241,241,241,
	241,241,241,241,241,241,241,241,241,241,
//	240,241,242,243,244,245,246,247,248,249,
	250,251,252,253,254,255,256,257,258,259,
	260,261,262,263,264,265,266,267,268,269,
	275 };
#endif


#ifdef SIM_DATA_FAIL_SOAK_FAST_MISS_END
// incremented 1C / sec model barely passing into soak but missing end of soak temp test
unsigned int simInput[]  =
	{20,20,20,20,20,20,20,20,20,20,
	150,150,150,151,152,153,154,155,155,155,
	155,156,157,158,159,160,160,160,160,160,
	161,162,163,164,165,165,165,165,165,166,
	167,168,169,170,170,170,170,170,171,172,
	173,174,175,175,175,175,175,176,177,178,
	179,180,180,180,180,180,181,182,183,184,
	185,185,185,185,185,185,186,187,188,189,
	190,190,190,190,191,192,193,194,195,195,
	195,195,195,196,197,198,199,200,200,189,
	189,189,189,189,189,189,189,189,189,189,
	210,211,212,213,214,215,216,217,218,219,
	220,221,222,223,224,225,226,227,228,229,
	230,231,232,233,234,235,236,237,238,239,
	241,241,241,241,241,241,241,241,241,241,
	241,241,241,241,241,241,241,241,241,241,
//	240,241,242,243,244,245,246,247,248,249,
	250,251,252,253,254,255,256,257,258,259,
	260,261,262,263,264,265,266,267,268,269,
	275 };
#endif


//
// ***** API *****
//
// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify LCD interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// Specify thermocouple interface
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, thermocoupleCLKPin);


//
// ******** MACROS ********
//
// Macros defined for debug 

#ifdef ALLPRINT
#define DEBUG_PRINT(string) \
	Serial.print(string); \
	Serial.print("Loop="); \
	Serial.print(loopIteration); \
	Serial.print(",time="); \
	Serial.println(millis());
#else
#define DEBUG_PRINT(string)
#endif

#ifdef ALLPRINT
#define DEBUG_PRINT_PID(string) \
	Serial.print(string); \
	Serial.print("Loop="); \
	Serial.print(loopIteration); \
	Serial.print(",time="); \
	Serial.print(millis()); \
	Serial.print(",output="); \
	Serial.print(output); \
	Serial.print(",Kp="); \
	Serial.print(reflowOvenPID.GetKp()); \
	Serial.print(",Ki="); \
	Serial.print(reflowOvenPID.GetKi()); \
	Serial.print(",Kd="); \
	Serial.print(reflowOvenPID.GetKd()); \
	Serial.print(",Mode="); \
	Serial.println(reflowOvenPID.GetMode()); 
#else
#define DEBUG_PRINT_PID(string)
#endif

#ifdef ALLPRINT_WIN
#define DEBUG_PRINTWIN(string) \
	Serial.print(string); \
	Serial.print("Loop="); \
	Serial.print(loopIteration); \
	Serial.print(",time="); \
	Serial.println(millis());
#else
#define DEBUG_PRINTWIN(string)
#endif


//
// ********************************** FUNCTIONS ***********************************
//

// improvements go here

//
// *********************************** RESET **************************************
//
void setup()
{
	// SSR pin initialization to ensure reflow oven is off
	digitalWrite(ssrPin, LOW);
	pinMode(ssrPin, OUTPUT);

	// Buzzer pin initialization to ensure annoying buzzer is off
	digitalWrite(buzzerPin, LOW);
	pinMode(buzzerPin, OUTPUT);

	// LED pins initialization and turn on upon start-up (active low)
	digitalWrite(ledRedPin, LOW);
	pinMode(ledRedPin, OUTPUT);

	// Start-up splash
	digitalWrite(buzzerPin, HIGH);
	lcd.begin(8, 2);
	lcd.createChar(0, degree);
	lcd.clear();
	lcd.print("Reflow");
	lcd.setCursor(0, 1);
	lcd.print("Oven ");
	lcd.print(MAJOR_VERSION);
	lcd.print(".");
	lcd.print(MINOR_VERSION);
	digitalWrite(buzzerPin, LOW);
	delay(DISPLAY_DELAY);
	lcd.clear();

	// Serial communication
	Serial.begin(115200);

	// Signon
	Serial.print("The MG Reflow Oven ");
	Serial.print(MAJOR_VERSION);
	Serial.print(".");
	Serial.print(MINOR_VERSION);
	Serial.println(" " __DATE__ " " __TIME__);
 
#ifdef NOHEAT
	Serial.println("Heat turned off");
#endif

#ifdef SIM
	Serial.println("Simulator turned on");
#endif

#ifdef ALLPRINT
	Serial.println("PID/SSR DEBUG turned on");
#endif

#ifdef ALLPRINT_WIN
	Serial.println("Window DEBUG turned on");
#endif

#ifdef EEPROM_RESET
	Serial.println("Reset EEPROM");
	value = 0xffff;
	EEPROM.put(EEPROM_KEY,value);
#endif

	// Check if EEPROM has ever been used
	EEPROM.get(EEPROM_KEY,value);
	if (value != 0xaa55)
	{
		// setup defaults for the current code version in EEPROM 
		value = 0xaa55;
		EEPROM.put(EEPROM_KEY,value);
		value = MAJOR_VERSION * 256 + MINOR_VERSION;
		EEPROM.put(EEPROM_VERSION,value);
		value = 0;
		EEPROM.put(EEPROM_CYCLES_ATTEMPTED,value);
		EEPROM.put(EEPROM_CYCLES_COMPLETED,value);
		EEPROM.put(EEPROM_CYCLES_CANCELLED,value);
		EEPROM.put(EEPROM_CYCLES_ABORTEDPH,value);
		EEPROM.put(EEPROM_CYCLES_ABORTEDSK,value);
		EEPROM.put(EEPROM_CYCLES_ABORTEDRF,value);
		EEPROM.put(EEPROM_CYCLES_ABORTEDCY,value);
		EEPROM.put(EEPROM_RESERVED,value);
	} else {
		EEPROM.get(EEPROM_VERSION,value);
		switch (value)
		{
		case 0x0001:
			// initial testing EEPROM format
			// don't touch the first two entries, init the rest
			value = MAJOR_VERSION * 256 + MINOR_VERSION;
			EEPROM.put(EEPROM_VERSION,value);
			value = 0;
			EEPROM.put(EEPROM_CYCLES_CANCELLED,value);
			EEPROM.put(EEPROM_CYCLES_ABORTEDPH,value);
			EEPROM.put(EEPROM_CYCLES_ABORTEDSK,value);
			EEPROM.put(EEPROM_CYCLES_ABORTEDRF,value);
			EEPROM.put(EEPROM_RESERVED,value);

			// put the discrepancy in the cycle counter
			EEPROM.get(EEPROM_CYCLES_ATTEMPTED,value);
			EEPROM.get(EEPROM_CYCLES_COMPLETED,tmpvalue);
			value = value - tmpvalue;
			EEPROM.put(EEPROM_CYCLES_ABORTEDCY,value);
			break;

		case 0x0208:
			// V2.8 code is here, do nothing other than update version as this is current format
			value = MAJOR_VERSION * 256 + MINOR_VERSION;
			EEPROM.put(EEPROM_VERSION,value);
			break;

		case 0x0300:
			// V3.0 code is here, do nothing as this is current revision
			break;

		default:
			break;
		}
	}

	// Dump EEPROM
#ifdef EEPROM_DUMP
	address = 0;
	while (address < EEPROM_LENGTH)
	{
		EEPROM.get(address,value);
		Serial.print(address,HEX);
		Serial.print(",");
		Serial.println(value,HEX);
		address = address + 2;
	}
#endif

/* Oven status moved to summary for now
	// Oven Stats
	Serial.print ("Reflow Attempts=");
	EEPROM.get(EEPROM_CYCLES_ATTEMPTED,value);
	Serial.println (value);

	Serial.print ("Reflow Success=");
	EEPROM.get(EEPROM_CYCLES_COMPLETED,value);
	Serial.println (value);

	Serial.print ("Cancelled=");
	EEPROM.get(EEPROM_CYCLES_CANCELLED,value);
	Serial.println (value);

	Serial.print ("Preheat Aborts=");
	EEPROM.get(EEPROM_CYCLES_ABORTEDPH,value);
	Serial.println (value);

	Serial.print ("Soak Aborts=");
	EEPROM.get(EEPROM_CYCLES_ABORTEDSK,value);
	Serial.println (value);

	Serial.print ("Reflow Aborts=");
	EEPROM.get(EEPROM_CYCLES_ABORTEDRF,value);
	Serial.println (value);

	Serial.print ("Cycles Aborted=");
	EEPROM.get(EEPROM_CYCLES_ABORTEDCY,value);
	Serial.println (value);
*/

	// Turn off LED (active low)
	digitalWrite(ledRedPin, HIGH);

	// Set window size
	windowSize = PID_WINDOW_SIZE;

	// Initialize time keeping variable
	// Please note this is used to determine when to print to the LCD and when to print serial
	nextCheck = millis();

	// Initialize thermocouple reading variable
	nextRead = millis();

	// loop counter
	loopIteration = 0;

	// long press switch flag reset
	switchLongPressFlag = SWITCH_LONG_PRESS_FLAG_NO;

	// Lead Free Profile default
	profileLead = PROFILE_LEAD_FREE;

	// Added this init which was not present, worked without it
	reflowState = REFLOW_STATE_IDLE;

	// Added the run status
	runCompletionStatus = RUN_COMPLETION_STATUS_NEVERRUN;

	// zero out moving average input variables
	input0 = input1 = input2 = 0;
	inputPtr = 0;

	// state printout
	DEBUG_PRINT("ChangeToIdle,");
}

//
// *********************************** MAIN **************************************
//
void loop()
{
	// Current time for the PID loop only
	unsigned long now;

	//
	// ****************** Read thermocouple? *********************
	//
	if (millis() >= nextRead)
	{
		// Read thermocouple next sampling period
		nextRead += SENSOR_SAMPLING_TIME;

		// Read current temperature
		inputtmp = thermocouple.readThermocouple(CELSIUS);
		
		// If thermocouple problem detected
		if((inputtmp == FAULT_OPEN) || (inputtmp == FAULT_SHORT_GND) || (inputtmp == FAULT_SHORT_VCC))
		{
			// Illegal operation
			reflowState = REFLOW_STATE_ERROR;
			reflowStatus = REFLOW_STATUS_OFF;
		}

		// no moving average
#ifndef MOVING_AVERAGE
		input = inputtmp;
#endif

		// moving average post process
#ifdef MOVING_AVERAGE
		// check for initial state and set all to the same
		if (!input0) {
			input0 = input1 = input2 = inputtmp;
		}
		// yeah, yeah, I know about arrays, don't go there, I'll get that next time I edit
		if (inputPtr == 0)
		{
			input0 = inputtmp;
			inputPtr++;
		} else
		if (inputPtr == 1)
		{
			input1 = inputtmp;
			inputPtr++;
		} else
		{
			input2 = inputtmp;
			inputPtr = 0;
		}
		input = (input0 + input1 + input2) / 3.0;
#endif

#ifdef MEDIAN_AVERAGE
		if ((input0 <= input1) && (input0 <= input2))
		{
			input = (input1 <= input2) ? input1 : input2;
		} else
		if ((input1 <= input0) && (input1 <= input2))
		{
			input = (input0 <= input2) ? input0 : input2;
		} else {
			input = (input0 <= input1) ? input0 : input1;
		}
#endif

#ifdef SIM
		// please note this is a little squirrely because the input loop is running async to the display loop
		input = simInput[timerSeconds];
#endif
	}

	//
	// *********************** Misc Metrics *********************
	//
	// count iterations thru loop for debug
	loopIteration++;

	// Metrics (keep track of peak temp and time), reset as necessary when starting the run
	// Please note that this keeps running into REFLOW_2 (IMPROVE?)
	if ((input > runPeakTemp) && (reflowStatus == REFLOW_STATUS_ON))
	{
		runPeakTime = millis();
		runPeakTemp = input;
	}

	//
	// *********************** Cycle Safety Checks *********************
	//
	// Check for maximum temp and for maximum time while running a cycle
	// Please note the time max test maybe should be tweaked to not include the REFLOW2 or COOL portion of cycle (IMPROVE?)
	if (((reflowStatus == REFLOW_STATUS_ON) && (input >= CYCLE_MAX_TEMP)) ||
	((reflowStatus == REFLOW_STATUS_ON) && (millis() > (runStartTime + (CYCLE_MAX_TIME * 1000.0)))))
	{
		// Proceed to error state handler
		reflowState = REFLOW_STATE_CYCLE_ERROR; 
		reflowStatus = REFLOW_STATUS_OFF;

		// Change to show error on the completion
		runCompletionStatus = RUN_COMPLETION_STATUS_CYCLE_ERROR;

		// setup the buzzerChirp
		buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
		buzzerPeriod = millis();

		// Turn the PID off but this holds output at last setting. It does not zero it
		reflowOvenPID.SetMode(MANUAL);
		output = 0;
		DEBUG_PRINT_PID("PIDModeChange,");

		// Metrics
		runEndTime = millis();
		runEndTemp = input;

		// state printout
		DEBUG_PRINT("ChangeToAbortCycle,")

		// save failure to EEPROM
		EEPROM.get(EEPROM_CYCLES_ABORTEDCY,value);
		value++;
		EEPROM.put(EEPROM_CYCLES_ABORTEDCY,value);
	}

	//
	// *********************** Display and serial heartbeat output *********************
	//
	if (millis() >= nextCheck)
	{
		// Check if we should output information
		// hardwired to 1 second
		nextCheck += 1000;

		// If reflow process is on, printout data to serial port
		if (reflowStatus == REFLOW_STATUS_ON)
		{
			// Toggle red LED as system heart beat
			digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));

			// Send temperature and time stamp to serial 
			Serial.print(lcdMessagesReflowStatus[reflowState]);
			Serial.print(",");
			Serial.print(timerSeconds);
			Serial.print(",");
			Serial.print(setpoint);
			Serial.print(",");
			Serial.print(input);
			Serial.print(",");
			Serial.print(input-lastinput);
			Serial.print(",");
			Serial.println(output / 10.);

			DEBUG_PRINTWIN("Heartbeat,");

			// Increment seconds timer serial output
			timerSeconds++;

		} else {
			// Turn off red LED
			digitalWrite(ledRedPin, HIGH);
		}

		// Print current system state on top line
		lcd.clear();
		lcd.print(lcdMessagesReflowStatus[reflowState]);

		// Check to see if we are in a mode that the door may be opened for cooling
		// If so, display a temp on top right side of display to cue for cooling rate
		if ((reflowState == REFLOW_STATE_REFLOW_2) && (input <= runPeakTemp - SLOWFAST_TEMP_WINDOW))
		{
			// first time?
			if (!slowFastTemp) 
			{
				slowFastTemp = input;
			}

			// Put cue on end of top line during cooling / reflow and print temp
			lcd.setCursor(5, 0);
			lcd.print(slowFastTemp,0);

			if (((slowFastTemp > TEMPERATURE_SOAK_MAX) && (profileLead == PROFILE_LEAD_FREE)) || 
			((slowFastTemp > TEMPERATURE_SOAK_MAX_LEAD) && (profileLead == PROFILE_LEAD_YES)))
			{
				// target temp is hardwired to 1C / sec decrease - not very flexible implementation (IMPROVE?)
				slowFastTemp--;
			}
		}

		// Save away temp for ramprate calculation
		lastinput = input;

		// Move the cursor to the second line
		lcd.setCursor(0, 1);

		// Decide what to put on the second line
		// If currently in error state
		if (reflowState == REFLOW_STATE_ERROR)
		{
			// No thermocouple wire connected
			lcd.print("TC Error!");
		} else {
			if (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_YES)
			{
				// ready for execution of command
				lcd.print("Ready!");
			} else {
				// Print current temperature on bottom line
				lcd.print(input);

				// Print degree Celsius symbol
				lcd.write((uint8_t)0);
				lcd.print("C ");
			}
		}
	}

	//
	// ******************* Reflow oven controller state machine ******************
	//
	switch (reflowState)
	{

	//
	// ************ Oven Idling State ********************
	//
	case REFLOW_STATE_IDLE:
	case REFLOW_STATE_IDLE_LEAD:
		// If oven temperature is still above room temperature
 		if (input >= TEMPERATURE_ROOM)
		{
			reflowState = REFLOW_STATE_TOO_HOT;
			break;
		} 

		// Check to see if profile switch or summary view
		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_NO))
		{
			if (reflowState == REFLOW_STATE_IDLE)
			{
				reflowState = REFLOW_STATE_IDLE_LEAD;
				profileLead = PROFILE_LEAD_YES;
			} else {
				reflowState = REFLOW_STATE_IDLE_SUMMARY;
			}
			break;
		}

		//
		// ************ Reflow starts here ********************
		//
		// If switch is long pressed, start reflow process
		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_YES))
		{
			// Print Run profile information
			Serial.print("Profile=");
			if (reflowState == REFLOW_STATE_IDLE_LEAD)
			{
				Serial.println("Lead");
				Serial.print("SoakTempMin=");
				Serial.print(TEMPERATURE_SOAK_MIN);
				Serial.print(",SoakTempMax=");
				Serial.print(TEMPERATURE_SOAK_MAX_LEAD);
				Serial.print(",ReflowMaxTemp=");
				Serial.println(TEMPERATURE_REFLOW_MAX_LEAD);
			} else {
				Serial.println("LeadFree");
				Serial.print("SoakTempMin=");
				Serial.print(TEMPERATURE_SOAK_MIN);
				Serial.print(",SoakTempMax=");
				Serial.print(TEMPERATURE_SOAK_MAX);
				Serial.print(",ReflowMaxTemp=");
				Serial.println(TEMPERATURE_REFLOW_MAX);
			}

			// Send header for CSV file
			Serial.println("State,Time,Setpoint,Temperature,Ramp,DutyCycle");

			// Intialize seconds timer for serial output timer
			timerSeconds = 0;

			// Init the last temp for ramp rate calculation
			lastinput = input;

			// Ramp up to minimum soaking temperature
			setpoint = TEMPERATURE_SOAK_MIN;

			// Tell the PID to range between 0 and the full window size
			reflowOvenPID.SetOutputLimits(0, windowSize);
			reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);

			// Set PID parameters for Pre-Heat ramp here and turn it on
			reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);
			reflowOvenPID.SetMode(AUTOMATIC);
			DEBUG_PRINT_PID("PIDModeChange,");

			// PID started
			lastPIDChange = 0;

			// Proceed to preheat stage
			reflowState = REFLOW_STATE_PREHEAT;

			// Note the REFLOW_STATUS_ON is set in the next case statement
			// There is a logic thing here.  If I uncomment the next line (not the original code)
			// then the oven thinks the abort button is pressed.
			// reflowStatus = REFLOW_STATUS_ON;

			// Start metrics
			runStartTime = millis();
			runStartTemp = input;

			// reset the display timer for output of data to serial and LCD
			nextCheck = millis();

			// and init the rest to zero
			runPreHeatEndTime = runSoakEndTime = runPeakTime = 0;
			runReflowEndTime = runEndTime = 0;
			runSoakEndTemp = runPreHeatEndTemp = runPeakTemp = runEndTemp = 0;

			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_NORM_STEPS;
			buzzerPeriod = millis();

			// init the slowFastTemp
			slowFastTemp = 0;

			// state printout
			DEBUG_PRINT("ChangeToPreHeat,");

			// Initialize PID control window starting time after all other work
			// to minimize time difference between PID loop and frame loop
			windowStartTime = 0;

			// and indicate that oven run in process
			// consider a handler for this case (IMPROVE?)
			runCompletionStatus = RUN_COMPLETION_STATUS_INPROCESS;

			// save attempt to EEPROM
			EEPROM.get(EEPROM_CYCLES_ATTEMPTED,value);
			value++;
			EEPROM.put(EEPROM_CYCLES_ATTEMPTED,value);
		}
		break;

	case REFLOW_STATE_PREHEAT:
		// Need to keep this status here so the switch gets cleared out before rerunning, otherwise you get an abort
		reflowStatus = REFLOW_STATUS_ON;

		// chirp the buzzer upon start of the oven, Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_NORM_TIME;
			buzzerChirp--;
		}

		// Error checking on PreHeat Stage for maximum time and check for ramp runup at 3 spots
		if (millis() > (runStartTime + (PREHEAT_STAGE_MAX_TIME_TO_SOAK * 1000.0)) ||
		(millis() >= (runStartTime + (PREHEAT_STAGE_ERROR_CHECK_TIME_INIT * 1000.0)) && (input < (runStartTemp + PREHEAT_STAGE_MIN_INCREASE_INIT))) ||
		(millis() >= (runStartTime + ((PREHEAT_STAGE_ERROR_CHECK_TIME_INIT + PREHEAT_STAGE_ERROR_CHECK_TIME) * 1000.0)) && 
		(input < (runStartTemp + PREHEAT_STAGE_MIN_INCREASE_INIT + PREHEAT_STAGE_MIN_INCREASE))) ||
		(millis() >= (runStartTime + ((PREHEAT_STAGE_ERROR_CHECK_TIME_INIT + (2.0 * PREHEAT_STAGE_ERROR_CHECK_TIME)) * 1000.0)) && 
		(input < (runStartTemp + PREHEAT_STAGE_MIN_INCREASE_INIT + (2.0 * PREHEAT_STAGE_MIN_INCREASE))))) 
		{
			// Proceed to error state handler
			reflowState = REFLOW_STATE_PREHEAT_ERROR; 
			reflowStatus = REFLOW_STATUS_OFF;

			// Change to show error on the completion
			runCompletionStatus = RUN_COMPLETION_STATUS_PREHEAT_ERROR;

			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
			buzzerPeriod = millis();

			// Turn the PID off 
			reflowOvenPID.SetMode(MANUAL);
			output = 0;
			DEBUG_PRINT_PID("PIDModeChange,");

			// Metrics
			runEndTime = millis();
			runEndTemp = input;

			// state printout
			DEBUG_PRINT("ChangeToAbortPreHeat,");

			// save failure to EEPROM
			EEPROM.get(EEPROM_CYCLES_ABORTEDPH,value);
			value++;
			EEPROM.put(EEPROM_CYCLES_ABORTEDPH,value);

			break;

			// Consider realtime recovery methods here but really the safest is simply cut power and bail out
			// We are way below reflow temps here (IMPROVE?)
		}
		
#ifdef PID_NEAR_FAR_PREHEAT
		// Adaptive Pre-heat Parameters
		// Check whether to modify the parameters of the PID as we get closer to the NEAR trigger point
		// Statement confirms we are inside +/- zone of setpoint and checks to see if the values are the NEAR ones
		// (IMPROVE?)
		if (((input >= (setpoint - PID_NEAR_FAR_TEMP_PREHEAT)) && (input <= (setpoint + PID_NEAR_FAR_TEMP_PREHEAT))) &&
		((reflowOvenPID.GetKp() != PID_KP_PREHEAT_2) ||
		(reflowOvenPID.GetKi() != PID_KI_PREHEAT_2) ||
		(reflowOvenPID.GetKd() != PID_KD_PREHEAT_2)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_PREHEAT_2, PID_KI_PREHEAT_2, PID_KD_PREHEAT_2);
			DEBUG_PRINT_PID("PIDModeChange,");
		}

		// Check whether to modify the parameters of the PID
		// as we fall outside of the NEAR trigger. Opposite of above.
		if (((input < (setpoint - PID_NEAR_FAR_TEMP_PREHEAT)) || (input > (setpoint + PID_NEAR_FAR_TEMP_PREHEAT))) &&
		((reflowOvenPID.GetKp() != PID_KP_PREHEAT) ||
		(reflowOvenPID.GetKi() != PID_KI_PREHEAT) ||
		(reflowOvenPID.GetKd() != PID_KD_PREHEAT)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);
			DEBUG_PRINT_PID("PIDModeChange,");
		}
#endif
		// If minimum soak temperature is achieved by preheat state, then move into
		// this section where we set to the next state which is soak.
		// If it isn't achieved, the timeout and ramp test above should have caused an abort and cooldown
		if (input >= TEMPERATURE_SOAK_MIN)
		{
			// Chop soaking period into smaller sub-period
			// The SOAK state has a ramp rate of ~ 0.5C/sec.
			// This implementation of ramping the setpoint 5C every 9 seconds 
			// accomplishes this assuming the oven keeps up with the requirement

			timerSoak = millis() + SOAK_MICRO_PERIOD;
#ifndef ONEPIDSET
			// Set PID parameters for soaking ramp
			reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
			DEBUG_PRINT_PID("PIDModeChange,");
#endif
			// Ramp up to first section of soaking temperature
			setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   

			// Proceed to soaking state
			reflowState = REFLOW_STATE_SOAK; 

			// Metrics
			runPreHeatEndTime = millis();
			runPreHeatEndTemp = input;

			// state printout
			DEBUG_PRINT("ChangeToSoak,");
		}
		break;

	case REFLOW_STATE_SOAK:     
		// Check to see if we have exceeded max time limit
		// Please note that this test is a bit contrived and in theory can never be executed with normal limit of 240 seconds time to here
		// To test, I set the time limit to 239.  This is due to max preheat of 150 + 10 steps of 9 second = Total of 240 is legal time
		// However, in the event of a coding error, I left this test in at 240
		if (millis() > (runStartTime + (PREHEAT_STAGE_MAX_TIME_TO_REFLOW * 1000.0)))
		{
			// Proceed to error state handler
			reflowState = REFLOW_STATE_SOAK_ERROR; 
			reflowStatus = REFLOW_STATUS_OFF;

			// Change to show error on the completion
			runCompletionStatus = RUN_COMPLETION_STATUS_SOAK_ERROR;

			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
			buzzerPeriod = millis();

			// Turn the PID off but this holds output at last setting
			reflowOvenPID.SetMode(MANUAL);
			output = 0;
			DEBUG_PRINT_PID("PIDModeChange,");

			// Metrics
			runEndTime = millis();
	 		runEndTemp = input;

			// state printout
			DEBUG_PRINT("ChangeToAbortSoak,");

			// save failure to EEPROM
			EEPROM.get(EEPROM_CYCLES_ABORTEDSK,value);
			value++;
			EEPROM.put(EEPROM_CYCLES_ABORTEDSK,value);

			break;
		}

		// Check to see if the 9 second timer has expired (timerSoak)
		// If yes, then set the next setpoint
		if (millis() >= timerSoak)
		{
			timerSoak = millis() + SOAK_MICRO_PERIOD;

			// And move the desired setpoint higher
			setpoint += SOAK_TEMPERATURE_STEP;

			// Test to see if we are done with the SOAK state.
			// Please note that this requires the setpoint to EXCEED the max setpoint
			// which means that it just did a microSOAK at the max temp which is 9 seconds
			// I think the intention is that this allows
			// the oven to catch up to setpoint which would be correct.

			// Since the exit here is based on setpoints (and therefore time) and not actual temp 
			// I have put checks above for time based abort and internal for temp based aborts
			if (((setpoint > TEMPERATURE_SOAK_MAX) && (profileLead == PROFILE_LEAD_FREE)) || 
			((setpoint > TEMPERATURE_SOAK_MAX_LEAD) && (profileLead == PROFILE_LEAD_YES)))
			{
				// Check to see if we are close enough to desired temp
				if (((input < (TEMPERATURE_SOAK_MAX - SOAK_STAGE_TEMP_WITHIN_MAX_ERROR)) && (profileLead == PROFILE_LEAD_FREE)) || 
				((input < (TEMPERATURE_SOAK_MAX_LEAD - SOAK_STAGE_TEMP_WITHIN_MAX_ERROR)) && (profileLead == PROFILE_LEAD_YES)))
				{
					// we are not, abort out now

					// Proceed to error state handler
					reflowState = REFLOW_STATE_SOAK_ERROR; 
					reflowStatus = REFLOW_STATUS_OFF;

					// Change to show error on the completion
					runCompletionStatus = RUN_COMPLETION_STATUS_SOAK_ERROR;

					// setup the buzzerChirp
					buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
					buzzerPeriod = millis();

					// Turn the PID off but this holds output at last setting
					reflowOvenPID.SetMode(MANUAL);
					output = 0;
					DEBUG_PRINT_PID("PIDModeChange,");

					// Metrics
					runEndTime = millis();
					runEndTemp = input;

					// state printout
					DEBUG_PRINT("ChangeToAbortSoak,");

					// save failure to EEPROM
					EEPROM.get(EEPROM_CYCLES_ABORTEDSK,value);
					value++;
					EEPROM.put(EEPROM_CYCLES_ABORTEDSK,value);

					break;
				}
#ifndef ONEPIDSET
				// Set agressive PID parameters for reflow ramp
				reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
				DEBUG_PRINT_PID("PIDModeChange,");
#endif
				// Ramp up to first section of soaking temperature
				setpoint = (profileLead == PROFILE_LEAD_FREE) ? TEMPERATURE_REFLOW_MAX : TEMPERATURE_REFLOW_MAX_LEAD;

				// Proceed to reflowing state
				reflowState = REFLOW_STATE_REFLOW; 

				// Metrics and bang bang control setup
				bangbangTime = runSoakEndTime = millis();
				bangbangInput = runSoakEndTemp = input;

				// state printout
				DEBUG_PRINT("ChangeToReflow,");
			}
		}
		break; 

	case REFLOW_STATE_REFLOW:
		// Check to see if we have exceeded allowable maximum time slot to peak
		if (millis() > (runStartTime + (PREHEAT_STAGE_MAX_TIME_TO_PEAK * 1000.0)))
		{
			// Proceed to error state handler
			reflowState = REFLOW_STATE_REFLOW_ERROR; 
			reflowStatus = REFLOW_STATUS_OFF;

			// Change to show error on the completion
			runCompletionStatus = RUN_COMPLETION_STATUS_REFLOW_ERROR;

			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
			buzzerPeriod = millis();

			// Turn the PID off but this holds output at last setting
			reflowOvenPID.SetMode(MANUAL);
			output = 0;
			DEBUG_PRINT_PID("PIDModeChange,");

			// Metrics
			runEndTime = millis();
	 		runEndTemp = input;

			// state printout
			DEBUG_PRINT("ChangeToAbortReflow,");

			// save failure to EEPROM
			EEPROM.get(EEPROM_CYCLES_ABORTEDRF,value);
			value++;
			EEPROM.put(EEPROM_CYCLES_ABORTEDRF,value);

			break;
		}

#ifdef PID_NEAR_FAR_REFLOW
		// Adaptive Reflow Parameters
		// Check whether to modify the parameters of the PID as we get closer to the NEAR trigger point
		// statement confirms we are inside +/- zone of setpoint and checks to see if the values are the NEAR ones
		if (((input >= (setpoint - PID_NEAR_FAR_TEMP_REFLOW)) && (input <= (setpoint + PID_NEAR_FAR_TEMP_REFLOW))) &&
		((reflowOvenPID.GetKp() != PID_KP_REFLOW_2) ||
		(reflowOvenPID.GetKi() != PID_KI_REFLOW_2) ||
		(reflowOvenPID.GetKd() != PID_KD_REFLOW_2)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_REFLOW_2, PID_KI_REFLOW_2, PID_KD_REFLOW_2);
			DEBUG_PRINT_PID("PIDModeChange,");
		}
		// Check whether to modify the parameters of the PID
		// as we fall outside of the NEAR trigger
		if (((input < (setpoint - PID_NEAR_FAR_TEMP_REFLOW)) || (input > (setpoint + PID_NEAR_FAR_TEMP_REFLOW))) &&
		((reflowOvenPID.GetKp() != PID_KP_REFLOW) ||
		(reflowOvenPID.GetKi() != PID_KI_REFLOW) ||
		(reflowOvenPID.GetKd() != PID_KD_REFLOW)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
			DEBUG_PRINT_PID("PIDModeChange,");
		}
#endif
		// Check to see if we need to put into bang bang to hit the max
		// First check to see if we increased 3 degrees in less than 10 seconds
		if ((input >= (bangbangInput + REFLOW_STAGE_MIN_INCREASE)) &&
		(millis() <= (bangbangTime + (REFLOW_STAGE_ERROR_CHECK_TIME * 1000.0))) &&
		(reflowOvenPID.GetMode() == AUTOMATIC))
		{
			// yes, reset the bangbang status 
			bangbangInput = input;
			bangbangTime = millis();
		}

		// Check to see if we have exceeded the 10 second counter
		if ((millis() > (bangbangTime + (REFLOW_STAGE_ERROR_CHECK_TIME * 1000.0))) &&
		(reflowOvenPID.GetMode() == AUTOMATIC))
		{
			// Turn the PID off 
			reflowOvenPID.SetMode(MANUAL);
			output = 1000;
			DEBUG_PRINT_PID("PIDModeChange,");

			// capture PID change
			lastPIDChange = millis();
		}

		// Check to see if we have hit max temp
		if (((input >= TEMPERATURE_REFLOW_MAX) && (profileLead == PROFILE_LEAD_FREE)) ||
		((input >+ TEMPERATURE_REFLOW_MAX_LEAD) && (profileLead == PROFILE_LEAD_YES)))
		{
			// Now, If we hit the max, then just shut it off and kill the PID
			// The display loop currently requires REFLOW_STATUS_ON
			// which also allows the SSR to engage without add'l checks. (see bottom)
			//
			// what we really want to do is turn the reflow off but still print out the temperatures.

			// Turn the PID off but this holds output at last setting
			reflowOvenPID.SetMode(MANUAL);
			output = 0;
			DEBUG_PRINT_PID("PIDModeChange,");

			// Slam the setpoint, note this doesn't do anything other than provide 
			// an indication of change on the serial output for better looking graphs
			setpoint = 0;

			// capture PID change
			lastPIDChange = millis();

			// Proceed to next reflow state which is actually cooling 
			reflowState = REFLOW_STATE_REFLOW_2; 

			// Metrics -> max temp and time and captured in loop

			// setup buzzer chirp;
			buzzerChirp = BUZZER_CHIRP_NORM_STEPS;
			buzzerPeriod = millis();

			// state printout
			DEBUG_PRINT("ChangeToReflow2,");
		}
		break;   

	case REFLOW_STATE_REFLOW_2:
		// Dummy state which is technically still REFLOW for the soldering process 
		// but the oven is cooling.  

		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_NORM_TIME;
			buzzerChirp--;
		}

		// have we reached the end of the reflow?
		if (((profileLead == PROFILE_LEAD_FREE) && (input <= TEMPERATURE_SOAK_MAX)) ||
		((profileLead == PROFILE_LEAD_YES) && (input <= TEMPERATURE_SOAK_MAX_LEAD)))
		{
			// Additional metrics accumulating on length of reflow zone
			runReflowEndTime = millis();

			// Proceed to cooling state
			reflowState = REFLOW_STATE_COOL; 

			// state printout
			DEBUG_PRINT("ChangeToCool,");
		}
		break;   

	case REFLOW_STATE_COOL:
		// End of cycle is more or less at this min temp (100C)
		if (input <= TEMPERATURE_COOL_MIN)
		{
			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_NORM_STEPS;
			buzzerPeriod = millis();

			// Turn off reflow process
			reflowStatus = REFLOW_STATUS_OFF;                

			// Proceed to reflow Completion state
			reflowState = REFLOW_STATE_COMPLETE; 

			// Metrics
			runEndTime = millis();
	 		runEndTemp = input;

			// state printout
			DEBUG_PRINT("ChangeToComplete,");
		}         
		break;    

	case REFLOW_STATE_COMPLETE:
		// this state turns the buzzer off and print out stats before returning to idle

		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_NORM_TIME;
			buzzerChirp--;
			break;
		}

		if (!buzzerChirp)
		{
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Change to show ok on the completion
			runCompletionStatus = RUN_COMPLETION_STATUS_OK;

			// Output done message
			Serial.println("Reflow normal finish");

			// save success to EEPROM
			EEPROM.get(EEPROM_CYCLES_COMPLETED,value);
			value++;
			EEPROM.put(EEPROM_CYCLES_COMPLETED,value);

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

#ifdef ALLPRINT
			// Metric printout
			Serial.print ("Start Time=");
			Serial.println (runStartTime/1000.);
			Serial.print ("Pre-Heat End Time=");
			Serial.println (runPreHeatEndTime/1000.);
			Serial.print ("Soak End Time=");
			Serial.println (runSoakEndTime/1000.);
			Serial.print ("Peak Time=");
			Serial.println (runPeakTime/1000.);
			Serial.print ("Reflow End Time=");
			Serial.println (runReflowEndTime/1000.);
			Serial.print ("End Time=");
			Serial.println (runEndTime/1000.);
#endif

			// Calculated Metrics
			Serial.print ("Start Temp=");
			Serial.println (runStartTemp);

			Serial.print ("Time in Pre-Heat=");
			Serial.println ((runPreHeatEndTime - runStartTime) / 1000.);
			Serial.print ("Pre-Heat End Temp=");
			Serial.println (runPreHeatEndTemp);

			Serial.print ("Time in Pre-Heat+Soak=");
			Serial.println ((runSoakEndTime - runStartTime) / 1000.);
			Serial.print ("Soak End Temp=");
			Serial.println (runSoakEndTemp);

			Serial.print ("Time to Peak=");
			Serial.println ((runPeakTime - runStartTime) / 1000.);
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);
			Serial.print ("Time in Reflow=");
			Serial.println ((runReflowEndTime - runSoakEndTime) / 1000.);

			Serial.print ("End Temp=");
			Serial.println (runEndTemp);
			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);

			Serial.print ("Ramp Rate Pre-Heat=");
			Serial.println ((runPreHeatEndTemp - runStartTemp) * 1000. /
			(runPreHeatEndTime - runStartTime));

			Serial.print ("Ramp Rate Reflow=");
			Serial.println ((runPeakTemp - runSoakEndTemp) * 1000. /
			(runPeakTime - runSoakEndTime));

			Serial.print ("Number Cycles Attempted by Oven=");
			EEPROM.get(EEPROM_CYCLES_ATTEMPTED,value);
			Serial.println (value);
			Serial.print ("Number Cycles Completed by Oven=");
			EEPROM.get(EEPROM_CYCLES_COMPLETED,value);
			Serial.println (value);
		}
		break;
	
	case REFLOW_STATE_TOO_HOT:
		// When oven temperature drops below room temperature, set back to the idle ready state
		// This prevents starting the oven if it is too warm from previous run
		if (input < TEMPERATURE_ROOM)
		{
			// Ready to reflow
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;
		}
		break;
		
	case REFLOW_STATE_ERROR:
		// If thermocouple problem is still present
		if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || (input == FAULT_SHORT_VCC))
		{
			// Wait until thermocouple wire is connected
			reflowState = REFLOW_STATE_ERROR; 
		} else {
			// Ready state - Clear to perform reflow process
			reflowState = REFLOW_STATE_IDLE; 
			profileLead = PROFILE_LEAD_FREE;
		}
		break;

	//
	// ************ Error handling from last run ********************
	//
	case REFLOW_STATE_PREHEAT_ERROR:
		// Test to see if to chirp buzzer
		// During this time an error message will be displayed
		// Not abortable b/c the oven reflow state is off
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_ERROR_TIME;
			buzzerChirp--;
			break;
		}
		if (!buzzerChirp)
		{
			// the error message will only be displayed during the buzzer above
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended, set oven back to ready
			// but the run state is left with an error
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Output done message
			Serial.println("Warning: Aborted PreHeat");

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

			// Calculated Metrics
			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);
#ifdef SIM
			// Intialize seconds timer so we can get the temp back down
			timerSeconds = 0;
#endif
		}
		break;

	case REFLOW_STATE_SOAK_ERROR:
		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_ERROR_TIME;
			buzzerChirp--;
			break;
		}
		if (!buzzerChirp)
		{
			// it's going to fly through this
			// the error message will only be displayed during the buzzer above
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended, set oven back to ready
			// but the run state is left with an error
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Output done message
			Serial.println("Warning: Aborted Soak");

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

			// Calculated Metrics
			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);

			Serial.print ("Time in Pre-Heat=");
			Serial.println ((runPreHeatEndTime - runStartTime) / 1000.);
			Serial.print ("Pre-Heat End Temp=");
			Serial.println (runPreHeatEndTemp);
#ifdef SIM
			// Intialize seconds timer so we can get the temp back down
			timerSeconds = 0;
#endif
		}
		break;
		

	case REFLOW_STATE_REFLOW_ERROR:
		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_ERROR_TIME;
			buzzerChirp--;
			break;
		}
		if (!buzzerChirp)
		{
			// the error message will only be displayed during the buzzer above
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended, set oven back to ready
			// but the run state is left with an error
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Output done message
			Serial.println("Warning: Aborted Reflow");

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

			// Calculated Metrics
			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);

			Serial.print ("Time in Pre-Heat=");
			Serial.println ((runPreHeatEndTime - runStartTime) / 1000.);
			Serial.print ("Pre-Heat End Temp=");
			Serial.println (runPreHeatEndTemp);

			Serial.print ("Time in Pre-Heat+Soak=");
			Serial.println ((runSoakEndTime - runStartTime) / 1000.);
			Serial.print ("Soak End Temp=");
			Serial.println (runSoakEndTemp);
#ifdef SIM
			// Intialize seconds timer so we can get the temp back down
			timerSeconds = 0;
#endif
		}
		break;

	case REFLOW_STATE_CYCLE_ERROR:
		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_ERROR_TIME;
			buzzerChirp--;
			break;
		}
		// the error message will only be displayed during the buzzer above
		if (!buzzerChirp)
		{
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended, set oven back to ready
			// but the run state is left with an error
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Output done message
			Serial.println("Warning: Aborted Cycle");

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

			// Calculated Metrics
			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);
#ifdef SIM
			// Intialize seconds timer so we can get the temp back down
			timerSeconds = 0;
#endif
		}
		break;

	case REFLOW_STATE_CANCEL:
		// Test to see if to chirp buzzer
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_ERROR_TIME;
			buzzerChirp--;
			break;
		}
		// the error message will only be displayed during the buzzer above
		if (!buzzerChirp)
		{
			// Turn off buzzer and green LED
			digitalWrite(buzzerPin, LOW);

			// Reflow process ended, set oven back to ready
			// but the run state is left with an error
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;

			// Output done message
			Serial.println("Warning: Manually cancelled cycle");

			// state printout
			DEBUG_PRINT("ChangeToIdle,");

			// Calculated Metrics
			Serial.print ("Peak Temp=");
			Serial.println (runPeakTemp);

			Serial.print ("Total Time=");
			Serial.println ((runEndTime - runStartTime) / 1000.);
#ifdef SIM
			// Intialize seconds timer so we can get the temp back down
			timerSeconds = 0;
#endif
		}
		break;

	//
	// ************ Summary Printout  ********************
	//

	case REFLOW_STATE_IDLE_SUMMARY:

		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_NO))
		{
#ifndef CONSTANT_TEMP
			// Check to see if profile switch back to IDLE non Lead
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;
#endif
#ifdef CONSTANT_TEMP
			// If debug compiled in for constant state temp option
			reflowState = REFLOW_STATE_IDLE_CONSTANT;
#endif
			break;
		}

		//
		// ************ Summary starts here ********************
		//
		// If switch is long pressed, start display of summary statistics
		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_YES))
		{
			// Oven Stats
			Serial.print ("Reflow Attempts=");
			EEPROM.get(EEPROM_CYCLES_ATTEMPTED,value);
			Serial.println (value);

			Serial.print ("Reflow Success=");
			EEPROM.get(EEPROM_CYCLES_COMPLETED,value);
			Serial.println (value);

			Serial.print ("Cancelled=");
			EEPROM.get(EEPROM_CYCLES_CANCELLED,value);
			Serial.println (value);

			Serial.print ("Preheat Aborts=");
			EEPROM.get(EEPROM_CYCLES_ABORTEDPH,value);
			Serial.println (value);

			Serial.print ("Soak Aborts=");
			EEPROM.get(EEPROM_CYCLES_ABORTEDSK,value);
			Serial.println (value);

			Serial.print ("Reflow Aborts=");
			EEPROM.get(EEPROM_CYCLES_ABORTEDRF,value);
			Serial.println (value);

			Serial.print ("Cycles Aborted=");
			EEPROM.get(EEPROM_CYCLES_ABORTEDCY,value);
			Serial.println (value);

			// decide what to print out on summary lcd
			switch (runCompletionStatus)
			{
			case RUN_COMPLETION_STATUS_NEVERRUN:
				// no run data
				lcd.clear();
				lcd.print("No Data");
				delay(DISPLAY_DELAY);
				break;

			case RUN_COMPLETION_STATUS_PREHEAT_ERROR:
			case RUN_COMPLETION_STATUS_SOAK_ERROR:
			case RUN_COMPLETION_STATUS_REFLOW_ERROR:
			case RUN_COMPLETION_STATUS_CYCLE_ERROR:
			case RUN_COMPLETION_STATUS_CANCEL:
				// Abort
				lcd.clear();
				if (runCompletionStatus == RUN_COMPLETION_STATUS_PREHEAT_ERROR)
					lcd.print("Abort PH");
				if (runCompletionStatus == RUN_COMPLETION_STATUS_SOAK_ERROR)
					lcd.print("Abort SK");
				if (runCompletionStatus == RUN_COMPLETION_STATUS_REFLOW_ERROR)
					lcd.print("Abort RF");
				if (runCompletionStatus == RUN_COMPLETION_STATUS_CYCLE_ERROR)
					lcd.print("Abort CY");
				if (runCompletionStatus == RUN_COMPLETION_STATUS_CANCEL)
					lcd.print("Canceled");
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("PeakTemp");
				lcd.setCursor(0, 1);
				lcd.print(runPeakTemp);
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("TotlTime");
				lcd.setCursor(0, 1);
				lcd.print((runEndTime - runStartTime) / 1000.);
				delay(DISPLAY_DELAY);
				break;

			case RUN_COMPLETION_STATUS_OK:
				// Success
				lcd.clear();
				lcd.print("Success");
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("PeakTemp");
				lcd.setCursor(0, 1);
				lcd.print(runPeakTemp);
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("PeakTime");
				lcd.setCursor(0, 1);
				lcd.print((runPeakTime - runStartTime) / 1000.);
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("ReflTime");
				lcd.setCursor(0, 1);
				lcd.print((runReflowEndTime - runSoakEndTime) / 1000.);
				delay(DISPLAY_DELAY);

				lcd.clear();
				lcd.print("TotlTime");
				lcd.setCursor(0, 1);
				lcd.print((runEndTime - runStartTime) / 1000.);
				delay(DISPLAY_DELAY);
				break;

			default:
				// should never get here
				lcd.clear();
				lcd.print("Error 3!");
				// buzzer on
				digitalWrite(buzzerPin, HIGH);
				// ensure oven is off
				digitalWrite(ssrPin, LOW);
				while (1); 
			} 

			// back to IDLE
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;
		}
		break;

#ifdef CONSTANT_TEMP
	//
	// ************ Constant Temperature option ********************
	//
	case REFLOW_STATE_IDLE_CONSTANT:

		// Check to see if profile switch back to IDLE non Lead
		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_NO))
		{
			reflowState = REFLOW_STATE_IDLE;
			profileLead = PROFILE_LEAD_FREE;
			break;
		}
		//
		// ************ Start of constant run setup here ********************
		//
		// If switch is long pressed, start 
		if ((switchStatus == SWITCH_1) && (switchLongPressFlag == SWITCH_LONG_PRESS_FLAG_YES))
		{
			// not elegant but its for debug, copied from setup run state above

			// Send header for CSV file
			Serial.println("State,Time,Setpoint,Temperature,Ramp,DutyCycle");

			// Intialize seconds timer for serial output timer
			timerSeconds = 0;

			// Init the last temp
			lastinput = input;

			// Ramp up to constant temp
			setpoint = TEMPERATURE_CONSTANT;

			// Tell the PID to range between 0 and the full window size
			reflowOvenPID.SetOutputLimits(0, windowSize);
			reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);

			// Set PID parameters for constant ramp up here
			reflowOvenPID.SetTunings(PID_KP_CONSTANT_FAR, PID_KI_CONSTANT_FAR, PID_KD_CONSTANT_FAR);

#ifndef PID_OFF_DURING_PREHEAT
			// Turn the PID on
			reflowOvenPID.SetMode(AUTOMATIC);

			// PID started
			lastPIDChange = 0;
#endif
			DEBUG_PRINT_PID("PIDModeChange,");

#ifdef PID_OFF_DURING_PREHEAT
			// Turn the PID on only when within PID_ON_WHEN_WITHIN (it better not be)
			if (input >= (TEMPERATURE_CONSTANT - PID_ON_WHEN_WITHIN))
			{
				reflowOvenPID.SetMode(AUTOMATIC);
				DEBUG_PRINT_PID("PIDModeChange,");

				// PID started
				lastPIDChange = 0;
			} else {
				// Should already be at Manual but do it again and max output
				reflowOvenPID.SetMode(MANUAL);
				output = CONSTANT_OUTPUT_1; 
				DEBUG_PRINT_PID("PIDModeChange,");

				// capture PID change
				lastPIDChange = millis();
			}
// closing endif PID_OFF_DURING_PREHEAT
#endif
			// Proceed to constant stage
			reflowState = REFLOW_STATE_RUN_CONSTANT;

			// Note the REFLOW_STATUS_ON is set in the next case statement

			// Start metrics
			runStartTime = millis();
			runStartTemp = input;

			// reset the display timer for output of data to serial and LCD
			nextCheck = millis();

			// and init the rest to zero
			runPreHeatEndTime = runSoakEndTime = runPeakTime = 0;
			runReflowEndTime = runEndTime = 0;
			runSoakEndTemp = runPreHeatEndTemp = runPeakTemp = runEndTemp = 0;

			// setup the buzzerChirp
			buzzerChirp = BUZZER_CHIRP_NORM_STEPS;
			buzzerPeriod = millis();

			// state printout
			DEBUG_PRINTWIN("StartConstant,");

			// Initialize PID control window starting time after all other work
			windowStartTime = 0;

			// and indicate that oven run in process
			// consider a handler for this case (IMPROVE?)
			runCompletionStatus = RUN_COMPLETION_STATUS_INPROCESS;
		}
		break;

	//
	// ************************ Constant run **********************************
	//
	case REFLOW_STATE_RUN_CONSTANT:
		reflowStatus = REFLOW_STATUS_ON;

		// chirp the buzzer upon start of the oven
		if (buzzerChirp && (buzzerPeriod <= millis())) {
			digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
			buzzerPeriod = millis() + BUZZER_CHIRP_NORM_TIME;
			buzzerChirp--;
		}

#ifdef PID_OFF_DURING_PREHEAT
		// Turn the PID on only when within PID_ON_WHEN_WITHIN value
		if ((input >= (TEMPERATURE_CONSTANT - PID_ON_WHEN_WITHIN)) &&
		(reflowOvenPID.GetMode() == MANUAL))
		{
			reflowOvenPID.SetMode(AUTOMATIC);
			DEBUG_PRINT_PID("PIDModeChange,");
		}
#endif

#ifdef PID_NEAR_FAR
		// Check whether to modify the parameters of the PID as we get closer to the NEAR trigger point
		// statement confirms we are inside +/- zone of setpoint and checks to see if the values are the NEAR ones
		if (((input >= (setpoint - PID_NEAR_FAR_TEMP)) && (input <= (setpoint + PID_NEAR_FAR_TEMP))) &&
		((reflowOvenPID.GetKp() != PID_KP_CONSTANT_NEAR) ||
		(reflowOvenPID.GetKi() != PID_KI_CONSTANT_NEAR) ||
		(reflowOvenPID.GetKd() != PID_KD_CONSTANT_NEAR)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_CONSTANT_NEAR, PID_KI_CONSTANT_NEAR, PID_KD_CONSTANT_NEAR);
			DEBUG_PRINT_PID("PIDModeChange,");
			break;
		}

		// Check whether to modify the parameters of the PID as we fall outside of the NEAR trigger
		// opposite of above.
		if (((input < (setpoint - PID_NEAR_FAR_TEMP)) || (input > (setpoint + PID_NEAR_FAR_TEMP))) &&
		((reflowOvenPID.GetKp() != PID_KP_CONSTANT_FAR) ||
		(reflowOvenPID.GetKi() != PID_KI_CONSTANT_FAR) ||
		(reflowOvenPID.GetKd() != PID_KD_CONSTANT_FAR)))
		{ 
			// Set PID parameters 
			reflowOvenPID.SetTunings(PID_KP_CONSTANT_FAR, PID_KI_CONSTANT_FAR, PID_KD_CONSTANT_FAR);
			DEBUG_PRINT_PID("PIDModeChange,");
			break;
		}
#endif
		// There is no end to this state other than abort button and cycle timeout
		break;

// CONSTANT_TEMP ifdef endif
#endif
	default:
		// should never get here
		lcd.clear();
		lcd.print("Error 1!");
		// buzzer on
		digitalWrite(buzzerPin, HIGH);
		// ensure oven is off
		digitalWrite(ssrPin, LOW);
		while (1); 
	}
	//
	// *********************** End of oven state machine loop **********************
	//

	//
	// *********************** Cancel run option ***********************************
	//
	// If switch 1 is pressed AND reflow is on, then cancel
	if ((switchStatus == SWITCH_1) && (reflowStatus == REFLOW_STATUS_ON))
	{
		// buzzer chirp cleanup
		digitalWrite(buzzerPin, LOW);

		// next state
		reflowState = REFLOW_STATE_CANCEL;
		reflowStatus = REFLOW_STATUS_OFF;

		// Change to show cancel on the completion
		runCompletionStatus = RUN_COMPLETION_STATUS_CANCEL;

		// setup the buzzerChirp
		buzzerChirp = BUZZER_CHIRP_ERROR_STEPS;
		buzzerPeriod = millis();

		// Do some PID cleanup
		reflowOvenPID.SetMode(MANUAL);
		output = 0;
		DEBUG_PRINT_PID("PIDModeChange,");

		// Pick up run end
		runEndTime = millis();
		runEndTemp = input;

		// state printout
		DEBUG_PRINT("ChangeToCancel,");

		// save failure to EEPROM
		EEPROM.get(EEPROM_CYCLES_CANCELLED,value);
		value++;
		EEPROM.put(EEPROM_CYCLES_CANCELLED,value);
	} 

	//
	// *********************** Switch handler ***********************************
	//
	// Please note the switch detection mechanism allows for one run through the loop for detection
	// The switch is always being checked.
	// Everytime through the loop the switch will be cleared immediately below.
	// The code prior to this must either handle a switch depress detection or assume that it will go away.
	// So there is exactly one loop to check for the switch depression.

	// Simple switch debounce state machine (for switch #1)
	switch (debounceState)
	{
	case DEBOUNCE_STATE_IDLE:
		// Polling to see if there is a switch being pressed
		switchStatus = SWITCH_NONE;
		switchLongPressFlag = SWITCH_LONG_PRESS_FLAG_NO;

		// If switch #1 is pressed, start of switch press
		if (analogRead(switchPin) == 0)
		{
			// Intialize debounce counter
			lastDebounceTime = millis();

			// Proceed to check validity of button press
			debounceState = DEBOUNCE_STATE_CHECK;

			// Setup long switch press counter
			switchLongPressTimer = millis();
		}	
		break;

	case DEBOUNCE_STATE_CHECK:
		if (analogRead(switchPin) == 0)
		{
			// If minimum debounce period is completed
			if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
			{
				// Proceed to wait for button release
				debounceState = DEBOUNCE_STATE_RELEASE;
			}
		} else {
		  	// False trigger
			// Reinitialize button debounce state machine
			debounceState = DEBOUNCE_STATE_IDLE; 
		}
		break;

	case DEBOUNCE_STATE_RELEASE:
		// no debounce on release, probably gets eaten up in state machine somewhere
		// or more likely gets discarded in the False trigger above

		// Check if to set long press flag
		if ((millis() - switchLongPressTimer) > LONG_SWITCH_PRESS)
		{
			switchLongPressFlag = SWITCH_LONG_PRESS_FLAG_YES;
		}
		if (analogRead(switchPin) > 0)
		{
			// Valid switch 1 press
			// Note valid switch press is defined on release of switch
			// switchStatus will get reset next time through this switch statement at the top
			switchStatus = SWITCH_1;

			// Reinitialize button debounce state machine
			debounceState = DEBOUNCE_STATE_IDLE; 
		}
		break;

	default:
		// should never get here
		lcd.clear();
		lcd.print("Error 2!");
		// buzzer on
		digitalWrite(buzzerPin, HIGH);
		// ensure oven is off
		digitalWrite(ssrPin, LOW);
		while (1); 
	}

	//
	// ******************** PID computation and SSR control **********************
	//
	// This is being run at loop speed which is recommended by library author.
	// The library will only calculate when the SampleTime has expired.
	// According to the author, this simplified the PID code because he can be assured
	// that the calculation will occur at periodic intervals.
	//
	// Loop speed is about 140uS on my ATmega when measured although it is probably longer due to code additions
	//
	// To continue to print out the data to the serial port, we need REFLOW_STATUS_ON
	// However, I don't want to turn on the relay when the oven is cooling
	// So I have 1) turned PID to manual during cooling stages, 2) set output to 0,
	// and 3) enforced rules below for heating states to be met
	if (reflowStatus == REFLOW_STATUS_ON)
	{
		now = millis();

		// save off previous output
		lastoutput = output;
		
		// init window if first time through, note this is to run in sync with PID
		if (!windowStartTime)
		{
			windowStartTime = now;
		}

		// and compute trying to keep window and PID close timewise
		reflowOvenPID.Compute();

		// state printout only when something significant changed
		if (output != lastoutput)
		{
			// capture PID change
			lastPIDChange = now;
#ifdef ALLPRINT
			Serial.print("PIDComputeOutputChange,");
			Serial.print("Loop=");
			Serial.print(loopIteration);
			Serial.print(",now=");
			Serial.print(now);
			Serial.print(",lastoutput=");
			Serial.print(lastoutput);
			Serial.print(",output=");
			Serial.print(output);
			Serial.print(",WindowStartTime=");
			Serial.print(windowStartTime);
			Serial.print(",Kp=");
			Serial.print(reflowOvenPID.GetKp());
			Serial.print(",Ki=");
			Serial.print(reflowOvenPID.GetKi());
			Serial.print(",Kd=");
			Serial.print(reflowOvenPID.GetKd());
			Serial.print(",Mode=");
			Serial.println(reflowOvenPID.GetMode());
#endif
		}

		if((now - windowStartTime) > windowSize)
		{ 
			// Time to shift the Relay Window
			windowStartTime += windowSize;

			// state printout, this is voluminous but fixed one bug
#ifdef ALLPRINT_WIN
			Serial.print("ChangeOfWindow,");
			Serial.print("Loop=");
			Serial.print(loopIteration);
			Serial.print(",now=");
			Serial.print(now);
			Serial.print(",WindowStartTime=");
			Serial.println(windowStartTime-windowSize);
#endif
		}

		//
		// This is where the SSR is turned on / off (or simply forced off)
		// 
#ifndef NOHEAT
		// Double check - turn on relay only during heating cycles
		// This test ignores the PID output in case there is a coding fault
		if((output >= (now - windowStartTime)) &&
		((reflowState == REFLOW_STATE_PREHEAT) ||
		(reflowState == REFLOW_STATE_SOAK) ||
		(reflowState == REFLOW_STATE_REFLOW) ||
// intentionally commented out this line to defeat potential SSR on during cooling cycle
//		(reflowState == REFLOW_STATE_REFLOW_2) ||
		(reflowState == REFLOW_STATE_RUN_CONSTANT)))
		{
#ifdef ALLPRINT
			// first, check to see if the SSR is currently low
			// because we are going to force it high
			// This was done to only print the following information on a change
			// and this routine is executed every loop
			if (digitalRead(ssrPin) == LOW)
			{
				Serial.print("SSRON,");
				Serial.print("Loop=");
				Serial.print(loopIteration);
				Serial.print(",output=");
				Serial.print(output);
				Serial.print(",now=");
				Serial.print(now);
				Serial.print(",time=");
				Serial.print(millis());
				Serial.print(",lastPIDChange=");
				Serial.print(lastPIDChange);
				Serial.print(",WindowStartTime=");
				Serial.println(windowStartTime);
			}
#endif
			digitalWrite(ssrPin, HIGH);
		} else {
#ifdef ALLPRINT
			if (digitalRead(ssrPin) == HIGH)
			{
				Serial.print("SSROFF,");
				Serial.print("Loop=");
				Serial.print(loopIteration);
				Serial.print(",output=");
				Serial.print(output);
				Serial.print(",now=");
				Serial.print(now);
				Serial.print(",time=");
				Serial.print(millis());
				Serial.print(",lastPIDChange=");
				Serial.print(lastPIDChange);
				Serial.print(",WindowStartTime=");
				Serial.println(windowStartTime);
			}
#endif
			digitalWrite(ssrPin, LOW);
		}
// closing endif for NOHEAT
#endif
	} else {
		// Reflow oven process is off, ensure oven is off
		digitalWrite(ssrPin, LOW);
	}
}
