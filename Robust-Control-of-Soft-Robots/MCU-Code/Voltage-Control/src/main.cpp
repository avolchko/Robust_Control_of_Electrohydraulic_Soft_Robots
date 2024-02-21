#include <Arduino.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <MCUStat_MIMO_SYSID.h>
#include <MCURef_MIMO_SYSID.h>
// #include <RecordedData.h>
#include <TimerOne.h>

// add filter
#include <Filters.h>
#include <Filters/SMA.hpp>

// #include <CircularBuffer.h> // for storing recorded data


ros::NodeHandle nh;


// assign pins as const ints
const int H1 = 22;
const int L1 = 23;
const int H2 = 19;
const int L2 = 18;
const int LED = 13;
const int HV_OFF = 12;
const int HV_ON = 11;
const int KILL = 17;
const int VD1 = 14;
const int VD2 = 20;  
const int HV_MON = 16; 
const int CAP_MON1 = 25; // change this after knowing analog pin
const int CAP_MON2 = 21; // was I_MON

// assign other constant variables
const int FREQ = 30000;     // pwm frequency
const int PUB_INT = 5000;       // ros publish frequency  (us) 5000 us = 5 ms = 200 Hz
const int CTRL_INT = 1000;   // control loop frequency (us) (1000 us = 1000 hz)
const int WRITE_RES = 12;   // PWM write resolution 
const int OUT_MIN = -pow(2, WRITE_RES-2); // (2^n) min pwm value
const int OUT_MAX = pow(2, WRITE_RES-2);  // (2^n) max pwm value 

// scaling factors

// const double VOLT_FACTOR = 1.0;
// const double PWM_FACTOR = 0.25; // changed from 0.25

const double VOLT_FACTOR = 1.0;
const double PWM_FACTOR = 0.6827; // 4096/6000 - > accounts for all scaling for ctrl equation

double KILL_status;
double HV_raw;
double HV_out;
bool ros_kill = 1;               // ROS killswitch bool
bool HV_kill = 0;                // HV killswitch bool

// initialize timing variables
unsigned long curr_time = 0;            // global current time set in loop()
unsigned long prev_time = 0;            // prev_time set for voltage controller
unsigned long prev_loop_time = 0;        // prev_pub_time set for publishing ros message
unsigned long prev_record_time = 0;      // prev_record_time set for recording data

// double KP, KI, KD set by msg mcu_ref
float kp = 0;
float ki = 0;
float kd = 0;

// add butterworth filter for capacitance
// Sampling frequency
const double FS = 1000; // Hz
// // Cut-off frequency (-3 dB)
// const double FC = 250; // Hz
// // Normalized cut-off frequency // ? 
// const double FN = 2 * FC / FS;

// const int SAMPLES = 5; // 200 Hz
const int SAMPLES = 10; // 20 Hz

// Simple Moving Average filter of length SAMPLES, initialized with a value of 0.
SMA<SAMPLES> average1 = {0};
SMA<SAMPLES> average2 = {0};
SMA<SAMPLES> average3 = {0};
SMA<SAMPLES> average4 = {0};

// // Fourth-order Butterworth filter
// auto filter1 = butter<4>(FN);
// auto filter2 = butter<4>(FN);

// // Circular buffers for data that is to be recorded at high frequency and sent after the fact
// CircularBuffer<int16_t, 20000> ch_duty_buf;
// CircularBuffer<int16_t, 20000> dis_duty_buf;
// CircularBuffer<int16_t, 20000> volt_buf;
// CircularBuffer<int16_t, 20000> volt_ref_buf;
// CircularBuffer<unsigned long, 20000> time_buf;

// bool saving = false;  // whether or not we are currently saving data over ros
// bool recording = false; // whether or not we are currently recording data

// std_msgs::String status_msg;
// ros::Publisher status_publisher("teensy_status", &status_msg);

void setup();                           // declare setup function

void DoNothing();

// class for both channels

class Channel {

  public:

    // Member variables
    bool sat = false;
    bool satA = false;
    bool satB = false;
    int pwm = 0;
    int volt_ref = 0;
    double volt_err = 0;
    int last_err = 0;
    int last_pwm = 0;
    int cap = 0;
    int cap_filt = 0;
    double volt_raw;
    double volt;
    double volt_filt;
    double Ch_Duty;
    double Dis_Duty;
    bool charge_state = false;

    // Member functions

    // // initialize PID outputs
    // int cp = 0;                    // P-control output
    // int ci = 0;                    // I-control output
    // int cd = 0;                    // D-control output


    void clamp(int signal) {
    // clamp integral term based on current signal value being clamped 
    // and direction of error versus control signal 

      // check sign of error and signal
      bool error_sign = signbit(volt_err);
      bool signal_sign = signbit(signal);
    
    // first check if the output is saturating, if so, set logic A to true
    // compare saturated value to controller output
      if (signal > OUT_MAX){
        satA = true;
      }
      else if (signal < OUT_MIN){
        satA = true;
      }
      else {
        satA = false;
      }


    // if error sign is same as biceps, output is saturated, set logic B to 1
      if (error_sign == signal_sign){
        satB = true;
      }
      else {
        satB = false;
      }

    // if logic A && B are true -> clamp.
      if (satA == true && satB == true) {
        sat = true;
      }
      else {
        sat = false;
      }

    }

    void setPWM(int pwm) {
      // Takes + or - pwm value and maps it to either charge or drain HASELs.

      if (pwm > 0) { // logic built in here for charge vs discharge state command
        Ch_Duty = pwm;   // set rate to charge
        Dis_Duty = 0;       
        charge_state = 1;       // set state to charge
      }

      else if (pwm <= 0) {
        Ch_Duty = 0;
        Dis_Duty = abs(pwm);     // set rate to drain
        charge_state = 0;        // set state to drain
      }
    }

    void Regulate(int ref) {

      // compare current voltage and command voltage
      volt_err = (ref - volt) ;

      if (ref == 0 && volt < 200)  {
        volt_err = 0;
        last_err = 0;
        last_pwm = 0; 
      }

      // updated antiwindup logic
      if (sat == true){
        volt_err = 0;
        last_err = 0;
      }
      else if (sat == false){
        ; // pass
      }
      
      // PID control
      // proportional control
      // cp = int(PWM_FACTOR * kp * volt_err);
      // derivative control
      // cd = int(PWM_FACTOR * kd * (volt_err - last_err) / CTRL_INT);
      // integral action
      // ci = ci + int(PWM_FACTOR * ki * (volt_err * CTRL_INT));
      // controller output in total pwm (0 - max pwA)
      // pwm = cp  + ci + cd;


      // k_lag z-transform controller 
      // pwm_dbl = PWM_FACTOR * (5 * volt_err - 2.506 * last_err + 0.995 * last_pwm)
      // pwm = int(PWM_FACTOR * (5.0 * volt_err - 2.506 * last_err + 0.995 * last_pwm));
      // new controller as of 10/06/23
      // pwm = int(PWM_FACTOR * (0.9236*last_pwm + 5.0 * volt_err - 3.792 * last_err));
      // new controller as of 10/07/23
      // pwm = int(PWM_FACTOR * (0.8876*last_pwm + 1.0 * volt_err - 0.6446 * last_err));
      // new controller as of 10/07/23
      // pwm = int(PWM_FACTOR * (0.8198*last_pwm + 1.0 * volt_err - 0.4302 * last_err));
      // new controller as of 10/07/23
      // pwm = int(PWM_FACTOR * (0.854*last_pwm + 1.0 * volt_err - 0.4188 * last_err));
      // new controller as of 10/18/23
      // pwm = int(PWM_FACTOR * (0.8943*last_pwm + 1.0 * volt_err - 0.4055 * last_err));

      // new controller for dual control 
      if (volt < 2000) {
        pwm = int(PWM_FACTOR * (0.7299*last_pwm + 1.0 * volt_err - 0.461 * last_err));
      }
      else if (volt >= 2000) {
        pwm = int(PWM_FACTOR * (0.7293*last_pwm + 1.0 * volt_err + 0.07763 * last_err));
      }

      clamp(pwm);

      // apply saturation to bound pwm command
      if (pwm > OUT_MAX){
        pwm = OUT_MAX;
      }
      else if (pwm < OUT_MIN) {
        pwm = OUT_MIN;
      }        

      setPWM(pwm);

      // update last-values
      last_err = volt_err;
      last_pwm = pwm;
    }


};

Channel channel1, channel2;

// **************************** start non-class specific functions ***************************** // 


void DoNothing() {

  if (channel1.volt > 80) {
    // drain charge
    channel1.Ch_Duty = 0;
    channel1.Dis_Duty = 1000;
    channel1.charge_state = 0;

    analogWrite(L1, channel1.Dis_Duty); 
    analogWrite(H1, channel1.Ch_Duty);
  }
  else {
    // turn off optos
    channel1.Ch_Duty = 0;
    channel1.Dis_Duty = 0;
    channel1.charge_state = 0;

    analogWrite(L1, 0);
    analogWrite(H1, 0);
  }
  
  if (channel2.volt > 80) {
     // drain charge
    channel2.Ch_Duty = 0;
    channel2.Dis_Duty = 1000;
    channel2.charge_state = 0;

    analogWrite(L2, channel2.Dis_Duty);
    analogWrite(H2, channel2.Ch_Duty);
  }
  else {
    // turn off optos
    channel2.Ch_Duty = 0;
    channel2.Dis_Duty = 0;
    channel2.charge_state = 0;

    analogWrite(L2, 0);
    analogWrite(H2, 0);
  }

}

void HV_safety() {
  // kill if either channel's voltage is above the typical rail voltage
  if (channel1.volt > 6000 || channel2.volt > 6000) {
    HV_kill = 1;
  }
}

void Actuate() {

  if (channel1.charge_state == 0) {
    analogWrite(L1, channel1.Dis_Duty);
    analogWrite(H1, 0);
  }
  else if (channel1.charge_state == 1) {
    analogWrite(L1, 0);
    analogWrite(H1, channel1.Ch_Duty);
  }

  if (channel2.charge_state == 0) {
    analogWrite(L2, channel2.Dis_Duty);
    analogWrite(H2, 0);
  }
  else if (channel2.charge_state == 1) {
    analogWrite(L2, 0);
    analogWrite(H2, channel2.Ch_Duty);
  }

}


void checkVolt() {
  // this function is connected to timer1

  // read kill pin // why does the logic seem backwards
  KILL_status = analogRead(KILL);

  if (KILL_status <= 50) {
    digitalWrite(HV_ON, HIGH);
  }
  else if (KILL_status > 50)
  {
    digitalWrite(HV_ON, LOW);
    DoNothing();
  }

  HV_raw = analogRead(HV_MON);
  HV_out = map(HV_raw, 0, 3244, 0, 10830); // convert raw voltage to kV
  // max reading is 3244/4096 due to signal amplifier just for HV input

  // read voltage
  channel1.volt_raw = analogRead(VD1);
  channel1.volt = map(channel1.volt_raw, 0, 4096, 0, 10830);
  channel2.volt_raw = analogRead(VD2);
  channel2.volt = map(channel2.volt_raw, 0, 4096, 0, 10830);


  channel1.volt_filt = average3(channel1.volt);
  channel2.volt_filt = average4(channel2.volt);

  // read capacitance 
  channel1.cap = analogRead(CAP_MON1);
  channel2.cap = analogRead(CAP_MON2);

  channel1.cap_filt = average1(channel1.cap);
  channel2.cap_filt = average2(channel2.cap);

  HV_safety();

// if ros kill and hv kill are both 0 then run controller
  if (ros_kill == 0 && HV_kill == 0) {
    channel1.Regulate(channel1.volt_ref);
    channel2.Regulate(channel2.volt_ref);
    Actuate();
  }

  else if (ros_kill == 1 || HV_kill == 1) {
    DoNothing();

    // reset integrator terms
    // volt_err last_err last_pwm pwm
    channel1.volt_err = 0;
    channel1.last_err = 0;
    channel1.last_pwm = 0;
    channel1.pwm = 0;
    channel2.volt_err = 0;
    channel2.last_err = 0;
    channel2.last_pwm = 0;
    channel2.pwm = 0;
  }

  prev_time = curr_time;

}

void messageCb(const sttr_phase2::MCURef_MIMO_SYSID &mcu_ref){  
  // rosrun rosserial_arduino make_libraries.py /home/angie/Documents/ mcu_ref
  
  channel1.volt_ref = mcu_ref.ch1_ref;    // set reference voltage
  channel2.volt_ref = mcu_ref.ch2_ref;    // set reference voltage
  ros_kill = mcu_ref.kill_ref;   // kill bool
  kp = mcu_ref.KP;
  ki = mcu_ref.KI;
  kd = mcu_ref.KD;

}

// void on_record(const std_msgs::Bool& record_msg) {
//   // publish whether or not recording is true to status message
//     // if (record_msg.data)
//     // {
//     //     status_msg.data = "recording true";
//     //     status_publisher.publish(&status_msg);
//     // } else {
//     //     status_msg.data = "recording false";
//     //     status_publisher.publish(&status_msg);
//     // }
    

//     // no longer recording so save data+
//     if(recording && !record_msg.data) { // falling edge
//         // status_msg.data = "calling savedata()";
//         // status_publisher.publish(&status_msg);
//       saving = true;
//       status_msg.data = String(time_buf.size()).c_str();
//       status_publisher.publish(&status_msg);
//       status_msg.data = String(micros()).c_str();
//       status_publisher.publish(&status_msg);
//     } else if (!recording && record_msg.data) { // rising edge
//       saving = false;
//     }
//     recording = record_msg.data;
// }

// ros::Subscriber<std_msgs::Bool> record_subscriber("recording", &on_record);

// instantiate subscriber with name s
ros::Subscriber<sttr_phase2::MCURef_MIMO_SYSID> s("mcu_ref", messageCb); // subscibe to topic mcu_ref

// instantiate publisher with name mcu_status  
sttr_phase2::MCUStat_MIMO_SYSID mcu_status;
ros::Publisher p("mcu_status", &mcu_status);

// sttr_phase2::RecordedData recordedData;
// ros::Publisher recordPub("recorded_data", &recordedData);

void setup() {

  // connect node to ros
  nh.initNode();
  nh.subscribe(s);
  // nh.subscribe(record_subscriber);
  nh.advertise(p);
  // nh.advertise(recordPub);
  nh.negotiateTopics();
  // nh.advertise(status_publisher);

  analogWriteResolution(WRITE_RES); // X bit res for all pwm pins

  //pins for optocoupler array
  pinMode(H1, OUTPUT); //1st channel charge
  analogWriteFrequency(H1, FREQ);
  pinMode(L1, OUTPUT); //1st channel discharge
  analogWriteFrequency(L1, FREQ);
  pinMode(H2, OUTPUT); //2nd channel charge
  analogWriteFrequency(H2, FREQ);
  pinMode(L2, OUTPUT); //2nd channel discharge
  analogWriteFrequency(L2, FREQ);

  //pins for trip status
  pinMode(KILL, INPUT); //detects status trip
  pinMode(HV_ON, OUTPUT); //illuminates red LED to indicate high voltage is on
  pinMode(HV_OFF, OUTPUT); // illuminates green LED to indicate high voltage is off

  //pins for voltage and current monitors
  pinMode(VD1, INPUT); //voltage monitor for 1st channel (left channel when HV input is top)
  pinMode(VD2, INPUT); //voltage monitor for 2nd channel (right channel when HV input is top)
  pinMode(HV_MON, INPUT); //voltage monitor for the output of the UltraVolt
  pinMode(CAP_MON1, INPUT); // capacitance monitor channel 1
  pinMode(CAP_MON2, INPUT); // capacitance monitor channel 2

  //turn built in LED on to signify power on
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // Set resolution and averaging of analog read for voltage monitor
  analogReadRes(12);           // Teensy 4.0: set ADC resolution to this many bits

  Timer1.initialize(CTRL_INT);    // set period in microseconds
  Timer1.attachInterrupt(checkVolt);
}

void loop() {
  curr_time = micros(); // micros can run for ~70 min before overflow

  // run main loop (callback)
  nh.spinOnce(); 

  // Write message to publish to ROS
  mcu_status.chg1 = channel1.Ch_Duty; // pwm value to charge //uint16
  mcu_status.chg2 = channel2.Ch_Duty; // single input system
  mcu_status.dis1 = channel1.Dis_Duty; // pwm value to drain //uint16
  mcu_status.dis2 = channel2.Dis_Duty; // single input system
  mcu_status.volt1 = channel1.volt_filt; // actuator voltage v_act //uint16
  mcu_status.volt2 = channel2.volt_filt; // single input system
  mcu_status.pol1 = ros_kill; // bool
  mcu_status.pol2 = channel1.sat; // bool
  mcu_status.hv = HV_out; // rail voltage so we can check it //int16
  mcu_status.cap1 = channel1.cap_filt; // int16
  mcu_status.cap2 = channel2.cap_filt; // int16

  // publish once every interval
  // using micros, we can run program for ~70 min before overflow
  if (curr_time - prev_loop_time > PUB_INT) {
    p.publish(&mcu_status);

    // update the previousTime value
    prev_loop_time = curr_time;
  }

  // // record data at 2000 hz (500 us)
  // if (curr_time - prev_record_time > 500) {
  //   // record data if recording is true
  //   if(recording){
  //     ch_duty_buf.push(channel1.Ch_Duty);
  //     dis_duty_buf.push(channel1.Dis_Duty);
  //     volt_buf.push(channel1.volt);
  //     volt_ref_buf.push(channel1.volt_ref);
  //     time_buf.push(curr_time);
  //   }

  //   // update the previousTime value
  //   prev_record_time = curr_time;
  // }

  // // if we are currently saving then publish one recordedData messages
  // if (saving)
  // {
  //   if (!time_buf.isEmpty())
  //   {
  //     recordedData.timestamp = time_buf.shift();
  //     recordedData.charge = ch_duty_buf.shift();
  //     recordedData.discharge = dis_duty_buf.shift();
  //     recordedData.volt = volt_buf.shift();
  //     recordedData.voltref = volt_ref_buf.shift();
  //     recordPub.publish(&recordedData);
  //     delay(5);
  //   }

  //   // if (!time_buf.isEmpty())
  //   // {
  //   //   recordedData.timestamp = time_buf.shift();
  //   //   recordedData.charge = ch_duty_buf.shift();
  //   //   recordedData.discharge = dis_duty_buf.shift();
  //   //   recordedData.volt = volt_buf.shift();
  //   //   recordedData.voltref = volt_ref_buf.shift();
  //   //   recordPub.publish(&recordedData);
  //   // }

  //   if (time_buf.isEmpty())
  //   {
  //     saving = false;
  //     ch_duty_buf.clear();
  //     dis_duty_buf.clear();
  //     volt_buf.clear();
  //     volt_ref_buf.clear();
  //   }
  // }
  
}