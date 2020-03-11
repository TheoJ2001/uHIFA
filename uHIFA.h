#ifndef uHIFA_h
#define uHIFA_h

#include <stdint.h> 
#include <Arduino.h>

#define NOT_DEF -1

#define VACUUM 0
#define CLAW 1

#define MIN -2
#define MAX -1
#define RETRACTED 0
#define EXTENDED 1
#define SAFE 2
#define HOLDING 3
#define POSITION 4
#define MOVING 5
#define DELIVERING 6
#define RESET_REQ 7
#define RESETING 8
#define DIRECTION 9
#define FORWARDS 10
#define BACKWARDS 11


class MachineModule{
  public:
    MachineModule() = default;
    bool wait(uint64_t dur);
    virtual void initiate() = 0; 
    virtual void maintain() = 0;
    virtual int16_t get(int8_t mode) = 0;
    int16_t status(int8_t mode);
  protected:
    virtual void read() = 0;
    bool waiting = false;
    uint64_t wait_start;
    uint64_t wait_time;
};

class Piston : public MachineModule{
  public:
    Piston() = default;
    void config(uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin);
    virtual void initiate();
    virtual void maintain();
    virtual int16_t get(int8_t mode);
    
    void push();
    void extend();
    void retract();
  protected:                        
    uint8_t ext_sens;			      //the sensor that checks if the arm is extended
    uint8_t rtd_sens;			      //the sensor that checks if the arm is retracted
    bool extended;
    bool retracted;

    uint8_t piston_pin;		//the actuator controling the extension and retraction of the arm 
    uint8_t piston_pressure; 
    virtual void read();
};

class Grabber : public Piston{
  public:
    Grabber() = default;
    void config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin, uint8_t hold_pin, uint8_t grab_pin);
    void initiate();
    void maintain();
    int16_t get(int8_t mode);

    void grab();
    void drop();
  protected:
    void read();
  private:
    uint8_t grabType; 
    uint8_t hold_sens; 
    bool holding;
    uint8_t grabber_pin;
    uint8_t grabber_pressure; 
  };

class Shuttle : public MachineModule{
  public:
	  Shuttle(uint8_t upper, uint8_t lower); 
    void initiate();
    void maintain();
    int16_t get(int8_t mode);;

	  void config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin); 
    void addStop(uint8_t s_index, int8_t s_pin);		 	 //add stops
    
    void move(uint8_t pos);
    void beginDeliv(uint8_t mode);
    void endDeliv(uint8_t mode);
  protected:
    void read();
  private:
    Grabber arm();
    uint8_t max_stops = 8;
    int8_t stops[8];		        //the pins of the stops, -1 undefined
    bool stop_get[8];	          //if index is true then it's stopped at index
    uint8_t stops_amt;				  //how many times will the Shuttle stop
    uint8_t upper_pin;
    uint8_t lower_pin;
    uint8_t upper_pressure = HIGH;
    uint8_t lower_pressure = HIGH;
    
    int8_t last_stop = -1;
    int8_t current_stop;
    bool delivering = false; 
    bool moving = false;
    
    void stop();
    void forward();
    void backward();
};

class Conveyor : public MachineModule{
  public:
    Conveyor(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom);
    void initiate();
    void setMax(uint16_t maxim);
    void maintain();
    void reset();
    void move(int16_t pos);

    int16_t get(int8_t mode);
    int16_t status(int8_t mode);
  private:
    uint8_t power_relay_pin;
    uint8_t polar_relay_pin;
    uint8_t tachometer_pin;
    uint8_t min_sens_pin;
    uint8_t max_sens_pin;
    
    bool at_min;
    bool at_max;
    
    bool req_reset = true;
    bool reseting;
    bool moving;
    int8_t direction = FORWARDS;
    
    uint8_t target_pos;

    uint16_t tachometer_max;
    uint16_t tachometer_val;
    uint16_t tachometer_val_mapped;
    
    bool tachometer_read;
    
    void start();
    void stop();
    void forward();
    void backward();
};
#endif

#ifdef CONTROLLINO_MINI
  #define _D0 4
  #define _D1 5
  #define _D2 6
  #define _D3 7
  #define _D4 8
  #define _D5 9
  #define _D6 18
  #define _D7 19
  
  #define _A0 14
  #define _A1 15
  #define _A2 16
  #define _A3 17
  #define _A4 20
  #define _A5 21
  #define _IN0 2
  #define _IN1 3
#endif

#ifdef CONTROLLINO_MAXI
  #define _D0 2
  #define _D1 3
  #define _D2 4
  #define _D3 5
  #define _D4 6
  #define _D5 7
  #define _D6 8
  #define _D7 9
  #define _D8 10
  #define _D9 11
  #define _D10 12
  #define _D11 13
  
  #define _A0 54
  #define _A1 55
  #define _A2 56
  #define _A3 57
  #define _A4 58
  #define _A5 59
  #define _A6 60
  #define _A7 61
  #define _A8 62
  #define _A9 63
  #define _IN0 18
  #define _IN1 19
  
  #define _MINUS 14
  #define _PLUS 15
  
  #define _R0 22
  #define _R1 23
  #define _R2 24
  #define _R3 25
  #define _R4 26
  #define _R5 27
  #define _R6 28
  #define _R7 29
  #define _R8 30
  #define _R9 31
#endif

#ifdef CONTROLLINO_MEGA
  #define _D0 2
  #define _D1 3
  #define _D2 4
  #define _D3 5
  #define _D4 6
  #define _D5 7
  #define _D6 8
  #define _D7 9
  #define _D8 10
  #define _D9 11
  #define _D10 12
  #define _D11 13
  #define _D12 42
  #define _D13 43
  #define _D14 44
  #define _D15 45
  #define _D16 46
  #define _D17 47
  #define _D18 48
  #define _D19 49
  #define _D20 77
  #define _D21 78
  #define _D22 79
  #define _D23 80
  
  #define _A0 54
  #define _A1 55
  #define _A2 56
  #define _A3 57
  #define _A4 58
  #define _A5 59
  #define _A6 60
  #define _A7 61
  #define _A8 62
  #define _A9 63
  #define _A10 64
  #define _A11 65
  #define _A12 66
  #define _A13 67
  #define _A14 68
  #define _A15 69
  #define _I16 38
  #define _I17 39
  #define _I18 40
  #define _IN0 18
  #define _IN1 19
  
  #define _MINUS 14
  #define _PLUS 15
  
  #define _R0 22
  #define _R1 23
  #define _R2 24
  #define _R3 25
  #define _R4 26
  #define _R5 27
  #define _R6 28
  #define _R7 29
  #define _R8 30
  #define _R9 31
  #define _R10 32
  #define _R11 33
  #define _R12 34
  #define _R13 35
  #define _R14 36
  #define _R15 37
#endif