#ifndef uHIFA_h
#define uHIFA_h

#include <stdint.h> 
#include <Arduino.h>

#define UNDEFINED -1

#define VACUUM 0
#define CLAW 1

#define MIN -1
#define MAX -2

#define RETRACTED 0x0
#define EXTENDED 0x1
#define SAFE 0x2
#define HOLDING 0x3
#define POSITION 0x4
#define MOVING 0x5
#define SEQUENCE_INDEX 0x6
#define DELIVERING 0x7
#define RESET_REQ 0x8
#define RESETING 0x9
#define DIRECTION 0xA
#define DIRECTION_DEFAULT 0xB
#define FORWARDS 0xC
#define BACKWARDS 0xD

class Piston{
  public:
    Piston() = default;
    bool wait(uint64_t dur);
    void config(uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin);
    virtual void init();
    void scan();
    virtual void update();
    virtual int16_t get(int8_t mode);
    
    void push();
    void extend();
    void retract();

    virtual ~Piston() = default;
  protected:  
    bool waiting = false;
    uint64_t wait_start;
    uint64_t wait_time;

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
    void init();
    void scan();
    void update();
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

  class Machine{
  public:
    Machine() = default;
    bool wait(uint64_t dur);
    virtual void init() = 0; 
    virtual void scan() = 0;
    virtual void update() = 0;
    virtual int16_t get(int8_t mode) = 0;
    int16_t status(int8_t mode);
    virtual ~Machine() = default;
  protected:
    bool waiting = false;
    uint64_t wait_start;
    uint64_t wait_time;
};

class Shuttle : public Machine{
  public:
	  Shuttle(uint8_t upper, uint8_t lower); 
    void init();
    void scan();
    void update();
    int16_t get(int8_t mode);;

	  void config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin); 
    void addStop(uint8_t s_index, int8_t s_pin);		 	 //add stops
    
    void move(uint8_t pos);
    void beginDeliv(uint8_t mode);
    void endDeliv(uint8_t mode);
  protected:
    void read();
  private:
    Grabber arm;
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
    uint8_t deliv_seq_index = 0;
    bool delivering = false; 
    bool moving = false;
    
    void stop();
    void forward();
    void backward();
};

class Conveyor : public Machine{
  public:
    Conveyor(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom);
    void init();
    void setMax(uint16_t maxim);
    void scan();
    void update();
    void move(int16_t pos);

    int16_t get(int8_t mode);
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
    bool overshot = false;
    bool moving;
    bool unsafe = false;
    bool in_safety_proc = false;
    bool default_direction = true;
    int8_t direction = FORWARDS;
    
    uint8_t target_pos;

    uint16_t tachometer_max;
    
    bool tachometer_state;
    bool tachometer_read;
    
    static uint16_t tachometer_val;
    static void tachometer_ISR();
    static Conveyor* tachometer;
    
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