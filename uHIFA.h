/**
 * @author Theo Johansson
 * @version 1.0
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 */
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
#define DIRECTION 0x8
#define DIRECTION_DEFAULT 0x9
#define FORWARDS 0xA
#define BACKWARDS 0xB

/**
 * @brief A class for maniging pnumatic 3:2 cylinders with a basic head i.e it can push things and only push things
 */
class Piston{
  public:
    /**
     * @brief Construct a new Piston object
     */
    Piston(void) = default;

    /**
     * @brief waits for the called number of milliseconds
     * @pre can only be used in loop()
     * @see waiting
     * @see wait_start
     * @see wait_time
     * @param dur number of milliseconds to wait
     * @return true
     * @return false
     * @details returns false until the the object has waited the requested time
     */ 
    bool wait(uint64_t dur);

    /**
     * @brief adds pins to control and read the status of the cylinder
     * @pre used in setup() before Piston::init()
     * @param rtd_pin   pin for sensor that becomes high when a Piston object is retracted
     * @param ext_pin   pin for sensor that becomes high when a Piston object is extended
     * @param push_pin  pin for actuator that opens valve
     */ 
    void config(uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin);

    /**
     * @brief initiates all pins
     * @pre used in setup() after Piston::config()
     * @see ext_sens
     * @see rtd_sens
     * @see piston_pin
     * @details uses the pinMode function from Arduino.h to set actuators to OUTPUT and set sensors to INPUT
     */ 
    virtual void init(void);

    /**
     * @brief reads all sensors and stores the values in private and/or protected variables
     * @pre called at the top of void loop()
     * @code 
     * void loop(){ 
     *    pistonObject.scan();
     *    //methods are used here
     *    pistonObject.update(); 
     * }
     * @endcode
     * @see ext_sens
     * @see rtd_sens
     * @see extended
     * @see retracted
     */ 
    virtual void scan(void);

    /**
     * @brief writes to the piston_pin depending on methods used 
     * @pre called at the bottom of void loop()
     * @code 
     * void loop(){ 
     *    pistonObject.scan();
     *    //methods are used here
     *    pistonObject.update(); 
     * }
     * @endcode
     * @see piston_pin
     * @see piston_pressure
     * 
     */ 
    virtual void update(void);

    /**
     * @brief gets the state  of a sensor or actuator
     * @pre mode must be RETRACTED, EXTENDED, SAFE to get a return value other than false
     * @param mode chooses which state to return
     * @return true 
     * @return false 
     * @details RETRACTED true if the cylinder is retracted,  EXTENDED true if the cylinder is extended, SAFE returns true if the cylinder isn't extended and is retracted
     */
    virtual bool get(int8_t mode);
    
    /**
     * @brief performs a pushing motion
     * @see extended
     * @see retracted
     * @see extend()
     * @see retract()
     */ 
    void push(void);

    /**
     * @brief extends the cyilinder
     * @see extended
     * @see retracted
     * @see piston_pressure
     */ 
    void extend(void);

    /**
     * @brief retracts the cyilinder
     * @see extended
     * @see retracted
     * @see piston_pressure
     */ 
    void retract(void);

    /**
     * @brief Destroy the Piston object
     */
    virtual ~Piston() = default;
  protected:  
    bool waiting = false;     ///< tells the wait() method that a waiting duration is in progress
    uint64_t wait_start;      ///< the time the wait started 
    uint64_t wait_time;       ///< the current waited time
    uint8_t ext_sens;	        ///< sensor that checks if the arm is extended
    uint8_t rtd_sens;			    ///< sensor that checks if the arm is retracted
    uint8_t piston_pin;		    ///< actuator controling the valve of the cylinder 
    bool extended;            ///< state of ext_sens
    bool retracted;           ///< state of rtd_sens
    uint8_t piston_pressure;  ///< value that will be written to piston_pin

    /**
     * @brief reads the sensors
     * @see ext_sens
     * @see rtd_sens
     * @see extended
     * @see retracted
     * @details stores the read values in protected varibles
     */ 
    virtual void read(void);
};

/**
 * @brief Similar to Piston, has the ability to grab
 */ 
class Grabber : public Piston{
  public:
    /**
     * @brief Construct a new Grabber object
     */
    Grabber(void) = default;

    /**
     * @brief adds pins to control and read the status of the cylinder and grabber
     * @pre is called in setup, needs to be used before Grabber::config() and type has to be VACUUM or CLAW
     * @param type determines what type of grabber is used
     * @param rtd_pin pin for sensor that becomes HIGH when a Grabber object is retracted
     * @param ext_pin pin for sensor that becomes HIGH when a Grabber object is extended
     * @param push_pin pin for actuator that opens valve
     * @param hold_pin pin for sensor that turns HIGH when a Grabber object is holding something
     * @param grab_pin pin for actuator that engages grabber
     */
    void config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin, uint8_t hold_pin, uint8_t grab_pin);

    /**
     * @brief initiates all pins
     * @pre is called in setup, used after Grabber::config()
     * @see ext_sens
     * @see rtd_sens
     * @see hold_sens
     * @see piston_pin
     * @see grabber_pin
     * @details uses the pinMode function from Arduino.h to set actuators to OUTPUT and set sensors to INPUT
     */ 
    void init(void);

     /**
     * @brief reads all sensors and stores the values in private and/or protected variables
     * @pre is called at the top of void loop()
     * @code 
     * void loop(){ 
     *    grabberObject.scan();
     *    //methods are used here
     *    grabberObject.update(); 
     * }
     * @endcode
     * @see ext_sens
     * @see rtd_sens
     * @see grabber_pin
     * @see extended
     * @see retracted
     * @see holding
     */ 
    void scan(void);

    /**
     * @brief writes to the piston_pin and grabber_pin depending on methods used 
     * @pre called at the bottom of void loop()
     * @code 
     * void loop(){ 
     *    grabberObject.scan();
     *    //methods are used here
     *    grabberObject.update(); 
     * }
     * @endcode
     * @see piston_pin
     * @see piston_pressure
     * @see grabber_pin
     * @see grabber_pressure
     * 
     */ 
    void update(void);

    /**
     * @brief gets the state  of a sensor or actuator
     * @pre mode must be RETRACTED, EXTENDED, SAFE, HOLDING to get a return value other than false
     * @param mode chooses which state to return 
     * @return true 
     * @return false 
     * @details RETRACTED true if the cylinder is retracted,  EXTENDED true if the cylinder is extended, SAFE returns true if the cylinder isn't extended and is retracted, HOLDING returns true when Grabber is holding something
     */
    bool get(int8_t mode);

    /**
     * @brief grabs something
     * @see grabber_pressure
     */
    void grab(void);

    /** 
     * @brief drops something
     * @see grabber_pressure
     */
    void drop(void);

    /**
     * @brief Destroy the Grabber object
     */
    ~Grabber() = default;
  protected:
    /**
     * @brief reads the sensors
     * @note same as Piston::read() with extended functionality
     * @see hold_sens
     * @see holding
     * @see grabber_pin
     * @see grabber_pressure
     * @details stores the read values in protected varibles
     */ 
    void read(void);
  private:
    uint8_t grabType;         ///< the type of grabber used vacuum or claw
    uint8_t hold_sens;        ///< sensor that checks if the grabbe ris holding something
    bool holding;             ///< state of hold_sens
    uint8_t grabber_pin;      ///< actuator controlling tha valve for the grab
    uint8_t grabber_pressure; ///< value that will be written to grabber_pin
  };


  /**
   * @brief a machine module
   */
  class Machine{
  public:
    /**
     * @brief Construct a new Machine object
     */
    Machine() = default;

    /**
     * @brief waits for the called number of milliseconds
     * @pre can only be used in loop()
     * @see waiting
     * @see wait_start
     * @see wait_time
     * @param dur number of milliseconds to wait
     * @return true
     * @return false
     * @details returns false until the the object has waited the requested time
     */ 
    bool wait(uint64_t dur);

    /**
     * @brief pure virtual function
     */
    virtual void init() = 0; 

    /**
     * @brief pure virtual function
     */
    virtual void scan() = 0;

    /**
     * @brief pure virtual function
     */
    virtual void update() = 0;

    /**
     * @brief pure virtual function
     * @param mode 
     * @return int16_t 
     */
    virtual int16_t get(int8_t mode) = 0;

    /**
     * @brief Destroy the Machine object
     */
    virtual ~Machine() = default;
  protected:
    bool waiting = false;     ///< tells the wait() method that a waiting duration is in progress
    uint64_t wait_start;      ///< the time the wait started 
    uint64_t wait_time;       ///< the current waited time
};

/**
 * @brief Machine module consisting of a shuttle with a 5:2 valve and a Grabber
 */
class Shuttle : public Machine{
  public:
    /**
     * @brief Construct a new Shuttle object
     * @param upper asigns an actuator to control the upper chamber of the shuttle
     * @param lower asigns an actuator to control the lower chamber of the shuttle
     */
	  Shuttle(uint8_t upper, uint8_t lower); 	 	 
    
    /**
     * @brief initiates all pins and sets all stops[] to be UNDEFINED
     * @pre is called in setup() and needs to be used before addStop()
     * @see upper_pin
     * @see lower_pin
     * @see stops[]
     * @details uses the pinMode function from Arduino.h to set actuators to OUTPUT and set sensors to INPUT
     */
    void init(void);

    /**
     * @brief initiates a arm and initiates its pins
     * @pre is called in setup()
     * @see Shuttle::arm
     * @see Grabber::config 
     * @param type 
     * @param rtd_pin 
     * @param ext_pin 
     * @param arm_pin 
     * @param hold_pin 
     * @param grab_pin 
     */
    void config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin); 

    /**
     * @brief adds and initiates a stop
     * @pre is called in setup, method must be called after init()
     * @param s_index index of the stop
     * @param s_pin pin of the stop
     * @see stops[]
     */
    void addStop(uint8_t s_index, int8_t s_pin);	

    /**
     * @brief reads all sensors and stores the values in private and/or protected variables
     * @pre called at the top of void loop()
     * @code 
     * void loop(){ 
     *    shuttleObject.scan();
     *    //methods are used here
     *    shuttleObject.update(); 
     * }
     * @endcode
     * @see read()
     * @see Grabber::scan()
     * @see upper_pressure
     * @see lower_pressure
     * @see moving
     */
    void scan(void);

    /**
     * @brief writes to the upper_pin and lower_pin, and updates arm depending on methods used
     * @pre called at the bottom of void loop()
     * @code 
     * void loop(){ 
     *    shuttleObject.scan();
     *    //methods are used here
     *    shuttleObject.update(); 
     * }
     * @endcode
     * @see Grabber::update()
     * @see upper_pin
     * @see lower_pin
     * @see upper_pressure
     * @see lower_pressure
     */
    void update(void);

    /**
     * @brief gets the state of a sensor or actuator
     * @pre mode must be POSITON, MOVING, DELIVERING, SAFE, SEQUENCE_INDEX to get a return value other than 0
     * @param mode 
     * @return int16_t 
     * @details POSITON returns the last stop the arm passed, MOVING returns true if arm is moving, DELIVERING returns true if the arm is delivering something, 
     * SAFE returns true if arm.get(SAFE) is true, SEQUENCE_INDEX returns the current step of the delivery sequence of endDeliv and beginDeliv
     */
    int16_t get(int8_t mode);

    /**
     * @brief moves the arm to the requested stop
     * @pre pos has to be an index of a used stop
     * @param pos the index of the desired stop
     * @see stop()
     * @see forward()
     * @see backward()
     */
    void move(uint8_t pos);

    /**
     * @brief starts a sequence to start a delivery
     * @pre mode has to be RETRACTED or EXTENDED to start a sequence
     * @param mode used to choose if the arm should grab an item extended or retracted
     * @see arm
     * @see delivering
     * @see deliv_seq_index
     * @details when the sequence is finished sets delivering to true
     */
    void beginDeliv(uint8_t mode);

    /**
     * @brief starts a sequence to finish a delivery
     * @pre mode has to be RETRACTED or EXTENDED to start a sequence
     * @param mode used to choose if the arm should drop an item extended or retracted
     * @see arm
     * @see delivering
     * @see deliv_seq_index
     * @details when the sequence is finished sets delivering to false
     */
    void endDeliv(uint8_t mode);
  protected:
    /**
     * @brief reads all defined stops[] and gives last_stop and current_stop their values
     * @see last_stop
     * @see current_stop
     */
    void read(void);
  private:
    uint8_t upper_pin;              ///< actuator controlling the valve of the upper chamber of the Shuttle
    uint8_t lower_pin;              ///< actuator controlling the valve of the lower chamber of the Shuttle
    Grabber arm;                    ///< the arm attatched to the Shuttle
    int8_t stops[8];                ///< sensors that triggers when the Shuttle::arm passes over them, are set to UNDEFINED if no sensor is added
    bool stop_state[8];	            ///< states of stops[]
    uint8_t stops_amt;			        ///< number of defined stops
    uint8_t upper_pressure = HIGH;  ///< value that will be written to the upper_pin
    uint8_t lower_pressure = HIGH;  ///< value that will be written to the lower_pin
    int8_t last_stop = UNDEFINED;   ///< last stop the arm stopped at, initially set to UNDEFINED becuase the arm won't be at a stop
    int8_t current_stop;            ///< current location of the arm, is UNDEFINED when moving between stops
    uint8_t deliv_seq_index = 0x0;  ///< current step of a sequence used in beginDeliv() and endDeliv()
    bool delivering = false;        ///< true when arm is delivering an item
    bool moving = false;            ///< true when the arm is moving
    
    /**
     * @brief stops the Shuttle
     * @see upper_pressure
     * @see lower_pressure
     * @details Shuttle stops when both upper_pressure and lower_pressure are HIGH
     */
    void stop(void);

    /**
     * @brief moves the Shuttle forwards
     * @see upper_pressure
     * @see lower_pressure
     * @details Shuttle moves forwards when upper_pressure is LOW and lower_pressure is HIGH
     */
    void forward(void);

    /**
     * @brief moves the Shuttle backwards
     * @see upper_pressure
     * @see lower_pressure
     * @details Shuttle moves backwards when upper_pressure is HIGH and lower_pressure is LOW
     */
    void backward(void);
};

/**
 * @brief interrupt sub-routine used for counting increments made by a tachometer
 * @see tachometer_val
 */
void tachometer_ISR(void);
static volatile int16_t tachometer_val; ///< stores counted incriments

/**
 * @brief a bidirectional auger conveyor
 * @note only one object of this class can exist due to the need of an interrupt and the attacthInterrupt() requires a static function
 * @todo find a way to circumvent the use of a static function if that fails make tachometer optional
 */
class Conveyor : public Machine{
  public:
    /**
     * @brief Construct a new Conveyor object
     * @param pwr relay supplying power to the conveyor:
     * @param plr relay changing the polarity of the motor
     * @param min sensor at the end of the Conveyor signaling its at the minimum position
     * @param max sensor at the end of the Conveyor signaling its at the maximum position
     * @param tachom sensor triggering tachometer_ISR()
     * @see power_relay_pin
     * @see polar_relay_pin
     * @see tachometer_pin
     * @see min_sens_pin
     * @see max_sens_pin
     */
    Conveyor(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom);

    /**
     * @brief initiates all actuators, sensors and attaches ISR
     * @see power_relay_pin
     * @see polar_relay_pin
     * @see tachometer_pin
     * @see min_sens_pin
     * @see max_sens_pin
     * @details uses the pinMode function from Arduino.h to set actuators to OUTPUT and set sensors to INPUT
     */
    void init(void);

    /**
     * @brief Set the Max val of the tachometer for bidirectionality
     * @param maxim the maximum value the tachometer counts to before reseting 
     * @todo make this dynamic
     * @see tachometer_max
     */
    void setMax(uint16_t maxim);

    /**
     * @brief reads all sensors and runs the ISR
     * @pre called at the top of void loop()
     * @code 
     * void loop(){ 
     *    conveyorObject.scan();
     *    //methods are used here
     *    conveyorObject.update(); 
     * }
     * @endcode
     * @see tachometer_pin
     * @see min_sens_pin
     * @see max_sens_pin
     */
    void scan(void);

    /**
     * @brief writes to the relays based on methods used
     * @note also has code to stop the conveyor to move past the max and min sensors
     * @pre called at the top of void loop()
     * @code 
     * void loop(){ 
     *    conveyorObject.scan();
     *    //methods are used here
     *    conveyorObject.update(); 
     * }
     * @endcode
     * @see power_relay_pin
     * @see polar_relay_pin
     * @see at_min
     * @see at_max
     */
    void update(void);

    /**
     * @brief moves the conveyor to the desired position
     * @pre pos has to be MAX, MIN or a positive integer
     * @param pos there is 3 types of positions MAX, MIN and an integer, MAX moves the Conveyor to the max sensor, MIN moves the conveyor to the the min sensor 
     * and an integer value moves it to that tachometer increment
     * @see at_min
     * @see at_max
     * @see target_pos
     * @see stop()
     * @see start()
     * @see forward()
     * @see backward()
     */
    void move(int16_t pos);

    /**
     * @brief stops the conveyor
     * @see moving
     * @see update
     */
    void stop(void);

    /**
     * @brief 
     * @pre mode must be POSITON, MOVING, DIRECTION, DIRECTION_DEFAULT, FORWARDS, BACKWARDS, SAFE to get a return value other than 0
     * @param mode 
     * @return int16_t 
     * @details POSITON returns current tachomter increment, MOVING returns true if moving, DIRECTION returns the hex value FORWARDS or BACKWARDS indicating direction, 
     * DIRECTION_DEFAULT returns true if moving in the initial direction used for bidirectionality, FORWARDS return true if moving forwards, BACKWARDS return true if moving backward, 
     * SAFE returns true if it's safe to move i.e Conveyor isn't changing direction
     */
    int16_t get(int8_t mode);
  private:
    uint8_t power_relay_pin;      ///< relay controling power for the conveyor motor
    uint8_t polar_relay_pin;      ///< relay controling polarity/direction of conveyor motor
    uint8_t min_sens_pin;         ///< sensor at the end of the Conveyor signaling its at the minimum position 
    uint8_t max_sens_pin;         ///< sensor at the end of the Conveyor signaling its at the maximum position
    uint8_t tachometer_pin;       ///< sensor/interrupt linked to tachometer_ISR() used to keep track of the position of the Conveyor
    bool at_min;                  ///< state of min_sens_pin
    bool at_max;                  ///< state of max_sens_pin
    bool moving;                  ///< if true update writes HIGH power_relay_pin otherwise false
    bool unsafe = false;          ///< if true the Conveyor is in a safety procedure, becomes false when safety procedure is finished
    bool in_safety_proc = false;  ///< if true a safety procedure is in progress
    bool default_direction = true;///< if true the conveyor is moving in the default direction
    int8_t direction = FORWARDS;  ///< current relative direction
    int16_t target_pos;           ///< requested position, if deafault direction is false target_pos is inverted
    int16_t tachometer_max;       ///< the max value the tachometer increments, used to invert target_pos
    
    /**
     * @brief starts the conveyor
     * @see moving
     * @see update
     */
    void start(void);

    /**
     * @brief change the direction to forward, is relative to the default direction
     * @see direction
     * @see update
     */
    void forward(void);

    /**
     * @brief change the direction to forward, is relative to the default direction
     * @see direction
     * @see update
     */
    void backward(void);
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
