#include "uHIFA.h"

PISTON::PISTON(){}

void PISTON::initiate(){
    pinMode(piston_pressure, OUTPUT);		
    pinMode(ext_sens, INPUT);
	pinMode(rtd_sens, INPUT);
}

void PISTON::config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t piston_pin){
    pistonType = type; 
    rtd_sens = rtd_pin;	
    ext_sens = ext_pin;				
    piston_pressure = piston_pin;
    pinMode(rtd_sens, INPUT);
    pinMode(ext_sens, INPUT);
    pinMode(piston_pressure, OUTPUT);	
}

void PISTON::addGrab(uint8_t hold_pin, uint8_t grab_pin){
    if(pistonType != PUSH){
        hold_sens = hold_pin;
        grab_pressure = grab_pin; 
		pinMode(hold_sens, INPUT);
        pinMode(grab_pressure, OUTPUT);
	}
}

void PISTON::read(){
	extended = digitalRead(ext_sens);
	retracted = digitalRead(rtd_sens);
    if(not PUSH){
        holding = digitalRead(hold_sens);
    }
}

void PISTON::extend(){
    digitalWrite(piston_pressure, 1);
    read();
}

void PISTON::retract(){
    digitalWrite(piston_pressure, 0); 
    read();
}

void PISTON::push(){
    read();
    if(get(RETRACTED)){
        extend();
    }
    if(get(EXTENDED)){
        retract();
    }
}

void PISTON::grab(){
    if(pistonType == VACUUM){
        digitalWrite(grab_pressure, 1);
    }else{
        if(holding){
            digitalWrite(grab_pressure, 1);
        }
    } 
    read();
}

void PISTON::drop(){
    if(pistonType != PUSH){
        digitalWrite(grab_pressure, 0); 
    }
}

int16_t PISTON::get(int8_t mode){
    read();
    if(mode == RETRACTED){
        return retracted;
    }else if(mode == EXTENDED){
        return extended;
    }else if(mode == SAFE){
        if(retracted and not extended){
            return true;
        }else{
            return false;
        }
    }else if(mode == HOLDING){
        return holding;
    }
} 

int16_t PISTON::status(int8_t mode){
   return get(mode);
}


SHUTTLE::SHUTTLE(uint8_t upper_pin, uint8_t lower_pin){
	upper_pressure = upper_pin;	
	lower_pressure = lower_pin;
    
    for(uint8_t i = 0; i<max_stops; i++){ //sets all pins for the stops to undefined
    	stops[i] = NOT_DEF;
    }
}

void SHUTTLE::initiate(){
    pinMode(upper_pressure, OUTPUT);
	pinMode(lower_pressure, OUTPUT);
}

void SHUTTLE::config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin){
    shuttle_arm.config(type, rtd_pin, ext_pin, arm_pin);
	shuttle_arm.addGrab(hold_pin, grab_pin);
}

void SHUTTLE::addStop(uint8_t s_index, int8_t s_pin){
	stops[s_index] = s_pin;
	stops_amt += (stops[s_index] != -1) ? 1:0;
    pinMode(stops[s_index], INPUT);
}

void SHUTTLE::read(){
	for(uint8_t i = 0; i<stops_amt; i++){
		stop_get[i] = digitalRead(stops[i]);
		if(stop_get[i] != 0){
            last_stop = i;
        }
	}
    current_stop = -1;
    for(uint8_t i = 0; i<stops_amt; i++){
		stop_get[i] = digitalRead(stops[i]);
		if(stop_get[i] != 0){
            current_stop = last_stop;
        }
	}
}

void SHUTTLE::forward(){
    if(shuttle_arm.get(SAFE)){
        digitalWrite(upper_pressure, 0);
        digitalWrite(lower_pressure, 1);
        moving = true;
    }else{
        digitalWrite(upper_pressure, 1);
        digitalWrite(lower_pressure, 1); 
    }
}

void SHUTTLE::backward(){
	if(shuttle_arm.get(SAFE)){
        digitalWrite(upper_pressure, 1);
        digitalWrite(lower_pressure, 0);
        moving = true;
    }else{
        digitalWrite(upper_pressure, 1);
        digitalWrite(lower_pressure, 1); 
        moving = false;
    }
}

void SHUTTLE::stop(){
    digitalWrite(upper_pressure, 1);
    digitalWrite(lower_pressure, 1);
    moving = false;
}

void SHUTTLE::maintain(){
    read();
}

void SHUTTLE::move(uint8_t pos){
    if(pos>last_stop){
        forward();
    }else if(pos<last_stop){
        backward();
    }else if(pos == current_stop){
        stop();
    }
}


void SHUTTLE::beginDeliv(uint8_t mode){
    if(not delivering){
        if(current_stop != -1){
            if(mode == EXTENDED){
               shuttle_arm.extend();
                if(not shuttle_arm.get(SAFE)){
                    shuttle_arm.grab();
                    if(shuttle_arm.get(HOLDING) and shuttle_arm.get(EXTENDED)){
                        shuttle_arm.retract();
                        delivering = true;
                    } 
                }     
            }else if(mode == RETRACTED){
                    shuttle_arm.retract();
                    if(shuttle_arm.get(SAFE)){
                        shuttle_arm.grab();
                    }
                    if(shuttle_arm.get(HOLDING)){
                        delivering = true;
                    }  
            }
        }
    }
}

void SHUTTLE::endDeliv(uint8_t mode){
    if(delivering){
        if(mode == EXTENDED){
            shuttle_arm.extend();
            if(shuttle_arm.get(EXTENDED)){
                shuttle_arm.drop();
                if(not shuttle_arm.get(HOLDING)){
                    shuttle_arm.retract();
                    delivering = false;
                } 
            }  
        }else if(mode == RETRACTED){
                shuttle_arm.retract();
                shuttle_arm.drop();
                delivering = false;
        }
    }
}

int16_t SHUTTLE::get(int8_t mode){
    read();
    if(mode == POSITION){
        return last_stop;
    }else if(mode == MOVING){
        if(current_stop == -1){
            return true;
        }else{
            return false;
        }
    }else if(mode == DELIVERING){
        return delivering;
    }else if(mode == SAFE){
        return shuttle_arm.get(SAFE);
    }
}

int16_t SHUTTLE::status(int8_t mode){
   return get(mode);
}


CONVEYOR::CONVEYOR(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom){
    power_relay_pin = pwr;
    polar_relay_pin = plr;
    min_sens_pin = min;
    max_sens_pin = max;
    tachometer_pin = tachom;
}

void CONVEYOR::initiate(){
    pinMode(power_relay_pin, OUTPUT);
    pinMode(polar_relay_pin, OUTPUT);
    pinMode(min_sens_pin, INPUT);
    pinMode(max_sens_pin, INPUT);
    pinMode(tachometer_pin, INPUT_PULLUP);
}

void CONVEYOR::setMax(uint16_t maxim){
    tachometer_max = maxim;
}

void CONVEYOR::start(){
    moving = true;
}

void CONVEYOR::stop(){
    moving = false;
}

void CONVEYOR::forward(){
    direction = FORWARDS;
}

void CONVEYOR::backward(){
    direction = BACKWARDS;
}

void CONVEYOR::reset(){
    reseting = true;
    start();
    backward();
    if(at_min){
        stop();
        tachometer_val = 0;
        reseting = false;
        if(req_reset){
            req_reset = false;
        }
    }
}

void CONVEYOR::maintain(){
    at_min = digitalRead(min_sens_pin);
    at_max = digitalRead(max_sens_pin);

    if(at_min){
        forward();
        tachometer_val = 0;
    }

    if(at_max){
        backward();
        tachometer_val = 0;
    }
    
    if(not req_reset){
        if(not reseting and moving){
            if(not tachometer_read){
                if(digitalRead(tachometer_pin)){
                    tachometer_val +=1;
                    tachometer_read = true;
                }
            }else{
                if(not digitalRead(tachometer_pin)){
                    tachometer_read = false;
                } 
            }
        }
    }else{
        reset();
    }

    if(moving){
        digitalWrite(power_relay_pin, 1);
    }else{
        digitalWrite(power_relay_pin, 0);
    }
    
    if(direction==FORWARDS){
        digitalWrite(polar_relay_pin, 0);
    }else{
        digitalWrite(polar_relay_pin, 1);
    }
}

void CONVEYOR::move(int16_t pos){
    start();
    tachometer_val_mapped = map(tachometer_val, 0, tachometer_max, 0, 100);

    if(pos != (MAX or MIN)){
        if(direction==BACKWARDS){
            target_pos = 100 - pos;
        }else{
            target_pos = pos;
        }
        
        if(target_pos>tachometer_val_mapped){
            forward();
        }else if(target_pos<tachometer_val_mapped){
            if(tachometer_val_mapped>50){
                if(direction==FORWARDS){
                    forward();
                }else{
                    backward();
                }
            }else{
                reset();
            }
        }else{
            stop();
        }
    }

    if(pos==MIN){
        if(at_min){
            stop();
        }else if(tachometer_val_mapped > 0){
            backward();
        }else{
            backward();
        }
    }else if(pos==MAX){
        if(at_min){
            stop();
        }else{
            forward();
        }
    }
}

int16_t CONVEYOR::get(int8_t mode){
    if(mode == RESET_REQ){
        return req_reset;
    }else if(mode == RESETING){
        return reseting;
    }else if(mode == MOVING){
        return moving;
    }else if(mode == MIN){
        return at_min;
    }else if(mode == MAX){
        return at_max;
    }else if(mode == POSITION){
        return tachometer_val_mapped;
    }else if(mode == DIRECTION){
        return direction;
    }else if(mode == FORWARDS){
        return (direction == FORWARDS) ? 1:0;
    }else if(mode == BACKWARDS){
        return (direction == BACKWARDS) ? 1:0;
    }
}

int16_t CONVEYOR::status(int8_t mode){
   return get(mode);
}