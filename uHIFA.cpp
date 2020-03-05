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
    if(status(RETRACTED)){
        extend();
    }
    if(status(EXTENDED)){
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

int16_t PISTON::status(uint8_t mode){
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
		stop_status[i] = digitalRead(stops[i]);
		if(stop_status[i] != 0){
            last_stop = i;
        }
	}
    current_stop = -1;
    for(uint8_t i = 0; i<stops_amt; i++){
		stop_status[i] = digitalRead(stops[i]);
		if(stop_status[i] != 0){
            current_stop = last_stop;
        }
	}
}

void SHUTTLE::forward(){
    if(shuttle_arm.status(SAFE)){
        digitalWrite(upper_pressure, 0);
        digitalWrite(lower_pressure, 1);
        moving = true;
    }else{
        digitalWrite(upper_pressure, 1);
        digitalWrite(lower_pressure, 1); 
    }
}

void SHUTTLE::backward(){
	if(shuttle_arm.status(SAFE)){
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
                if(not shuttle_arm.status(SAFE)){
                    shuttle_arm.grab();
                    if(shuttle_arm.status(HOLDING) and shuttle_arm.status(EXTENDED)){
                        shuttle_arm.retract();
                        delivering = true;
                    } 
                }else if(mode == RETRACTED){
                    shuttle_arm.retract();
                    if(shuttle_arm.status(SAFE)){
                        shuttle_arm.grab();
                    }
                    if(shuttle_arm.status(HOLDING)){
                        delivering = true;
                    }  
                }     
            }
        }
    }
}

void SHUTTLE::endDeliv(uint8_t mode){
    if(delivering){
        if(mode == EXTENDED){
            shuttle_arm.extend();
            if(shuttle_arm.status(EXTENDED)){
                shuttle_arm.drop();
                if(not shuttle_arm.status(HOLDING)){
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

int16_t SHUTTLE::status(uint8_t mode){
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
        return shuttle_arm.status(SAFE);
    }
}


CONVEYOR::CONVEYOR(uint8_t pwr, uint8_t plr, uint8_t str, uint8_t end, uint8_t tachom){
    power_relay_pin = pwr;
    polar_relay_pin = plr;
    start_sens_pin = str;
    end_sens_pin = end;
    tachometer_pin = tachom;
}

void CONVEYOR::initiate(){
    pinMode(power_relay_pin, OUTPUT);
    pinMode(polar_relay_pin, OUTPUT);
    pinMode(start_sens_pin, INPUT);
    pinMode(end_sens_pin, INPUT);
    pinMode(tachometer_pin, INPUT_PULLUP);
}

void CONVEYOR::setMax(uint16_t maxim){
    tachometer_max = maxim;
}

void CONVEYOR::start(){
    digitalWrite(power_relay_pin, 1);
    stopped = false;
}

void CONVEYOR::stop(){
    digitalWrite(power_relay_pin, 0);
    stopped = true;
}

void CONVEYOR::forward(){
    digitalWrite(polar_relay_pin, 0);
}

void CONVEYOR::backward(){
    digitalWrite(polar_relay_pin, 1);
}

void CONVEYOR::reset(){
    reseting = true;
    start();
    backward();
    if(at_start){
        stop();
        tachometer_val = 0;
        reseting = false;
        if(req_reset){
            req_reset = false;
        }
    }
}

void CONVEYOR::maintain(){
    at_start = digitalRead(start_sens_pin);
    at_end = digitalRead(end_sens_pin);
    if(at_end){
        req_reset = true;
    }
    
    if(not req_reset){
        if(not reseting and not stopped){
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
}

void CONVEYOR::move(uint16_t pos){
    start();
    tachometer_val_mapped = map(tachometer_val, 0, tachometer_max, 0, 100);
    if(pos>tachometer_val_mapped){
        forward();
    }else if(pos<tachometer_val_mapped){
        req_reset = true;
    }else{
        stop();
    }
}

int16_t CONVEYOR::status(uint8_t mode){
    if(mode == RESET_REQ){
        return req_reset;
    }else if(mode == RESETING){
        return reseting;
    }else if(mode == STOPPED){
        return stopped;
    }else if(mode == START){
        return at_start;
    }else if(mode == END){
        return at_end;
    }else if(mode == POSITION){
        return tachometer_val_mapped;
    }
}
