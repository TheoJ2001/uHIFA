#include "uHIFA.h"

bool MachineModule::wait(uint64_t dur){
    wait_time = millis();
    if(not waiting){
        wait_start = millis();
        waiting = true;
    }
    if(waiting){
        if((wait_time-wait_start)==dur){
            waiting = false;
            return true;
        }
    }
}

int16_t MachineModule::status(int8_t mode){
   return get(mode);
}



void Piston::config(uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin){
    rtd_sens = rtd_pin;	
    ext_sens = ext_pin;				
    piston_pin = push_pin;
}

void Piston::initiate(){
    pinMode(piston_pin, OUTPUT);		
    pinMode(ext_sens, INPUT);
	pinMode(rtd_sens, INPUT);
}

void Piston::read(){
	extended = digitalRead(ext_sens);
	retracted = digitalRead(rtd_sens);
}

void Piston::maintain(){
    digitalWrite(piston_pin, piston_pressure);
}

void Piston::extend(){
    piston_pressure = HIGH;
}

void Piston::retract(){
    piston_pressure = LOW;
}

void Piston::push(){
    if(get(RETRACTED)){
        extend();
    }
    if(get(EXTENDED)){
        retract();
    }
}

int16_t Piston::get(int8_t mode){
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
    }
}



void Grabber::config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin, uint8_t hold_pin, uint8_t grab_pin){
    rtd_sens = rtd_pin;	
    ext_sens = ext_pin;				
    piston_pin = push_pin;
    hold_sens = hold_pin;
    grabber_pin = grab_pin; 
}

void Grabber::initiate(){
    pinMode(rtd_sens, INPUT);
    pinMode(ext_sens, INPUT);
    pinMode(hold_sens, INPUT);
    pinMode(piston_pin, OUTPUT);	
    pinMode(grabber_pin, OUTPUT);
}

void Grabber::read(){
	extended = digitalRead(ext_sens);
	retracted = digitalRead(rtd_sens);
    holding = digitalRead(hold_sens);
}

void Grabber::maintain(){
    read();
    digitalWrite(piston_pin, piston_pressure);
    digitalWrite(grabber_pin, grabber_pressure);
}

void Grabber::grab(){
    if(grabType == VACUUM){
        grabber_pressure = HIGH;
    }else{
        if(holding){
            grabber_pressure = HIGH;
        }
    } 
}

void Grabber::drop(){
     grabber_pressure = LOW;
}

int16_t Grabber::get(int8_t mode){
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



Shuttle::Shuttle(uint8_t upper, uint8_t lower){
	upper_pin = upper;	
	lower_pin = lower;
    
    for(uint8_t i = 0; i<max_stops; i++){ //sets all pins for the stops to undefined
    	stops[i] = NOT_DEF;
    }
}

void Shuttle::initiate(){
    pinMode(upper_pin, OUTPUT);
	pinMode(lower_pin, OUTPUT);
    digitalWrite(upper_pin, upper_pressure);
    digitalWrite(lower_pin, lower_pressure);
}

void Shuttle::config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin){
    this->arm.config(type, rtd_pin, ext_pin, arm_pin, hold_pin, grab_pin);
}

void Shuttle::maintain(){
    read();
    if((upper_pressure*lower_pressure) != HIGH){
        moving = true;
    }else{
        moving = false;
    }
    digitalWrite(upper_pin, upper_pressure);
    digitalWrite(lower_pin, lower_pressure);
}

void Shuttle::addStop(uint8_t s_index, int8_t s_pin){
	stops[s_index] = s_pin;
	stops_amt += (stops[s_index] != -1) ? 1:0;
    pinMode(stops[s_index], INPUT);
}

void Shuttle::read(){
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

void Shuttle::forward(){
    if(this->arm.get(SAFE)){
        upper_pressure  =  LOW;
        lower_pressure = HIGH;
    }else{
        stop(); 
    }
}

void Shuttle::backward(){
	if(this->arm.get(SAFE)){
        upper_pressure  =  HIGH;
        lower_pressure = LOW;
    }else{
        stop();
    }
}

void Shuttle::stop(){
    upper_pressure  =  HIGH;
    lower_pressure = HIGH;
}

void Shuttle::move(uint8_t pos){
    if(pos>last_stop){
        forward();
    }else if(pos<last_stop){
        backward();
    }else if(pos == current_stop){
        stop();
    }
}


void Shuttle::beginDeliv(uint8_t mode){
    if(not delivering){
        if(current_stop != -1){
            if(mode == EXTENDED){
               this->arm.extend();
                if(not this->arm.get(SAFE) and wait(1000)){
                    this->arm.grab();
                    if(this->arm.get(HOLDING) and this->arm.get(EXTENDED)){
                        this->arm.retract();
                        delivering = true;
                    } 
                }     
            }else if(mode == RETRACTED){
                    this->arm.retract();
                    if(this->arm.get(SAFE) and wait(1000)){
                        this->arm.grab();
                    }
                    if(this->arm.get(HOLDING)){
                        delivering = true;
                    }  
            }
        }
    }
}

void Shuttle::endDeliv(uint8_t mode){
    if(delivering){
        if(mode == EXTENDED){
            this->arm.extend();
            if(this->arm.get(EXTENDED) and wait(1000)){
                this->arm.drop();
                if(not this->arm.get(HOLDING)){
                    this->arm.retract();
                    delivering = false;
                } 
            }  
        }else if(mode == RETRACTED){
            this->arm.retract();
            if(this->arm.get(RETRACTED) and wait(1000)){
                this->arm.drop();
                delivering = false;
            }
        }
    }
}

int16_t Shuttle::get(int8_t mode){
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
        return this->arm.get(SAFE);
    }
}


Conveyor::Conveyor(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom){
    power_relay_pin = pwr;
    polar_relay_pin = plr;
    min_sens_pin = min;
    max_sens_pin = max;
    tachometer_pin = tachom;
}

void Conveyor::initiate(){
    pinMode(power_relay_pin, OUTPUT);
    pinMode(polar_relay_pin, OUTPUT);
    pinMode(min_sens_pin, INPUT);
    pinMode(max_sens_pin, INPUT);
    pinMode(tachometer_pin, INPUT_PULLUP);
}

void Conveyor::setMax(uint16_t maxim){
    tachometer_max = maxim;
}

void Conveyor::start(){
    moving = true;
}

void Conveyor::stop(){
    moving = false;
}

void Conveyor::forward(){
    direction = FORWARDS;
}

void Conveyor::backward(){
    direction = BACKWARDS;
}

void Conveyor::reset(){
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

void Conveyor::maintain(){
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
        digitalWrite(power_relay_pin, HIGH);
    }else{
        digitalWrite(power_relay_pin, LOW);
    }
    
    if(direction==FORWARDS){
        digitalWrite(polar_relay_pin, LOW);
    }else{
        digitalWrite(polar_relay_pin, HIGH);
    }
}

void Conveyor::move(int16_t pos){
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

int16_t Conveyor::get(int8_t mode){
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