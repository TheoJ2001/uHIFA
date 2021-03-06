#include "uHIFA.h"

void tachometer_ISR(){
    tachometer_val++;
}

bool Piston::wait(uint64_t dur){
    wait_time = millis();
    if(not waiting){
        wait_start = millis();
        waiting = true;
    }
    if(waiting){
        if((wait_time-wait_start)>=dur){
            waiting = false;
            dur = 0;
            return true;
        }
    }
}

void Piston::config(uint8_t rtd_pin, uint8_t ext_pin, uint8_t push_pin){
    rtd_sens = rtd_pin;	
    ext_sens = ext_pin;				
    piston_pin = push_pin;
}

void Piston::init(){
    pinMode(piston_pin, OUTPUT);		
    pinMode(ext_sens, INPUT);
	pinMode(rtd_sens, INPUT);
}

void Piston::read(){
	extended = digitalRead(ext_sens);
	retracted = digitalRead(rtd_sens);
}

void Piston::scan(){
    read();
}

void Piston::update(){
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

bool Piston::get(int8_t mode){
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

void Grabber::init(){
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

void Grabber::scan(){
    read();
}

void Grabber::update(){
    digitalWrite(piston_pin, piston_pressure);
    digitalWrite(grabber_pin, grabber_pressure);
}

void Grabber::grab(){
    if(grabType == VACUUM){
        grabber_pressure = HIGH;
    }
    if(grabType == CLAW){
        if(holding==HIGH){
            if(wait(1000)){
                grabber_pressure = HIGH;
            }
        }  
    }
}

void Grabber::drop(){
     grabber_pressure = LOW;
}

bool Grabber::get(int8_t mode){
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

bool Machine::wait(uint64_t dur){
    wait_time = millis();
    if(not waiting){
        wait_start = millis();
        waiting = true;
    }
    if(waiting){
        if((wait_time-wait_start)>=dur){
            waiting = false;
            dur = 0;
            return true;
        }
    }
}

Shuttle::Shuttle(uint8_t upper, uint8_t lower){
	upper_pin = upper;	
	lower_pin = lower;
}

void Shuttle::init(){
    for(uint8_t i = 0; i<8; i++){ //sets all pins for the stops to undefined
    	stops[i] = UNDEFINED;
    }
    pinMode(upper_pin, OUTPUT);
	pinMode(lower_pin, OUTPUT);
    digitalWrite(upper_pin, upper_pressure);
    digitalWrite(lower_pin, lower_pressure);
}

void Shuttle::config(uint8_t type, uint8_t rtd_pin, uint8_t ext_pin, uint8_t arm_pin, uint8_t hold_pin, uint8_t grab_pin){
    arm.config(type, rtd_pin, ext_pin, arm_pin, hold_pin, grab_pin);
    arm.init();
}

void Shuttle::scan(){
    read();
    arm.scan();
    if((upper_pressure*lower_pressure) != HIGH){
        moving = true;
    }else{
        moving = false;
    }
}

void Shuttle::update(){
    arm.update();
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
		stop_state[i] = digitalRead(stops[i]);
		if(stop_state[i] != 0){
            last_stop = i;
        }
	}
    current_stop = UNDEFINED;
    for(uint8_t i = 0; i<stops_amt; i++){
		stop_state[i] = digitalRead(stops[i]);
		if(stop_state[i] != 0){
            current_stop = last_stop;
        }
	}
}

void Shuttle::forward(){
    if(arm.get(SAFE)){
        upper_pressure  =  LOW;
        lower_pressure = HIGH;
    }else{
        stop();
        arm.retract(); 
    }
}

void Shuttle::backward(){
	if(arm.get(SAFE)){
        upper_pressure  =  HIGH;
        lower_pressure = LOW;
    }else{
        stop();
        arm.retract();
    }
}

void Shuttle::stop(){
    upper_pressure  = HIGH;
    lower_pressure = HIGH;
}

void Shuttle::move(uint8_t pos){
    if(pos>last_stop){
        arm.retract();
        forward();
    }else if(pos<last_stop){
        arm.retract();
        backward();
    }else if(pos == current_stop){
        arm.retract();
        stop();
    }
}


void Shuttle::beginDeliv(uint8_t mode){
    if(not delivering){
        if(current_stop != UNDEFINED){
            if(mode == EXTENDED){
                if(not arm.get(EXTENDED) and deliv_seq_index==0x0){
                    arm.extend();
                    deliv_seq_index = 0x1;
                }
                if(arm.get(EXTENDED) and deliv_seq_index==0x1){
                    arm.grab();
                    deliv_seq_index = 0x2;
                }
                if(arm.get(HOLDING) and wait(1000) and deliv_seq_index==0x2){
                    arm.retract();
                    delivering = true;
                    deliv_seq_index = 0x0;
                }      
            }else if(mode == RETRACTED){
                if(not arm.get(RETRACTED) and deliv_seq_index==0x0){
                    arm.retract();
                    deliv_seq_index = 0x1;
                }else{
                    deliv_seq_index = 0x1;
                }
                if(arm.get(SAFE) and wait(1000) and (deliv_seq_index==0x1)){
                    arm.grab();
                    deliv_seq_index = 0x2;
                }
                if(arm.get(HOLDING) and (deliv_seq_index==0x2)){
                    delivering = true;
                    deliv_seq_index = 0;
                }  
            }
        }
    }
}

void Shuttle::endDeliv(uint8_t mode){
    if(delivering){
        if(mode == EXTENDED){
            arm.extend();
            if(arm.get(EXTENDED) and wait(1000)){
                arm.drop();
                if(not arm.get(HOLDING)){
                    arm.retract();
                    delivering = false;
                } 
            }  
        }else if(mode == RETRACTED){
            arm.retract();
            if(arm.get(RETRACTED) and wait(1000)){
                arm.drop();
                if(not arm.get(HOLDING)){
                    arm.retract();
                    delivering = false;
                } 
            }
        }
    }
}

int16_t Shuttle::get(int8_t mode){
    if(mode == POSITION){
        return last_stop;
    }else if(mode == MOVING){
        return moving;
    }else if(mode == DELIVERING){
        return delivering;
    }else if(mode == SAFE){
        return arm.get(SAFE);
    }else if(mode == SEQUENCE_INDEX){
        return deliv_seq_index;
    }
}


Conveyor::Conveyor(uint8_t pwr, uint8_t plr, uint8_t min, uint8_t max, uint8_t tachom){
    power_relay_pin = pwr;
    polar_relay_pin = plr;
    min_sens_pin = min;
    max_sens_pin = max;
    tachometer_pin = tachom;
}

void Conveyor::init(){
    pinMode(power_relay_pin, OUTPUT);
    pinMode(polar_relay_pin, OUTPUT);
    pinMode(min_sens_pin, INPUT);
    pinMode(max_sens_pin, INPUT);
    pinMode(tachometer_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(tachometer_pin), tachometer_ISR, RISING);
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
    if(default_direction){
        direction = FORWARDS;
    }else{
        direction = BACKWARDS;
    }
}

void Conveyor::backward(){
    if(default_direction){
        direction = BACKWARDS;
    }else{
        direction = FORWARDS;
    }
}

void Conveyor::scan(){
    at_min = digitalRead(min_sens_pin);
    at_max = digitalRead(max_sens_pin); 
}

void Conveyor::update(){
    if(at_min and (not in_safety_proc and get(BACKWARDS))){
        stop(); 
        in_safety_proc = true;
        default_direction = true;
        tachometer_val = 0;
    }

    if(at_max and (not in_safety_proc and get(FORWARDS))){
        stop();
        in_safety_proc = true;
        default_direction = false;
        tachometer_val = 0;
    }

    if(in_safety_proc){
        forward();
    }

    if((at_max or at_min) and in_safety_proc){
        if(default_direction and get(FORWARDS)){
            in_safety_proc = false;
        }else if(!default_direction and get(BACKWARDS)){
            in_safety_proc = false;
        }
    }

    if(in_safety_proc){
        unsafe = true;
    }else{
        unsafe = false;
    }

    if(not moving){
        digitalWrite(power_relay_pin, LOW);
    }else{
        digitalWrite(power_relay_pin, HIGH);
    }
    
    if(direction==FORWARDS){
        digitalWrite(polar_relay_pin, LOW);
    }else{
        digitalWrite(polar_relay_pin, HIGH);
    }
}

void Conveyor::move(int16_t pos){
    if(get(SAFE)){
        start();
    }
    if(pos != (MAX or MIN) and get(SAFE)){
        if(not default_direction){
            target_pos = tachometer_max - pos;
        }else{
            target_pos = pos;
        }
        if(target_pos<=tachometer_val){
            stop();
        }else{
            forward();
        }
    }else if(pos==MIN and get(SAFE)){
        if(default_direction){
            backward();
        }else{
            forward();
        }
        if(at_min){
            stop();
        }
    }else if(pos==MAX and get(SAFE)){
        if(default_direction){
            forward();
        }else{
            backward();
        }
        if(at_max){
            stop();
        }
    }
}

int16_t Conveyor::get(int8_t mode){
    if(mode == MOVING){
        return moving;
    }else if(mode == MIN){
        return at_min;
    }else if(mode == MAX){
        return at_max;
    }else if(mode == POSITION){
        return tachometer_val;
    }else if(mode == DIRECTION){
        return direction;
    }else if(mode == DIRECTION_DEFAULT){
        return default_direction;
    }else if(mode == FORWARDS){
        return (direction == FORWARDS) ? 1:0;
    }else if(mode == BACKWARDS){
        return (direction == BACKWARDS) ? 1:0;
    }else if(mode == SAFE){
        return unsafe ? 0:1;
    }
}