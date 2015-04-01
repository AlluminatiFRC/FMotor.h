#include "WPILib.h"

#ifndef SRC_FMOTOR_H_
#define SRC_FMOTOR_H_

class FMotor: public Controller, public SpeedController {
private:
	PIDOutput* motor;
	float factor; //Filter coefficient
	float target = 0.0f; //Target value
	float current = 0.0f;
public:
	FMotor(float fact, PIDOutput* mot) :
			motor(mot) {
		if (fact >= 1.0f)
			fact = 1.0f; //Don't allow anything that could cause a runaway
		factor = fact;
	}

	virtual void Set(float val, uint8_t syncGroup = 0) {
		SetTarget(val);
		update();
	}

	virtual void SetTarget(float val, uint8_t syncGroup = 0) {
		target = val;
	}

	virtual float Get() {
		return current;
	}

	virtual void Disable() {
		target = 0;
		motor->PIDWrite(0);
	}

	virtual void PIDWrite (float output){
		target = output;
	}

	virtual void Enable() {
	}

	void Halt(){
		target = 0;
		motor->PIDWrite(0);
	}

	virtual void update() {
		current += factor * (target - current);
		motor->PIDWrite(current);
	}
};

#endif /* SRC_FMOTOR_H_ */
