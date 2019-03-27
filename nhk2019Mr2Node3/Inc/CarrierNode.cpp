/*
 * CarrierNode.cpp
 *
 *  Created on: May 30, 2018
 *      Author: yusaku
 */

#include "stm32f1xx_hal.h"
#include "solenoid_driver.h"
#include "CarrierNode.hpp"

void CarrierNode::Control(void)
{

	if(this->_m_status == CarrierStatus::shutdown
			&& this->_m_cmd != CarrierCommands::reset_cmd)
	{
		return;
	}

	switch(this->_m_cmd)
	{
	case CarrierCommands::shutdown_cmd:
		this->shutdown();
		break;

	case CarrierCommands::reset_cmd:
		this->reset();
		break;

	case CarrierCommands::disarm_cmd:
		this->unchuck_right();
		this->unchuck_left();
		this->base_retract();

		this->_m_status = CarrierStatus::disarmed;
		break;

	case CarrierCommands::pickup_cmd:
		this->chuck_right();
		this->chuck_left();
		this->_m_status = CarrierStatus::pickedup;
		break;

	case CarrierCommands::deliver_r_cmd:
		if(this->_m_status == CarrierStatus::pickedup)
		{
			this->base_retract();
			this->_m_status = CarrierStatus::delivering_r;
		}
		break;

	case CarrierCommands::deliver_l_cmd:
		//if(this->_m_status == CarrierStatus::delivered_r)
		//{
			this->base_extend();
			this->_m_status = CarrierStatus::delivering_l;
		//}
		break;

	case CarrierCommands::deliver_r_force_cmd:
		if(this->_m_status == CarrierStatus::delivering_r)
		{
			this->unchuck_right();
			this->_m_status = CarrierStatus::delivered_r;
		}
		break;

	case CarrierCommands::deliver_l_force_cmd:
		if(this->_m_status == CarrierStatus::delivering_l)
		{
			this->unchuck_left();
			this->_m_status = CarrierStatus::delivered_l;
		}
		break;
	}
}

void CarrierNode::SetCommand(const CarrierCommands command)
{
	this->_m_cmd = command;
}
const CarrierStatus CarrierNode::GetStatus(void) const
{
	return this->_m_status;
}

void CarrierNode::OnRightChuckSensorEXTInt(void)
{
	if(this->_m_status != CarrierStatus::delivering_r)
	{
		return;
	}

	this->unchuck_right();
	this->_m_status = CarrierStatus::delivered_r;
}

void CarrierNode::OnLeftChuckSensorEXTInt(void)
{
	if(this->_m_status != CarrierStatus::delivering_l)
	{
		return;
	}

	this->unchuck_left();
	this->_m_status = CarrierStatus::delivered_l;
}

void CarrierNode::shutdown(void)
{
	solenoid_disable();
	this->_m_solenoid_pattern = 0x00;
	solenoid_drive(this->_m_solenoid_pattern);

	this->_m_status = CarrierStatus::shutdown;
}

void CarrierNode::reset(void)
{
	solenoid_enable();
	this->_m_status = CarrierStatus::reset;
}

void CarrierNode::base_extend(void)
{
	this->_m_solenoid_pattern |=  BaseASolenoidPattern;
	this->_m_solenoid_pattern &= (uint8_t)~BaseBSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}

void CarrierNode::base_retract(void)
{
	this->_m_solenoid_pattern &= (uint8_t)~BaseASolenoidPattern;
	this->_m_solenoid_pattern |=  BaseBSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}

void CarrierNode::chuck_right(void)
{
	this->_m_solenoid_pattern |= RightChuckSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}

void CarrierNode::chuck_left(void)
{
	this->_m_solenoid_pattern |= LeftChuckSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}

void CarrierNode::unchuck_right(void)
{
	this->_m_solenoid_pattern &= ~RightChuckSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}

void CarrierNode::unchuck_left(void)
{
	this->_m_solenoid_pattern &= ~LeftChuckSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}


