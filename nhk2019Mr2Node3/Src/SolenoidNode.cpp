/*
 * SolenoidNode.cpp
 *
 *  Created on: May 30, 2018
 *      Author: yusaku
 */
//���̃v���O�����͋��炭�M�����󂯎���Ă��̐M������d���قɑ���M�������߂Ă���Ƃ���
//���ہA���̕��͂̉��̕�������ƁAsolenoid_driver.c�ɐM���𑗂��Ă���
#include "solenoid_driver.h"
#include "stm32f1xx_hal.h"
#include "SolenoidNode.hpp"

void SolenoidNode::Control(void)//�����͋��炭CAN�ʐM�ł����M������A�ǂ̐M����solenoid_driver.c�ɑ��邩�������Ă���Ƃ���
{//�����͂����M���ɑ΂��Ă̏������Ǝv���̂ŁA�����Ɏ�����̋@�\�̓z�����������΂����H

	if(this->_m_status == SolenoidStatus::shutdown
			&& this->_m_cmd != SolenoidCommands::reset_cmd)
	{
		return;
	}

	switch(this->_m_cmd)//�����ɏ��������΂����H
	{
	case SolenoidCommands::shutdown_cmd:
		this->shutdown();
		break;

	case SolenoidCommands::reset_cmd:
		this->reset();
		break;
	//�����Ɏ����̂����������čŌ��NHK�̓z���R�����g�A�E�g���Ȃ񂩂��Ă���
	case SolenoidCommands::apploaching_s_cmd://�����͏����[���瓮��
		//�����Ŏ��s�����݂����̂�����H
		this->s_arm_extend();
		this->_m_status = SolenoidStatus::apploaching_s;
		break;

	case SolenoidCommands::picked_s_cmd:
		if(this->_m_status == SolenoidStatus::apploaching_s)
		{
			this->catch_s();
			this->_m_status = SolenoidStatus::picked_s;
		}
		break;

	case SolenoidCommands::up_s_cmd:
		if(this->_m_status == SolenoidStatus::picked_s)
		{
			this->s_arm_retract();
			this->_m_status = SolenoidStatus::up_s;
		}
		break;

	case SolenoidCommands::apploaching_f_cmd:
		if(this->_m_status == SolenoidStatus::up_s)
		{
			this->uncatch_f();
			this->_m_status = SolenoidStatus::apploaching_f;
		}
		break;

	case SolenoidCommands::picked_f_1_cmd:
		if(this->_m_status == SolenoidStatus::apploaching_f ||this->_m_status == SolenoidStatus::up_f_2)
		{
			this->f_arm_down();
			this->_m_status = SolenoidStatus::picked_f_1;
		}
		break;

	case SolenoidCommands::picked_f_2_cmd:
		if(this->_m_status == SolenoidStatus::picked_f_1)
		{
			this->catch_f();
			this->_m_status = SolenoidStatus::picked_f_2;
		}
		break;

	case SolenoidCommands::up_f_1_cmd:
		if(this->_m_status == SolenoidStatus::picked_f_2)
		{
			this->f_arm_up();
			this->_m_status = SolenoidStatus::up_f_1;
		}
		break;

	case SolenoidCommands::up_f_2_cmd:
		if(this->_m_status == SolenoidStatus::up_f_1)
		{
			this->uncatch_f();
			this->_m_status = SolenoidStatus::up_f_2;
		}
		break;

	case SolenoidCommands::apploaching_a_cmd:
		if(this->_m_status == SolenoidStatus::up_f_2)
		{
			this->uncatch_s();
			this->_m_status == SolenoidStatus::apploaching_a;
		}
		break;

	case SolenoidCommands::setting_s_cmd:
		if(this->_m_status == SolenoidStatus::apploaching_a)
		{
			this->push();
			this->_m_status = SolenoidStatus::setting_s;
		}
		break;
	}
}

void SolenoidNode::SetCommand(const SolenoidCommands command)
{
	this->_m_cmd = command;
}
const SolenoidStatus SolenoidNode::GetStatus(void) const
{
	return this->_m_status;
}

void SolenoidNode::shutdown(void)
{
	solenoid_disable();
	this->_m_solenoid_pattern = 0x00;
	solenoid_drive(this->_m_solenoid_pattern);

	this->_m_status = SolenoidStatus::shutdown;
}

void SolenoidNode::reset(void)
{
	solenoid_enable();
	this->_m_status = SolenoidStatus::reset;
}
//��̂Ɠ����悤��SolenoidNode�ŏ������֐��̒��g���L�q���Ă����H
void SolenoidNode::catch_s(void)
{
	this->_m_solenoid_pattern |= SArmCatchSolenoidPattern;
	this->_m_solenoid_pattern &= ~SArmCatchSolenoidPattern2;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::uncatch_s(void)
{
	this->_m_solenoid_pattern &= ~SArmCatchSolenoidPattern;
	this->_m_solenoid_pattern |= SArmCatchSolenoidPattern2;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::catch_f(void)
{
	this->_m_solenoid_pattern |= FArmCatchSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::uncatch_f(void)
{
	this->_m_solenoid_pattern &= ~FArmCatchSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::s_arm_extend(void)
{
	this->_m_solenoid_pattern |= SArmExtendSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::s_arm_retract(void)
{
	this->_m_solenoid_pattern &= ~SArmExtendSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::f_arm_down(void)
{
	this->_m_solenoid_pattern |= FArmExtendSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::f_arm_up(void)
{
	this->_m_solenoid_pattern &= ~FArmExtendSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::push(void)
{
	this->_m_solenoid_pattern |= SPushSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}
void SolenoidNode::pull(void)
{
	this->_m_solenoid_pattern &= ~SPushSolenoidPattern;
	solenoid_drive(this->_m_solenoid_pattern);
}


