/*
 * CarrierNode.hpp
 *
 *  Created on: May 30, 2018
 *      Author: yusaku
 */

#ifndef CARRIERNODE_HPP_
#define CARRIERNODE_HPP_

enum class CarrierStatus : uint16_t
{
	shutdown			= 0x0000,
	reset				= 0x0001,

	sensing				= 0x0020,

	//operational			= 0x0010,
	disarmed			= 0x0010,
	pickedup			= 0x0011,
	delivered_r			= 0x0012,
	delivered_l			= 0x0013,

	delivering_r		= 0x0081,
	delivering_l		= 0x0082,
};

enum class CarrierCommands : uint16_t
{
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,

	disarm_cmd			= 0x0010,
	pickup_cmd			= 0x0011,

	deliver_r_cmd		= 0x0020,
	deliver_l_cmd		= 0x0021,
	deliver_r_force_cmd	= 0x0022,
	deliver_l_force_cmd	= 0x0023,
};

class CarrierNode
{
public:
	void Control(void);
	void SetCommand(const CarrierCommands command);
	const CarrierStatus GetStatus(void) const;

	void OnRightChuckSensorEXTInt(void);
	void OnLeftChuckSensorEXTInt(void);

private:
	void shutdown(void);
	void reset(void);

	void base_extend(void);
	void base_retract(void);
	void chuck_right(void);
	void chuck_left(void);
	void unchuck_right(void);
	void unchuck_left(void);

	uint8_t _m_solenoid_pattern = 0x00;

	CarrierCommands _m_cmd = CarrierCommands::shutdown_cmd;
	CarrierStatus _m_status = CarrierStatus::shutdown;

	static constexpr uint8_t RightChuckSolenoidPattern 	= 0x01;
	static constexpr uint8_t LeftChuckSolenoidPattern 	= 0x02;
	static constexpr uint8_t BaseASolenoidPattern 		= 0x04;
	static constexpr uint8_t BaseBSolenoidPattern 		= 0x08;
};



#endif /* CARRIERNODE_HPP_ */
