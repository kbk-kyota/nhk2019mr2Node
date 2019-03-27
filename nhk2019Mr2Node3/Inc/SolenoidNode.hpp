#ifndef SOLENOIDNODE_HPP_
#define SOLENOIDNODE_HPP_

enum class SolenoidStatus : uint16_t//�����͏�ԕ\���݂����Ȃ��́H
{
	shutdown			= 0x0000,//16�i���\�L�@��������ES�łȂ���H
	reset				= 0x0001,//�����炪������ԂƂ�����H

	sensing				= 0x0020,//�Z���T�ŕ������m����H

	apploaching_s		= 0x010,//�܂����܂��ĂȂ��̂œK��
	apploaching_f		= 0x011,//ry
	apploaching_a		= 0x012,//ry
	picked_s			= 0x013,//ry
	picked_f_1			= 0x014,//ry
	picked_f_2			= 0x015,//ry
	up_s				= 0x016,//ry
	up_f_1				= 0x017,//ry
	up_f_2				= 0x018,//ry
	setting_s				= 0x019,//RY
};

enum class SolenoidCommands : uint16_t//���炭�������d���قɒ��ڑ����Ă���M������
{//�d���قɑ�����Ƃ���2�i���\�L�ő�����
//�Ȃ񂩂��ꂪ���ړd���قɑ����Ă���킯�ł͂Ȃ��݂���
	shutdown_cmd		= 0x0000,
	reset_cmd			= 0x0001,

	apploaching_s_cmd		= 0x020,//�O��֐ڋ�
	apploaching_f_cmd		= 0x021,//�؂̎��֐ڋ�
	apploaching_a_cmd		= 0x022,//������֐ڋ�
	picked_s_cmd			= 0x023,//�O��c��
	picked_f_1_cmd			= 0x024,//�؂̎��c��1
	picked_f_2_cmd			= 0x025,//�؂̎��c��2
	up_s_cmd				= 0x026,//�O��u�����
	up_f_1_cmd				= 0x027,//�؂̎����[1
	up_f_2_cmd				= 0x028,//�؂̎����[2
	setting_s_cmd				= 0x029,//�O��u��
};

class SolenoidNode
{
public:
	void Control(void);
	void SetCommand(const SolenoidCommands command);
	const SolenoidStatus GetStatus(void) const;
/*
	void OnRightChuckSensorEXTInt(void);//����͉��H
	void OnLeftChuckSensorEXTInt(void);
*/
private:
	void shutdown(void);
	void reset(void);

	//�����Ɉ�A�̓�����֐��\�L����΂����H
	void catch_s(void);
	void uncatch_s(void);
	void catch_f(void);
	void uncatch_f(void);
	void s_arm_extend(void);
	void s_arm_retract(void);
	void f_arm_down(void);
	void f_arm_up(void);
	void push(void);
	void pull(void);

	uint8_t _m_solenoid_pattern = 0x00;

	SolenoidCommands _m_cmd = SolenoidCommands::shutdown_cmd;
	SolenoidStatus _m_status = SolenoidStatus::shutdown;
//���������Ă��������\���m�C�h�ɑ����Ă����H
//CarrierNode.cpp��solenoid_driver�ɂ������Ă�̂��������������@�܂��H
//�M���͂܂����܂��ĂȂ��̂œK��
/*	static constexpr uint8_t RightChuckSolenoidPattern 	= 0x01;
	static constexpr uint8_t LeftChuckSolenoidPattern 	= 0x02;//10
	static constexpr uint8_t BaseASolenoidPattern 		= 0x04;//100
	static constexpr uint8_t BaseBSolenoidPattern 		= 0x08;//1000
*/
//������ւ񂩂�
	static constexpr uint8_t SArmCatchSolenoidPattern 	= 0x01;
	static constexpr uint8_t SArmCatchSolenoidPattern2 	= 0x20;
	static constexpr uint8_t SArmExtendSolenoidPattern 	= 0x02;
	static constexpr uint8_t FArmCatchSolenoidPattern 	= 0x04;
	static constexpr uint8_t FArmExtendSolenoidPattern 	= 0x08;
	static constexpr uint8_t SPushSolenoidPattern 		= 0x16;
};



#endif /* SOLENOIDNODE_HPP_ */
