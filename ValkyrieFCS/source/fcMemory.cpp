

/* Project Includes */
#include <ValkyrieFCS/include/fcMemory.hpp>

#define FLASH_PID_BASE		0x00000000



namespace FCS
{
	FCSettings fcSettings;
	uint32_t flashMemObjAddr[MemoryObject::MAX_OBJECTS_IN_MEM];

	void assignFlashAddress()
	{
		flashMemObjAddr[MemoryObject::PID_TERMS_ROLL]	= FLASH_PID_BASE;
		flashMemObjAddr[MemoryObject::PID_TERMS_PITCH]	= FLASH_PID_BASE + 1 * sizeof(PID::PIDSettings);
		flashMemObjAddr[MemoryObject::PID_TERMS_YAW]	= FLASH_PID_BASE + 2 * sizeof(PID::PIDSettings);

		
	}

	FCSettings::Status FCSettings::initialize(Adesto::FlashChip chip, const int& spiChannel, uint32_t clockFreq)
	{
		assignFlashAddress();

		flash = Chimera::make_unique<Adesto::NORFlash::AT45>(chip, spiChannel);

		if (flash->initialize(clockFreq) != Adesto::FLASH_OK)
			return FLASH_INIT_FAIL;

		maxFlashAddress = flash->getFlashSize();

		return FLASH_INIT_SUCCESS;
	}

	FCSettings::Status FCSettings::get(MemoryObject objType, uint8_t* objAddr, size_t objLen)
	{
		if ((flashMemObjAddr[objType] + objLen) > maxFlashAddress)
			return FLASH_MAX_ADDRESS_EXCEEDED;

		read(flashMemObjAddr[objType], objAddr, objLen);

		while (!flash->isReadComplete())
		{
			Chimera::delayMilliseconds(1);
		}

		return FLASH_OK;
	}

	FCSettings::Status FCSettings::set(MemoryObject objType, uint8_t* objAddr, size_t objLen)
	{
		if ((flashMemObjAddr[objType] + objLen) > maxFlashAddress)
			return FLASH_MAX_ADDRESS_EXCEEDED;

		write(flashMemObjAddr[objType], objAddr, objLen);

		while (!flash->isWriteComplete())
		{
			Chimera::delayMilliseconds(1);
		}

		if (flash->isErasePgmError())
			return FLASH_PROGRAMMING_ERROR;

		return FLASH_OK;
	}

	uint32_t FCSettings::maxValidAddress()
	{
		return flash->getFlashSize();
	}

	void FCSettings::write(uint32_t address, uint8_t* in, size_t len)
	{
		flash->write(address, in, len);
	}

	void FCSettings::read(uint32_t address, uint8_t* out, size_t len)
	{
		flash->read(address, out, len);
	}
	

}