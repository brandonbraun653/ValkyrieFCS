/* Chimera Includes */
#include <Chimera/logging.hpp>


/* Project Includes */
#include <ValkyrieFCS/include/fcMemory.hpp>

#define FLASH_PID_BASE		0x00000000

using namespace Chimera::Logging;

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
		{
			Console.log(Level::ERROR, "Address invalid for accessing current flash chip. Max: %d, Requested: %d\r\n",
				maxFlashAddress, (flashMemObjAddr[objType] + objLen));
			return FLASH_MAX_ADDRESS_EXCEEDED;
		}

		Adesto::Status errorCode = static_cast<Adesto::Status>(read(flashMemObjAddr[objType], objAddr, objLen));

		if (errorCode != Adesto::Status::FLASH_OK)
		{
			//Left as a placeholder for now. Waiting for more development on the actual read function first.
			Console.log(Level::ERROR, "Unhandled error on flash read\r\n");
		}

		return FLASH_OK;
	}

	FCSettings::Status FCSettings::set(MemoryObject objType, uint8_t* objAddr, size_t objLen)
	{
		if ((flashMemObjAddr[objType] + objLen) > maxFlashAddress)
		{
			Console.log(Level::ERROR, "Address invalid for accessing current flash chip. Max: %d, Requested: %d\r\n", 
				maxFlashAddress, (flashMemObjAddr[objType] + objLen));
			return FLASH_MAX_ADDRESS_EXCEEDED;
		}
			
		Adesto::Status errorCode = static_cast<Adesto::Status>(write(flashMemObjAddr[objType], objAddr, objLen));
		
		if (errorCode != Adesto::Status::FLASH_OK)
		{
			switch (errorCode)
			{
			case Adesto::Status::ERROR_WRITE_FAILURE:
				Console.log(Level::ERROR, "Failed writing FCS parameter to flash memory!\r\n");
				return FLASH_PROGRAMMING_ERROR;
				break;

			//Add more as needed.

			default: break;
			}
		}
			
		return FLASH_OK;
	}

	uint32_t FCSettings::maxValidAddress()
	{
		return flash->getFlashSize();
	}

	int FCSettings::write(uint32_t address, uint8_t* in, size_t len)
	{
		return (int)flash->write(address, in, len);
	}

	int FCSettings::read(uint32_t address, uint8_t* out, size_t len)
	{
		return (int)flash->read(address, out, len);
	}
	

}