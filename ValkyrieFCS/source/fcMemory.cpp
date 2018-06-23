

/* Project Includes */
#include <ValkyrieFCS/include/fcMemory.hpp>

namespace FCS
{
	FCSettings fcSettings;


	FCSettings::Status FCSettings::initialize(Adesto::FlashChip chip, const int& spiChannel, uint32_t clockFreq)
	{
		flash = Chimera::make_unique<Adesto::NORFlash::AT45>(chip, spiChannel);

		if (flash->initialize(clockFreq) != Adesto::FLASH_OK)
			return FLASH_INIT_FAIL;

		return FLASH_INIT_SUCCESS;
	}


	int FCSettings::write(uint32_t address, uint8_t* in, size_t len)
	{
		return 0;
	}

	int FCSettings::read(uint32_t address, uint8_t* out, size_t len)
	{
		return 0;
	}

}