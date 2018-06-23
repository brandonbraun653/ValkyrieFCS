#pragma once
#ifndef FC_MEMORY_HPP
#define FC_MEMORY_HPP

#include "at45db081.hpp"


namespace FCS
{
	/** This class is intended to provide an easy to use interface for getting and setting 
	 *	Flight Controller parameters stored in flash memory. Examples of such parameters could
	 *	be PID constants, Kalman Filter matrices, ESC max/min commands, etc etc.
	 *
	 *	@note	Currently only the AT45 line of flash chips from Adesto are supported, but the class
	 *			can easily be modified to use a different device. Simply modify the stubs for read/write.
	 **/
	class FCSettings
	{
	public:
		enum Status
		{
			FLASH_INIT_SUCCESS,
			FLASH_INIT_FAIL,
		};

		/**	Starts up the low level interface to the specified flash chip
		 *	@param[in]	chip		Which chip expected to connect to 
		 *	@param[in]	spiChannel	Which spi channel to use
		 *	@param[in]	clockFreq	Desired spi clock frequency (may not be exact)
		 *	@return	Status code from FCSettings::Status. If successful, returns FLASH_INIT_SUCCESS
		 **/
		Status initialize(Adesto::FlashChip chip, const int& spiChannel, uint32_t clockFreq);

		FCSettings() = default;
		~FCSettings() = default;

	private:
		Adesto::NORFlash::AT45_uPtr flash;


		/** Writes data to a given address
		 *	@param[in]	address	Starting address to write at
		 *	@param[in]	in		Address of data to be written
		 *	@param[in]	len		Number of bytes to write
		 *	@return Number of bytes actually written
		 **/
		int write(uint32_t address, uint8_t* in, size_t len);

		/**	Reads data starting from a given address
		 *	@param[in]	address	Starting address to read from
		 *	@param[out]	out		Address of data to write to
		 *	@param[in]	len		Number of bytes to read
		 *	@return Number of bytes actually read
		 **/
		int read(uint32_t address, uint8_t* out, size_t len);
	};

	extern FCSettings fcSettings;
}


#endif /* !FC_MEMORY_HPP */