#pragma once
#ifndef FC_MEMORY_HPP
#define FC_MEMORY_HPP

/* Project Includes */
#include <ValkyrieFCS/include/fcTypes.hpp>

/* Device Driver Includes */
#include "at45db081.hpp"


namespace FCS
{
	enum MemoryObject : uint8_t
	{
		PID_TERMS_ROLL,
		PID_TERMS_PITCH,
		PID_TERMS_YAW,
		MAX_OBJECTS_IN_MEM
	};

	/** Stores the current address in Flash Memory of a specific object referenced by FCS::MemoryObject. Initial
	 *	values are set in assignFlashAddress() in fcMemory.cpp
	 *	@note This object is not const and can be changed at run time. 
	 **/
	extern uint32_t flashMemObjAddr[MemoryObject::MAX_OBJECTS_IN_MEM];

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
			FLASH_OK,
			FLASH_INIT_SUCCESS,
			FLASH_INIT_FAIL,
			FLASH_MAX_ADDRESS_EXCEEDED,
			FLASH_PROGRAMMING_ERROR,
		};

		/**	Starts up the low level interface to the specified flash chip
		 *	@param[in]	chip		Which chip expected to connect to 
		 *	@param[in]	spiChannel	Which spi channel to use
		 *	@param[in]	clockFreq	Desired spi clock frequency (may not be exact)
		 *	@return	Status code from FCSettings::Status. If successful, returns FLASH_INIT_SUCCESS
		 **/
		Status initialize(Adesto::FlashChip chip, const int& spiChannel, uint32_t clockFreq);

		/**	Grabs a specific object from flash memory using a pointer style output format
		 *	@param[in]	objType	The type of object to read as specified in FCS::MemoryObject
		 *	@param[out]	objAddr	Address of where to write the flash memory data to
		 *	@param[in]	objLen	The size of the object in bytes
		 *	@return	Success or error message from FCS::FCSettings::Status
		 **/
		Status get(MemoryObject objType, uint8_t* objAddr, size_t objLen);

		/**	A far more generic version of "get" that is intended to be used with more complex data types.
		 *	Automatic reference conversion and sizing information is supported.
		 *	@param[in]	objType	The type of object to read as specified in FCS::MemoryObject
		 *	@param[out]	obj		Reference to the actual object to be read into
		 *	@return	Success or error message from FCS::FCSettings::Status
		 **/
		template<typename T>
		Status get(MemoryObject objType, T& obj)
		{
			return get(objType, reinterpret_cast<uint8_t*>(&obj), sizeof(T));
		}

		/**	Writes a specific object into flash memory using a pointer style input format 
		*	@param[in]	objType	The type of object to write as specified in FCS::MemoryObject
		*	@param[in]	objAddr	Address of where to write the flash memory data from
		*	@param[in]	objLen	The size of the object in bytes
		*	@return	Success or error message from FCS::FCSettings::Status
		**/
		Status set(MemoryObject objType, uint8_t* objAddr, size_t objLen);

		/**	A far more generic version of "set" that is intended to be used with more complex data types.
		*	Automatic reference conversion and sizing information is supported.
		*	@param[in]	objType	The type of object to write as specified in FCS::MemoryObject
		*	@param[in]	obj		Reference to the actual object to write from
		*	@return	Success or error message from FCS::FCSettings::Status
		**/
		template<typename T>
		Status set(MemoryObject objType, T& obj)
		{
			return set(objType, reinterpret_cast<uint8_t*>(&obj), sizeof(T));
		}

		/** Gets the max valid address value for the current flash chip
		*	@return largest address chip supports
		**/
		uint32_t maxValidAddress();

		FCSettings() = default;
		~FCSettings() = default;

	private:
		Adesto::NORFlash::AT45_uPtr flash;
		uint32_t maxFlashAddress = 0;


		/** Writes data to a given address
		 *	@param[in]	address	Starting address to write at
		 *	@param[in]	in		Address of data to be written
		 *	@param[in]	len		Number of bytes to write
		 **/
		int write(uint32_t address, uint8_t* in, size_t len);

		/**	Reads data starting from a given address
		 *	@param[in]	address	Starting address to read from
		 *	@param[out]	out		Address of data to write to
		 *	@param[in]	len		Number of bytes to read
		 **/
		int read(uint32_t address, uint8_t* out, size_t len);

		
	};

	extern FCSettings fcSettings;
}


#endif /* !FC_MEMORY_HPP */