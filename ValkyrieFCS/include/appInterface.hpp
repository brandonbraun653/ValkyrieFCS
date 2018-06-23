#pragma once
#ifndef FCS_APPINTERFACE_HPP
#define FCS_APPINTERFACE_HPP


namespace FCS
{
	class FCSApp
	{
	public:

		void parseCommand(/* command typedef */);
		void sendResponse(/* packet type? should probably define a specific TX/RX packet with error correction */);

		FCSApp() = default;
		~FCSApp() = default;

	private:

	};
}

#endif /* !FCS_APPINTERFACE_HPP */