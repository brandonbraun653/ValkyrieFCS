#pragma once
#ifndef FCS_TYPES_HPP
#define FCS_TYPES_HPP

namespace FCS
{
	namespace PID
	{
		struct __attribute__((packed)) PIDSettings
		{
			float kp = 0.0f;
			float ki = 0.0f;
			float kd = 0.0f;
		};
	}
}

#endif /* !FCS_TYPES_HPP */