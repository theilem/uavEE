/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <xsens/xsresultvalue.h>
#include <xsens/xsbytearray.h>
#include <xsens/xsmessagearray.h>
#include <xsens/xsdeviceid.h>
#include <xsens/xsportinfo.h>
#include <xsens/xsoutputmode.h>
#include <xsens/xsoutputsettings.h>
#include <xsens/xsoutputconfigurationarray.h>

class StreamInterface;

class DeviceClass
{
public:
	DeviceClass(void);
	~DeviceClass(void);

	bool openPort(const XsPortInfo& portInfo);
	void close();

	XsResultValue readDataToBuffer(XsByteArray& raw);
	XsResultValue processBufferedData(XsByteArray& rawIn, XsMessageArray& messages);
	bool waitForMessage(XsXbusMessageId xmid, XsMessage& rcv);
	bool writeMessage(const XsMessage& msg);
	bool gotoConfig();
	bool gotoMeasurement();
	XsString getProductCode();
	XsDeviceId getDeviceId();
	bool setDeviceMode(const XsOutputMode& outputMode, const XsOutputSettings& outputSettings);
	bool setOutputConfiguration(XsOutputConfigurationArray& config);

private:
	StreamInterface *m_streamInterface;
	XsByteArray m_dataBuffer;
};

