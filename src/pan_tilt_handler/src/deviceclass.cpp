////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
﻿/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
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

#include "pan_tilt_handler/deviceclass.h"
#include <include/xcommunication/protocolhandler.h>
#include <include/xcommunication/usbinterface.h>
#include <include/xcommunication/serialinterface.h>
#include <include/xcommunication/streaminterface.h>

DeviceClass::DeviceClass(void)
{
}

DeviceClass::~DeviceClass(void)
{
	if (m_streamInterface)
		delete m_streamInterface;
}

/*! rief Open an IO device
 \param portInfo The info to use for opening the port
 eturn True when successful
 */
bool
DeviceClass::openPort(const XsPortInfo& portInfo)
{
	if (portInfo.isUsb())
		m_streamInterface = new UsbInterface();
	else
		m_streamInterface = new SerialInterface();

	if (m_streamInterface->open(portInfo) != XRV_OK)
		return false;
	return true;
}

/*! rief Close an IO device
 */
void
DeviceClass::close()
{
	m_streamInterface->close();
}

/*! rief Read available data from the open IO device
 \details This function will attempt to read all available data from the open device (COM port
 or USB port).
 The function will read from the device, but it won't wait for data to become available.
 \param raw A XsByteArray to where the read data will be stored.
 eturn Whether data has been read from the IO device
 */
XsResultValue
DeviceClass::readDataToBuffer(XsByteArray& raw)
{
	// always read data and append it to the cache before doing analysis
	const int maxSz = 8192;
	XsResultValue res = m_streamInterface->readData(maxSz, raw);
	if (raw.size())
		return XRV_OK;

	return res;
}

/*! rief Read all messages from the buffered read data after adding new data supplied in  rawIn
 \details This function will read all present messages in the read buffer. In order for this function
 to work, you need to call readDataToBuffer() first.
 \param rawIn The buffered data in which to search for messages
 \param messages The messages found in the data
 eturn The messages that were read.
 */
XsResultValue
DeviceClass::processBufferedData(XsByteArray& rawIn, XsMessageArray& messages)
{
	ProtocolHandler protocol;

	if (rawIn.size())
		m_dataBuffer.append(rawIn);

	int popped = 0;
	messages.clear();

	for (;;)
	{
		XsByteArray raw(m_dataBuffer.data() + popped, m_dataBuffer.size() - popped);
		XsMessage message;
		MessageLocation location = protocol.findMessage(message, raw);

		if (location.isValid())
		{
			// message is valid, remove data from cache
			popped += location.m_size + location.m_startPos;
			messages.push_back(message);
		}
		else
		{
			if (popped)
				m_dataBuffer.pop_front(popped);

			if (messages.empty())
				return XRV_TIMEOUTNODATA;

			return XRV_OK;
		}
	}
}

/*! rief Wait for the requested XsXbusMessageId
 \param xmid The message id to wait for
 \param rcv  The received message
 eturn Whether the requested message was found
 */
bool
DeviceClass::waitForMessage(XsXbusMessageId xmid, XsMessage& rcv)
{
	XsByteArray data;
	XsMessageArray msgs;
	bool foundAck = false;
	do
	{
		readDataToBuffer(data);
		processBufferedData(data, msgs);
		for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			if ((*it).getMessageId() == xmid)
			{
				foundAck = true;
				rcv = *it;
			}
	} while (!foundAck);
	return foundAck;
}

/*! rief Write a message to the IO device
 \param msg The message to write
 eturn Whether the message could be written
 */
bool
DeviceClass::writeMessage(const XsMessage& msg)
{
	XsByteArray raw;
	if (ProtocolHandler::composeMessage(raw, msg) < 0)
		return false;

	return (m_streamInterface->writeData(raw) == XRV_OK);
}

/*! rief Put a device in config mode
 eturn True when the device acknowledged config mode
 */
bool
DeviceClass::gotoConfig()
{
	XsMessage snd(XMID_GotoConfig, 0), rcv;
	writeMessage(snd);

	return waitForMessage(XMID_GotoConfigAck, rcv);
}

/*! rief Put a device in measurement mode
 eturn True when the device acknowledged measurement mode
 */
bool
DeviceClass::gotoMeasurement()
{
	XsMessage snd(XMID_GotoMeasurement, 0), rcv;
	writeMessage(snd);

	return waitForMessage(XMID_GotoMeasurementAck, rcv);
}

/*! rief Request the product code from a device
 eturn The product code when ok, otherwise an empty XsString
 */
XsString
DeviceClass::getProductCode()
{
	XsMessage snd(XMID_ReqProductCode, 0), rcv;
	writeMessage(snd);

	if (waitForMessage(XMID_ProductCode, rcv))
	{
		const char* pc = (const char*) rcv.getDataBuffer(0);
		std::string result(pc ? pc : "", rcv.getDataSize());
		std::string::size_type thingy = result.find(" ");
		if (thingy < 20)
			result.erase(result.begin() + thingy, result.end());	//lint !e534
		return XsString(result);
	}
	else
		return XsString();
}

/*! rief Request the device id from a device
 eturn The device id (XsDeviceId) when ok, otherwise an empty XsDeviceId
 */
XsDeviceId
DeviceClass::getDeviceId()
{
	XsMessage snd(XMID_ReqDid, 0), rcv;
	writeMessage(snd);

	if (waitForMessage(XMID_DeviceId, rcv))
	{
		return rcv.getDataLong();
	}
	else
		return XsDeviceId();
}

/*! rief Set the device mode of a device (outputmode and outputsettings)
 \param outputMode The XsOutputMode to set
 \param outputSettings The XsOutputSettings to set
 eturn True when successful
 */
bool
DeviceClass::setDeviceMode(const XsOutputMode& outputMode, const XsOutputSettings& outputSettings)
{
	XsMessage sndOM(XMID_SetOutputMode), sndOS(XMID_SetOutputSettings), rcv;

	sndOM.resizeData(2);
	sndOM.setDataShort((uint16_t) outputMode);
	writeMessage(sndOM);
	if (!waitForMessage(XMID_SetOutputModeAck, rcv))
		return false;

	XsMessage snd(XMID_SetOutputSettings);
	snd.resizeData(4);
	snd.setDataLong((uint32_t) outputSettings);
	writeMessage(sndOS);
	if (!waitForMessage(XMID_SetOutputSettingsAck, rcv))
		return false;

	return true;
}

/*! rief Set the output configuration of a device
 \param config An array XsOutputConfigurationArray) containing the one or multiple XsOutputConfigurations
 eturn True when successful
 */
bool
DeviceClass::setOutputConfiguration(XsOutputConfigurationArray& config)
{
	XsMessage snd(XMID_SetOutputConfiguration, 4), rcv;
	if (config.size() == 0)
	{
		snd.setDataShort((uint16_t) XDI_None, 0);
		snd.setDataShort(0, 2);
	}
	else
	{
		for (XsSize i = 0; i < (XsSize) config.size(); ++i)
		{
			snd.setDataShort((uint16_t) config[i].m_dataIdentifier, i * 4);
			snd.setDataShort(config[i].m_frequency, i * 4 + 2);
		}
	}
	writeMessage(snd);

	return waitForMessage(XMID_SetOutputConfigurationAck, rcv);
}
