#include "StreamBytesBuffer.hpp"

using namespace stream;

StreamBytesBuffer::StreamBytesBuffer() :m_cmd(StreamBufferCMD::NullCMD),
	m_float_min(1.0f),m_float_pre(1.0f),
	m_float_compress_flag(false),m_CRC_todo_data_count(0)
{
	;
}

void StreamBytesBuffer::AddCRC(StreamBufferCMD cmd)
{
	uint8_t* p_temp_data = nullptr;
	std::deque<uint8_t>::iterator it;

	p_temp_data = new uint8_t[m_CRC_todo_data_count];
	it = m_buffer.end() - m_CRC_todo_data_count;

	for (int i = 0; i < m_CRC_todo_data_count; i++, it++)
	{
		p_temp_data[i] = *(it);
	}

	if (cmd == StreamBufferCMD::CRC8)
	{
		(*this) << Crc::Get_CRC8_Check_Sum(p_temp_data, m_CRC_todo_data_count, Crc::CRC8_INIT);
	}
	else if (cmd == StreamBufferCMD::CRC16)
	{
		(*this) << Crc::Get_CRC16_Check_Sum(p_temp_data, m_CRC_todo_data_count, Crc::CRC16_INIT);
	}
	delete[] p_temp_data;
}

StreamBytesBuffer& StreamBytesBuffer::operator<<(StreamBufferCMD cmd)
{
	switch (cmd)
	{
	case StreamBufferCMD::BeginFlag:
		m_CRC_todo_data_count = 0;
		break;
	case StreamBufferCMD::Clear:
		m_buffer.clear();
		break;
	case StreamBufferCMD::CRC8:
	case StreamBufferCMD::CRC16:
		AddCRC(cmd);
		break;
	case StreamBufferCMD::FloatCompress:
		m_float_compress_flag = true;
		break;
	case StreamBufferCMD::FloatNoCompress:
		m_float_compress_flag = false;
		break;
	case StreamBufferCMD::FloatMin:
	case StreamBufferCMD::FloatPre:
	case StreamBufferCMD::NullCMD:
		m_cmd = cmd;
		break;
	default:
		break;
	}
	return (*this);
}

StreamBytesBuffer& StreamBytesBuffer::operator<<(StreamBytesBuffer& object)
{
	m_buffer.insert(m_buffer.end(),object.m_buffer.begin(),object.m_buffer.end());
	return (*this);
}

StreamBytesBuffer& StreamBytesBuffer::operator>>(StreamBytesBuffer& object)
{
	object.m_buffer.insert(object.m_buffer.end(),m_buffer.begin(),m_buffer.end());
	return (*this);
}

bool StreamBytesBuffer::CheckSOF()
{
	if (m_buffer.size()!=0)
	{
		if ((*m_buffer.begin())==0xA5)
		{
			return true;
		}
	}
	return false;
}

static int error_count = 0;

bool StreamBytesBuffer::CheckCRC8(uint8_t size)
{
	if (m_buffer.size() > size)
	{
		uint8_t* temp_data = new uint8_t[size + 1];

		auto it = m_buffer.begin();
		bool result;

		for (int i = 0; i < size + 1; i++, it++)
		{
			temp_data[i] = (*it);
		}

		result = Crc::VerifyCrc8CheckSum(temp_data, size + 1);
		delete[] temp_data;

		if (!result)
		{
			error_count++;
		}

		return result;
	}
	return false;
}

bool StreamBytesBuffer::CheckCRC16(uint8_t size)
{
	if (m_buffer.size() > (size+1))
	{
		uint8_t* temp_data = new uint8_t[size + 2];

		auto it = m_buffer.begin();
		bool result;

		for (int i = 0; i < size + 2; i++, it++)
		{
			temp_data[i] = (*it);
		}

		result = Crc::VerifyCrc8CheckSum(temp_data, size + 2);
		delete[] temp_data;
		return result;
	}
	return false;
}

void StreamBytesBuffer::ReceiveInputData(int length, uint8_t* ptr)
{
	for (; length > 0; length--, ptr++)
	{
		m_buffer.push_back(*ptr);
	}
}