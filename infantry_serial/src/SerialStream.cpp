#include "SerialStream.hpp"

using namespace stream;

void SerialOutputStream::Send()
{
	Serialize();

	int length = m_buffer.BufferSize();
	std::deque<uint8_t>* buffer = nullptr;
	uint8_t* p_data = nullptr;

	if (length!=0)
	{
		buffer = m_buffer.GetBuffer();
		p_data = new uint8_t[length];

		auto iterator = buffer->begin();
		
		for (size_t i = 0; i < length; i++, iterator++)
		{
			p_data[i] = (*iterator);
			// std::cout << "data[" << i << "]" <<(int) p_data[i] << std::endl;
		}

		p_serial->write(p_data,length);

		m_buffer << StreamBufferCMD::Clear;
		
		delete[] p_data;
	}
}

void SerialOutputStream::Serialize()
{
	auto iterator = m_registered_packet_list.begin();

	while (iterator!=m_registered_packet_list.end())
	{
		if ((*iterator)->GetUpdateFlag())
		{
			(*iterator)->SetUpdateFlag(false);
			(*iterator)->ToStream(this);
		}
		iterator++;
	}
}

void SerialInputStream::Receive()
{
	int length = p_serial->read(m_bytes_buffer, 5120);

	if (length >0)
	{
		//std::cout << "length:" << length << std::endl;

		m_buffer.ReceiveInputData(length, m_bytes_buffer);

		Deserialize();
	}
}

void SerialInputStream::Deserialize()
{
	uint8_t nulldata = 0, datalength = 0, id = 0,CRC8 = 0;
	uint8_t* p_data = nullptr;
	uint16_t Seq = 0;

	while (m_buffer.BufferSize() > 7)
	{
		m_buffer << StreamBufferCMD::BeginFlag;

		if (m_buffer.CheckSOF())
		{
			if (m_buffer.CheckCRC8(4))
			{
				m_buffer >> nulldata >> datalength >> Seq >> CRC8 >> id;

				if (m_buffer.BufferSize() < datalength + 2)
				{
					int todo_size = m_buffer.BufferSize();

					uint8_t* temp_data = new uint8_t[todo_size];

					for (int i = 0; i < todo_size; i++)
					{
						m_buffer >> temp_data[i];
					}

					m_buffer << (uint8_t)0xA5 << datalength << Seq << CRC8 << id;

					for (int i = 0; i < todo_size; i++)
					{
						m_buffer << temp_data[i];
					}
					delete[] temp_data;

					break;
				}

				auto register_packet = std::find_if(m_registered_packet_list.begin(),
					m_registered_packet_list.end(),
					[id](PacketBehaviorProto* packet) {return packet->GetID() == id; });

				if (register_packet != m_registered_packet_list.end())
				{
					//(*register_packet)->SetDataLength((uint8_t)datalength);
					(*register_packet)->FromStream(this);
					(*register_packet)->SetUpdateFlag(true);
					m_buffer >> nulldata >> nulldata;
				}
				else
				{
					std::cerr << "null id: " << (int)id << std::endl;
					for (int i = 0; i < datalength + 2; i++)
					{
						m_buffer >> nulldata;
					}
				}
			}
			else
			{
				m_buffer >> nulldata;
				std::cerr << "null CRC: " << (int)nulldata << std::endl;
			}
		}
		else
		{
			m_buffer >> nulldata;
			std::cerr << "null SOF: " << (int)nulldata<<std::endl;
		}
	}
}