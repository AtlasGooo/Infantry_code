#include "PacketBehavior.hpp"
#include "LogStream.hpp"
#include "SerialStream.hpp"

using namespace stream;

void LogPacketBehavior::ToStream(Stream<StreamType::PacketStreamType>* stream)
{
	auto true_stream = dynamic_cast<LogOutputStream*>(stream);

	auto unit_iterator = m_model->GetUnitList()->begin();
	auto end = m_model->GetUnitList()->end();

	true_stream->output_file_stream.open("/home/spaceman/Documents/Test/"+m_file_name+".csv",std::ofstream::app);

	if (true_stream->output_file_stream.is_open())
	{
		ToStream(&true_stream->output_file_stream);

		true_stream->output_file_stream.flush(); 
		true_stream->output_file_stream.close();
	}
}

void LogPacketBehavior::FromStream(Stream<StreamType::PacketStreamType>* stream)
{

}

void LogPacketBehavior::ToStream(std::ofstream *ofstream)
{
	auto iterator = m_model->GetUnitList()->begin();
	auto end = m_model->GetUnitList()->end();

	ReplacementUnit unit;

	while (iterator != end)
	{
		(*iterator)->ToStream(&unit);

		switch (unit.update_type)
		{
		case BaseTypeEnum::uint8:
			(*ofstream) << unit.base_union.u8;
			break;
		case BaseTypeEnum::int8:
			(*ofstream) << unit.base_union.i8;
			break;
		case BaseTypeEnum::uint16:
			(*ofstream) << unit.base_union.u16;
			break;
		case BaseTypeEnum::int16:
			(*ofstream) << unit.base_union.i16;
			break;
		case BaseTypeEnum::uint32:
			(*ofstream) << unit.base_union.u32;
			break;
		case BaseTypeEnum::int32:
			(*ofstream) << unit.base_union.i32;
			break;
		case BaseTypeEnum::fp32:
			(*ofstream) << unit.base_union.fp32;
			break;
		case BaseTypeEnum::fp64:
			(*ofstream) << unit.base_union.fp64;
			break;
		default:
			std::cerr<<"LogStream toStream null type"<<std::endl;
			break;
		}

		iterator++;

		if(iterator!=end)
		{
			(*ofstream)<<",";
		}
	}
	(*ofstream)<<"\n";
}

void LogPacketBehavior::FromStream(std::ifstream *ifstream)
{
	;
}


void SerialPacketBehavior::ToStream(Stream<StreamType::BytesStreamType>* stream)
{
	stream->m_buffer <<StreamBufferCMD::BeginFlag<< (uint8_t)0xA5<< (uint8_t)GetDataLength()
		<< (uint16_t)0<< StreamBufferCMD::CRC8<<(uint8_t)GetID();

	if (stream->m_buffer.GetFloatCompressFlag())
	{
		stream->m_buffer << StreamBufferCMD::FloatMin << m_float_min << StreamBufferCMD::FloatPre
			<< m_float_pre;
	}

	PacketBehaviorProto::ToStream(stream);

	stream->m_buffer << StreamBufferCMD::CRC16;
}

void RefereePacketBehavior::ToStream(Stream<StreamType::BytesStreamType>* stream)
{
	stream->m_buffer << StreamBufferCMD::BeginFlag << (uint8_t)0xA5 << (uint16_t)GetDataLength()
		<< (uint8_t)0 << StreamBufferCMD::CRC8 << (uint16_t)GetID();

	PacketBehaviorProto::ToStream(stream);

	stream->m_buffer << StreamBufferCMD::CRC16;
}
