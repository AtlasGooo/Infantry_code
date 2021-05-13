#ifndef STREAM_BYTES_BUFFER_HPP
#define STREAM_BYTES_BUFFER_HPP

#include <stdint.h>
#include <typeinfo>
#include <vector>
#include <deque>
#include <string>
#include <list>
#include <algorithm>
#include <iostream>
#include "Crc.hpp"

namespace stream
{
	class StreamBytesBuffer;

	template <class T>
	struct DataUnit;

	enum class StreamBufferCMD
	{
		BeginFlag,
		Clear,
		CRC8,
		CRC16,
		FloatCompress,
		FloatNoCompress,
		FloatMin,
		FloatPre,
		NullCMD,
	};

	enum class BaseTypeEnum
	{
		uint8,
		int8,
		uint16,
		int16,
		uint32,
		int32,
		fp32,
		fp64,
		null
	};

	union BaseTypeUnion
	{
		uint8_t u8;
		int8_t i8;
		uint16_t u16;
		int16_t i16;
		uint32_t u32;
		int32_t i32;
		float fp32;
		double fp64;
	};

	struct ReplacementUnit
	{
		ReplacementUnit() : update_type(BaseTypeEnum::null) { ; }

		BaseTypeUnion base_union;
		BaseTypeEnum update_type;
	};

	struct DataUnitProto
	{
		virtual void FromStream(DataUnitProto *unit_proto) = 0;
		virtual void FromStream(StreamBytesBuffer *buffer) = 0;
		virtual void FromStream(ReplacementUnit *replacement_unit) = 0;
		virtual void ToStream(DataUnitProto *unit_proto) = 0;
		virtual void ToStream(StreamBytesBuffer *unit_proto) = 0;
		virtual void ToStream(ReplacementUnit *replacement_unit) = 0;
	};

	class StreamBytesBuffer
	{
	public:
		StreamBytesBuffer();
		~StreamBytesBuffer() { ; }

		StreamBytesBuffer &operator<<(StreamBufferCMD);
		StreamBytesBuffer &operator<<(StreamBytesBuffer &object);
		StreamBytesBuffer &operator>>(StreamBytesBuffer &object);

		template <typename T>
		StreamBytesBuffer &operator<<(T *object);

		template <typename T>
		StreamBytesBuffer &operator<<(T object);

		template <typename T>
		StreamBytesBuffer &operator<<(DataUnit<T> &object);

		template <typename T>
		StreamBytesBuffer &operator>>(T *object);

		template <typename T>
		StreamBytesBuffer &operator>>(T &object);

		template <typename T>
		StreamBytesBuffer &operator>>(DataUnit<T> &object);

		bool CheckSOF();
		bool CheckCRC8(uint8_t size);
		bool CheckCRC16(uint8_t size);

		bool GetFloatCompressFlag() { return m_float_compress_flag; }

		int BufferSize() { return m_buffer.size(); }
		std::deque<uint8_t> *GetBuffer() { return &m_buffer; }

		void ReceiveInputData(int length, uint8_t *ptr);

	private:
		void AddCRC(StreamBufferCMD);

		std::deque<uint8_t> m_buffer;
		StreamBufferCMD m_cmd;
		int m_CRC_todo_data_count;
		float m_float_min;
		float m_float_pre;
		bool m_float_compress_flag;
	};

	template <class T>
	struct DataUnit : public DataUnitProto
	{
		T data;

		void FromStream(StreamBytesBuffer *buffer) override
		{
			(*buffer) >> data;
		}

		void FromStream(DataUnitProto *unit_proto) override
		{
			if (typeid(*unit_proto) != typeid(DataUnit<T>))
			{
				std::cerr<<"Unit type not equal!"<<std::endl;
				return;
			}

			DataUnit<T> *true_unit = dynamic_cast<DataUnit<T> *>(unit_proto);
			data = true_unit->data;
		}

		void FromStream(ReplacementUnit *replacement_unit) override
		{
			if (replacement_unit->update_type == BaseTypeEnum::uint8)
			{
				data = replacement_unit->base_union.u8;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::int8)
			{
				data = replacement_unit->base_union.i8;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::uint16)
			{
				data = replacement_unit->base_union.u16;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::int16)
			{
				data = replacement_unit->base_union.i16;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::uint32)
			{
				data = replacement_unit->base_union.u32;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::int32)
			{
				data = replacement_unit->base_union.i32;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::fp32)
			{
				data = replacement_unit->base_union.fp32;
			}
			else if (replacement_unit->update_type == BaseTypeEnum::fp64)
			{
				data = replacement_unit->base_union.fp64;
			}
			else
			{
				replacement_unit->update_type = BaseTypeEnum::null;
			}
		}

		void ToStream(StreamBytesBuffer *buffer) override
		{
			(*buffer) << data;
		}

		void ToStream(DataUnitProto *unit_proto) override
		{
			if (typeid(*unit_proto) != typeid(DataUnit<T>))
			{
				std::cerr<<"Unit type not equal!"<<std::endl;
				return;
			}

			DataUnit<T> *true_unit = dynamic_cast<DataUnit<T> *>(unit_proto);
			true_unit->data = data;
		}

		void ToStream(ReplacementUnit *replacement_unit) override
		{
			if (typeid(data) == typeid(uint8_t))
			{
				replacement_unit->base_union.u8 = data;
				replacement_unit->update_type = BaseTypeEnum::uint8;
			}
			else if (typeid(data) == typeid(int8_t))
			{
				replacement_unit->base_union.i8 = data;
				replacement_unit->update_type = BaseTypeEnum::int8;
			}
			else if (typeid(data) == typeid(uint16_t))
			{
				replacement_unit->base_union.u16 = data;
				replacement_unit->update_type = BaseTypeEnum::uint16;
			}
			else if (typeid(T) == typeid(int16_t))
			{
				replacement_unit->base_union.i16 = data;
				replacement_unit->update_type = BaseTypeEnum::int16;
			}
			else if (typeid(T) == typeid(uint32_t))
			{
				replacement_unit->base_union.u32 = data;
				replacement_unit->update_type = BaseTypeEnum::uint32;
			}
			else if (typeid(T) == typeid(int32_t))
			{
				replacement_unit->base_union.i32 = data;
				replacement_unit->update_type = BaseTypeEnum::int32;
			}
			else if (typeid(T) == typeid(float))
			{
				replacement_unit->base_union.fp32 = data;
				replacement_unit->update_type = BaseTypeEnum::fp32;
			}
			else if (typeid(T) == typeid(double))
			{
				replacement_unit->base_union.fp64 = data;
				replacement_unit->update_type = BaseTypeEnum::fp64;
			}
			else
			{
				replacement_unit->update_type = BaseTypeEnum::null;
			}
		}
	};

	template <>
	struct DataUnit<std::vector<uint8_t>> : public DataUnitProto
	{
		std::vector<uint8_t> data;
		uint16_t data_length;

		void FromStream(StreamBytesBuffer *buffer) override
		{
			uint8_t temp_data;

			data.clear();

			for (int i = 0; i < data_length; i++)
			{
				(*buffer) >> temp_data;
				data.push_back(temp_data);
			}
		}

		void FromStream(DataUnitProto *unit_proto) override
		{
			if (typeid(*unit_proto) == typeid(DataUnit<std::vector<uint8_t>>))
			{
				DataUnit<std::vector<uint8_t>> *true_unit =
					dynamic_cast<DataUnit<std::vector<uint8_t>> *>(unit_proto);

				data.clear();
				data.insert(data.end(), true_unit->data.begin(), true_unit->data.end());
			}
		}

		void FromStream(ReplacementUnit *replacement_unit) override
		{

		}

		void ToStream(StreamBytesBuffer *buffer) override
		{
			auto iterator = data.begin();

			while (iterator != data.end())
			{
				(*buffer) << (*iterator);
				iterator++;
			}
		}

		void ToStream(DataUnitProto *unit_proto) override
		{
			if (typeid(*unit_proto) == typeid(DataUnit<std::vector<uint8_t>>))
			{
				DataUnit<std::vector<uint8_t>> *true_unit =
					dynamic_cast<DataUnit<std::vector<uint8_t>> *>(unit_proto);

				data.clear();
				true_unit->data.insert(true_unit->data.end(), data.begin(), data.end());
			}
		}

		void ToStream(ReplacementUnit *replacement_unit) override
		{

		}
	};

	template <class T>
	inline StreamBytesBuffer &StreamBytesBuffer::operator<<(T *object)
	{
		uint8_t *p = (uint8_t *)object;
		for (int i = 0; i < sizeof(T); i++, p++)
		{
			m_buffer.push_back(*p);
		}
		m_CRC_todo_data_count += sizeof(T);
		return (*this);
	}

	template <class T>
	inline StreamBytesBuffer &StreamBytesBuffer::operator<<(T object)
	{
		uint8_t *p = (uint8_t *)(&object);
		for (int i = 0; i < sizeof(T); i++, p++)
		{
			m_buffer.push_back(*p);
		}
		m_CRC_todo_data_count += sizeof(T);
		return (*this);
	}

	template <typename T>
	StreamBytesBuffer &StreamBytesBuffer::operator<<(DataUnit<T> &object)
	{
		(*this) << object.data;
		return (*this);
	}

	template <>
	inline StreamBytesBuffer &StreamBytesBuffer::operator<<(float object)
	{
		if (m_float_compress_flag)
		{
			if (m_cmd == StreamBufferCMD::FloatMin)
			{
				m_float_min = object;
				m_cmd = StreamBufferCMD::NullCMD;
				return (*this);
			}
			else if (m_cmd == StreamBufferCMD::FloatPre)
			{
				m_float_pre = object;
				m_cmd = StreamBufferCMD::NullCMD;
				return (*this);
			}
			uint16_t temp_float = (uint16_t)((object - m_float_min) / m_float_pre);

			m_buffer.push_back(temp_float & 0xFF);
			m_buffer.push_back(temp_float >> 8);
			m_CRC_todo_data_count += 2;
		}
		else
		{
			uint8_t *p = (uint8_t *)(&object);
			for (int i = 0; i < 4; i++, p++)
			{
				m_buffer.push_back(*p);
			}
			m_CRC_todo_data_count += 4;
		}
		return (*this);
	}

	template <typename T>
	inline StreamBytesBuffer &StreamBytesBuffer::operator>>(T *object)
	{
		uint8_t *p = (uint8_t *)object;

		auto it = m_buffer.begin();
		for (int i = 0; i < sizeof(T); i++, p++, it++)
		{
			(*p) = (*it);
		}
		for (size_t i = 0; i < sizeof(T); i++)
		{
			m_buffer.pop_front();
		}
		return (*this);
	}

	template <typename T>
	inline StreamBytesBuffer &StreamBytesBuffer::operator>>(T &object)
	{
		auto it = m_buffer.begin();
		uint8_t temp_data = 0;
		object = 0;
		for (int i = 0; i < sizeof(T); i++, it++)
		{
			temp_data = (*it);
			object = object | (temp_data << (8 * i));
		}

		for (int i = 0; i < sizeof(T); ++i)
		{
			m_buffer.pop_front();
		}
		return (*this);
	}

	template <typename T>
	StreamBytesBuffer &StreamBytesBuffer::operator>>(DataUnit<T> &object)
	{
		(*this) >> object.data;
		return (*this);
	}

	template <>
	inline StreamBytesBuffer &StreamBytesBuffer::operator>>(float &object)
	{
		auto it = m_buffer.begin();
		float temp_float = 0.0f;
		uint8_t *p = (uint8_t *)&temp_float;
		uint16_t tenp_zip_float = 0;

		if (m_float_compress_flag)
		{
			(*this) >> tenp_zip_float;
			object = ((float)tenp_zip_float) * m_float_pre + m_float_min;
		}
		else
		{
			for (int i = 0; i < 4; i++, p++, it++)
			{
				(*p) = (*it);
			}
			for (size_t i = 0; i < 4; i++)
			{
				m_buffer.pop_front();
			}
			object = temp_float;
		}
		return (*this);
	}
} // namespace stream

#endif // ! STREAM_BYTES_BUFFER_HPP
