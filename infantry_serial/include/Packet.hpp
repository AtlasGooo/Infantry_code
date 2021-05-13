#ifndef PACKET_HPP
#define PACKET_HPP

#include "StreamBytesBuffer.hpp"

namespace stream
{
	class PacketBehaviorProto;

	enum class StreamType
	{
		PacketStreamType = 1,
		BytesStreamType = 2,
	};

	template<StreamType type>
	class Stream;


	template<class PacketProtoType>
	class PacketTypeList
	{
	public:
		static PacketTypeList* Instance()
		{
			static PacketTypeList instance;
			return &instance;
		}
		~PacketTypeList() { ; }

		void AddType(PacketProtoType*) { ; }

	private:
		PacketTypeList() { ; }

		std::vector<PacketProtoType*> m_packet_type_list;
	};

	class PacketModelProto
	{
	friend class StreamBytesBuffer;
	friend class PacketBehaviorProto;
	public:
		PacketModelProto(uint16_t _id,uint8_t _data_length) :m_id(_id) , m_data_length(_data_length){ ; }
		~PacketModelProto() { ; }

		std::vector<DataUnitProto*>* GetUnitList() { return &m_unit_list; }
	protected:
		uint16_t m_id;
		uint8_t m_data_length;
		std::vector<DataUnitProto*> m_unit_list;
	};

	class PacketBehaviorProto
	{
	public:
		PacketBehaviorProto(PacketModelProto* _unit_list) :m_model(_unit_list),m_update_flag(false),m_used_flag(false)
		{
			;
		}
		~PacketBehaviorProto() { ; }

		PacketBehaviorProto& operator<<(PacketBehaviorProto&);
		PacketBehaviorProto& operator>>(PacketBehaviorProto&);

		virtual void ToStream(Stream<StreamType::PacketStreamType>*) = 0;
		virtual void ToStream(Stream<StreamType::BytesStreamType>*);
		void ToStream(PacketBehaviorProto*);

		virtual void FromStream(Stream<StreamType::PacketStreamType>*) = 0;
		virtual void FromStream(Stream<StreamType::BytesStreamType>*);
		void FromStream(PacketBehaviorProto*);

		void SetUpdateFlag(bool _set) { m_update_flag = _set; }
		bool GetUpdateFlag() { return m_update_flag; }

		void SetUsedFlag(bool _set) { m_used_flag = _set; }
		bool GetUsedFlag() { return m_used_flag; }

		uint16_t GetID() { return m_model->m_id; }

		void SetDataLength(uint8_t _set){m_model->m_data_length = _set;}
		uint8_t GetDataLength(){return m_model->m_data_length;}
		
		PacketModelProto* GetModel() { return m_model; }
	protected:
		PacketModelProto* m_model;
		bool m_update_flag;
		bool m_used_flag;
	};
}

#endif
