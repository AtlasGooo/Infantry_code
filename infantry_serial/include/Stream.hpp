#ifndef STREAM_HPP
#define STREAM_HPP

#include "Packet.hpp"

namespace stream
{

	template <StreamType type>
	class Stream;

	template <>
	class Stream<StreamType::PacketStreamType>;

	template <>
	class Stream<StreamType::BytesStreamType>;

	template <StreamType type>
	class Stream
	{
	public:
		Stream() { ; }
		~Stream() { ; }

		virtual void Send() { ; }
		virtual void Receive() { ; }
		virtual void Serialize() { ; }
		virtual void Deserialize() { ; }
		void CleanUsedPacket();

		virtual Stream<type> &operator<<(Stream<StreamType::BytesStreamType> *);
		virtual Stream<type> &operator<<(Stream<StreamType::PacketStreamType> *);
		virtual Stream<type> &operator>>(Stream<StreamType::BytesStreamType> *);
		virtual Stream<type> &operator>>(Stream<StreamType::PacketStreamType> *);

		template <StreamType stream_type>
		Stream<type> &operator<<(Stream<stream_type> *stream) { ; }

		template <StreamType stream_type>
		Stream<type> &operator>>(Stream<stream_type> *stream) { ; }

		void RegisterPacket(PacketBehaviorProto *packet) { m_registered_packet_list.push_back(packet); }
		StreamType GetStreamType() { return m_type; }

		std::list<PacketBehaviorProto *> *GetRegisteredPacketList()
		{
			return &m_registered_packet_list;
		}

	protected:
		StreamType m_type;
		std::list<PacketBehaviorProto *> m_registered_packet_list;
	};

	template <>
	class Stream<StreamType::PacketStreamType>;

	template <>
	class Stream<StreamType::BytesStreamType>;

	template <>
	class Stream<StreamType::BytesStreamType>
	{
	public:
		Stream() : m_type(StreamType::BytesStreamType), m_buffer() { ; }
		~Stream() { ; }

		virtual void Send() { ; }
		virtual void Receive() { ; }
		virtual void Serialize() { ; }
		virtual void Deserialize() { ; }
		void CleanUsedPacket();

		virtual Stream<StreamType::BytesStreamType> &operator<<(Stream<StreamType::BytesStreamType> *stream);
		virtual Stream<StreamType::BytesStreamType> &operator<<(Stream<StreamType::PacketStreamType> *stream);
		virtual Stream<StreamType::BytesStreamType> &operator>>(Stream<StreamType::BytesStreamType> *stream);
		virtual Stream<StreamType::BytesStreamType> &operator>>(Stream<StreamType::PacketStreamType> *stream);

		template <StreamType stream_type>
		Stream<StreamType::BytesStreamType> &operator<<(Stream<stream_type> *stream);

		template <StreamType stream_type>
		Stream<StreamType::BytesStreamType> &operator>>(Stream<stream_type> *stream);

		void RegisterPacket(PacketBehaviorProto *packet) { m_registered_packet_list.push_back(packet); }

		StreamType GetStreamType() { return m_type; }

		StreamBytesBuffer m_buffer;

		std::list<PacketBehaviorProto *> *GetRegisteredPacketList()
		{
			return &m_registered_packet_list;
		}

	protected:
		StreamType m_type;
		std::list<PacketBehaviorProto *> m_registered_packet_list;
	};

	template <>
	class Stream<StreamType::PacketStreamType>
	{
		template <StreamType stream_type>
		friend std::list<PacketBehaviorProto *> *GetRegisteredPacketList(Stream<stream_type> &stream);

	public:
		Stream() : m_type(StreamType::PacketStreamType) { ; }
		~Stream() { ; }

		virtual void Send() { ; }
		virtual void Receive() { ; }
		virtual void Serialize() { ; }
		virtual void Deserialize() { ; }
		void CleanUsedPacket();

		virtual Stream<StreamType::PacketStreamType> &operator<<(Stream<StreamType::BytesStreamType> *stream)
		{
			auto giver_packet_list = stream->GetRegisteredPacketList();
			auto giver_packet_list_iterator = giver_packet_list->begin();

			while (giver_packet_list_iterator != giver_packet_list->end())
			{
				if ((*giver_packet_list_iterator)->GetUpdateFlag())
				{
					uint16_t id = (*giver_packet_list_iterator)->GetID();

					auto my_packet = std::find_if(m_registered_packet_list.begin(),
												  m_registered_packet_list.end(), [id](PacketBehaviorProto *packet) { return (packet->GetID() == id) && (!packet->GetUpdateFlag()); });

					if (my_packet != m_registered_packet_list.end())
					{
						(**my_packet) << (**giver_packet_list_iterator);

						(*my_packet)->SetUpdateFlag(true);
						(*giver_packet_list_iterator)->SetUsedFlag(true);
					}
				}
				giver_packet_list_iterator++;
			}
			return (*this);
		}

		virtual Stream<StreamType::PacketStreamType> &operator<<(Stream<StreamType::PacketStreamType> *stream)
		{
			return (*this) << (Stream<StreamType::BytesStreamType> *)stream;
		}

		virtual Stream<StreamType::PacketStreamType> &operator>>(Stream<StreamType::BytesStreamType> *stream)
		{
			auto my_packet_list_iterator = m_registered_packet_list.begin();
			auto receiver_packet_list = stream->GetRegisteredPacketList();

			while (my_packet_list_iterator != m_registered_packet_list.end())
			{
				if ((*my_packet_list_iterator)->GetUpdateFlag())
				{
					uint16_t id = (*my_packet_list_iterator)->GetID();

					auto receiver_packet = std::find_if(receiver_packet_list->begin(),
														receiver_packet_list->end(), [id](PacketBehaviorProto *packet) { return (packet->GetID() == id) && (!packet->GetUpdateFlag()); });

					if (receiver_packet != m_registered_packet_list.end())
					{
						(**receiver_packet) << (**my_packet_list_iterator);

						(*receiver_packet)->SetUpdateFlag(true);
						(*my_packet_list_iterator)->SetUsedFlag(true);
					}
				}

				my_packet_list_iterator++;
			}
			return (*this);
		}

		virtual Stream<StreamType::PacketStreamType> &operator>>(Stream<StreamType::PacketStreamType> *stream)
		{
			return (*this) >> (Stream<StreamType::BytesStreamType> *)stream;
		}

		template <StreamType stream_type>
		Stream<StreamType::PacketStreamType> &operator<<(Stream<stream_type> *stream)
		{
			if (stream->GetStreamType() == StreamType::BytesStreamType)
			{
				(*this) << (Stream<StreamType::BytesStreamType> *)stream;
			}
			else if (stream->GetStreamType() == StreamType::PacketStreamType)
			{
				(*this) << (Stream<StreamType::PacketStreamType> *)stream;
			}

			return (*this);
		}

		template <StreamType stream_type>
		Stream<StreamType::PacketStreamType> &operator>>(Stream<stream_type> *stream)
		{
			if (stream->GetStreamType() == StreamType::BytesStreamType)
			{
				(*this) >> (Stream<StreamType::BytesStreamType> *)stream;
			}
			else if (stream->GetStreamType() == StreamType::PacketStreamType)
			{
				(*this) >> (Stream<StreamType::PacketStreamType> *)stream;
			}

			return (*this);
		}

		void RegisterPacket(PacketBehaviorProto *packet) { m_registered_packet_list.push_back(packet); }

		StreamType GetStreamType() { return m_type; }

		std::list<PacketBehaviorProto *> *GetRegisteredPacketList()
		{
			return &m_registered_packet_list;
		}

	protected:
		StreamType m_type;
		StreamBytesBuffer m_buffer;
		std::list<PacketBehaviorProto *> m_registered_packet_list;
	};

	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator<<(Stream<StreamType::BytesStreamType> *stream)
	{
		m_buffer << stream->m_buffer;
		return (*this);
	}

	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator<<(Stream<StreamType::PacketStreamType> *stream)
	{
		auto giver_packet_list = stream->GetRegisteredPacketList();
		auto giver_packet_list_iterator = giver_packet_list->begin();

		while (giver_packet_list_iterator != giver_packet_list->end())
		{
			if ((*giver_packet_list_iterator)->GetUpdateFlag())
			{
				uint16_t id = (*giver_packet_list_iterator)->GetID();

				auto my_packet = std::find_if(m_registered_packet_list.begin(),
											  m_registered_packet_list.end(), [id](PacketBehaviorProto *packet) { return (packet->GetID() == id) && (!packet->GetUpdateFlag()); });

				if (my_packet != m_registered_packet_list.end())
				{
					(**my_packet) << (**giver_packet_list_iterator);

					(*my_packet)->SetUpdateFlag(true);
					(*giver_packet_list_iterator)->SetUsedFlag(true);
				}
			}
			giver_packet_list_iterator++;
		}
		return (*this);
	}

	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator>>(Stream<StreamType::BytesStreamType> *stream)
	{
		stream->m_buffer << m_buffer;
		return (*this);
	}

	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator>>(Stream<StreamType::PacketStreamType> *stream)
	{
		auto my_packet_list_iterator = m_registered_packet_list.begin();
		auto receiver_packet_list = stream->GetRegisteredPacketList();

		while (my_packet_list_iterator != m_registered_packet_list.end())
		{
			if ((*my_packet_list_iterator)->GetUpdateFlag())
			{
				uint16_t id = (*my_packet_list_iterator)->GetID();

				auto receiver_packet = std::find_if(receiver_packet_list->begin(),
													receiver_packet_list->end(), [id](PacketBehaviorProto *packet) { return (packet->GetID() == id) && (!packet->GetUpdateFlag()); });

				if (receiver_packet != m_registered_packet_list.end())
				{
					(**receiver_packet) << (**my_packet_list_iterator);

					(*receiver_packet)->SetUpdateFlag(true);
					(*my_packet_list_iterator)->SetUsedFlag(true);
				}
			}

			my_packet_list_iterator++;
		}
		return (*this);
	}

	template <StreamType stream_type>
	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator<<(Stream<stream_type> *stream)
	{
		if (stream->GetStreamType() == StreamType::BytesStreamType)
		{
			(*this) << (Stream<StreamType::BytesStreamType> *)stream;
		}
		else if (stream->GetStreamType() == StreamType::PacketStreamType)
		{
			(*this) << (Stream<StreamType::PacketStreamType> *)stream;
		}

		return (*this);
	}

	template <StreamType stream_type>
	inline Stream<StreamType::BytesStreamType> &Stream<StreamType::BytesStreamType>::
		operator>>(Stream<stream_type> *stream)
	{
		if (stream->GetStreamType() == StreamType::BytesStreamType)
		{
			(*this) >> (Stream<StreamType::BytesStreamType> *)stream;
		}
		else if (stream->GetStreamType() == StreamType::PacketStreamType)
		{
			(*this) >> (Stream<StreamType::PacketStreamType> *)stream;
		}

		return (*this);
	}
} // namespace stream

#endif