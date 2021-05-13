#ifndef PACKET_BEHAVIOR_HPP
#define PACKET_BEHAVIOR_HPP

#include "Packet.hpp"
#include <ros/ros.h>

namespace stream
{
	class LogPacketBehavior : public PacketBehaviorProto
	{
	public:
		LogPacketBehavior(PacketModelProto *_unit_list, std::string _name) : PacketBehaviorProto(_unit_list), m_file_name(_name) { ; }
		~LogPacketBehavior() { ; }

		void ToStream(Stream<StreamType::PacketStreamType> *) override;
		void FromStream(Stream<StreamType::PacketStreamType> *) override;

		virtual void ToStream(std::ofstream *ofstream);
		virtual void FromStream(std::ifstream *ifstream);

	protected:
		std::string m_file_name;
	};

	class SerialPacketBehavior : public PacketBehaviorProto
	{
	public:
		SerialPacketBehavior(PacketModelProto *_unit_list) : PacketBehaviorProto(_unit_list), m_float_min(1.0f), m_float_pre(1.0f)
		{
			;
		}

		~SerialPacketBehavior() { ; }

		void ToStream(Stream<StreamType::PacketStreamType> *) override { ; }
		void FromStream(Stream<StreamType::PacketStreamType> *) override { ; }

		void ToStream(Stream<StreamType::BytesStreamType> *) override;

	protected:
		float m_float_min;
		float m_float_pre;
	};

	class RefereePacketBehavior : public PacketBehaviorProto
	{
	public:
		RefereePacketBehavior(PacketModelProto *_unit_list) : PacketBehaviorProto(_unit_list), m_data_id(0) { ; }

		~RefereePacketBehavior() { ; }

		void ToStream(Stream<StreamType::PacketStreamType> *) override { ; }
		void FromStream(Stream<StreamType::PacketStreamType> *) override { ; }

		virtual void ToStream(Stream<StreamType::BytesStreamType> *) override;

	protected:
		uint16_t m_data_id;
	};

	class RostopicPacketBehaviorProto : public PacketBehaviorProto
	{
	public:
		RostopicPacketBehaviorProto(PacketModelProto *_unit_list, std::string _topic_name) : PacketBehaviorProto(_unit_list), m_topic_name(_topic_name) { ; }
		RostopicPacketBehaviorProto() : PacketBehaviorProto(nullptr), m_topic_name(""){;}
		~RostopicPacketBehaviorProto() { ; }

		virtual void PublisherInit(ros::NodeHandle *) = 0;
		virtual void SubscriberInit(ros::NodeHandle *) = 0;
		virtual void Publish() = 0;
		virtual uint16_t GetDataID() = 0;

	protected:
		std::string m_topic_name;
		ros::Publisher publisher;
		ros::Subscriber subscriber;
	};

	template <class TopicType>
	class RostopicPacketBehavior : public RostopicPacketBehaviorProto
	{
	protected:
		std::vector<DataUnitProto *>::iterator m_unit_iterator;
		std::vector<DataUnitProto *>::iterator m_unit_end;
		bool m_direction_flag; // true: ToStream, false: FromStream
		ros::serialization::Serializer<TopicType> m_serializer;
		TopicType m_message;
	public:
		RostopicPacketBehavior(PacketModelProto *_unit_list, std::string _topic_name) : RostopicPacketBehaviorProto(_unit_list, _topic_name),
			m_unit_end(_unit_list->GetUnitList()->end()) 
			{
				m_packet_list.push_back(this);
			}

		RostopicPacketBehavior() : RostopicPacketBehaviorProto()
		{
			m_packet_list.push_back(this);	
		}

		~RostopicPacketBehavior() { ; }

		virtual void PublisherInit(ros::NodeHandle *n) override
		{
			publisher = n->advertise<TopicType>(m_topic_name, 256);
		}
		
		virtual void SubscriberInit(ros::NodeHandle *n) override
		{
			subscriber = n->subscribe(m_topic_name,1000,RosCallBack);
		}

		virtual void Publish() override
		{
			publisher.publish(m_message);
		}

		virtual uint16_t GetDataID() override { return 0; }

		template <class T>
		void next(T& data)
		{
			ReplacementUnit unit;

			if (m_unit_iterator == m_unit_end)
			{
				std::cerr << "RostopicPacketBehavior next iterator out of range" << std::endl;
				return;
			}
			if (m_direction_flag)
			{
				(*m_unit_iterator)->ToStream(&unit);

				switch (unit.update_type)
				{
				case BaseTypeEnum::uint8:
					data = unit.base_union.u8;
					break;
				case BaseTypeEnum::int8:
					data = unit.base_union.i8;
					break;
				case BaseTypeEnum::uint16:
					data = unit.base_union.u16;
					break;
				case BaseTypeEnum::int16:
					data = unit.base_union.i16;
					break;
				case BaseTypeEnum::uint32:
					data = unit.base_union.u32;
					break;
				case BaseTypeEnum::int32:
					data = unit.base_union.i32;
					break;
				case BaseTypeEnum::fp32:
					data = unit.base_union.fp32;
					break;
				case BaseTypeEnum::fp64:
					data = unit.base_union.fp64;
					break;
				default:
					std::cerr << "RostopicPacketBehavior ToStream null type" << std::endl;
					break;
				}
			}
			else
			{
				(*m_unit_iterator)->ToStream(&unit);

				switch (unit.update_type)
				{
				case BaseTypeEnum::uint8:
					unit.base_union.u8 = data;
					break;
				case BaseTypeEnum::int8:
					unit.base_union.i8 = data;
					break;
				case BaseTypeEnum::uint16:
					unit.base_union.u16 = data;
					break;
				case BaseTypeEnum::int16:
					unit.base_union.i16 = data;
					break;
				case BaseTypeEnum::uint32:
					unit.base_union.u32 = data;
					break;
				case BaseTypeEnum::int32:
					unit.base_union.i32 = data;
					break;
				case BaseTypeEnum::fp32:
					unit.base_union.fp32 = data;
					break;
				case BaseTypeEnum::fp64:
					unit.base_union.fp64 = data;
					break;
				default:
					std::cerr << "RostopicPacketBehavior FromStream null type" << std::endl;
					break;
				}

				(*m_unit_iterator)->FromStream(&unit);
			}
			m_unit_iterator++;
		}

		void FromStream(Stream<StreamType::PacketStreamType> *) override
		{
			;
		}

		virtual void FromStream(const typename TopicType::ConstPtr& msg)
		{
			ROS_INFO("message updated");
			m_unit_iterator = m_model->GetUnitList()->begin();
			m_direction_flag = false;
			m_serializer.allInOne((*this), *msg);
		}

		void ToStream(Stream<StreamType::PacketStreamType> *) override
		{
			m_unit_iterator = m_model->GetUnitList()->begin();
			m_direction_flag = true;
			m_serializer.read((*this),m_message);
		}

		TopicType* GetMessage(){return &m_message;}

		static void RosCallBack(const typename TopicType::ConstPtr& msg)
		{
			auto packet = m_packet_list.begin();

			while (packet!=m_packet_list.end())
			{
				if(!(*packet)->GetUpdateFlag())
				{
					// ROS_INFO("message updated");
					(*packet)->FromStream(msg);
					(*packet)->SetUpdateFlag(true);
					break;
				}
				packet++;
			}
		}

		static std::vector<RostopicPacketBehavior<TopicType>*> m_packet_list;
	};

	//magical template class static member variables are initialized out of class
	template<class TopicType> std::vector<RostopicPacketBehavior<TopicType>*> RostopicPacketBehavior<TopicType>::m_packet_list;

} // namespace stream

#endif
