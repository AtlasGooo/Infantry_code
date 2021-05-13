#ifndef ROSTOPIC_PACKET_HPP
#define ROSTOPIC_PACKET_HPP

#include "PacketBehavior.hpp"

namespace stream
{
    template<class TopicType,class ModelType>
    class RostopicTemplatePacket : public RostopicPacketBehavior<TopicType>,public ModelType
    {
    public:
        RostopicTemplatePacket(std::string topic_name): 
            RostopicPacketBehavior<TopicType>::RostopicPacketBehavior(this,topic_name)
        {
            // RostopicTemplatePacket(std::string topic_name):RostopicPacketBehavior(this,topic_name) failed.
            // Why does it look so stupid : 
            // https://www.zhihu.com/question/28139230
            // https://blog.csdn.net/gettogetto/article/details/52955741
            
            //RostopicPacketBehavior<TopicType>::RostopicPacketBehavior(this,topic_name) equal to next lines.
            //this->m_model = this;
            //this->m_topic_name = topic_name;
            //this->m_unit_end = this->m_model->GetUnitList()->end();
        }

        ~RostopicTemplatePacket(){;}
    };

} // namespace stream
#endif