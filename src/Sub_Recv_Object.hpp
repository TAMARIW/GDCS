#ifndef SUB_RECV_OBJECT_HPP_
#define SUB_RECV_OBJECT_HPP_

#include <stdint.h>

#include "rodos-debug.h"
#include "rodos-semaphore.h"
#include "topic.h"
#include "netmsginfo.h"


/**
 * @author Christopher Steffen
 * @date 07.2024 
 * 
 * @brief Similar to subscriber receiver but works with specific objects and not just static/global functions.
 */

namespace RODOS {

/**
 * Simple Subscriber interface for users. Similar to SubscriberReceiver but works with objects and not static/global functions. 
 * @tparam Type The data type of the topic message that shall be received by the %SubscriberObjRecv.
 * @tparam ObjectType The object type to receive the message %SubscriberObjRecv.
 */
template <class Type, class ObjectType>
class SubscriberObjRecv : public Subscriber {
    void (ObjectType::*receiverFunc)(Type &msg); ///< an object function to be called for each message (optional)
    ObjectType *receiverObject; ///< the object ot call the function on.

public:
    /**
     * This constructor is usually used when developers inherit from this class in order to implement message handling
     * in the overridden put() method.
     * @param topic A reference to the topic that shall be subscribed.
     * @param name The name of the subscriber.
     */
    SubscriberObjRecv(TopicInterface &topic, const char* name = "anonymSubscriber") :
        Subscriber(topic, name) {
        receiverFunc = 0;
        receiverObject = 0;
    }

    /**
     * This constructor is usually used when developers just create an instance in order to call the passed message
     * handling function.
     * @param topic A reference to the topic that shall be subscribed.
     * @param funcPtr The pointer to the function that is called every time a topic message arrives.
     * @param object The object to have the function called onto.
     * @param name The name of the subscriber.
     */
    SubscriberObjRecv(TopicInterface &topic,  void (ObjectType::*funcPtr)(Type&), ObjectType *object, const char* name = "anonymSubscriber") :
        Subscriber(topic, name) {
        receiverFunc = funcPtr;
        receiverObject = object;
    }

    /**
     * This method is called every time a topic message arrives.
     * @param msg The message that was published to the topic.
     */
    virtual void put(Type &msg) {
        if(receiverFunc && receiverObject) (receiverObject->*receiverFunc)(msg);
    }

    /**
     * This method is called every time a topic message arrives.
     * @param msg The message that was published to the topic.
     * @param netMsgInfo Meta information about the message (path).
     */
    virtual void put(Type &msg, [[gnu::unused]] const NetMsgInfo& netMsgInfo) {
        put(msg);
    }

    uint32_t put([[gnu::unused]] const uint32_t topicId, [[gnu::unused]] const size_t len, void* data, const NetMsgInfo& netMsgInfo) override {
        put(*(Type*)data,netMsgInfo);
        return 1;
    }
};

}


#endif
