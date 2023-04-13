//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class ByteMultiArrayMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/ByteMultiArray";
        public override string RosMessageName => k_RosMessageName;

        //  This was originally provided as an example message.
        //  It is deprecated as of Foxy
        //  It is recommended to create your own semantically meaningful message.
        //  However if you would like to continue using this please use the equivalent in example_msgs.
        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayoutMsg layout;
        //  specification of data layout
        public sbyte[] data;
        //  array of data

        public ByteMultiArrayMsg()
        {
            this.layout = new MultiArrayLayoutMsg();
            this.data = new sbyte[0];
        }

        public ByteMultiArrayMsg(MultiArrayLayoutMsg layout, sbyte[] data)
        {
            this.layout = layout;
            this.data = data;
        }

        public static ByteMultiArrayMsg Deserialize(MessageDeserializer deserializer) => new ByteMultiArrayMsg(deserializer);

        private ByteMultiArrayMsg(MessageDeserializer deserializer)
        {
            this.layout = MultiArrayLayoutMsg.Deserialize(deserializer);
            deserializer.Read(out this.data, sizeof(sbyte), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.layout);
            serializer.WriteLength(this.data);
            serializer.Write(this.data);
        }

        public override string ToString()
        {
            return "ByteMultiArrayMsg: " +
            "\nlayout: " + layout.ToString() +
            "\ndata: " + System.String.Join(", ", data.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}