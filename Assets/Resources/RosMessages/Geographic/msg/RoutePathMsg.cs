//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geographic
{
    [Serializable]
    public class RoutePathMsg : Message
    {
        public const string k_RosMessageName = "geographic_msgs/RoutePath";
        public override string RosMessageName => k_RosMessageName;

        //  Path through a route network.
        // 
        //  A path is a sequence of RouteSegment edges.  This information is
        //  extracted from a RouteNetwork graph.  A RoutePath lists the route
        //  segments needed to reach some chosen goal.
        public HeaderMsg header;
        public Uuid.UniqueIDMsg network;
        //  Route network containing this path
        public Uuid.UniqueIDMsg[] segments;
        //  Sequence of RouteSegment IDs
        public KeyValueMsg[] props;
        //  Key/value properties

        public RoutePathMsg()
        {
            this.header = new HeaderMsg();
            this.network = new Uuid.UniqueIDMsg();
            this.segments = new Uuid.UniqueIDMsg[0];
            this.props = new KeyValueMsg[0];
        }

        public RoutePathMsg(HeaderMsg header, Uuid.UniqueIDMsg network, Uuid.UniqueIDMsg[] segments, KeyValueMsg[] props)
        {
            this.header = header;
            this.network = network;
            this.segments = segments;
            this.props = props;
        }

        public static RoutePathMsg Deserialize(MessageDeserializer deserializer) => new RoutePathMsg(deserializer);

        private RoutePathMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.network = Uuid.UniqueIDMsg.Deserialize(deserializer);
            deserializer.Read(out this.segments, Uuid.UniqueIDMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.props, KeyValueMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.network);
            serializer.WriteLength(this.segments);
            serializer.Write(this.segments);
            serializer.WriteLength(this.props);
            serializer.Write(this.props);
        }

        public override string ToString()
        {
            return "RoutePathMsg: " +
            "\nheader: " + header.ToString() +
            "\nnetwork: " + network.ToString() +
            "\nsegments: " + System.String.Join(", ", segments.ToList()) +
            "\nprops: " + System.String.Join(", ", props.ToList());
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