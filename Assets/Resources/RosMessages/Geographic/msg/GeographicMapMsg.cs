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
    public class GeographicMapMsg : Message
    {
        public const string k_RosMessageName = "geographic_msgs/GeographicMap";
        public override string RosMessageName => k_RosMessageName;

        //  Geographic map for a specified region.
        public HeaderMsg header;
        //  stamp specifies time
        //  frame_id (normally /map)
        public Uuid.UniqueIDMsg id;
        //  identifier for this map
        public BoundingBoxMsg bounds;
        //  2D bounding box containing map
        public WayPointMsg[] points;
        //  way-points
        public MapFeatureMsg[] features;
        //  map features
        public KeyValueMsg[] props;
        //  map properties

        public GeographicMapMsg()
        {
            this.header = new HeaderMsg();
            this.id = new Uuid.UniqueIDMsg();
            this.bounds = new BoundingBoxMsg();
            this.points = new WayPointMsg[0];
            this.features = new MapFeatureMsg[0];
            this.props = new KeyValueMsg[0];
        }

        public GeographicMapMsg(HeaderMsg header, Uuid.UniqueIDMsg id, BoundingBoxMsg bounds, WayPointMsg[] points, MapFeatureMsg[] features, KeyValueMsg[] props)
        {
            this.header = header;
            this.id = id;
            this.bounds = bounds;
            this.points = points;
            this.features = features;
            this.props = props;
        }

        public static GeographicMapMsg Deserialize(MessageDeserializer deserializer) => new GeographicMapMsg(deserializer);

        private GeographicMapMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.id = Uuid.UniqueIDMsg.Deserialize(deserializer);
            this.bounds = BoundingBoxMsg.Deserialize(deserializer);
            deserializer.Read(out this.points, WayPointMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.features, MapFeatureMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.props, KeyValueMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.id);
            serializer.Write(this.bounds);
            serializer.WriteLength(this.points);
            serializer.Write(this.points);
            serializer.WriteLength(this.features);
            serializer.Write(this.features);
            serializer.WriteLength(this.props);
            serializer.Write(this.props);
        }

        public override string ToString()
        {
            return "GeographicMapMsg: " +
            "\nheader: " + header.ToString() +
            "\nid: " + id.ToString() +
            "\nbounds: " + bounds.ToString() +
            "\npoints: " + System.String.Join(", ", points.ToList()) +
            "\nfeatures: " + System.String.Join(", ", features.ToList()) +
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
