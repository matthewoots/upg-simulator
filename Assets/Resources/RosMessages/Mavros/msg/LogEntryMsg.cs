//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class LogEntryMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/LogEntry";
        public override string RosMessageName => k_RosMessageName;

        //  Information about a single log
        // 
        //   :id: - log id
        //   :num_logs: - total number of logs
        //   :last_log_num: - id of last log
        //   :time_utc: - UTC timestamp of log (ros::Time(0) if not available)
        //   :size: - size of log in bytes (may be approximate)
        public Std.HeaderMsg header;
        public ushort id;
        public ushort num_logs;
        public ushort last_log_num;
        public TimeMsg time_utc;
        public uint size;

        public LogEntryMsg()
        {
            this.header = new Std.HeaderMsg();
            this.id = 0;
            this.num_logs = 0;
            this.last_log_num = 0;
            this.time_utc = new TimeMsg();
            this.size = 0;
        }

        public LogEntryMsg(Std.HeaderMsg header, ushort id, ushort num_logs, ushort last_log_num, TimeMsg time_utc, uint size)
        {
            this.header = header;
            this.id = id;
            this.num_logs = num_logs;
            this.last_log_num = last_log_num;
            this.time_utc = time_utc;
            this.size = size;
        }

        public static LogEntryMsg Deserialize(MessageDeserializer deserializer) => new LogEntryMsg(deserializer);

        private LogEntryMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.id);
            deserializer.Read(out this.num_logs);
            deserializer.Read(out this.last_log_num);
            this.time_utc = TimeMsg.Deserialize(deserializer);
            deserializer.Read(out this.size);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.id);
            serializer.Write(this.num_logs);
            serializer.Write(this.last_log_num);
            serializer.Write(this.time_utc);
            serializer.Write(this.size);
        }

        public override string ToString()
        {
            return "LogEntryMsg: " +
            "\nheader: " + header.ToString() +
            "\nid: " + id.ToString() +
            "\nnum_logs: " + num_logs.ToString() +
            "\nlast_log_num: " + last_log_num.ToString() +
            "\ntime_utc: " + time_utc.ToString() +
            "\nsize: " + size.ToString();
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
