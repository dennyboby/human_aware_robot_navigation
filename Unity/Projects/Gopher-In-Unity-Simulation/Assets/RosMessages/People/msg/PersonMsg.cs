//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.People
{
    [Serializable]
    public class PersonMsg : Message
    {
        public const string k_RosMessageName = "people_msgs/Person";
        public override string RosMessageName => k_RosMessageName;

        public string name;
        public Geometry.PointMsg position;
        public Geometry.PointMsg velocity;
        public double reliability;
        public string[] tagnames;
        public string[] tags;

        public PersonMsg()
        {
            this.name = "";
            this.position = new Geometry.PointMsg();
            this.velocity = new Geometry.PointMsg();
            this.reliability = 0.0;
            this.tagnames = new string[0];
            this.tags = new string[0];
        }

        public PersonMsg(string name, Geometry.PointMsg position, Geometry.PointMsg velocity, double reliability, string[] tagnames, string[] tags)
        {
            this.name = name;
            this.position = position;
            this.velocity = velocity;
            this.reliability = reliability;
            this.tagnames = tagnames;
            this.tags = tags;
        }

        public static PersonMsg Deserialize(MessageDeserializer deserializer) => new PersonMsg(deserializer);

        private PersonMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            this.position = Geometry.PointMsg.Deserialize(deserializer);
            this.velocity = Geometry.PointMsg.Deserialize(deserializer);
            deserializer.Read(out this.reliability);
            deserializer.Read(out this.tagnames, deserializer.ReadLength());
            deserializer.Read(out this.tags, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.Write(this.position);
            serializer.Write(this.velocity);
            serializer.Write(this.reliability);
            serializer.WriteLength(this.tagnames);
            serializer.Write(this.tagnames);
            serializer.WriteLength(this.tags);
            serializer.Write(this.tags);
        }

        public override string ToString()
        {
            return "PersonMsg: " +
            "\nname: " + name.ToString() +
            "\nposition: " + position.ToString() +
            "\nvelocity: " + velocity.ToString() +
            "\nreliability: " + reliability.ToString() +
            "\ntagnames: " + System.String.Join(", ", tagnames.ToList()) +
            "\ntags: " + System.String.Join(", ", tags.ToList());
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
