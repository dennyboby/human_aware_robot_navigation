using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.People;

public class PeoplePublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    // Variables required for ROS communication
    public string peopleTopicName = "people";
    // Transform
    public Transform publishedTransform;
    // Message
    private PeopleMsg peopleMsg;
    // private PersonMsg personMsg;
    private string frameID = "map";
    public float publishRate = 10f;
    public CharacterWalk script;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PeopleMsg>(peopleTopicName);

        // Initialize message
        peopleMsg = new PeopleMsg
        {
            header = new HeaderMsg(Clock.GetCount(),
                                   new TimeStamp(Clock.time), frameID)
        };
        InvokeRepeating("PublishPoseStamped", 1f, 1f/publishRate);
    }

    private void PublishPoseStamped()
    {
        peopleMsg.header = new HeaderMsg(Clock.GetCount(), 
                                           new TimeStamp(Clock.time), frameID);
        var orientation = publishedTransform.eulerAngles;
        var personMsg = new PersonMsg
        {
            name = "person",
            position = publishedTransform.position.To<FLU>(),
            velocity = new PointMsg(script.speed*Mathf.Cos(orientation[1]*Mathf.Deg2Rad),
                    -script.speed*Mathf.Sin(orientation[1]*Mathf.Deg2Rad),0),
            reliability = 1.0,
            tagnames = new string[] { "tag1", "tag2" },
            tags = new string[] { "tag1", "tag2" }
        };
        peopleMsg.people = new PersonMsg[] { personMsg };

        ros.Publish(peopleTopicName, peopleMsg);
    }
}
