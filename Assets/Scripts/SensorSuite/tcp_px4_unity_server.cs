using System;
using System.Collections; 
using System.Collections.Generic; 
using System.Net; 
using System.Net.Sockets; 
using System.Text; 
using System.Threading; 
using UnityEngine;  

using sensors_suite;

public class tcp_px4_unity_server : MonoBehaviour {  	
	#region private members 	
	/// <summary> 	
	/// TCPListener to listen for incomming TCP connection 	
	/// requests. 	
	/// </summary> 	
	private TcpListener tcpListener; 
	/// <summary> 
	/// Background thread for TcpServer workload. 	
	/// </summary> 	
	private Thread tcpListenerThread;  	
	/// <summary> 	
	/// Create handle to connected tcp client. 	
	/// </summary> 	
	private TcpClient connectedTcpClient; 	
	#endregion 	

    //the name of the connection, not required but better for overview if you have more than 1 connections running
	public string conName = "localhost";
	
	//ip/address of the server, 127.0.0.1 is for your own computer
	public string conHost = "127.0.0.1";
	
	//port for the server, make sure to unblock this in your router firewall if you want to allow external connections
	public int conPort = 4560;
	public GameObject imu, state, gps;

	private bool isOpen;
	public MAVLink.mavlink_heartbeat_t Hb_data;
	public MAVLink.mavlink_hil_actuator_controls_t controls_data;
	public MAVLink.mavlink_hil_gps_t gps_data;
	public MAVLink.mavlink_hil_sensor_t sensors_data;
	public MAVLink.mavlink_hil_state_quaternion_t state_data;
	private float start_delay = 0.0f;
		
	// Use this for initialization
	void Start () { 		
		isOpen = true;
		// Start TcpServer background thread 		
		tcpListenerThread = new Thread (new ThreadStart(ListenForIncommingRequests)); 		
		tcpListenerThread.IsBackground = true; 		
		tcpListenerThread.Start(); 	
	}  	
	
	// Update is called once per frame
	void Update () { 
		start_delay = Time.deltaTime + start_delay; 
		// Don't send until sensor suite is stabilized, which means that it must initialized to send home state to PX4
		if (start_delay < 2.0f)
            return;

		SendMessage();         
	}  	
	
	/// <summary> 	
	/// Runs in background TcpServerThread; Handles incomming TcpClient requests 	
	/// </summary> 	
	private void ListenForIncommingRequests () { 	
		try { 			
			// Create listener on localhost port 8052. 			
			tcpListener = new TcpListener(IPAddress.Parse(conHost), conPort); 			
			tcpListener.Start();    
			// Server starting          
			Debug.Log("Server is listening");              
			Byte[] bytes = new Byte[1024];  			
			while (isOpen) { 				
                MAVLink.MAVLinkMessage packet;
				using (connectedTcpClient = tcpListener.AcceptTcpClient()) { 					
					// Get a stream object for reading 					
					using (NetworkStream stream = connectedTcpClient.GetStream()) { 
                        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
                        packet = mavlink.ReadPacket(stream);	
						int length = packet.Length;
						
                        // Read incomming stream into byte arrary. 						
                        while ((length = stream.Read(bytes, 0, bytes.Length)) != 0) { 							
                        	// check its valid
							if (packet == null || packet.data == null)
								continue;	
							uint msgid = packet.msgid;
							Debug.Log("Client msgid " + msgid);					
                        } 					
					} 				
				} 			
			} 		
		} 		
		catch (SocketException socketException) { 			
			Debug.Log("SocketException " + socketException.ToString()); 		
		}     
	}  	
	/// <summary> 	
	/// Send message to client using socket connection. 	
	/// </summary> 	
	private void SendMessage() { 		
		if (connectedTcpClient == null) {             
			return;         
		}  		
		
		try { 			
			MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
			imu_baro_mag_sensor imu_tmp = new imu_baro_mag_sensor();
			imu_tmp = imu.GetComponent<imu_baro_mag_sensor>();
			sensors_data = imu_tmp.px4_full_sensor_data(true, true, true);
			
			state_sensor state_tmp = new state_sensor();
			state_tmp = state.GetComponent<state_sensor>();
			state_data = state_tmp.px4_state_sensor_data();
			
			gps_sensor gps_tmp = new gps_sensor();
			gps_tmp = gps.GetComponent<gps_sensor>();
			gps_data = gps_tmp.px4_gps_sensor_data();
			Debug.Log("Data initialized");

			// Convert message to byte array.
			byte[] buffer_sensors = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_SENSOR, sensors_data);
			byte[] buffer_state = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_STATE, state_data);
			byte[] buffer_gps = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HIL_GPS, gps_data);
			
			NetworkStream stream = connectedTcpClient.GetStream();

			// Write byte array to socket connection stream
			stream.Write(buffer_gps, 0, buffer_gps.Length); 
			stream.Write(buffer_sensors, 0, buffer_sensors.Length);   
			stream.Write(buffer_state, 0, buffer_state.Length);   
			         
			Debug.Log("Server sent his message - should be received by client");      
		} 		
		catch (SocketException socketException) {             
			Debug.Log("Socket exception: " + socketException);         
		} 	
	} 

	void OnApplicationQuit()
	{
		isOpen = false;
		tcpListener.Stop();
		// wait for listening thread to terminate (max. 500ms)
		tcpListenerThread.Join(500);
	}
}