using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

[ExecuteInEditMode]
public class ObjectProperties : MonoBehaviour {
	//public virtual void ApplyProperties(float time) {}
	public virtual void Run() {}
	public virtual void Setup() {}
	[NonSerialized] private bool hasSetup;

	private bool awakeRan;

	public void ApplyProperties(float timeIn, TimeOfDayManager timeOfDayManagerIn) {
		timeOfDayManager = timeOfDayManagerIn;
		//timeOfDayManagero = timeOfDayManagerIn.gameObject;
		time = timeIn;
		if (!awakeRan) return;
		if (!enabled) return; // Only Runs if object is enabled, but if disabled will update the time variable ready for when it is enabled
		if (!hasSetup) OnEnable();
		Run();
	}

	[SerializeField] public TimeOfDayManager timeOfDayManager;

	//[SerializeField] private GameObject timeOfDayManagero;

	[Range(0.0f, 1.0f)] public float time;

	internal void Awake() {
		awakeRan = true;
		if (mirrorLight) mirror();
		Setup();
	}

	public bool debugLogness;

	public void OnEnable() {
		if (debugLogness) Debug.Log("Enable Properties", this);
		if (timeOfDayManager == null) {
			Debug.LogWarning("no timeOfDayManager found, so finding one now ( you should make sure TimeOfDayManager runs first and finds all objects it wants to control", this); // If a TimeOfDayManager fails to claim this as something for it to control, the item itself can find a TimeOfDayManager and tell it to control it, this is expensive at the moment as each time this happens it requests the TimeOfDayManager seek all scene objects, so is used in emergency only hence the above warning
			timeOfDayManager = FindObjectOfType<TimeOfDayManager>(); // may have multiple managers in future so bad
		}


		if (timeOfDayManager != null) timeOfDayManager.AmIadded(this); // For editor use
		if (mirrorLight) mirror();
		Setup();
		hasSetup = true;
		Run();
	}

	[NonSerialized]
	private float lastSentTime;
	internal void OnValidate() { // For editor use


		if (debugLogness) Debug.Log("Validate Object: " + hasSetup, this);
		if (hasSetup) {
			if (!mirrorLight) lightProp = null;
			if (mirrorLight && lightProp == null) mirror();

			//Run();
			if (timeOfDayManager != null) {
				if (lastSentTime != time && time != 0 ) { // TODO hack, we want manual alteration of the time slider to cause an update, but for exampple 'AgentProperties' causes an 'OnValidate' to be called as part of it's setup

					//Debug.Log("lastSentTime:"+lastSentTime+":"+time,this);

					timeOfDayManager.SetTime(time);
					lastSentTime = time;
				}
				timeOfDayManager.AmIadded(this); // This checks even if timeOfDayManager is not null, does it also have a reference to me, as its possible it got orphaned/forgotten by it

				Run();
			} else {
				Debug.Log("timeOfDayManager null, new object, run Manager first, running all of them now", this);
				FindObjectsOfType<TimeOfDayManager>().ToList().ForEach(o => o.Setup());
			}
			if (timeOfDayManager == null) Debug.LogWarning("no timeOfDayManager wants me!", this);
		}
	}

	[Tooltip("looks on self and in childen for Light ( LightProperties ) to copy values from")] public bool mirrorLight;
	public LightProperties lightProp; // gets wiped each setup

	public void mirror() {
		//if (mirrorLight) {
		//lightProp = GetComponent<LightProperties>();
		//if (!(this is LightProperties)) lightProp = GetComponent<LightProperties>();
		lightProp = null;
		List<LightProperties> props = null;


		props = GetComponentsInChildren<LightProperties>().ToList();
		if (props.Count > 0) {
			foreach (var lightProperties in props) {
				if (lightProperties != this && lightProperties.mirrorLight == false) {
					lightProp = lightProperties;
					break;
				}
			}
		}

		if (lightProp == null) props = transform.parent.GetComponentsInChildren<LightProperties>().ToList();
		if (props.Count > 0) {
			foreach (var lightProperties in props) {
				if (lightProperties != this && lightProperties.mirrorLight == false) {
					lightProp = lightProperties;
					break;
				}
			}
		}

		//if (lightProp == null) props = GetComponentsInParent<LightProperties>().ToList();
		//if (props.Count > 0) {
		//	foreach (var lightProperties in props) {
		//		if (lightProperties != this && lightProperties.mirrorLight == false) {
		//			lightProp = lightProperties;
		//			break;
		//		}
		//	}
		//}

		if (lightProp == null) {
			//mirrorLight = false;
			Debug.Log("failed to mirrorLight, no Light found", this);
		} else {
			lightProp.AddToFollowers(this);
			//_color = lightProp._color;
			//randomDelay = lightProp._randomDelayRange;
		}
		//}
	}
}