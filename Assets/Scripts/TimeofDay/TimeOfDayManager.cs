using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

//using UnityEditor;
using UnityEngine.Rendering;
using UnityEngine.UI;

// TODO Scale of timeOfday gradients/curves are awkward should adhere to a real 24 hour cycle where sunset and sunrise time can be moved easily, have hard coded times and looping also making it awkward to edit/understand
[ExecuteInEditMode]
public class TimeOfDayManager : MonoBehaviour {
	public bool debug;
	public Slider timeSlider;
	public Slider speedSlider;
	public Vector2 speedCurveMinMax;
	public Toggle autoCycleTimeToggle;

	public float defaultTime = 0.2f;
	public Gradient updateCurve;

	[Tooltip("--- AllFacesAtOnce - render all faces at once spread remaining work over next 8 frames. total take 9 frames. \r" +
	         "--- IndividualFaces - each face over several frames total 14 frames and may produce incorrect results if thingschange quickly.\r" +
	         "--- NoTimeSlicing render the probe entirely in one frame")]
	public ReflectionProbeTimeSlicingMode timeSliceProbes= ReflectionProbeTimeSlicingMode.IndividualFaces;
	public float sensetivityProbes = 20f;
	public float sensetivityProps = 100f;
	public float sensetivitySuns = 1000f;

	public SunProperties sunProperties;


	public bool findAllProbes;
	public List<ReflectionProbe> probes;


	[Range(0.0f, 0.8f)] public float time; // TODO make private but show in inspector as read only

	public bool autoCycleTime = false;
	public float cycleTime = 60.0f;
	[NonSerialized] private bool hasSetup;

	//[SerializeField, HideInInspector] private float lastTime;
	public List<ObjectProperties> props, suns, instantUpdate;

	//[NonSerialized]
	public bool animationControl = false; // Set by AnimationHub

	private void Start() {
		Setup();
	}

	public void Awake() {
		SetTime(defaultTime);
#if UNITY_EDITOR
		if (!Application.isPlaying) {
			UnityEditor.EditorApplication.update -= Update;
			UnityEditor.EditorApplication.update += Update;
		}
#endif
	}


	public void OnEnable() {
		Setup();
		ForceUpdate();
	}

	public void OnDisable() {
		#if UNITY_EDITOR
		if (!Application.isPlaying) UnityEditor.EditorApplication.update -= Update;
#endif
	}

	public void Setup() {
		hasSetup = true;
		SeekItems();
		StaggeredUpdate();
	}

	public void SeekItems() {
		//TODO should limit this to one per frame, end of frame/lateUpdate
		props = FindObjectsOfType<ObjectProperties>().ToList();
		suns = FindObjectsOfType<SunProperties>().Cast<ObjectProperties>().ToList();
		if (findAllProbes) probes = FindObjectsOfType<ReflectionProbe>().ToList();

		props.ForEach(o => o.timeOfDayManager = this);
		suns.ForEach(o => o.timeOfDayManager = this);
		instantUpdate.ForEach(o => o.timeOfDayManager = this);

		Debug.Log("TimeOfDay SeekItems: prop:" + props.Count + " sun:" + suns.Count + " probe:" + probes.Count + " inst:" + instantUpdate.Count(),this);
	}

	// called on slider change value
	public void ChangeTime() {
		if(timeSlider!=null)time = timeSlider.value;
		//UpdateLabel(time.ToString());
		//Debug.Log("ChangeTime:"+time);
		StaggeredUpdate();

	}

	// called on slider change value
	public void ChangeSpeed() {
		if(speedSlider!=null)cycleTime = Mathf.Lerp(speedCurveMinMax.x,speedCurveMinMax.y,speedSlider.value);
		//UpdateLabel(time.ToString());
		//StaggeredUpdate();
	}

	public void SetSpeed(float speed) {

	}


	// called on toggle change value
	public void ChangeContinuousUpdates(bool value) {
		autoCycleTime = value;
	}

	public void Update() {
		if (Input.GetButtonDown("AutoTimeOfDay")) {
			autoCycleTime = !autoCycleTime;
		}
		autoCycleTimeToggle.isOn = autoCycleTime;
		if (Input.GetButton("TimeReverse")) {
			SetTime(time - 0.005f);
		}
		if (Input.GetButton("TimeAdvance")) {
			SetTime(time + 0.005f);
		}
		if (Input.GetButton("SpeedDown")) {
			if (speedSlider != null) speedSlider.value -= 0.025f;
			ChangeSpeed();
		}
		if (Input.GetButton("SpeedUp")) {
			if (speedSlider != null) speedSlider.value += 0.025f;
			ChangeSpeed();
		}
		if (Application.isPlaying && !animationControl) if (autoCycleTime) SetTime(time + Time.deltaTime/cycleTime);
	}

	public void SetTime(float timeIn) {
		if (debug) Debug.Log("timeOfDay: " + timeIn,this);

		var ttime = timeIn;
		var tmax = timeSlider.maxValue;

		if (ttime < 0) {
			ttime = ((ttime%tmax) + tmax)%tmax; // Ugh math, this is how you keep in 0-1 range when a number goes negative, tip of the day ;-)
		} else {
			ttime = ttime%tmax;
		}

		//UpdateLabel(time.ToString());
		timeSlider.value = ttime;
		time = ttime;


		StaggeredUpdate();
	}

	private float lastSun1, lastSun2;

	public void StaggeredUpdate() {
		var ttime = time;

		UpdateInstant();
		//if (sunProperties != null) sunProperties = FindObjectOfType<SunProperties>();

		//var currentSun = 0f;

		//if (sunProperties != null) currentSun = sunProperties._sun.color.r;
		//currentSun = updateCurve.Evaluate(ttime).r;

		//Debug.Log("		sunChange:" + currentSun + ":" + (lastSun1 - currentSun));


		var ttimerounded2 = Mathf.Round(ttime*sensetivityProps)/sensetivityProps;
		if (Math.Abs(ttimerounded2 - propsLastUpdate) > 0.00001f) {
			UpdateProps();
			propsLastUpdate = ttimerounded2;
		}

		//if ((lastSun1 - currentSun) > (1/sensetivityProps)) {
		//	//UpdateProps();
		//	lastSun1 = currentSun;
		//}

		var ttimerounded = Mathf.Round(ttime*sensetivityProbes)/sensetivityProbes;
		if (Math.Abs(ttimerounded - probesLastUpdated) > 0.00001f) {
			UpdateProbes();
			probesLastUpdated = ttimerounded;
		}

		//if ((lastSun2 - currentSun) > (1/sensetivityProbes)) {
		//	UpdateProbes();
		//	lastSun2 = currentSun;
		//}

		var ttimerounded3 = Mathf.Round(ttime*sensetivitySuns)/sensetivitySuns;
		if (Math.Abs(ttimerounded3 - sunsLastUpdated) > 0.00001f) {
			UpdateSuns();
			sunsLastUpdated = ttimerounded3;
		}
	}

	public void ForceUpdate() {
		UpdateSuns();
		sunsLastUpdated = time;
		UpdateProbes();
		probesLastUpdated = time;
		UpdateProps();
		propsLastUpdate = time;
		UpdateInstant();
	}


	private float probesLastUpdated, propsLastUpdate, sunsLastUpdated;

	private void UpdateProbes() {
		if (debug) Debug.Log("Update Probes: " + time + ":" + probesLastUpdated);
		
		foreach (var reflectionProbe in probes) {
		
			if (reflectionProbe != null) {
				
				if (!Application.isPlaying) {
					reflectionProbe.refreshMode = ReflectionProbeRefreshMode.OnAwake;
					reflectionProbe.RenderProbe();
					// TODO Hack to get it to properly refresh probes in Editor when not playing by toggling refreshMode
				}

				if (reflectionProbe.refreshMode != ReflectionProbeRefreshMode.ViaScripting)
					reflectionProbe.refreshMode = ReflectionProbeRefreshMode.ViaScripting;

				/*
				AllFacesAtOnce	Instructs Unity to use time-slicing by first rendering all faces at once, then spreading the remaining work over the nex 8 frames. Using this option, updating the probe will take 9 frames.

				IndividualFaces	Instructs Unity to spread the rendering of each face over several frames. Using this option, updating the cubemap will take 14 frames. This option greatly reduces the impact on frame rate, however it may produce incorrect results, especially in scenes where lighting conditions change over these 14 frames.

				NoTimeSlicing	Unity will render the probe entirely in one frame.
				*/
				reflectionProbe.timeSlicingMode = timeSliceProbes;
				
				// Only update probe if it doesnt have a customBakedTexture
				if (reflectionProbe.customBakedTexture == null) {
					reflectionProbe.RenderProbe();
					//Debug.Log("Probe update: " +reflectionProbe.name,reflectionProbe );
				}
			}
		}
	}

	private void UpdateInstant() {
		//if (debug) Debug.Log("Update Instant: " + time);
		foreach (ObjectProperties prop in instantUpdate) {
			//if (prop != null && prop.isActiveAndEnabled) prop.ApplyProperties(time, this);
			if (prop != null) prop.ApplyProperties(time, this);
		}
	}

	private void UpdateProps() {
		if (debug) Debug.Log("Update Props: " + time + ":" + propsLastUpdate);
		foreach (ObjectProperties prop in props) {
			//if (prop != null && prop.isActiveAndEnabled) prop.ApplyProperties(time, this);
			if (prop != null) prop.ApplyProperties(time, this);
		}
	}

	private void UpdateSuns() {
		//if (debug) Debug.Log("Update Suns: " + time + ":" + propsLastUpdate);
		foreach (ObjectProperties prop in props) {
			if (prop != null && prop.isActiveAndEnabled) prop.ApplyProperties(time, this);
			if (prop != null) prop.ApplyProperties(time, this);
		}
	}


	// Called when float is changed in inspector
	public void OnValidate() {
		if (hasSetup) {
			//Debug.Log("lastTime:" + lastTime + ":" + time);
			//lastTime = time;
			timeSlider.value = time;
			//UpdateLabel(time.ToString());
			StaggeredUpdate();
		}
	}

	// If a TimeOfDayManager fails to claim an item as something for it to control, the item itself can find a TimeOfDayManager and tell it to control it, this is expensive at the moment as each time this happens it requests the TimeOfDayManager seek all scene objects, so is used in emergency only
	public void AmIadded(ObjectProperties objectProperties) {
		//if (props != null)
		if (!props.Contains(objectProperties) && !suns.Contains(objectProperties) && !instantUpdate.Contains(objectProperties)) {
			// TODO move to event system or send message or observables etc
			Debug.LogWarning(objectProperties.gameObject.name + " was not added so finding all ObjectProperties, bad for performance!", objectProperties.gameObject);
			SeekItems();
		}
	}
}
