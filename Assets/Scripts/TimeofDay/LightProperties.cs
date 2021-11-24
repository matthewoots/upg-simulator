using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Random = UnityEngine.Random;

[ExecuteInEditMode]
public class LightProperties : ObjectProperties {
	public Gradient activePeriod;
	//public float? brightness;
	public bool alwaysOn;

	public AudioSource sound, enableSound;
	private float soundVol,enableSoundVol;
	private float soundPitch,enableSoundPitch;

	public Light _light;
	public Vector2 _randomDelayRange;

	[NonSerialized] public List<ObjectProperties> followers = new List<ObjectProperties>();


	public void AddToFollowers(ObjectProperties item) {
		if (followers == null) followers = new List<ObjectProperties>();
		if (!followers.Contains(item)) followers.Add(item);
	}

	private float randomDelayVal, randomDelayVal2;
	private const float _lightEnableThreshold = 0.01f;
	private const float frameRate = 60f;

	public float enableLength, disableLength;

	public AnimationCurve enableCurve = new AnimationCurve(new Keyframe(0, 0), new Keyframe(1, 1));
	public AnimationCurve disableCurve = new AnimationCurve(new Keyframe(1, 1), new Keyframe(0, 0));

	public override void Setup() {
		if (sound != null) soundVol = sound.volume;
		if (enableSound != null) enableSoundVol = enableSound.volume;
		if (sound != null) soundPitch = sound.pitch;
		if (enableSound != null) enableSoundPitch = enableSound.pitch;

		_light = GetComponent<Light>();
		if (_light == null) {
			Debug.LogWarning("No light found", this);
			return;
		}
		if (targetColor == new Color(0, 0, 0)) targetColor = _light.color;
		//enabledTarget = _light.enabled;
		if(alwaysOn)StartSound();
		//if (brightness == null) brightness = _light.intensity;
	}

	public bool useGradientColor = false; // TODO not implimented, should switch to Alpha then and keep color solid
	public Color32 targetColor;

	[NonSerialized]
	private bool? enabledTarget = null; // This is like _light.enabled however as light animates on and off, I need to know if it is turningoff/or off and turning on or on

	public override void Run() {
		if (_light == null) return;


		float tdimmer = 1;
		if(activePeriod!=null) tdimmer = activePeriod.Evaluate(time).r; // does not use the Color, just on and off state

		if (alwaysOn) tdimmer = 1;
		if (tdimmer > _lightEnableThreshold) {
			tdimmer = 1;
		} else {
			tdimmer = 0;
		}
		if(debugLogness)Debug.Log("Run " + tdimmer+":"+_lightEnableThreshold+":"+enabledTarget,this);


		if (mirrorLight && lightProp != null) {
			//Debug.Log("mirroring: " + lightProp.dimmer, this);
			_light.color = Color.Lerp(new Color(0, 0, 0), targetColor, lightProp.dimmer);
			//updateFollowers(); // Shouldnt really have followers if it is itself following something
		} else {
			//currentColor = _color.Evaluate(time);

			if (tdimmer > _lightEnableThreshold) {
				//if (alwaysOn) dimmer = tdimmer;
				if (alwaysOn||enabledTarget==null||(bool) !enabledTarget) {
					//Debug.Log("turning on");

					enabledTarget = true;
					if(debugLogness)Debug.Log("Run " + tdimmer+":"+_lightEnableThreshold,this);
					//if (enableLength > 0) Debug.Log("blar", this);


					//_light.color = lightProp._
					//} else {
					if (alwaysOn||enableLength == 0 || (Application.isEditor && !Application.isPlaying)) {
						dimmer = tdimmer;
						_light.color = targetColor;
						_light.enabled = true;
						if (debugLogness) Debug.Log("enable" + " --- " + tdimmer, this);
						updateFollowers();
					} else {
						randomDelayVal = Random.Range(_randomDelayRange.x, _randomDelayRange.y);
						StartCoroutine(Enable());
					}
					//}

					//enableLoop = 0;
					//CancelInvoke("Enable");
					//CancelInvoke("Disable");
					//if (enableLength > 0) InvokeRepeating("Enable", enableLength, 1f/frameRate);
				}
			} else {
				if (alwaysOn||enabledTarget==null||(bool) enabledTarget) {
					if (debugLogness)Debug.Log("turning off");
					enabledTarget = false;

					//

					if (alwaysOn||disableLength == 0 || (Application.isEditor && !Application.isPlaying)) {
						dimmer = tdimmer;
						_light.color = targetColor;
						_light.enabled = false;
						if (debugLogness) Debug.Log("disable" + " --- " + tdimmer, this);
						updateFollowers();
					} else {

						randomDelayVal2 = Random.Range(_randomDelayRange.x, _randomDelayRange.y);

						StartCoroutine(Disable());
					}

					//disableLoop = 0;
					//CancelInvoke("Enable");
					//CancelInvoke("Disable");
					//if (disableLength > 0) InvokeRepeating("Disable", disableLength, 1f/frameRate);
				}
			}
		}
		//_light.enabled = evalColor.r > _lightEnableThreshold ? true : false;
	}

	public void updateFollowers() {
		if (debugLogness) if (followers.Count > 0) Debug.Log("updateFollowers: " + followers.Count + " --- " + dimmer, this);

		foreach (var objectProperties in followers) {
			if (objectProperties != null) objectProperties.Run();
		}
	}

	//private int enableLoop, disableLoop;
	//[NonSerialized]
	public float dimmer = 1; // TODO set to NonSerialized

	public enum animState {
		none,
		disabling,
		enabling
	}

	public animState currentState = animState.none;

	public void StartSound() {
		if (!Application.isPlaying) return;
		var random = Random.Range(-0.025f,0.025f);
		
		var pitch = soundPitch + random;
		var enablePitch = enableSoundPitch + random;

		if (sound != null) {
			sound.pitch = pitch;
			sound.PlayScheduled(Random.Range(0f, sound.clip.length));
		}
		if (enableSound != null) {
			enableSound.pitch = enablePitch;
			enableSound.Play();
		}
	}

	public void StopSound() {
		if (sound != null) sound.Stop();
	}

	public IEnumerator Enable() {
		float i = 0;
		currentState = animState.enabling;
		if(debugLogness)Debug.Log("Enable: " + i, this);

		yield return new WaitForSeconds(randomDelayVal);
		//print("WaitAndPrint " + Time.time);
		StartSound();

		_light.enabled = true;
		while ((1f/(enableLength*frameRate)*i) < 1f) {
			i++;
			if (currentState == animState.disabling) break;
			dimmer = enableCurve.Evaluate((1/(enableLength*frameRate))*i);
			_light.color = Color.Lerp(new Color(0, 0, 0), targetColor, dimmer);
			if (sound != null) sound.volume = soundVol*(enableCurve.Evaluate((1/(enableLength*frameRate))*i));
			if (enableSound != null) enableSound.volume = enableSoundVol;

			if (debugLogness) Debug.Log("enable: " + i + ":" + (1f/(enableLength*frameRate)*i) + " --- " + dimmer, this);
			updateFollowers();
			yield return new WaitForSeconds(1f/frameRate);
		}
		if (currentState == animState.enabling) {
			if (debugLogness) Debug.Log("Done: " + i, this);
			currentState = animState.none;
		}
		yield return true;
	}

	public IEnumerator Disable() {
		float i = 0;

		currentState = animState.disabling;
		yield return new WaitForSeconds(randomDelayVal2);

		

		while ((1f/(enableLength*frameRate)*i) < 1f) {
			i++;
			if (currentState == animState.enabling) break;
			dimmer = 1 - disableCurve.Evaluate((1/(disableLength*frameRate))*i);
			_light.color = Color.Lerp(new Color(0, 0, 0), targetColor, dimmer);
			if (sound != null) sound.volume = soundVol*(1 - disableCurve.Evaluate((1/(disableLength*frameRate))*i));
			if (enableSound != null) enableSound.volume = enableSoundVol;
			if (debugLogness) Debug.Log("disable: " + i + ":" + (1f/(disableLength*frameRate)*i) + " --- " + dimmer, this);
			updateFollowers();
			yield return new WaitForSeconds(1f/frameRate);
		}
		if (currentState == animState.disabling) {
			_light.enabled = false;
			StopSound();
			currentState = animState.none;
		}
		yield return true;
	}
}