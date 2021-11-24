using System;
using UnityEngine;


[ExecuteInEditMode]
public class SunProperties : ObjectProperties {
	public Light _sun;
	public Light _moon;

	public Gradient _sunColor;
	public Gradient _moonColor;
	
	//public AnimationCurve _shadowStrength;
	//public AnimationCurve _shadowStrengthMoon;
	public AnimationCurve bounceBoost;
	public AnimationCurve indirectScale;
	public AnimationCurve ambientIntensity;
	public float ambientMultiply = 1;
	public AnimationCurve skyExposure;
	public AnimationCurve sunExposure;

	public float skyExposureMult = 15;
	public float sunExposureMult = 150;
	public Vector2 angles = new Vector2(-20, 210);
	public Vector2 anglesMoon = new Vector2(-20, 210);
	[NonSerialized] private static readonly Vector3 startSunRotation = new Vector3(0, 0, 0);
	private const float _sunEnableThreshold = 0.01f;
	private const float _moonEnableThreshold = 0.01f;
	public Material skyMaterial;


	public new void Awake() {
		_sun = GetComponent<Light>();
		
		base.Awake();
	}

	public override void Run() {
		UpdateSun(time);
		UpdateMoon(time);
		UpdateSky(time);
	}


	private void UpdateSky(float timeIn) {
		if (skyMaterial == null) return;
		//Debug.Log("Sky:" + time);
		skyMaterial.SetFloat("_RotationPitch2", Mathf.Lerp(180, -180, time*1.25f)); //TODO look up max automatically for this to loop instead of 1.25
		skyMaterial.SetFloat("_Exposure", skyExposure.Evaluate(timeIn)*skyExposureMult);
		skyMaterial.SetFloat("_SunExposure", sunExposure.Evaluate(timeIn)*sunExposureMult);
		//skyMaterial.SetFloat("_Rotation2", Mathf.Lerp(360, 0, time*1.25f));
	}

	private void UpdateSun(float timeIn) {
		if (_sun == null) return;
		_sun.color = _sunColor.Evaluate(timeIn);

		if (ambientIntensity != null) RenderSettings.ambientIntensity = ambientIntensity.Evaluate(timeIn)* ambientMultiply;
		if (indirectScale != null) DynamicGI.indirectScale = indirectScale.Evaluate(timeIn);
		//if(bounceBoost!=null)LightmapEditorSettings.bounceBoost = bounceBoost.Evaluate(timeIn); // TODO not added!??

		_sun.transform.localEulerAngles = new Vector3(Mathf.Lerp(angles.x, angles.y, Mathf.Clamp(timeIn*2, 0, 1)), startSunRotation.y, startSunRotation.z);

		//_sun.shadowStrength = Mathf.Clamp(_shadowStrength.Evaluate(Mathf.Clamp(timeIn*2, 0, 1)), 0, 1);
		//if (_sun.shadowStrength == 0) {
		//	_sun.shadows = LightShadows.None;
		//} else {
		//	_sun.shadows = LightShadows.Soft;
		//}


		if (_sun.color.r > _sunEnableThreshold) {
			_sun.enabled = true;
			_sun.shadows = LightShadows.Soft;
		} else {
			//_sun.enabled = false;
			_sun.shadows = LightShadows.None;
		}
	}

	private void UpdateMoon(float timeIn) {
		_moon.color = _moonColor.Evaluate(timeIn);
		//_moon.enabled = _moon.color.r > _moonEnableThreshold ? true : false;
		if (_moon.color.r > _moonEnableThreshold) {
			_moon.enabled = true;
			_moon.transform.localEulerAngles = new Vector3(Mathf.Lerp(anglesMoon.x, anglesMoon.y, Mathf.Clamp(timeIn - 0.5f, 0, 1)) + startSunRotation.x, startSunRotation.y, startSunRotation.z);
			//_moon.shadowStrength = Mathf.Clamp(_shadowStrengthMoon.Evaluate(Mathf.Clamp(timeIn - 0.5f, 0, 1)), 0, 1);
			//if (_moon.shadowStrength == 0) {
			//	_moon.shadows = LightShadows.None;
			//} else {
			//	_moon.shadows = LightShadows.Soft;
			_moon.shadows = LightShadows.Soft;
			//}
		} else {
			_moon.enabled = false;
			_moon.shadows = LightShadows.None;
		}
	}
}