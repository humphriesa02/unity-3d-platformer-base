using UnityEngine;

[RequireComponent (typeof(Camera))]
public class OrbitCamera : MonoBehaviour
{
    [SerializeField, Tooltip("Object for the camera to focus on / follow.")]
    Transform focus = default;

    [SerializeField, Range(1f, 20f), Tooltip("Distance from the camera to the object being followed.")]
    float distance = 5f;

	[SerializeField, Min(0f), Tooltip("Camera will only move when its focus point differs too much from ideal focus. This radius describes that distance.")]
	float focusRadius = 1f;

	[SerializeField, Range(0f, 1f), Tooltip("Pulls the camera towards the focused object slowly, moving exponential, when the object stops moving. This is the rate of pull.")]
	float focusCentering = 0.5f;

	[SerializeField, Range(1f, 360f), Tooltip("Rotation speed configuration option, expressed in degrees per second")]
	float rotationSpeed = 90f;

	[SerializeField, Range(-89f, 89f), Tooltip("Min and max angle to constrain the vertical camera rotation.")]
	float minVerticalAngle = -45f, maxVerticalAngle = 45f;

	[SerializeField, Min(0f), Tooltip("Automatically aligns the horizontal axis for the target after the delay.")]
	float alignDelay = 5f;

	[SerializeField, Range(0f, 90f), Tooltip("Angle at which we scale to full speed camera alignment.")]
	float alignSmoothRange = 45f;

	[SerializeField, Tooltip("Layer mask ignored by the camera")]
	LayerMask obstructionMask = -1;

	Camera regularCamera;

	// Point the camera is focusing on, relative to the focus object
	Vector3 focusPoint, previousFocusPoint;

	// Describes the orientation of the camera in angles. X represents vertical orientation
	// where 0 looks straight ahead and 90 degrees looks straight down.
	// Y angle describes horizontal orientation with 0 being straight ahead.
	Vector2 orbitAngles = new Vector2(45f, 0f);

	// Last time that a manual rotation happened
	float lastManualRotationTime;

	// For box casting the camera raycast. Contains
	// half extends of a box. Found by taking the tan
	// of half the camera's field of view angle in radians,
	// scaled by its near clip plane distance. 
	Vector3 CameraHalfExtends
	{
		get
		{
			Vector3 halfExtends;
			halfExtends.y = regularCamera.nearClipPlane *
				Mathf.Tan(0.5f * Mathf.Deg2Rad * regularCamera.fieldOfView);
			halfExtends.x = halfExtends.y * regularCamera.aspect;
			halfExtends.z = 0f;
			return halfExtends;
		}
	}

	private void OnValidate()
	{
		// Sanitizes angle configuration in editor
		if (maxVerticalAngle < minVerticalAngle)
		{
			maxVerticalAngle = minVerticalAngle;
		}
	}

	private void Awake()
	{
		regularCamera = GetComponent<Camera>();
		focusPoint = focus.position;
		Cursor.lockState = CursorLockMode.Locked;
		transform.localRotation = Quaternion.Euler(orbitAngles);
	}

	private void LateUpdate()
	{
		UpdateFocusPoint();
		Quaternion lookRotation;

		if (ManualRotation() || AutomaticRotation())
		{
			ConstrainAngles();
			lookRotation = Quaternion.Euler(orbitAngles);
		}
		else
		{
			lookRotation = transform.localRotation;
		}

		// Skew the look direction by the look rotation
		Vector3 lookDirection = lookRotation * Vector3.forward;
		Vector3 lookPosition = focusPoint - lookDirection * distance;

		Vector3 rectOffset = lookDirection * regularCamera.nearClipPlane;
		Vector3 rectPosition = lookPosition + rectOffset;
		Vector3 castFrom = focus.position;
		Vector3 castLine = rectPosition - castFrom;
		float castDistance = castLine.magnitude;
		Vector3 castDirection = castLine / castDistance;

		if (Physics.BoxCast(
			castFrom, CameraHalfExtends, castDirection, out RaycastHit hit,
			lookRotation, castDistance, obstructionMask
		))
		{
			rectPosition = castFrom + castDirection * hit.distance;
			lookPosition = rectPosition - rectOffset;
		}

		transform.SetPositionAndRotation(lookPosition, lookRotation);
	}

	void UpdateFocusPoint()
	{
		previousFocusPoint = focusPoint;
		Vector3 targetPoint = focus.position;
		
		// Check if we're using deadzones, else just snap the focus point back
		if (focusRadius > 0f)
		{
			// Distance from where we want to look, to where we currently are looking
			float distance = Vector3.Distance(targetPoint, focusPoint);
			float t = 1f;

			// If the camera is not yet focused on the target object, and we have
			// a focusCentering value, set the timestep for the focust point movement
            if (distance > 0.01f && focusCentering > 0f)
            {
				t = Mathf.Pow(1f - focusCentering, Time.unscaledDeltaTime); // Camera will still move normal when in game time slows / stops
            }

            // If the distance exceeds the radius defined
            if (distance > focusRadius)
			{
				// Set timestep for moving the focusPoint to the target
				// Either the existing focusCentering timestep, or focus / distance
				// Whichever is less
				t = Mathf.Min(t, focusRadius / distance);
			}
			// Lerp the focus point back to the target 
			focusPoint = Vector3.Lerp(targetPoint, focusPoint, t);
		}
		else
		{
			focusPoint = targetPoint;
		}
	}

	bool ManualRotation()
	{
		Vector2 input = new Vector2(
			Input.GetAxis("Vertical Camera"),
			Input.GetAxis("Horizontal Camera")
			);
		const float e = 0.001f;
		if (input.x < -e || input.x > e || input.y < -e || input.y > e)
		{
			orbitAngles += rotationSpeed * Time.unscaledDeltaTime * input;
			lastManualRotationTime = Time.unscaledTime;
			return true;
		}
		return false;
	}

	bool AutomaticRotation()
	{
		if (Time.unscaledTime - lastManualRotationTime < alignDelay)
		{
			return false;
		}

		Vector2 movement = new Vector2(
			focusPoint.x - previousFocusPoint.x,
			focusPoint.z - previousFocusPoint.z
			);
		float movementDeltaSqr = movement.sqrMagnitude;
		if (movementDeltaSqr < 0.0001f)
		{
			return false;
		}

		// Give a new angle to automatically return to
		float headingAngle = GetAngle(movement / Mathf.Sqrt(movementDeltaSqr));
		float deltaAbs = Mathf.Abs(Mathf.DeltaAngle(orbitAngles.y, headingAngle));
		float rotationChange = rotationSpeed * Mathf.Min(Time.unscaledDeltaTime, movementDeltaSqr);
		if (deltaAbs < alignSmoothRange)
		{
			rotationChange *= deltaAbs / alignSmoothRange;
		}
		else if (180f - deltaAbs < alignSmoothRange)
		{
			rotationChange *= (180f - deltaAbs) / alignSmoothRange;
		}
		orbitAngles.y = Mathf.MoveTowardsAngle(orbitAngles.y, headingAngle, rotationChange);
		return true;
	}

	// Figure out the horizontal angle matching the current direction
	// Converts 2D direction to an angle
	static float GetAngle (Vector2 direction)
	{
		float angle = Mathf.Acos(direction.y) * Mathf.Rad2Deg;
		return direction.x < 0f ? 360f - angle : angle;
	}

	void ConstrainAngles()
	{
		orbitAngles.x = Mathf.Clamp( orbitAngles.x, minVerticalAngle, maxVerticalAngle);

		if (orbitAngles.y < 0f)
		{
			orbitAngles.y += 360f;
		}
		else if (orbitAngles.y > 360f)
		{
			orbitAngles.y -= 360f;
		}
	}
}
