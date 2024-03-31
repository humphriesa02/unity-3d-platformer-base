using UnityEngine;

/**
 * Made by Jasper Flick A.K.A CatLikeCoding
 * https://catlikecoding.com/unity/tutorials/movement/
 * 
 * Adapted by Alex Humphries
 * 
 * 3D Character Controller
 */
public class MovingSphere : MonoBehaviour
{
	[SerializeField, Range(0f, 100f), Tooltip("How fast the sphere is. Used to calculate velocity.")]
	float maxSpeed = 10f;

	[SerializeField, Range(0f, 100f), Tooltip("Controls how quickly we reach max speed.")]
	float maxAcceleration = 10f;

	[SerializeField, Range(0f, 100f), Tooltip("Controls how much movement sphere has in air.")]
	float maxAirAcceleration = 1f;

	[SerializeField, Range(0f, 10f), Tooltip("Controls how high the spehere jumps.")]
	float jumpHeight = 2f;

	[SerializeField, Range(0, 5), Tooltip("Controls how many jumps the sphere has after an initial jump.")]
	int maxAirJumps = 2;

	[SerializeField, Range(0f, 90f), 
		Tooltip("Controls the angle in which the sphere is considered 'on the ground'." +
		" Any slope above this angle will not be considered ground.")]
	float maxGroundAngle = 25f;

	[SerializeField, Range(0f, 90f), Tooltip("Controls the angle in which the sphere is considered 'on stairs'." +
		" Any slope above this angle will not be considered on stairs. Allows for a secondary angle for steeper stair slopes")]
	float maxStairsAngle = 50f;

	[SerializeField, Range(0f, 100f), Tooltip("The speed the sphere needs to move at to miss snapping to the ground.")]
	float maxSnapSpeed = 100f;

	[SerializeField, Min(0f), Tooltip("Distance to shoot the ray to check for snapping to ground.")]
	float probeDistance = 1f;

	[SerializeField, Tooltip("Include any ground objects - things the sphere can snap to. Used in raycast.")]
	LayerMask probeMask = -1;

	[SerializeField, Tooltip("Used to determine if we're on stairs or not, so we can use the stair angle. Assign to stairs layer.")]
	LayerMask stairsMask = -1;

	[SerializeField, Tooltip("Custom space to define player input")]
	Transform playerInputSpace = default;

	// Part of collision with ground / stairs detection
	float minGroundDotProduct, minStairsDotProduct;

	// Velocity and what we want velocity to be
	Vector3 velocity, desiredVelocity;

	// Are we trying to jump?
	bool desiredJump;

	// Direction away from collision. I.e. contact normal ~ opposite direction of
	// when colliding with ground. Steep is opposite when colliding with wall
	Vector3 contactNormal, steepNormal;

	// Number of collision contacts for ground and walls
	int groundContactCount, steepContactCount;

	// If we have collision contacts on the ground OnGround is true.
	bool OnGround => groundContactCount > 0;

	// If we have collision contacts on the walls OnSteep is true.
	bool OnSteep => steepContactCount > 0;

	// Tracks how many jumps we've used.
	int jumpPhase;

	// Counts how many 'steps' we've taken since we've been on the ground
	// How many frames
	int stepsSinceLastGrounded = 0;
	// Counts how many 'steps' we've taken since we've last jumped
	// How many frames
	int stepsSinceLastJump = 0;

	Rigidbody body;

	
	private void OnValidate()
	{
		// Calculate the min dot products for ground and stairs on first awake
		minGroundDotProduct = Mathf.Cos(maxGroundAngle * Mathf.Deg2Rad);
		minStairsDotProduct = Mathf.Cos(maxStairsAngle * Mathf.Deg2Rad);
	}

	private void Awake()
	{
		body = GetComponent<Rigidbody>();
		OnValidate();
	}

	private void Update()
	{
		// Input gathering
		Vector2 playerInput;
		playerInput.x = Input.GetAxis("Horizontal");
		playerInput.y = Input.GetAxis("Vertical");
		// Clamp input so diagonals aren't faster
		playerInput = Vector2.ClampMagnitude(playerInput, 1f);

		if (playerInputSpace)
		{
			Vector3 forward = playerInputSpace.forward;
			forward.y = 0f;
			forward.Normalize();

			Vector3 right = playerInputSpace.right;
			right.y = 0f;
			right.Normalize();
			desiredVelocity =
				(forward * playerInput.y + right * playerInput.x) * maxSpeed;
		}
		else
		{
			// The velocity we wish to go based on speed and input
			desiredVelocity = new Vector3(playerInput.x, 0f, playerInput.y) * maxSpeed;
		}
		
		// Polling for jump input
		desiredJump |= Input.GetButtonDown("Jump");
	}

	private void FixedUpdate()
	{
		UpdateState();
		AdjustVelocity();

		if (desiredJump)
		{
			desiredJump = false;
			Jump();
		}

		// Apply calculated velocity. This is how movement is applied
		body.velocity = velocity;

		ClearState();
	}

	/**
	 * Calculates velocity and modifies it here
	 * Since velocity is a member variable, it is not returned here
	 * but instead just modified.
	 */
	void AdjustVelocity()
	{
		// determining the projected X and Z axes by projecting
		// the right and forward vectors on the contact plane.
		// Vectors are normalized to get proper directions
		Vector3 xAxis = ProjectOnContactPlane(Vector3.right).normalized;
		Vector3 zAxis = ProjectOnContactPlane(Vector3.forward).normalized;

		// project the current velocity on both vectors to get the relative X and Z speeds.
		float currentX = Vector3.Dot(velocity, xAxis);
		float currentZ = Vector3.Dot(velocity, zAxis);

		// Acceleration speed dependent on if we are on the ground
		float acceleration = OnGround ? maxAcceleration : maxAirAcceleration;

		// Apply delta time to acceleration for speed change
		float maxSpeedChange = acceleration * Time.deltaTime;

		// Move from the current x and z speeds to the desired speeds using speed change
		float newX = Mathf.MoveTowards(currentX, desiredVelocity.x, maxSpeedChange);
		float newZ = Mathf.MoveTowards(currentZ, desiredVelocity.z, maxSpeedChange);

		// Fancy velocity calculation
		velocity += xAxis * (newX - currentX) + zAxis * (newZ - currentZ);
	}

	/**
	 * Used for correctly calculating velocity on slopes
	 * Returns a vector correlating to the axis
	 * achieved by determining how much slope we have.
	 * More info here: https://catlikecoding.com/unity/tutorials/movement/physics/
	 * and image describing here: https://catlikecoding.com/unity/tutorials/movement/physics/slopes/projecting-vector.png
	 */
	Vector3 ProjectOnContactPlane(Vector3 vector)
	{
		return vector - contactNormal * Vector3.Dot(vector, contactNormal);
	}

	// Clears collision data
	void ClearState()
	{
		groundContactCount = steepContactCount =  0;
		contactNormal = steepNormal = Vector3.zero;
	}

	void UpdateState()
	{
		stepsSinceLastGrounded += 1;
		stepsSinceLastJump += 1;
		velocity = body.velocity;
		if (OnGround || SnapToGround() || CheckSteepContacts())
		{
			stepsSinceLastGrounded = 0;
			if (stepsSinceLastJump > 1) // Avoid false landing
			{
				jumpPhase = 0;
			}
			if (groundContactCount > 1)
			{
				contactNormal.Normalize();
			}
		}
		else
		{
			contactNormal = Vector3.up;
		}
	}

	void Jump()
	{
		Vector3 jumpDirection;
		if (OnGround)
		{
			jumpDirection = contactNormal;
		}
		else if (OnSteep)
		{
			jumpDirection = steepNormal;
			jumpPhase = 0;
		}
		else if (maxAirJumps > 0 && jumpPhase <= maxAirJumps)
		{
			if (jumpPhase == 0)
			{
				jumpPhase = 1;
			}
			jumpDirection = contactNormal;
		}
		else
		{
			return;
		}

		stepsSinceLastJump = 0;
		jumpPhase += 1;
		float jumpSpeed = Mathf.Sqrt(-2f * Physics.gravity.y * jumpHeight);
		jumpDirection = (jumpDirection + Vector3.up).normalized; // Upwards jump bias, for wall jumps
		float alignedSpeed = Vector3.Dot(velocity, jumpDirection);
		if (alignedSpeed > 0f)
		{
			jumpSpeed = Mathf.Max(jumpSpeed - alignedSpeed, 0f);
		}
		velocity += jumpDirection * jumpSpeed;
	}

	private void OnCollisionEnter(Collision collision)
	{
		EvaluateCollision(collision);
	}

	private void OnCollisionStay(Collision collision)
	{
		EvaluateCollision(collision);
	}

	void EvaluateCollision(Collision collision)
	{
		float minDot = GetMinDot(collision.gameObject.layer);
		for (int i = 0; i < collision.contactCount; i++)
		{
			Vector3 normal = collision.GetContact(i).normal;
			if (normal.y >= minDot)
			{
				groundContactCount += 1;
				contactNormal += normal;
			}
			else if (normal.y > -0.01f)
			{
				steepContactCount += 1;
				steepNormal += normal;
			}
		}
	}

	bool SnapToGround()
	{
		// Determine when NOT to snap to ground

		// If not immediately after leaving the ground
		if (stepsSinceLastGrounded > 1 || stepsSinceLastJump <= 2)
		{
			return false;
		}
		float speed = velocity.magnitude;
		if (speed > maxSnapSpeed)
		{
			return false;
		}
		// If there is nothing below the sphere
		if (!Physics.Raycast(body.position, Vector3.down, out RaycastHit hit, probeDistance, probeMask))
		{
			return false;
		}

		// Determine if what we hit is ground
		if (hit.normal.y < GetMinDot(hit.collider.gameObject.layer))
		{
			return false;
		}

		// snap to ground
		groundContactCount = 1;
		contactNormal = hit.normal;
		float dot = Vector3.Dot(velocity, hit.normal);
		if (dot > 0f)
		{
			velocity = (velocity - hit.normal * dot).normalized * speed;
		}
		return true;
	}

	float GetMinDot (int layer)
	{
		return (stairsMask & (1 << layer)) == 0 ? minGroundDotProduct : minStairsDotProduct;
	}

	bool CheckSteepContacts()
	{
		if (steepContactCount > 1)
		{
			steepNormal.Normalize();
			if (steepNormal.y >= minGroundDotProduct)
			{
				groundContactCount = 1;
				contactNormal = steepNormal;
				return true;
			}
		}
		return false;
	}
}
