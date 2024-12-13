using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plane : MonoBehaviour
{
    [Header("Thrust")]
   [SerializeField]
   float maxThrust ;
   [SerializeField]
   float throttleSpeed; 
   [SerializeField]
   float thrustAcceleration;
   [SerializeField]
   float filterFactor;
   [SerializeField]
   float rudderPower;
   [SerializeField]
   AnimationCurve rudderAoACurve;

   [Header("Lift")]
   [SerializeField]
   float liftPower;
   [SerializeField]
   AnimationCurve liftAoACurve;
   [SerializeField]
   float inducedDrag;
   [SerializeField]
   AnimationCurve inducedDragCurve;
   [SerializeField]
   float flapsLiftPower;
   [SerializeField]
   float flapsAOABias;


   [Header("Steering")]
   [SerializeField]
   Vector3 turnSpeed;
   [SerializeField]
   Vector3 turnAcceleration;
   [SerializeField]
   AnimationCurve steeringCurve;

   Vector3 controlInput ;

   public Rigidbody Rigidbody {get; private set;}
   public Vector3 Velocity {get; private set;}
   public Vector3 LocalVelocity{get; private set;}
   public Vector3 LocalAngularVelocity {get; private set;}
   public float AngleofAttack {get; private set;}
   public float AngleofAttackYaw {get; private set;}




    void Start()
    {
        Rigidbody = GetComponent<Rigidbody>(); // Initialize target rotation

    }


    void CalculateAngleOfAttack(){
        if(LocalVelocity.sqrMagnitude<0.1f){
            AngleofAttack = 0;
            AngleofAttackYaw = 0;
            return ;
        }
        AngleofAttack = Mathf.Atan2(-LocalVelocity.y ,  LocalVelocity.z);
        AngleofAttackYaw = Mathf.Atan2(LocalVelocity.x, LocalVelocity.z);
        Debug.Log(AngleofAttack);
        Debug.Log(AngleofAttackYaw);

    }

    void CalculateState(float dt){
        var invRotation = Quaternion.Inverse(Rigidbody.rotation);
        Velocity = Rigidbody.linearVelocity;
        LocalVelocity = invRotation * Velocity;
        LocalAngularVelocity = invRotation * Rigidbody.angularVelocity;

        CalculateAngleOfAttack();
    }

    Vector3 CalculateLift(float angleofAttack, Vector3 rightAxis, float liftPower, AnimationCurve aoaCurve){
        var liftVelocity = Vector3.ProjectOnPlane(LocalVelocity, rightAxis);
        var v2 = liftVelocity.sqrMagnitude;

        var liftCoefficient = aoaCurve.Evaluate(angleofAttack * Mathf.Rad2Deg);
        var liftForce = v2 * liftCoefficient* liftPower;

        var liftDirection = Vector3.Cross(liftVelocity.normalized, rightAxis);
        var lift = liftDirection * liftForce;

        // var dragForce = liftCoefficient * liftCoefficient * this.inducedDrag;
        // var dragDirection = - liftVelocity.normalized;
        // var inducedDrag = dragDirection * v2 * dragForce;

        return lift ;
    }

    Vector3 inputsteering(){
//   float rollBalancer = 0f;
//   float yawBalancer = 0f;
//   float pitchBalancer = 0f;
     if (Input.GetKey(KeyCode.W)){
            controlInput.x += 0.1f;
            controlInput.x = Mathf.Clamp(controlInput.x, -1f, 1f);
            Debug.Log(controlInput);

        }
        if (Input.GetKey(KeyCode.S)){
            controlInput.x -= 0.1f;
            controlInput.x = Mathf.Clamp(controlInput.x, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.LeftArrow)){
            controlInput.y -= 0.1f;
            controlInput.y = Mathf.Clamp(controlInput.y, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.RightArrow)){
            controlInput.y += 0.1f;
            controlInput.y = Mathf.Clamp(controlInput.y, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.A)){
            controlInput.z += 0.1f;
            controlInput.z = Mathf.Clamp(controlInput.z, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.D)){
            controlInput.z -= 0.1f;
            controlInput.z = Mathf.Clamp(controlInput.z, -1f, 1f);
            
        }

        return controlInput;
   }

    float calculateSteering(float dt, float angularVelocity, float targetVelocity, float acceleration){
        var error = targetVelocity - angularVelocity;
        var accel = acceleration * dt ;
        return Mathf.Clamp(error, -accel, accel);
    }

    void UpdateSteering(float dt){

        var speed = Mathf.Max(0, LocalVelocity.z);
        var steerPower = steeringCurve.Evaluate(speed);

        var targetAV = Vector3.Scale(controlInput, turnSpeed * steerPower);
        var av = LocalAngularVelocity * Mathf.Deg2Rad;

        var steering = new Vector3(calculateSteering(dt, av.x, targetAV.x, turnAcceleration.x * steerPower),
        calculateSteering(dt, av.y, targetAV.y, turnAcceleration.y * steerPower),
        calculateSteering(dt, av.z, targetAV.z, turnAcceleration.z * steerPower)
        );

        Rigidbody.AddRelativeTorque(steering * Mathf.Deg2Rad, ForceMode.VelocityChange);

        
        controlInput = new Vector3 (0, 0, 0);

    }

    //  void UpdateThrottle(float dt)
    // {
    //     // Determine the target throttle based on input
    //     targetThrottle = Input.GetKey(KeyCode.T) ? 1f : 0f;

    //     // Smoothly interpolate the Throttle value towards the targetThrottle
    //     Throttle = Mathf.Lerp(Throttle, targetThrottle, throttleSpeed * dt);

    //     // Optionally clamp the value of Throttle between 0 and 1
    //     Throttle = Mathf.Clamp(Throttle, 0f, 1f);

    //     Debug.Log($"Current Throttle: {Throttle}");
    // }

    void UpdateThrust(){

        if (Input.GetKey(KeyCode.T)){
            throttleSpeed += thrustAcceleration;
            throttleSpeed = Mathf.Lerp(throttleSpeed, Mathf.Clamp(throttleSpeed, 0, maxThrust), filterFactor);
            Debug.Log(throttleSpeed);
           
        }
        if (Input.GetKey(KeyCode.B)){
            throttleSpeed-= thrustAcceleration;
            throttleSpeed = Mathf.Lerp(throttleSpeed, Mathf.Clamp(throttleSpeed, 0, maxThrust), filterFactor);
        }
          Rigidbody.AddRelativeForce(throttleSpeed *  Vector3.forward);
        
    }
    void UpdateLift(){

        if (LocalVelocity.sqrMagnitude < 1f) return ;
        
        if(Input.GetKey(KeyCode.F)==true){
            var LiftForce = CalculateLift(
            AngleofAttack + (flapsAOABias * Mathf.Deg2Rad), Vector3.right,
            liftPower + flapsLiftPower,
            liftAoACurve
        );
        Rigidbody.AddRelativeForce(LiftForce);
        }
        else {
        var LiftForce = CalculateLift(
            AngleofAttack , Vector3.right,
            liftPower,
            liftAoACurve
        );
        Rigidbody.AddRelativeForce(LiftForce);
        } 
        var yawForce = CalculateLift(AngleofAttackYaw, Vector3.up, rudderPower, rudderAoACurve);

        
        Rigidbody.AddRelativeForce(yawForce);

    }

    void updateDrag(){
    var v = LocalVelocity;
    var v2 = v.sqrMagnitude;
    float dragCoefficient = 5f;

    Vector3 dragForce = -v.normalized * v2 * dragCoefficient;

    Rigidbody.AddRelativeForce(dragForce);




    }
    void FixedUpdate() {
    float dt = Time.fixedDeltaTime;
    CalculateState(dt);

    // UpdateThrottle(dt);
    UpdateThrust();
    UpdateLift();
    inputsteering();
    UpdateSteering(dt);  
    updateDrag();

    }



     void Update()
    {

    }
   
    
}



