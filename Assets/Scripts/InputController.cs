using UnityEngine;
using UnityEngine.InputSystem;

public class InputController : MonoBehaviour
{

    public Vector3 controlInput;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.A)){
            controlInput.x += 0.1f;
            controlInput.x = Mathf.Clamp(controlInput.x, -1f, 1f);

        }
        if (Input.GetKey(KeyCode.D)){
            controlInput.x -= 0.1f;
            controlInput.x = Mathf.Clamp(controlInput.x, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.W)){
            controlInput.y -= 0.1f;
            controlInput.y = Mathf.Clamp(controlInput.y, -1f, 1f);
            
        }
        if (Input.GetKey(KeyCode.S)){
            controlInput.y += 0.1f;
            controlInput.y = Mathf.Clamp(controlInput.y, -1f, 1f);
            
        }
        if (Keyboard.current.xKey.isPressed){
            controlInput.z += 0.1f;
            controlInput.z = Mathf.Clamp(controlInput.z, -1f, 1f);
            
        }
        if (Keyboard.current.zKey.isPressed){
            controlInput.z += 0.1f;
            controlInput.z = Mathf.Clamp(controlInput.z, -1f, 1f);
            
        }
    }
}
