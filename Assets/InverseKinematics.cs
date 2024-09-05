using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using System.Linq;

public class InverseKinematics : MonoBehaviour
{
    // Define robot arm geometry and kinematic parameters
    // a: the distance from the previous joint to the current joint along the x-axis (in meters)
    //alpha: the angle between the previous z-axis and the current z-axis around the x-axis(in radians)
    //d: the distance from the previous joint to the current joint along the z-axis(in meters)
    //theta: the angle between the previous x-axis and the current x-axis around the z-axis(in radians)

    public float gripper_length = 0.07f;
    public Toggle objective_control;
    public bool toggle_on = true;
    public GameObject Tatget_vis; // to control the visulization of the target

    private float[] linkLengths_z = { 0.13f, 0, 0, 0.064f, 0.073f, 0.0486f };
    private float[] linkLengths_x = { 0, 0.11f, 0.096f, 0,  0, 0f }; 
    private float[] alpha = { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };
        
    private float[] jointAngles_initial = { 0, Mathf.PI / 2, 0, Mathf.PI / 2, Mathf.PI / 2, 0 }; // initial joint angles in radians

    private static float[] jointAngles;

    public float[] jointLimits = { -Mathf.PI, Mathf.PI }; // joint angle limits in radians

    private float[] low_jointLimits = { -Mathf.PI, -Mathf.PI/2, -Mathf.PI, -Mathf.PI, -Mathf.PI, -Mathf.PI }; // joint angle limits in radians

    private float[] up_jointLimits = { Mathf.PI, Mathf.PI/2, Mathf.PI, Mathf.PI, Mathf.PI, Mathf.PI }; // joint angle limits in radians

    private float[] low_jointLimits_adjusted;
    private float[] up_jointLimits_adjusted;

    private float offsset_z = 0.04f; // to compensate the offset of the object
    private float offsset_y = 0f;
    private float offsset_x = -0.01f;

    //public float Dampper_factor = 0.8f;


    // public Transform[] links = new Transform[6]; // array of link GameObjects
    public Transform joints_base; // array of joint GameObjects
    public Transform[] joints = new Transform[6]; // array of joint GameObjects
    public Transform EndEffector; // end-effector GameObject

    private Transform Target;
    // private variables

    private Matrix4x4[] T = new Matrix4x4[7];
      
    public Slider[] sliders;

    Vector3 lastPos;
    Quaternion lastRot;

    private float[] lastTargetAngles;
    float[] currentAngles;
    int skipCounter = 0;
    public int skipCounterThreshold = 5;
    int cout = 1;

    // Implement inverse kinematics solver using Jacobian method
    private float[] error = { 0, 1, 0 }; // error between current and desired end-effector pose
    private float[] deltaTheta = new float[6]; // change in joint angles
    private float[] deltaPose = new float[6]; // change in end-effector pose
    private float[,] J = new float[6, 6]; // Jacobian matrix
    private float[,] Jinv = new float[6, 6]; // inverse Jacobian matrix

    // Start is called before the first frame update
    void Start()
    {
        linkLengths_z[5] = linkLengths_z[5] + gripper_length; // add the gripper_length
        low_jointLimits_adjusted = new float[low_jointLimits.Length];
        up_jointLimits_adjusted = new float[up_jointLimits.Length];

        jointAngles = jointAngles_initial.ToArray();

        Tatget_vis.SetActive(false);

        for (int i = 0; i < low_jointLimits.Length; i++)
        {
            low_jointLimits_adjusted[i] = low_jointLimits[i] + jointAngles_initial[i];
            up_jointLimits_adjusted[i] = up_jointLimits[i] + jointAngles_initial[i];

        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        try
        {

            if (objective_control.isOn)
            {

            
                if ( (Target.transform.position != lastPos || Target.transform.rotation != lastRot) && toggle_on ) // || Target.transform.rotation != lastRot
                {
                    Debug.Log(cout++);
                    if (skipCounter++ > skipCounterThreshold)
                    {
                        skipCounter = 0;
                        //Inverse Kinematics
                        Vector3 r_des = Target.transform.position - joints_base.position; // assume the original coordinate original point on robot base

                        Vector3 new_targetPosition = r_des;

                        // offset setting
                        r_des = new Vector3(new_targetPosition.x + offsset_x, new_targetPosition.y + offsset_y, new_targetPosition.z + offsset_z);

                        r_des.z = r_des.z - gripper_length; // consider the gripper_length

                        Debug.Log(r_des.ToString());

                        // float[] jointAngles = jointAngles_initial;

                        //Quaternion C_des = Target.transform.rotation; //  Quaternion C_des = Quaternion.Inverse(joints[0].rotaion) * Target.transform.rotation;

                        //InverseKinematics_closedform(r_des, C_des, linkLengths_x, linkLengths_z, alpha, out jointAngles);

                        Vector3 C_des = Target.transform.localEulerAngles;

                        if ((Target.transform.position - lastPos).magnitude > 0.15)
                        {
                            jointAngles = jointAngles_initial.ToArray(); // initial joint angles in radians
                        }
                    
                        jointAngles = ComputeInverseKinematics(r_des, C_des);

                        lastPos = Target.transform.position;
                        lastRot = Target.transform.rotation;

                        if (jointAngles.Any(x => float.IsNaN(x)))
                        {
                            Debug.Log("was NaN value");
                            return;
                        }

                        float[] currentAngles = new float[6];

                        for (int ii = 0; ii < jointAngles.Length; ii++) { currentAngles[ii] = jointAngles[ii] - jointAngles_initial[ii]; jointAngles[ii] = angle_autotransform(jointAngles[ii]); }

                    

                        //lastTargetAngles = q;

                        // pass the angle value to slider
                        int i = 0;

                        if (currentAngles != null && currentAngles.Length != 0 )
                        {
                            foreach (float value in currentAngles)
                            {
                                Debug.Log(value.ToString());
                                sliders[i].value = ((float)value * 180f / Mathf.PI);
                                i++;
                            }
                        }

                    
                    }
                
                }
            }

        }
        catch
        {

        }
    }

    public void Toggle_on()
    {
        toggle_on = true;
    }

    public void Toggle_off()
    {
        toggle_on = false;
    }

    // Compute forward kinematics of robot arm
    private void ComputeForwardKinematics() {

        //T = GetMatrix4x4();

        // T[0] = Matrix4x4.identity; // base frame
        for (int i = 0; i < 6; i++) {
        // compute transformation matrix from i-th joint to (i+1)-th joint
      //  T[i+1] = T[i] * DHTransform(linkLengths[i], jointAngles[i], 0, 0);
        // set position and rotation of i-th link and joint GameObjects
                 
        //links[i].position = GetPosition(T[i + 1]);

        //links[i].rotation = GetRotation(T[i + 1]);
        joints[i].position = GetPosition(T[i + 1]);
        joints[i].rotation = GetRotation(T[i + 1]);
    }
    // set position and rotation of end-effector GameObject
    EndEffector.position = GetPosition(T[6]);
    EndEffector.rotation = GetRotation(T[6]);
    }

       
    public float[] ComputeInverseKinematics(Vector3 position, Vector3 euler)
    {         
        // compute desired end-effector pose
        float[] desiredPose = { position.z,  -position.x,  position.y, euler.z, -euler.x, euler.y }; // the coordinate should be changed

        float threshold = 0.05f; // error threshold for convergence
        int maxIterations = 400; // maximum number of iterations
        int iteration = 1; // current iteration
        float weight = 1000f; // the weight for position

        // Vector3 error = Vector3.up;

        T = GetMatrix4x4();
        Vector3 P_tem0 = GetPosition(T[6]);

        Vector3 error = new Vector3 ( desiredPose[0] - P_tem0.x, desiredPose[1] - P_tem0.y, desiredPose[2] - P_tem0.z );

        Vector3 error_rotation = Vector3.zero;

        // float[] jointAngles = jointAngles_initial; // initial joint angles in radians

        Vector3 R_tem_initial = GetRotation(T[6]).eulerAngles;


        while ( error.magnitude + error_rotation.magnitude > threshold && iteration < maxIterations)
        {
            /*
            // the Jacobian method (unsuccess)
            // compute Jacobian matrix
            for (int i = 0; i < 6; i++)
            {
                // compute partial derivatives of end-effector pose w.r.t. i-th joint angle
                deltaPose[0] = T[6].m03 - T[i].m03;
                deltaPose[1] = T[6].m13 - T[i].m13;
                deltaPose[2] = T[6].m23 - T[i].m23;
                deltaPose[3] = T[6].m01 * T[i].m02 - T[6].m02 * T[i].m01;
                deltaPose[4] = T[6].m01 * T[i].m12 - T[6].m02 * T[i].m11;
                deltaPose[5] = T[6].m01 * T[i].m22 - T[6].m02 * T[i].m21;
                J[0, i] = deltaPose[0];
                J[1, i] = deltaPose[1];
                J[2, i] = deltaPose[2];
                J[3, i] = deltaPose[3];
                J[4, i] = deltaPose[4];
                J[5, i] = deltaPose[5];
            }

            // compute inverse Jacobian matrix with damping factor
            float lambda = 0.0001f; // set damping factor
            for (int i = 0; i < 6; i++)
            {
                J[i, i] += lambda;
            }


            // compute inverse Jacobian matrix
            Jinv = InverseJacobian_2(J);
                        
            Vector3 P_tem = GetPosition(T[6]);
            deltaPose[0] = desiredPose[0] - P_tem.x;
            deltaPose[1] = desiredPose[1] - P_tem.y;
            deltaPose[2] = desiredPose[2] - P_tem.z;

            Quaternion R_tem = GetRotation(T[6]);
            deltaPose[3] = desiredPose[3] - R_tem.x;
            deltaPose[5] = desiredPose[4] - R_tem.y;
            deltaPose[4] = desiredPose[5] - R_tem.z;
            // contrain the rotation
            deltaPose[3] = 0;
            deltaPose[5] = 0;
            deltaPose[4] = 0;


            // compute change in joint angles
            for (int i = 0; i < 6; i++)
            {
                deltaTheta[i] = 0;
                for (int j = 0; j < 6; j++)
                {
                    deltaTheta[i] += Math.Max(Math.Min(Jinv[i, j] * deltaPose[j], 0.1f), -0.1f);
                }
                // limit joint angle within valid range
                jointAngles[i] = Mathf.Clamp(jointAngles[i] + deltaTheta[i], jointLimits[0], jointLimits[1]);
            }
            

            // update forward kinematics
            T = GetMatrix4x4();
            error = new Vector3(desiredPose[0] - P_tem.x, desiredPose[1] - P_tem.y, desiredPose[2] - P_tem.z);

            */

            //ComputeForwardKinematics();


            // the interative method using numerical gradient (success)
            // compute error between current and desired end-effector pose

            //float error_old = error.magnitude;
            float error_old = error.magnitude * weight + error_rotation.magnitude;

            float it_f = (float) iteration;
            
            for (int i = 0; i < 6; i++)
            {
                jointAngles[i] = Mathf.Clamp(jointAngles[i] + 0.1f, low_jointLimits_adjusted[i], up_jointLimits_adjusted[i]);
                //jointAngles[i] = jointAngles[i] + 0.1f;
                T = GetMatrix4x4();
                Vector3 P_tem = GetPosition(T[6]);
                
                error = new Vector3(desiredPose[0] - P_tem.x, desiredPose[1] - P_tem.y, desiredPose[2] - P_tem.z);

                Vector3 R_tem = GetRotation(T[6]).eulerAngles - R_tem_initial; // set the iinitial pose as zero
                error_rotation = new Vector3(desiredPose[3] - R_tem.x, desiredPose[4] - R_tem.y, desiredPose[5] - R_tem.z)/ 180f * Mathf.PI;

                if (error.magnitude * weight + error_rotation.magnitude > error_old)
                {
                    jointAngles[i] = Mathf.Clamp(jointAngles[i] - 0.2f, low_jointLimits_adjusted[i], up_jointLimits_adjusted[i]);
                    //jointAngles[i] = jointAngles[i] - 0.2f;

                    T = GetMatrix4x4();
                    Vector3 P_tem1 = GetPosition(T[6]);
                    
                    error = new Vector3(desiredPose[0] - P_tem1.x, desiredPose[1] - P_tem1.y, desiredPose[2] - P_tem1.z);

                    Vector3 R_tem1 = GetRotation(T[6]).eulerAngles - R_tem_initial;
                    error_rotation = new Vector3(desiredPose[3] - R_tem1.x, desiredPose[4] - R_tem1.y, desiredPose[5] - R_tem1.z)/180f*Mathf.PI;

                    
                    if (error.magnitude * weight + error_rotation.magnitude > error_old)
                    {
                        jointAngles[i] = Mathf.Clamp(jointAngles[i] + 0.1f, low_jointLimits_adjusted[i], up_jointLimits_adjusted[i]);                        
                    }
                    

                }
                //jointAngles[i] = angle_autotransform(jointAngles[i]);

                if (i != 1 )
                {
                    if (jointAngles[i] > up_jointLimits_adjusted[i] - 0.1f)
                    {
                        jointAngles[i] = low_jointLimits_adjusted[i] + 0.1f;
                    }
                    else if (jointAngles[i] < low_jointLimits_adjusted[i] + 0.1f)
                    {
                        jointAngles[i] = up_jointLimits_adjusted[i] - 0.1f;
                    }
                }
                
                
            }


            

            // error[0] = desiredPose[0] - P_tem.x;
            // error[1] = desiredPose[1] - P_tem.y;
            // error[2] = desiredPose[2] - P_tem.z;
            // increment iteration counter

            iteration++;
        }


        // output final joint angles
        Debug.Log("Joint angles: " + jointAngles.ToString());

        return jointAngles;

    }




    private float angle_autotransform(float angle)
    {
       if (angle < jointLimits[0])
        {
            angle = angle + Mathf.PI;
        }
       else if (angle > jointLimits[1])
        {
            angle = angle - Mathf.PI;
        }
        
        return angle;
    }

    // Helper function to compute DH transformation matrix
    private Matrix4x4 DHTransform(float a, float alpha, float d, float theta)
    {
        Matrix4x4 T_tem = Matrix4x4.identity;
        T_tem.m00 = Mathf.Cos(theta);
        T_tem.m01 = -Mathf.Sin(theta) * Mathf.Cos(alpha);
        T_tem.m02 = Mathf.Sin(theta) * Mathf.Sin(alpha);
        T_tem.m03 = a * Mathf.Cos(theta);

        T_tem.m10 = Mathf.Sin(theta);
        T_tem.m11 = Mathf.Cos(theta) * Mathf.Cos(alpha);
        T_tem.m12 = -Mathf.Cos(theta) * Mathf.Sin(alpha);
        T_tem.m13 = a * Mathf.Sin(theta);

        T_tem.m21 = Mathf.Sin(alpha);
        T_tem.m22 = Mathf.Cos(alpha);
        T_tem.m23 = d;

        return T_tem;
    }

    private Vector3 GetPosition(Matrix4x4 matrix)
    {
        return new Vector3(matrix.m03, matrix.m13, matrix.m23);
    }

    private Quaternion GetRotation(Matrix4x4 matrix)
    {
        Vector3 forward = new Vector3(matrix.m02, matrix.m12, matrix.m22);
        Vector3 upwards = new Vector3(matrix.m01, matrix.m11, matrix.m21);
        return Quaternion.LookRotation(forward, upwards);
    }

    public static float Magnitude(float[] array)
    {
        float sumOfSquares = 0.0f;

        for (int i = 0; i < array.Length; i++)
        {
            sumOfSquares += array[i] * array[i];
        }

        return (float)Math.Sqrt(sumOfSquares);
    }

    /*
    public float[,] InverseJacobian6x6(float[,] J)
    {
        float[,] Jinv = new float[6, 6]; 
        float detJ = J[0, 0] * (J[1, 1] * J[2, 2] - J[2, 1] * J[1, 2]) - J[0, 1] * (J[1, 0] * J[2, 2] - J[1, 2] * J[2, 0]) + J[0, 2] * (J[1, 0] * J[2, 1] - J[1, 1] * J[2, 0]);

        if (Mathf.Abs(detJ) < 0.0001f)
        {
            Debug.LogWarning("Jacobian matrix is singular, cannot compute inverse");
            //Jinv = PseudoInverse(J);
            return Jinv;
        }

        float invDetJ = 1f / detJ;

        Jinv[0, 0] = (J[1, 1] * J[2, 2] - J[2, 1] * J[1, 2]) * invDetJ;
        Jinv[0, 1] = (J[0, 2] * J[2, 1] - J[0, 1] * J[2, 2]) * invDetJ;
        Jinv[0, 2] = (J[0, 1] * J[1, 2] - J[0, 2] * J[1, 1]) * invDetJ;
        Jinv[1, 0] = (J[1, 2] * J[2, 0] - J[1, 0] * J[2, 2]) * invDetJ;
        Jinv[1, 1] = (J[0, 0] * J[2, 2] - J[0, 2] * J[2, 0]) * invDetJ;
        Jinv[1, 2] = (J[1, 0] * J[0, 2] - J[0, 0] * J[1, 2]) * invDetJ;
        Jinv[2, 0] = (J[1, 0] * J[2, 1] - J[2, 0] * J[1, 1]) * invDetJ;
        Jinv[2, 1] = (J[2, 0] * J[0, 1] - J[0, 0] * J[2, 1]) * invDetJ;
        Jinv[2, 2] = (J[0, 0] * J[1, 1] - J[1, 0] * J[0, 1]) * invDetJ;
        Jinv[3, 3] = 1f;
        Jinv[4, 4] = 1f;
        Jinv[5, 5] = 1f;

        return Jinv;
    }

    
    public float[,] PseudoInverse(float[,] J) { 
        Matrix Jmat = new Matrix(J); 
        Matrix Jt = Jmat.Transpose(); 
        Matrix JJt = Jmat.Multiply(Jt); 
        SVD svd = new SVD(JJt); Matrix U = svd.U; 
        Matrix S = svd.S; 
        Matrix V = svd.V; 
        Matrix Sinv = new Matrix(S.Rows, S.Cols); 
        for (int i = 0; i < S.Rows; i++) { 
            if (S[i, i] > 0.0001f) { 
                Sinv[i, i] = 1f / S[i, i]; 
            } 
            else {
                Sinv[i, i] = 0f; 
            } 
        } 
        Matrix Jinv = V.Multiply(Sinv).Multiply(U.Transpose()).Multiply(Jmat); 
        return Jinv.ToArray(); 
    }
    */

            // inverse compute
            private float[,] InverseJacobian(float[,] J)
    {
        float det = Determinant(J);
        float[,] adj = Adjoint(J);
        float[,] invJ = ScalarMultiply(adj, 1 / det);

        if (det == 0)
        {
            // Matrix is not invertible
            Debug.Log("Matrix is not invertible");
        }

        return invJ;
    }

    private float Determinant(float[,] matrix)
    {
        int n = matrix.GetLength(0);
        float det = 0;
        if (n == 1)
        {
            det = matrix[0, 0];
        }
        else if (n == 2)
        {
            det = matrix[0, 0] * matrix[1, 1] - matrix[1, 0] * matrix[0, 1];
        }
        else
        {
            for (int j = 0; j < n; j++)
            {
                float[,] submatrix = new float[n - 1, n - 1];
                for (int i = 1; i < n; i++)
                {
                    for (int k = 0; k < n; k++)
                    {
                        if (k < j)
                        {
                            submatrix[i - 1, k] = matrix[i, k];
                        }
                        else if (k > j)
                        {
                            submatrix[i - 1, k - 1] = matrix[i, k];
                        }
                    }
                }
                det += matrix[0, j] * (float)Math.Pow(-1, j) * Determinant(submatrix);
            }
        }
        return det;
    }

    private float[,] Adjoint(float[,] matrix)
    {
        int n = matrix.GetLength(0);
        float[,] adj = new float[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                float[,] submatrix = new float[n - 1, n - 1];
                for (int k = 0; k < n; k++)
                {
                    for (int l = 0; l < n; l++)
                    {
                        if (k < i && l < j)
                        {
                            submatrix[k, l] = matrix[k, l];
                        }
                        else if (k < i && l > j)
                        {
                            submatrix[k, l - 1] = matrix[k, l];
                        }
                        else if (k > i && l < j)
                        {
                            submatrix[k - 1, l] = matrix[k, l];
                        }
                        else if (k > i && l > j)
                        {
                            submatrix[k - 1, l - 1] = matrix[k, l];
                        }
                    }
                }
                adj[j, i] = (float)Math.Pow(-1, i + j) * Determinant(submatrix);
            }
        }
        return adj;
    }

    private float[,] ScalarMultiply(float[,] matrix, float scalar)
    {
        int n = matrix.GetLength(0);
        int m = matrix.GetLength(1);
        float[,] result = new float[n, m];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                result[i, j] = matrix[i, j] * scalar;
            }
        }
        return result;
    }

    private Matrix4x4[] GetMatrix4x4()
    {
        T[0] = Matrix4x4.identity; // base frame

        if (jointAngles.Any(x => float.IsNaN(x)))
        {
            Debug.Log("was NaN value");
            float[] jointAngles = jointAngles_initial; // initial joint angles in radians
        }
        

        for (int i = 0; i < 6; i++)
        {
            // compute transformation matrix from i-th joint to (i+1)-th joint
            T[i + 1] = T[i] * DHTransform(linkLengths_x[i], alpha[i], linkLengths_z[i], jointAngles[i]);            
        }
        return T;
    }

    public bool InverseKinematics_closedform(Vector3 targetPosition, Quaternion targetOrientation, float[] d, float[] a, float[] alpha, out float[] jointAngles)
    {
        //float[] a = { 0, -0.425f, -0.39225f, 0, 0, 0 };
        //float[] alpha = { -Mathf.PI / 2, 0, 0, -Mathf.PI / 2, Mathf.PI / 2, 0 };
        //float[] d = { 0.089159f, 0, 0, 0.10915f, 0.09465f, 0.0823f };

        float[] theta = new float[6];
        

        // Compute transformation matrices for each joint
        Matrix4x4[] T = new Matrix4x4[6];

        T = GetMatrix4x4();

        /*
        for (int i = 0; i < 6; i++)
        {
            T[i] = DHTransform(a[i], alpha[i], d[i], theta[i]);
        }
        */


        ComputeForwardKinematics();

        // Compute target pose matrix
        Matrix4x4 T06 = Matrix4x4.TRS(targetPosition, targetOrientation, Vector3.one);

        // Compute wrist position and orientation
        Vector3 wristPosition = T06.GetColumn(3) - T06.GetColumn(2) * d[5];
        Quaternion wristOrientation = Quaternion.LookRotation(T06.GetColumn(2), T06.GetColumn(1));

        // Compute joint angles using closed-form inverse kinematics
        float[] q = new float[6];
        float r13 = wristOrientation.eulerAngles.x * Mathf.Deg2Rad;
        float r23 = wristOrientation.eulerAngles.y * Mathf.Deg2Rad;
        float r33 = wristOrientation.eulerAngles.z * Mathf.Deg2Rad;
        float r31 = Mathf.Sin(r23) * Mathf.Cos(r13);
        float r32 = Mathf.Sin(r23) * Mathf.Sin(r13);
        float r21 = Mathf.Cos(r23) * Mathf.Sin(r33) - Mathf.Sin(r23) * Mathf.Cos(r13) * Mathf.Cos(r33);
        float r11 = Mathf.Cos(r23) * Mathf.Cos(r33) + Mathf.Sin(r23) * Mathf.Cos(r13) * Mathf.Sin(r33);
        float px = wristPosition.x - a[5] * r13 * r21 - a[5] * r23 * r11;
        float py = wristPosition.y - a[5] * r13 * r31 - a[5] * r23 * r21;
        float pz = wristPosition.z - d[0] - d[4] - d[5] - a[5] * r33;
        float phi = Mathf.Atan2(py, px);
        float c3 = (px * px + py * py + pz * pz - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]);
        float s3 = Mathf.Sqrt(1 - c3 * c3); //wrong
        q[2] = Mathf.Atan2(s3, c3);
        float k1 = a[1] + a[2] * c3;
        float k2 = a[2] * s3;
        float c2 = (k1 * pz - k2 * Mathf.Sin(q[2])) / (k1 * k1 + k2 * k2);
        float s2 = Mathf.Sqrt(1 - c2 * c2);
        q[1] = Mathf.Atan2(s2, c2);
        float c23 = Mathf.Cos(q[1] + q[2]);
        float s23 = Mathf.Sin(q[1] + q[2]);
        q[0] = phi - Mathf.Atan2(py * (a[1] + a[2] * c23) - px * a[2] * s23,
                                  px * (a[1] + a[2] * c23) + py * a[2] * s23);
        q[3] = r33 - q[1] - q[2];
        q[4] = r23 - q[0];
        q[5] = r13;

        // Check if joint angles are within limits
        bool withinLimits = true;
        for (int i = 0; i < 6; i++)
        {
            if (q[i] < jointLimits[0] || q[i] > jointLimits[1])
            {
                withinLimits = false;
                break;
            }
        }

        // Return joint angles if within limits
        if (withinLimits)
        {
            jointAngles = q;
            return true;
        }
        else
        {
            jointAngles = null;
            return false;
        }
    }

    //method 1
    private float[,] InverseJacobian_1(float[,] J)
    {
        // compute determinant of 3x3 submatrix
        float det3x3 = J[0, 0] * (J[1, 1] * J[2, 2] - J[1, 2] * J[2, 1])
                     - J[0, 1] * (J[1, 0] * J[2, 2] - J[1, 2] * J[2, 0])
                     + J[0, 2] * (J[1, 0] * J[2, 1] - J[1, 1] * J[2, 0]);

        // compute inverse of 3x3 submatrix
        float[,] inv3x3 = new float[3, 3];
        inv3x3[0, 0] = (J[1, 1] * J[2, 2] - J[1, 2] * J[2, 1]) / det3x3;
        inv3x3[0, 1] = (J[0, 2] * J[2, 1] - J[0, 1] * J[2, 2]) / det3x3;
        inv3x3[0, 2] = (J[0, 1] * J[1, 2] - J[0, 2] * J[1, 1]) / det3x3;
        inv3x3[1, 0] = (J[1, 2] * J[2, 0] - J[1, 0] * J[2, 2]) / det3x3;
        inv3x3[1, 1] = (J[0, 0] * J[2, 2] - J[0, 2] * J[2, 0]) / det3x3;
        inv3x3[1, 2] = (J[1, 0] * J[0, 2] - J[0, 0] * J[1, 2]) / det3x3;
        inv3x3[2, 0] = (J[1, 0] * J[2, 1] - J[2, 0] * J[1, 1]) / det3x3;
        inv3x3[2, 1] = (J[2, 0] * J[0, 1] - J[0, 0] * J[2, 1]) / det3x3;
        inv3x3[2, 2] = (J[0, 0] * J[1, 1] - J[1, 0] * J[0, 1]) / det3x3;

        // compute inverse of full 6x6 matrix
        float[,] invJ = new float[6, 6];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                invJ[i, j] = inv3x3[i, j];
            }
            invJ[i + 3, i + 3] = 1.0f;
        }

        // compute remaining elements of inverse matrix
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                {
                    sum += inv3x3[i, k] * J[k + 3, j];
                }
                invJ[i, j + 3] = -sum;
            }
        }

        return invJ;
    }

    public static float[,] InverseJacobian_2(float [,] J)
    {
        int n = J.GetLength(0);
        float[,] Jinv = new float[n, n];

        // compute determinant of J
        float det = J[0, 0] * (J[1, 1] * J[2, 2] - J[1, 2] * J[2, 1])
                   - J[0, 1] * (J[1, 0] * J[2, 2] - J[1, 2] * J[2, 0])
                   + J[0, 2] * (J[1, 0] * J[2, 1] - J[1, 1] * J[2, 0])
                   - J[0, 3] * (J[1, 1] * J[2, 5] - J[1, 5] * J[2, 1])
                   + J[0, 4] * (J[1, 0] * J[2, 5] - J[1, 5] * J[2, 0])
                   - J[0, 5] * (J[1, 0] * J[2, 4] - J[1, 4] * J[2, 0]);

        // compute adjugate of J
        float[,] adj = new float[n, n];
        adj[0, 0] = J[1, 1] * J[2, 2] - J[1, 2] * J[2, 1];
        adj[0, 1] = -(J[0, 1] * J[2, 2] - J[0, 2] * J[2, 1]);
        adj[0, 2] = J[0, 1] * J[1, 2] - J[0, 2] * J[1, 1];
        adj[0, 3] = -(J[1, 1] * J[2, 5] - J[1, 5] * J[2, 1]);
        adj[0, 4] = J[0, 1] * J[2, 5] - J[0, 5] * J[2, 1];
        adj[0, 5] = -(J[0, 1] * J[1, 5] - J[0, 5] * J[1, 1]);
        adj[1, 0] = -(J[1, 0] * J[2, 2] - J[1, 2] * J[2, 0]);
        adj[1, 1] = J[0, 0] * J[2, 2] - J[0, 2] * J[2, 0];
        adj[1, 2] = -(J[0, 0] * J[1, 2] - J[0, 2] * J[1, 0]);
        adj[1, 3] = J[1, 0] * J[2, 5] - J[1, 5] * J[2, 0];
        adj[1, 4] = -(J[0, 0] * J[2, 5] - J[0, 5] * J[2, 0]);
        adj[1, 5] = J[0, 0] * J[1, 5] - J[0, 5] * J[1, 0];
        adj[2, 0] = J[1, 0] * J[2, 1] - J[1, 1] * J[2, 0];
        adj[2, 1] = -(J[0, 0] * J[2, 1] - J[0, 1] * J[2, 0]);
        adj[2, 2] = J[0, 0] * J[1, 1] - J[0, 1] * J[1, 0];
        adj[2, 3] = -(J[1, 0] * J[2, 4] - J[1, 4] * J[2, 0]);
        adj[2, 4] = J[0, 0] * J[2, 4] - J[0, 4] * J[2, 0];
        adj[2, 5] = -(J[0, 0] * J[1, 4] - J[0, 4] * J[1, 0]);
        adj[3, 0] = -(J[1, 1] * J[2, 5] - J[1, 5] * J[2, 1]);
        adj[3, 1] = J[0, 1] * J[2, 5] - J[0, 5] * J[2, 1];
        adj[3, 2] = -(J[0, 1] * J[1, 5] - J[0, 5] * J[1, 1]);
        adj[3, 3] = J[1, 1] * J[2, 2] - J[1, 2] * J[2, 1];
        adj[3, 4] = -(J[0, 1] * J[2, 2] - J[0, 2] * J[2, 1]);
        adj[3, 5] = J[0, 1] * J[1, 2] - J[0, 2] * J[1, 1];
        adj[4, 0] = J[1, 0] * J[2, 5] - J[1, 5] * J[2, 0];
        adj[4, 1] = -(J[0, 0] * J[2, 5] - J[0, 5] * J[2, 0]);
        adj[4, 2] = J[0, 0] * J[1, 5] - J[0, 5] * J[1, 0];
        adj[4, 3] = -(J[1, 0] * J[2, 2] - J[1, 2] * J[2, 0]);
        adj[4, 4] = J[0, 0] * J[2, 2] - J[0, 2] * J[2, 0];
        adj[4, 5] = -(J[0, 0] * J[1, 2] - J[0, 2] * J[1, 0]);
        adj[5, 0] = -(J[1, 0] * J[2, 4] - J[1, 4] * J[2, 0]);
        adj[5, 1] = J[0, 0] * J[2, 4] - J[0, 4] * J[2, 0];
        adj[5, 2] = -(J[0, 0] * J[1, 4] - J[0, 4] * J[1, 0]);
        adj[5, 3] = J[1, 0] * J[2, 1] - J[1, 1] * J[2, 0];
        adj[5, 4] = -(J[0, 0] * J[2, 1] - J[0, 1] * J[2, 0]);
        adj[5, 5] = J[0, 0] * J[1, 1] - J[0, 1] * J[1, 0];

        // compute inverse of J
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                Jinv[i, j] = adj[i, j] / det;
            }
        }

        return Jinv;
    }
}