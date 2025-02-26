using System.Collections.Generic;
using UnityEngine;

namespace ArcadeVehicleController
{
    public class Vehicle : MonoBehaviour
    {
        private class SpringData
        {
            public float CurrentLength;
            public float CurrentVelocity;
        }

        private static readonly Wheel[] s_Wheels = new Wheel[]
        {
            Wheel.FrontLeft, Wheel.FrontRight, Wheel.BackLeft, Wheel.BackRight
        };

        private static readonly Wheel[] s_FrontWheels = new Wheel[] { Wheel.FrontLeft, Wheel.FrontRight };
        private static readonly Wheel[] s_BackWheels = new Wheel[] { Wheel.BackLeft, Wheel.BackRight };

        [SerializeField] private VehicleSettings m_Settings;

        private Transform m_Transform;
        private BoxCollider m_BoxCollider;
        private Rigidbody m_Rigidbody;
        private Dictionary<Wheel, SpringData> m_SpringDatas;

        private float m_SteerInput;
        private bool m_ReverseInput;
        private float m_ReverseToFloat;
        private bool m_AccelerateInput;
        private float m_AccelerateToFloat;
        private bool m_BrakeInput;

        public VehicleSettings Settings => m_Settings;
        public Vector3 Forward => m_Transform.forward;
        public Vector3 Velocity => m_Rigidbody.velocity;

        private void Awake()
        {
            m_Transform = transform;
            InitializeCollider();
            InitializeBody();

            m_SpringDatas = new Dictionary<Wheel, SpringData>();
            foreach (Wheel wheel in s_Wheels)
            {
                m_SpringDatas.Add(wheel, new());
            }
        }

        private void FixedUpdate()
        {
            UpdateSuspension();

            UpdateSteering();

            UpdateAccelerate();

            UpdateAirResistance();

            UpdateRotation();

            UpdateBrakeSlideBoost();
        }

        public void SetSteerInput(float steerInput)
        {
            m_SteerInput = Mathf.Clamp(steerInput, -1.0f, 1.0f);
        }

        public void SetReverseInput(bool reverseInput)
        {
            m_ReverseInput = reverseInput;
            if (m_ReverseInput == true)
            {
                m_ReverseToFloat = -1.0f;
            }
            else
            {
                m_ReverseToFloat = 0.0f;
            }
        }

        public void SetAccelerateInput(bool accelerateInput)
        {
            m_AccelerateInput = accelerateInput;
            if (m_AccelerateInput == true)
            {
                m_AccelerateToFloat = 1.0f;
            }
            else
            {
                m_AccelerateToFloat = 0.0f;
            }
        }

        public void SetBrakeInput(bool brakeinput)
        {
            m_BrakeInput = brakeinput;

        }

        public float GetSpringCurrentLength(Wheel wheel)
        {
            return m_SpringDatas[wheel].CurrentLength;
        }

        private void InitializeCollider()
        {
            if (!TryGetComponent(out m_BoxCollider))
            {
                m_BoxCollider = gameObject.AddComponent<BoxCollider>();
            }

            m_BoxCollider.center = Vector3.zero;
            m_BoxCollider.size = new Vector3(m_Settings.Width, m_Settings.Height, m_Settings.Length);
            m_BoxCollider.isTrigger = false;
            m_BoxCollider.enabled = true;
        }

        private void InitializeBody()
        {
            if (!TryGetComponent(out m_Rigidbody))
            {
                m_Rigidbody = gameObject.AddComponent<Rigidbody>();
            }

            const int WHEELS_COUNT = 4;
            m_Rigidbody.mass = m_Settings.ChassiMass + m_Settings.TireMass * WHEELS_COUNT;
            m_Rigidbody.isKinematic = false;
            m_Rigidbody.useGravity = true;
            m_Rigidbody.drag = 0.0f;
            m_Rigidbody.angularDrag = 0.0f;
            m_Rigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            m_Rigidbody.collisionDetectionMode = CollisionDetectionMode.Discrete;
            m_Rigidbody.constraints = RigidbodyConstraints.None;
        }

        // To be called once per physics frame per spring.
        // Updates the spring currentVelocity and currentLength.
        private void CastSpring(Wheel wheel)
        {
            Vector3 position = GetSpringPosition(wheel);

            float previousLength = m_SpringDatas[wheel].CurrentLength;

            float currentLength;

            if (Physics.Raycast(position, -m_Transform.up, out var hit, m_Settings.SpringRestLength))
            {
                currentLength = hit.distance;
            }
            else
            {
                currentLength = m_Settings.SpringRestLength;
            }

            m_SpringDatas[wheel].CurrentVelocity = (currentLength - previousLength) / Time.fixedDeltaTime;
            m_SpringDatas[wheel].CurrentLength = currentLength;
        }

        private Vector3 GetSpringRelativePosition(Wheel wheel)
        {
            Vector3 boxSize = m_BoxCollider.size;
            float boxBottom = boxSize.y * -0.5f;

            float paddingX = m_Settings.WheelsPaddingX;
            float paddingZ = m_Settings.WheelsPaddingZ;

            switch (wheel)
            {
                case Wheel.FrontLeft:
                    return new Vector3(boxSize.x * (paddingX - 0.5f), boxBottom, boxSize.z * (0.5f - paddingZ));
                case Wheel.FrontRight:
                    return new Vector3(boxSize.x * (0.5f - paddingX), boxBottom, boxSize.z * (0.5f - paddingZ));
                case Wheel.BackLeft:
                    return new Vector3(boxSize.x * (paddingX - 0.5f), boxBottom, boxSize.z * (paddingZ - 0.5f));
                case Wheel.BackRight:
                    return new Vector3(boxSize.x * (0.5f - paddingX), boxBottom, boxSize.z * (paddingZ - 0.5f));
                default:
                    return default;
            }
        }

        private Vector3 GetSpringPosition(Wheel wheel)
        {
            return m_Transform.localToWorldMatrix.MultiplyPoint3x4(GetSpringRelativePosition(wheel));
        }

        private Vector3 GetSpringHitPosition(Wheel wheel)
        {
            Vector3 vehicleDown = -m_Transform.up;
            return GetSpringPosition(wheel) + m_SpringDatas[wheel].CurrentLength * vehicleDown;
        }

        private Vector3 GetWheelRollDirection(Wheel wheel)
        {
            bool frontWheel = wheel == Wheel.FrontLeft || wheel == Wheel.FrontRight;

            if (frontWheel)
            {
                var steerQuaternion = Quaternion.AngleAxis(m_SteerInput * m_Settings.SteerAngle, Vector3.up);
                return steerQuaternion * m_Transform.forward;
            }
            else
            {
                return m_Transform.forward;
            }
        }

        private Vector3 GetWheelSlideDirection(Wheel wheel)
        {
            Vector3 forward = GetWheelRollDirection(wheel);

            if (m_AccelerateInput == true)
            {
                return Vector3.Cross(m_Transform.up, forward);
            }
            else
            {
                return (m_Rigidbody.velocity.normalized);
            }
        }

        private Vector3 GetWheelTorqueRelativePosition(Wheel wheel)
        {
            Vector3 boxSize = m_BoxCollider.size;

            float paddingX = m_Settings.WheelsPaddingX;
            float paddingZ = m_Settings.WheelsPaddingZ;

            switch (wheel)
            {
                case Wheel.FrontLeft:
                    return new Vector3(boxSize.x * (paddingX - 0.5f), 0.0f, boxSize.z * (0.5f - paddingZ));
                case Wheel.FrontRight:
                    return new Vector3(boxSize.x * (0.5f - paddingX), 0.0f, boxSize.z * (0.5f - paddingZ));
                case Wheel.BackLeft:
                    return new Vector3(boxSize.x * (paddingX - 0.5f), 0.0f, boxSize.z * (paddingZ - 0.5f));
                case Wheel.BackRight:
                    return new Vector3(boxSize.x * (0.5f - paddingX), 0.0f, boxSize.z * (paddingZ - 0.5f));
                default:
                    return default;
            }
        }

        private Vector3 GetWheelTorquePosition(Wheel wheel)
        {
            return m_Transform.localToWorldMatrix.MultiplyPoint3x4(GetWheelTorqueRelativePosition(wheel));
        }

        private float GetWheelGripFactor(Wheel wheel)
        {
            bool frontWheel = wheel == Wheel.FrontLeft || wheel == Wheel.FrontRight;
            return frontWheel ? m_Settings.FrontWheelsGripFactor : m_Settings.RearWheelsGripFactor;
        }

        private bool IsGrounded(Wheel wheel)
        {
            int WheelIsGrounded = 0;
            if (m_SpringDatas[wheel].CurrentLength < m_Settings.SpringRestLength)
            {
                WheelIsGrounded++;
            }
            else
            {
                WheelIsGrounded--;
            }

            return ((m_SpringDatas[wheel].CurrentLength < 100) && WheelIsGrounded >= 0);
        }

        private void UpdateSuspension()
        {
            foreach (Wheel id in m_SpringDatas.Keys)
            {
                CastSpring(id);
                float currentLength = m_SpringDatas[id].CurrentLength;
                float currentVelocity = m_SpringDatas[id].CurrentVelocity;

                float force = SpringMath.CalculateForceDamped(currentLength, currentVelocity,
                    m_Settings.SpringRestLength, m_Settings.SpringStrength,
                    m_Settings.SpringDamper);

                m_Rigidbody.AddForceAtPosition(force * m_Transform.up, GetSpringPosition(id));
            }
        }

        private void UpdateSteering()
        {
            foreach (Wheel wheel in s_Wheels)
            {
                if (!IsGrounded(wheel))
                {
                    continue;
                }

                Vector3 springPosition = GetSpringPosition(wheel);

                Vector3 slideDirection = GetWheelSlideDirection(wheel);
                float slideVelocity = Vector3.Dot(slideDirection, m_Rigidbody.GetPointVelocity(springPosition));

                float desiredVelocityChange = -slideVelocity * GetWheelGripFactor(wheel);
                float desiredAcceleration = desiredVelocityChange / Time.fixedDeltaTime;

                Vector3 force = desiredAcceleration * m_Settings.TireMass * slideDirection;
                m_Rigidbody.AddForceAtPosition(force, GetWheelTorquePosition(wheel));
            }
        }
        float speedforAcc = 0;
        private void UpdateAccelerate()
        {
            if ((m_AccelerateInput = false) && (m_ReverseInput = false))
            {
                return;
            }
            foreach (Wheel wheel in s_Wheels)
            {
                if (!IsGrounded(wheel))
                {
                    continue;
                }

                Vector3 LocalVelocity = m_Rigidbody.transform.InverseTransformDirection(m_Rigidbody.velocity);

                if (LocalVelocity.x > 0 || LocalVelocity.z > 0)
                {
                    speedforAcc = Mathf.Abs(LocalVelocity.x) + Mathf.Abs(LocalVelocity.z);
                }
                else
                {
                    speedforAcc = LocalVelocity.x + LocalVelocity.z;
                }

                if (speedforAcc > m_Settings.MaxSpeed)
                {
                    return;
                }
                else if (speedforAcc < m_Settings.MaxReverseSpeed)
                {
                    return;
                }



                    Vector3 position = GetWheelTorquePosition(wheel);
                    Vector3 wheelForward = GetWheelRollDirection(wheel);
                    m_Rigidbody.AddForceAtPosition(m_AccelerateToFloat * m_Settings.AcceleratePower * wheelForward, position);
                    m_Rigidbody.AddForceAtPosition(m_ReverseToFloat * m_Settings.ReversePower * wheelForward, position);
                }
            }

        private void UpdateAirResistance()
        {
            m_Rigidbody.AddForce(m_BoxCollider.size.magnitude * m_Settings.AirResistance * -m_Rigidbody.velocity);
        }

        void UpdateRotation()
        {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, -Vector3.up, out hit))
            {
                // Calculate rotation to align with ground normal
                Quaternion targetRotation = Quaternion.FromToRotation(transform.up, hit.normal) * transform.rotation;

                Quaternion newRotation = Quaternion.Slerp(m_Rigidbody.rotation, targetRotation, Time.deltaTime * 5);

                // Apply rotation to Rigidbody
                m_Rigidbody.MoveRotation(newRotation);
            }
            else
            {
                // If no ground detected, maintain current rotation
                m_Rigidbody.MoveRotation(transform.rotation);
            }



        }
        float PowerCharged = 0;
        float BoostPowerLevel = 0;

        void UpdateBrakeSlideBoost()
        {
            if (m_BrakeInput == true)
            {
                
                foreach (Wheel wheel in s_Wheels)
                {
                    if (!IsGrounded(wheel))
                    {
                        continue;
                    }

                   
                    
                    m_Rigidbody.velocity -= m_Rigidbody.velocity * GetWheelGripFactor(wheel)/m_Settings.BrakesPower * Time.fixedDeltaTime;
                    PowerCharged += m_Rigidbody.velocity.magnitude * GetWheelGripFactor(wheel)/m_Settings.BrakesPower * m_Settings.PowerChargeScale * Time.fixedDeltaTime;
                    Debug.Log("Boost is" + PowerCharged);

                }



            }
            if ((m_BrakeInput == false) && PowerCharged > 2f)
            {
                foreach (Wheel wheel in s_Wheels)
                {

                    if (!IsGrounded(wheel))
                    {
                        continue;
                    }

                    if (PowerCharged < 20)
                    {
                        BoostPowerLevel = m_Settings.BoostPowerT1;
                    }
                    if ((PowerCharged >= 20) && (PowerCharged < 30))
                    {
                        BoostPowerLevel = m_Settings.BoostPowerT2;
                    }
                    if ((PowerCharged >= 30) && (PowerCharged < 40))
                    {
                        BoostPowerLevel = m_Settings.BoostPowerT3;
                    }
                    if (PowerCharged >= 40)
                    {
                        BoostPowerLevel = m_Settings.BoostPowerT4;
                    }

                    int WHEELS_COUNT = 4;
                    Vector3 position = GetWheelTorquePosition(wheel);
                    Vector3 wheelForward = GetWheelRollDirection(wheel);
                    m_Rigidbody.AddForceAtPosition(BoostPowerLevel / WHEELS_COUNT * wheelForward, position, ForceMode.VelocityChange);
                }
            }
            if ((m_BrakeInput == false) && (BoostPowerLevel > 0))
            {
                BoostPowerLevel = 0;
                PowerCharged = 0;
            }
        }
    }
}